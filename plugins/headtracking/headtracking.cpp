// license:GPLv3+
// vpx-headtracking: 6-DOF head tracking for VPX Standalone.
// Reads a head pose over UDP in opentrack's "UDP over network" format
// (6 little-endian doubles: x,y,z,yaw,pitch,roll) and, on each OnPrepareFrame,
// writes the VPX eye position (viewX/Y/Z) so VLM_WINDOW mode renders an
// off-axis "window into the cabinet" perspective. Tracker-agnostic: any source
// that speaks opentrack UDP (webcam/neuralnet, Kinect, TrackIR, ...) drives it.
//
// Config via env: HT_UDP_PORT (default 4242), HT_SCALE_X/Y/Z (VPU per cm, default 1).

#include "plugins/MsgPlugin.h"
#include "plugins/VPXPlugin.h"

#include <atomic>
#include <mutex>
#include <thread>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <sys/stat.h>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

namespace HeadTracking {

const MsgPluginAPI* msgApi = nullptr;
VPXPluginAPI* vpxApi = nullptr;
uint32_t endpointId;
unsigned int getVpxApiId, onGameStartId, onGameEndId, onPrepareFrameId;

static std::atomic<bool> g_running{false};
static std::thread       g_udpThread;
static int               g_sock = -1;
static std::mutex        g_poseMtx;
static double            g_pose[6] = {0,0,0,0,0,0};
static std::atomic<bool> g_haveBase{false};
static float             g_baseX=0.f, g_baseY=0.f, g_baseZ=0.f;
static float             g_tableLength = 2000.f;   // VPU; captured with the base
static float             g_rotSign = 0.f;           // 0 = unproven; ±1 once self-tested
static float             g_zOffset = 0.f;           // wbot*sceneScaleY/realToVirtual, captured
static long              g_frames = 0;
static double            g_poseTime = 0.0;

constexpr double HT_STALE_S = 1.0;   // no packet for this long => tracker is gone

static float envF(const char* name, float dflt) { const char* v = getenv(name); return v ? (float)atof(v) : dflt; }

// LIVE GAIN. 1:1 parallax (18.527 VPU/cm, i.e. the view moves exactly as the real
// world would) is physically honest and, on a cab, wildly too much — Joe's words were
// "way too much". Everyone runs this well under 1. Rather than bake a number in and
// iterate through rebuilds, re-read a tuning file so the gain can be dialled WHILE A
// TABLE IS RUNNING and settled by feel, which is the only way this is ever settled.
//
//   /userdata/system/.config/vpinfe/ht-tune.conf     gain = 0.35
//
// Checked by mtime a few times a second: no cost, and no restart to try a value.
static float  g_gain = 1.0f;
static time_t g_tuneMtime = 0;
static const char* tune_path() {
   static std::string p;
   if (p.empty()) {
      const char* home = getenv("HOME");
      p = std::string(home ? home : "/userdata/system") + "/.config/vpinfe/ht-tune.conf";
   }
   return p.c_str();
}
static void reloadTune() {
   struct stat st;
   if (stat(tune_path(), &st) != 0) return;
   if (st.st_mtime == g_tuneMtime) return;
   g_tuneMtime = st.st_mtime;
   FILE* f = fopen(tune_path(), "r");
   if (!f) return;
   char line[128];
   while (fgets(line, sizeof(line), f)) {
      float v;
      if (sscanf(line, " gain = %f", &v) == 1 || sscanf(line, " gain=%f", &v) == 1) {
         g_gain = v;
         fprintf(stderr, "HEADTRACK: gain -> %.3f\n", g_gain); fflush(stderr);
      }
   }
   fclose(f);
}
static int   envI(const char* name, int   dflt) { const char* v = getenv(name); return v ? atoi(v) : dflt; }
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static double nowSec() {
   timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
   return double(ts.tv_sec) + double(ts.tv_nsec) * 1e-9;
}

static void udpListener() {
   int s = socket(AF_INET, SOCK_DGRAM, 0);
   if (s < 0) return;
   g_sock = s;
   const int port = envI("HT_UDP_PORT", 4242);
   sockaddr_in addr; memset(&addr, 0, sizeof(addr));
   addr.sin_family = AF_INET; addr.sin_addr.s_addr = htonl(INADDR_ANY); addr.sin_port = htons((uint16_t)port);
   if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) { close(s); g_sock=-1; return; }
   timeval tv; tv.tv_sec=1; tv.tv_usec=0; setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
   fprintf(stderr, "HEADTRACK: UDP listener bound on :%d\n", port); fflush(stderr);
   double buf[6]; long rx=0;
   while (g_running.load()) {
      ssize_t n = recv(s, buf, sizeof(buf), 0);
      if (n == (ssize_t)sizeof(buf)) {
         { std::lock_guard<std::mutex> lk(g_poseMtx); memcpy(g_pose, buf, sizeof(buf)); g_poseTime = nowSec(); }
         if ((rx++ % 60) == 0) { fprintf(stderr, "HEADTRACK: rx #%ld x=%.1f y=%.1f z=%.1f\n", rx, buf[0], buf[1], buf[2]); fflush(stderr); }
      }
   }
   close(s); g_sock=-1;
}

void onGameStart(const unsigned int, void*, void*) {
   g_haveBase.store(false); g_frames = 0; g_poseTime = 0.0;
   if (!g_running.exchange(true)) g_udpThread = std::thread(udpListener);
   if (vpxApi) {
      // MANDATORY when moving the eye. The static prepass is a baked image of the table's
      // static parts rendered ONCE from the camera as it was; the dynamic parts are then
      // composited over it every frame from the CURRENT camera. Move the camera per frame
      // and the two disagree — the playfield visibly separates from the objects sitting on
      // it and flashes, on every table. VPX's own point-of-view page does exactly this for
      // as long as its camera is being dragged.
      vpxApi->DisableStaticPrerendering(1);
      vpxApi->PushNotification("Head tracking active", 3000);
   }
}
void onGameEnd(const unsigned int, void*, void*) {
   if (vpxApi) vpxApi->DisableStaticPrerendering(0);   // refcounted — give it back
   if (g_running.exchange(false)) { if (g_sock >= 0) shutdown(g_sock, SHUT_RDWR); if (g_udpThread.joinable()) g_udpThread.join(); }
}
void onPrepareFrame(const unsigned int, void*, void*) {
   if (!vpxApi) return;
   VPXViewSetupDef view; vpxApi->GetActiveViewSetup(&view);
   if (!g_haveBase.exchange(true)) {
      g_baseX = view.viewX; g_baseY = view.viewY; g_baseZ = view.viewZ;
      VPXTableInfo ti{};
      vpxApi->GetTableInfo(&ti);
      if (ti.tableHeight > 1.0f) g_tableLength = ti.tableHeight;   // m_bottom, VPU

      // SELF-PROOF of the rotation convention. The launcher exports HT_ANCHOR as the ini
      // ScreenPlayerX,Y,Z (cm). VPX has ALREADY transformed exactly those numbers into
      // the base view we just read — so transform them ourselves with both matrix
      // conventions and keep the one that reproduces VPX's own answer. No human has to
      // lean to settle a sign bit ever again.
      const char* anch = getenv("HT_ANCHOR");
      float ax=0, ay=0, az=0;
      if (anch && sscanf(anch, "%f,%f,%f", &ax, &ay, &az) == 3) {
         const float CMTOVPU = 50.0f / (2.54f * 1.0625f);
         const float rad = (float)M_PI / 180.0f;
         const float ang = atan2f(view.windowTopZOfs - view.windowBottomZOfs, g_tableLength)
                         - view.screenInclination * rad;
         const float c = cosf(ang), sn = sinf(ang);
         const float Y = ay * CMTOVPU, Z = az * CMTOVPU;
         // solve the z offset from the base too (it also verifies wbot*scale/r2v)
         for (float sign : {1.0f, -1.0f}) {
            const float vy = Y * c - Z * sn * sign;
            const float vz = Y * sn * sign + Z * c;
            if (fabsf(vy - g_baseY) < 2.0f) {   // Y has no offset term — clean discriminator
               g_rotSign = sign;
               g_zOffset = g_baseZ - vz;
               break;
            }
         }
      }
      if (g_rotSign != 0.f)
         fprintf(stderr, "HEADTRACK: rotation convention PROVEN sign=%+.0f zOffset=%.1f (base %.1f,%.1f,%.1f)\n",
                 g_rotSign, g_zOffset, g_baseX, g_baseY, g_baseZ);
      else
         fprintf(stderr, "HEADTRACK: convention NOT proven (HT_ANCHOR=%s base=%.1f,%.1f,%.1f) — absolute mode disabled\n",
                 anch ? anch : "unset", g_baseX, g_baseY, g_baseZ);
      fflush(stderr);
   }
   double p[6]; double age;
   { std::lock_guard<std::mutex> lk(g_poseMtx); memcpy(p, g_pose, sizeof(p)); age = nowSec() - g_poseTime; }

   // Stale pose = no tracker. Fall back to the neutral eye rather than freezing the
   // view at whatever offset it happened to die on.
   if (age > HT_STALE_S) { p[0] = p[1] = p[2] = 0.0; }

   // AXES. The tracker speaks opentrack: x=lateral, y=UP, z=DEPTH.
   // VPX does NOT: in ViewSetup, Y is DEPTH and Z is HEIGHT (the cab settings say so
   // outright — ScreenPlayerY is "toward the player", ScreenPlayerZ is "up"). Mapping
   // y->viewY and z->viewZ, as this plugin originally did, therefore dollies the camera
   // when you nod and slides it vertically when you lean in. Swap them.
   //
   // Signs, MEASURED, not reasoned. I first argued that the Kinect faces the player so
   // its +x must be the player's left, and defaulted X to -1. Then I logged an actual
   // human leaning: left gives x=-19, right gives x=+20. It is NOT mirrored. Depth IS
   // inverted (leaning in drops z 107->92cm, and "in" means further INTO the table,
   // i.e. VPX +Y). Trust the trace over the geometry argument.
   if ((g_frames % 20) == 0) reloadTune();     // pick up a live gain change

   // ABSOLUTE mode (tracker calibrated to the screen): p[0..2] is the eye in VPX player
   // space (cm), flagged by p[3]~1000. Full SetViewPosFromPlayerPosition equivalent,
   // using the convention and offset PROVEN against VPX itself at base capture.
   if (p[3] > 900.0 && g_rotSign != 0.f) {
      const float CMTOVPU = 50.0f / (2.54f * 1.0625f);
      const float rad = (float)M_PI / 180.0f;
      const float ang = atan2f(view.windowTopZOfs - view.windowBottomZOfs, g_tableLength)
                      - view.screenInclination * rad;
      const float c = cosf(ang), sn = sinf(ang) * g_rotSign;
      const float X = (float)p[0] * CMTOVPU;
      const float Y = (float)p[1] * CMTOVPU;
      const float Z = (float)p[2] * CMTOVPU;
      view.viewX = X;
      view.viewY = Y * c - Z * sn;
      view.viewZ = Y * sn + Z * c + g_zOffset;
      vpxApi->SetActiveViewSetup(&view);
      if ((g_frames++ % 60) == 0) {
         fprintf(stderr, "HEADTRACK: ABS eye(%.1f,%.1f,%.1f)cm -> view(%.1f,%.1f,%.1f)\n",
                 p[0], p[1], p[2], view.viewX, view.viewY, view.viewZ); fflush(stderr);
      }
      return;
   }

   const float g = g_gain;

   // Head delta in PLAYER-SPACE centimetres (the space ScreenPlayerX/Y/Z live in):
   //   pdx  lateral, +right    pdy  +INTO the table (away from player)    pdz  +up
   const float pdx = (float)p[0] * envF("HT_SIGN_X", -1.0f) * g;
   const float pdz = (float)p[1] * envF("HT_SIGN_Y", 1.0f) * g;
   const float pdy = (float)p[2] * envF("HT_SIGN_Z", -1.0f) * g;

   // GEOMETRY-TRUE mapping. VPX converts a player position to an eye by rotating it
   // around X by (angle between playfield glass and the horizon) minus the screen's
   // physical inclination — see ViewSetup::SetViewPosFromPlayerPosition. The first
   // version of this plugin skipped that rotation and added deltas straight onto the
   // post-transform view axes: every head move landed in a coordinate frame that was
   // wrong by the glass-slope angle, so height leaked into depth and the scene
   // STRETCHED instead of behaving like a fixed box behind a window. A comfort gain
   // of 0.175 was masking a coordinate bug.
   //
   // The transform is linear, so the base view (computed by VPX from the measured
   // static player point) already carries the offset — only the ROTATED delta is
   // added. Geometry comes live from the view setup itself.
   const float rad = (float)M_PI / 180.0f;
   const float ang = atan2f(view.windowTopZOfs - view.windowBottomZOfs, g_tableLength)
                   - view.screenInclination * rad;
   const float c = cosf(ang), sn = sinf(ang) * envF("HT_ROT_SIGN", 1.0f);
   const float CMTOVPU = 50.0f / (2.54f * 1.0625f);   // VPX's own constant, 18.527/cm
   const float dX = pdx * CMTOVPU;
   const float dY = (pdy * c - pdz * sn) * CMTOVPU;
   const float dZ = (pdy * sn + pdz * c) * CMTOVPU;

   // Clamp: one bad depth sample must not hurl the camera across the room.
   const float lim = envF("HT_LIMIT_VPU", 700.0f);   // ~38cm at 18.5 VPU/cm
   view.viewX = g_baseX + clampf(dX, -lim, lim);
   view.viewY = g_baseY + clampf(dY, -lim, lim);
   view.viewZ = g_baseZ + clampf(dZ, -lim, lim);
   vpxApi->SetActiveViewSetup(&view);
   if ((g_frames++ % 60) == 0) {
      fprintf(stderr, "HEADTRACK: frame %ld pose(x=%.1f up=%.1f depth=%.1f age=%.1fs) -> eye(%.2f,%.2f,%.2f)\n",
              g_frames, p[0], p[1], p[2], age, view.viewX, view.viewY, view.viewZ); fflush(stderr);
   }
}

}

using namespace HeadTracking;

MSGPI_EXPORT void MSGPIAPI HeadTrackingPluginLoad(const uint32_t sessionId, const MsgPluginAPI* api) {
   msgApi = api; endpointId = sessionId;
   msgApi->BroadcastMsg(endpointId, getVpxApiId = msgApi->GetMsgID(VPXPI_NAMESPACE, VPXPI_MSG_GET_API), &vpxApi);
   msgApi->SubscribeMsg(endpointId, onGameStartId = msgApi->GetMsgID(VPXPI_NAMESPACE, VPXPI_EVT_ON_GAME_START), onGameStart, nullptr);
   msgApi->SubscribeMsg(endpointId, onGameEndId = msgApi->GetMsgID(VPXPI_NAMESPACE, VPXPI_EVT_ON_GAME_END), onGameEnd, nullptr);
   msgApi->SubscribeMsg(endpointId, onPrepareFrameId = msgApi->GetMsgID(VPXPI_NAMESPACE, VPXPI_EVT_ON_PREPARE_FRAME), onPrepareFrame, nullptr);
}

MSGPI_EXPORT void MSGPIAPI HeadTrackingPluginUnload() {
   if (g_running.exchange(false)) { if (g_sock >= 0) shutdown(g_sock, SHUT_RDWR); if (g_udpThread.joinable()) g_udpThread.join(); }
   msgApi->UnsubscribeMsg(onGameStartId, onGameStart, nullptr);
   msgApi->UnsubscribeMsg(onGameEndId, onGameEnd, nullptr);
   msgApi->UnsubscribeMsg(onPrepareFrameId, onPrepareFrame, nullptr);
   msgApi->ReleaseMsgID(getVpxApiId); msgApi->ReleaseMsgID(onGameStartId); msgApi->ReleaseMsgID(onGameEndId); msgApi->ReleaseMsgID(onPrepareFrameId);
   vpxApi = nullptr; msgApi = nullptr;
}
