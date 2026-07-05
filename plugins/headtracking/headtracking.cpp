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
#include <cstdlib>
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
static long              g_frames = 0;

static float envF(const char* name, float dflt) { const char* v = getenv(name); return v ? (float)atof(v) : dflt; }
static int   envI(const char* name, int   dflt) { const char* v = getenv(name); return v ? atoi(v) : dflt; }

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
         { std::lock_guard<std::mutex> lk(g_poseMtx); memcpy(g_pose, buf, sizeof(buf)); }
         if ((rx++ % 60) == 0) { fprintf(stderr, "HEADTRACK: rx #%ld x=%.1f y=%.1f z=%.1f\n", rx, buf[0], buf[1], buf[2]); fflush(stderr); }
      }
   }
   close(s); g_sock=-1;
}

void onGameStart(const unsigned int, void*, void*) {
   g_haveBase.store(false); g_frames = 0;
   if (!g_running.exchange(true)) g_udpThread = std::thread(udpListener);
   if (vpxApi) vpxApi->PushNotification("Head tracking active", 3000);
}
void onGameEnd(const unsigned int, void*, void*) {
   if (g_running.exchange(false)) { if (g_sock >= 0) shutdown(g_sock, SHUT_RDWR); if (g_udpThread.joinable()) g_udpThread.join(); }
}
void onPrepareFrame(const unsigned int, void*, void*) {
   if (!vpxApi) return;
   VPXViewSetupDef view; vpxApi->GetActiveViewSetup(&view);
   if (!g_haveBase.exchange(true)) {
      g_baseX = view.viewX; g_baseY = view.viewY; g_baseZ = view.viewZ;
      fprintf(stderr, "HEADTRACK: base eye=(%.2f,%.2f,%.2f) viewMode=%d\n", g_baseX, g_baseY, g_baseZ, view.viewMode); fflush(stderr);
   }
   double p[6]; { std::lock_guard<std::mutex> lk(g_poseMtx); memcpy(p, g_pose, sizeof(p)); }
   view.viewX = g_baseX + (float)p[0]*envF("HT_SCALE_X", 1.0f);
   view.viewY = g_baseY + (float)p[1]*envF("HT_SCALE_Y", 1.0f);
   view.viewZ = g_baseZ + (float)p[2]*envF("HT_SCALE_Z", 1.0f);
   vpxApi->SetActiveViewSetup(&view);
   if ((g_frames++ % 60) == 0) {
      fprintf(stderr, "HEADTRACK: frame %ld pose(x=%.1f y=%.1f z=%.1f) -> eye(%.2f,%.2f,%.2f)\n", g_frames, p[0], p[1], p[2], view.viewX, view.viewY, view.viewZ); fflush(stderr);
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
