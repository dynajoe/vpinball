// license:GPLv3+

#include "plugins/VPXPlugin.h"
#include "plugins/B2SPluginEventStream.h"
#include "plugins/ControllerPlugin.h"
#include "plugins/LoggingPlugin.h"
#include "pinmame/PinMAMEPlugin.h"

#pragma warning(push)
#pragma warning(disable : 4251) // xxx needs dll-interface
#include "DOF/DOF.h"
#pragma warning(pop)

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <algorithm>
#include <cctype>
#include <atomic>
#include <cstdio>
#include <cassert>
#include <cctype>
#include <cstdlib>
#include <chrono>
#include <cstring>
#include <charconv>
#include <format>
#if defined(__APPLE__) || defined(__linux__) || defined(__ANDROID__)
#include <pthread.h>
#endif

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <locale>
#endif

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//
// Direct Output Framework plugin
//

namespace DOFPlugin {

static const MsgPluginAPI* msgApi = nullptr;
static VPXPluginAPI* vpxApi = nullptr;
static uint32_t endpointId;

static std::unique_ptr<PinballPlugin::Controller::CtrlItemConsumer<ControllerDef>> controllers;
static std::unique_ptr<DOF::DOF> pDOF;

LPI_USE_CPP();
#define LOGD DOFPlugin::LPI_LOGD_CPP
#define LOGI DOFPlugin::LPI_LOGI_CPP
#define LOGW DOFPlugin::LPI_LOGW_CPP
#define LOGE DOFPlugin::LPI_LOGE_CPP

LPI_IMPLEMENT_CPP // Implement shared log support

#ifdef _WIN32
   static void SetThreadName(const std::string& name)
   {
      const int size_needed = MultiByteToWideChar(CP_UTF8, 0, name.c_str(), -1, nullptr, 0);
      if (size_needed <= 1)
         return;
      std::wstring wstr(size_needed - 1, L'\0');
      if (MultiByteToWideChar(CP_UTF8, 0, name.c_str(), -1, wstr.data(), size_needed) == 0)
         return;
      HRESULT hr = SetThreadDescription(GetCurrentThread(), wstr.c_str());
   }
#else
   static void SetThreadName(const std::string& name)
   {
#ifdef __APPLE__
      pthread_setname_np(name.c_str());
#elif defined(__linux__) || defined(__ANDROID__)
      pthread_setname_np(pthread_self(), name.c_str());
#endif
   }
#endif

void LIBDOFCALLBACK OnDOFLog(DOF_LogLevel logLevel, const char* format, va_list args)
{
   va_list args_copy;
   va_copy(args_copy, args);
   int size = vsnprintf(nullptr, 0, format, args_copy);
   va_end(args_copy);
   if (size > 0) {
      string buffer(size + 1, '\0');
      vsnprintf(buffer.data(), size + 1, format, args);
      buffer.pop_back(); // remove null terminator
      switch(logLevel) {
         case DOF_LogLevel_INFO:
            LOGI(buffer);
            break;
         case DOF_LogLevel_DEBUG:
            LOGD(buffer);
            break;
         case DOF_LogLevel_ERROR:
            LOGE(buffer);
            break;
         default:
            break;
      }
   }
}

class DOFEventConsumer
{
public:
   DOFEventConsumer(const string& tablePath, const string& gameId)
      : m_tablePath(tablePath)
      , m_gameId(gameId)
      , b2sPluginEventStream(std::make_unique<B2SPluginEventStream>(msgApi, endpointId, [this](char type, int index, int value) { PostEvent({ static_cast<uint8_t>(type), index, value }); }))
   {
      m_thread = std::thread(&DOFEventConsumer::Run, this);
   }

   ~DOFEventConsumer()
   {
      b2sPluginEventStream = nullptr;
      {
         std::lock_guard lock(m_mutex);
         m_stopRequested = true;
      }
      m_cv.notify_one();

      if (m_thread.joinable())
         m_thread.join();
   }

   // Non-copyable, non-movable (owns a thread + sync primitives)
   DOFEventConsumer(const DOFEventConsumer&) = delete;
   DOFEventConsumer& operator=(const DOFEventConsumer&) = delete;

   struct B2SPluginEvent
   {
      uint8_t type;
      int32_t index;
      int32_t value;
   };

   void PostEvent(const B2SPluginEvent& ev)
   {
      {
         std::lock_guard<std::mutex> lock(m_mutex);
         m_queue.push(ev);
      }
      m_cv.notify_one();
   }

private:
   void Run()
   {
      SetThreadName("DOF.EventQueue"s);
      pDOF->Init(m_tablePath.c_str(), m_gameId.c_str());
      while (!m_stopRequested)
      {
         std::unique_lock lock(m_mutex);

         m_cv.wait(lock, [this] { return !m_queue.empty() || m_stopRequested; });

         if (m_stopRequested)
            break;

         while (!m_queue.empty())
         {
            B2SPluginEvent ev = m_queue.front();
            m_queue.pop();
            lock.unlock();
            pDOF->DataReceive(ev.type, ev.index, ev.value);
            lock.lock();
         }
      }
      pDOF->Finish();
   }

   const string m_tablePath;
   const string m_gameId;

   std::queue<B2SPluginEvent> m_queue;
   std::mutex m_mutex;
   std::condition_variable m_cv;
   bool m_stopRequested = false;
   std::thread m_thread;

   std::unique_ptr<B2SPluginEventStream> b2sPluginEventStream;
};

static std::unique_ptr<DOFEventConsumer> dofThread;

// B2SServer/B2SLegacy register a controller with an UNNAMED game as "b2s::<random
// RFC4122 GUID>" (SetB2SName generates one while the name is empty) and re-broadcast
// controllers-changed once the script supplies the real name. Selecting the GUID
// would Init DOF against a name that matches no directoutputconfig row, open the
// output hardware, then tear it all down and reopen milliseconds later when the
// real name arrives — the new-consumer shape of the old "empty gameId" problem (a
// B2S announces BEFORE its ROM name is set). Recognize the placeholder and wait
// for the re-broadcast instead (the consumer filter drops it, so the list stays
// empty until the real name arrives). A real B2S name is a ROM-ish short name;
// none has the 8-4-4-4-12 hex layout.
static bool IsUnnamedB2SGameId(const string& gameId)
{
   if (gameId.length() != 36)
      return false;
   for (size_t i = 0; i < 36; i++)
   {
      if (i == 8 || i == 13 || i == 18 || i == 23)
      {
         if (gameId[i] != '-')
            return false;
      }
      else if (!isxdigit(static_cast<unsigned char>(gameId[i])))
         return false;
   }
   return true;
}

static string currentGameId; // identity the running DOFEventConsumer was initialized for

static void SetupDOF()
{
   const ControllerDef controller = controllers->With([](const std::vector<ControllerDef>& items) { return items.empty() ? ControllerDef { } : items.front(); });
   // The consumer filter admits PinMAME games and named B2S games, PinMAME first,
   // so front() is the best identity available. Strip whichever prefix it carries.
   // The b2s:: id is lowercased at the source and libdof matches ROM names
   // case-insensitively (LedControlConfigList upper-cases both sides), so the
   // stripped id hits the same config row a PinMAME identity would.
   string gameId;
   if (controller.gameId != nullptr && controller.endpointId != 0)
   {
      const string pinmamePrefix(PMPI_GAMEID_PREFIX);
      const string b2sPrefix("b2s::"); // same literal B2SServer.cpp / b2slegacy/Server.cpp register with; there is no shared define
      const string raw(controller.gameId);
      if (raw.starts_with(pinmamePrefix))
         gameId = raw.substr(pinmamePrefix.length());
      else if (raw.starts_with(b2sPrefix))
         gameId = raw.substr(b2sPrefix.length());
   }

   // Identity-level dedup: the controller LIST churns without the identity
   // changing — a ROM table's B2S announces after PinMAME ([pm] -> [pm, b2s]),
   // and late re-broadcasts arrive mid-game. Since the filter above admits B2S
   // controllers, list-level dedup alone would tear down and rebuild the
   // consumer on such churn: output hardware close/reopen and a full libdof
   // re-Init at table load, with the SAME gameId. Only touch the running
   // consumer when the selected identity actually changed.
   if (gameId == currentGameId)
      return;
   dofThread = nullptr;
   currentGameId = gameId;
   if (gameId.empty() || pDOF == nullptr)
      return;

   LOGI("New controller game started: gameId=" + gameId);
   VPXTableInfo tableInfo;
   vpxApi->GetTableInfo(&tableInfo);
   string path = tableInfo.path;
   dofThread = std::make_unique<DOFEventConsumer>(path, gameId);
}

}

using namespace DOFPlugin;

MSGPI_EXPORT void MSGPIAPI DOFPluginLoad(const uint32_t sessionId, const MsgPluginAPI* api)
{
   msgApi = api;
   endpointId = sessionId;

   LPISetup(endpointId, msgApi);

   unsigned int getVpxApiId = msgApi->GetMsgID(VPXPI_NAMESPACE, VPXPI_MSG_GET_API);
   msgApi->BroadcastMsg(endpointId, getVpxApiId, &vpxApi);
   msgApi->ReleaseMsgID(getVpxApiId);

   VPXInfo vpxInfo;
   vpxApi->GetVpxInfo(&vpxInfo);

   DOF::Config* pConfig = DOF::Config::GetInstance();
   pConfig->SetLogCallback(OnDOFLog);
   pConfig->SetBasePath(vpxInfo.prefPath);

   pDOF = std::make_unique<DOF::DOF>();

   controllers = std::make_unique<PinballPlugin::Controller::CtrlItemConsumer<ControllerDef>>(
      msgApi, endpointId, CTLPI_CONTROLLERS_GET_MSG, CTLPI_CONTROLLERS_ON_CHG_MSG,
      [](std::vector<ControllerDef>& items)
      {
         // Keep PinMAME games, and NAMED B2S games as a fallback identity: on ROM-less
         // tables (Stern SPIKE-era originals like Guardians of the Galaxy) B2S.Server is
         // the ONLY source of the game's identity — there is no PinMAME controller at
         // all. A PinMAME-only filter silently kills DOF on every such table: Init()
         // never runs, so no table config, 0 toys, and the output hardware never opens,
         // while the log stays clean (libdof cannot complain about a missing RomName if
         // it is never initialized). Measured on the cab: Guardians 40 toys -> 0.
         const string pinmamePrefix(PMPI_GAMEID_PREFIX);
         const string b2sPrefix("b2s::");
         std::erase_if(items, [&pinmamePrefix, &b2sPrefix](const ControllerDef& src)
         {
            const string gameId(src.gameId);
            if (gameId.starts_with(pinmamePrefix))
               return false;
            if (gameId.starts_with(b2sPrefix))
               return IsUnnamedB2SGameId(gameId.substr(b2sPrefix.length()));
            return true;
         });
         // A PinMAME identity wins when both are present: SetupDOF uses items.front()
         std::stable_partition(items.begin(), items.end(),
            [&pinmamePrefix](const ControllerDef& src) { return string(src.gameId).starts_with(pinmamePrefix); });
      },
      // Deliberately NOT tearing dofThread down here: DOFEventConsumer copies its
      // identity strings and borrows nothing from the item list, so it may outlive
      // a list change. SetupDOF (below) destroys it iff the selected identity
      // actually changed — see the dedup comment there.
      []() { },
      []() { SetupDOF(); });
   controllers->Subscribe();
}

MSGPI_EXPORT void MSGPIAPI DOFPluginUnload()
{
   controllers->Unsubscribe();
   controllers = nullptr;
   pDOF = nullptr;

   msgApi = nullptr;
   vpxApi = nullptr;
}
