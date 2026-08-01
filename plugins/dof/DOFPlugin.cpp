// license:GPLv3+

#include "plugins/VPXPlugin.h"
#include "plugins/B2SPluginEventStream.h"
#include "plugins/ControllerPlugin.h"
#include "plugins/LoggingPlugin.h"

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
#include <algorithm>
#include <chrono>
#include <cstring>
#include <charconv>
#include <cctype>
#include <filesystem>
#include <format>
#include <unordered_set>
#include <vector>

#include "plugins/PluginStrings.h"

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

///////////////////////////////////////////////////////////////////////////////
// DOF rom list
//
// libDOF resolves the rom name given to Init against the table configuration
// lines of the ledcontrol ini files it loads (first CSV column of the
// [Config DOF] / [Config outs] section, see LedControlConfigList). The plugin
// asks libDOF for that list to pick the rom name handed to Init: ns::rom
// resolves to "ns_rom" when that name is declared, "rom" otherwise.

static std::string ToUpper(const std::string_view& s)
{
   std::string r(s);
   std::transform(r.begin(), r.end(), r.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
   return r;
}

static std::unordered_set<std::string> cachedRomList;
static std::filesystem::path cachedRomListTablePath;
static bool cachedRomListValid = false;

static const std::unordered_set<std::string>& LoadRomList(const std::filesystem::path& tablePath)
{
   if (!cachedRomListValid || cachedRomListTablePath != tablePath)
   {
      cachedRomList.clear();
      for (const std::string& romName : pDOF->GetLedControlRomNames(PluginStrings::PathToNative(tablePath).c_str()))
         cachedRomList.insert(ToUpper(romName));
      cachedRomListTablePath = tablePath;
      cachedRomListValid = true;
   }
   return cachedRomList;
}

// Whether a rom name is declared in the rom list. Exact case-insensitive
// match only: anything looser would let ns_rom bind a config declared for ns,
// which is not the game this name was built for.
static bool RomListContains(const std::unordered_set<std::string>& romList, const std::string_view& romName) { return romList.contains(ToUpper(romName)); }

static std::string ResolveRomName(const std::string_view& gameNs, const std::string_view& gameKey, const std::unordered_set<std::string>& romList)
{
   if (!gameNs.empty())
   {
      const std::string nsRom = std::string(gameNs) + "_" + std::string(gameKey);
      if (RomListContains(romList, nsRom))
         return nsRom;
   }
   return std::string(gameKey);
}

class DOFEventConsumer
{
public:
   DOFEventConsumer(const string& tablePath, const string& gameId, const ControllerDef& controller)
      : m_tablePath(tablePath)
      , m_gameId(gameId)
      , b2sPluginEventStream(
           std::make_unique<B2SPluginEventStream>(msgApi, endpointId, controller, [this](char type, int index, int value) { PostEvent({ static_cast<uint8_t>(type), index, value }); }))
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

static string currentRomName; // resolved ROM identity the running DOFEventConsumer was initialized for

static void SetupDOF()
{
   const ControllerDef controller = controllers->With([](const std::vector<ControllerDef>& items) { return items.empty() ? ControllerDef { } : items.front(); });

   // Resolve the selected controller to the ROM identity DOF would be initialized for.
   // Upstream 7f2c70ad3 now covers the cab's B2S fallback (SPIKE tables with no PinMAME
   // controller): the consumer filter scores every namespace against the DOF rom list and
   // falls back to any named controller, and ResolveRomName falls back to the bare game
   // key — the same config row the cab's stripped b2s:: id hit (libdof matches ROM names
   // case-insensitively).
   string romName;
   std::string_view gameNs, gameKey;
   if (controller.gameId != nullptr && controller.endpointId != 0)
   {
      gameNs = PinballPlugin::Controller::CtrlGetGameNamespace(controller.gameId);
      gameKey = PinballPlugin::Controller::CtrlGetGameKey(controller.gameId);
      if (!gameKey.empty())
      {
         VPXTableInfo tableInfo {};
         vpxApi->GetTableInfo(&tableInfo);
         romName = ResolveRomName(gameNs, gameKey, LoadRomList(PluginStrings::PathFromNative(tableInfo.path)));
      }
   }

   // Identity-level dedup (cab behaviour preserved across the 2026-10-03 rebase): the
   // controller LIST churns without the ROM identity changing — a ROM table's B2S
   // announces after PinMAME, PinMAME can announce after an already-named B2S, and late
   // re-broadcasts arrive mid-game. List-level teardown would close and reopen the output
   // hardware and fully re-Init libdof at table load with the SAME identity. The
   // on-change callback is therefore a no-op and only an actual change of the resolved
   // ROM identity touches the running consumer.
   if (romName == currentRomName)
      return;
   dofThread = nullptr;
   currentRomName = romName;
   if (romName.empty())
      return;

   VPXTableInfo tableInfo {};
   vpxApi->GetTableInfo(&tableInfo);
   const string path = tableInfo.path ? tableInfo.path : ""; // Native narrow path, as libDOF expects

   LOGI("New game started: gameId="s + controller.gameId + ", romName=" + romName);
   dofThread = std::make_unique<DOFEventConsumer>(path, romName, controller);
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

   VPXInfo vpxInfo {};
   vpxApi->GetVpxInfo(&vpxInfo);

   DOF::Config* pConfig = DOF::Config::GetInstance();
   pConfig->SetLogCallback(OnDOFLog);
   pConfig->SetBasePath(vpxInfo.prefPath ? vpxInfo.prefPath : ""); // Native narrow path, as libDOF expects

   pDOF = std::make_unique<DOF::DOF>();

   controllers = std::make_unique<PinballPlugin::Controller::CtrlItemConsumer<ControllerDef>>(
      msgApi, endpointId, CTLPI_CONTROLLERS_GET_MSG, CTLPI_CONTROLLERS_ON_CHG_MSG,
      [](std::vector<ControllerDef>& items)
      {
         // Select the controller for which a DOF table config is declared
         // (ns_rom or rom in the rom list), a pinmame:: one winning over other
         // namespaces on ties (selection order is otherwise undefined). When no
         // controller resolves to a declared config, keep the legacy behavior
         // of binding the first pinmame one, then any controller: DOF can still
         // find table-filename mappings the rom list does not describe.
         const std::unordered_set<std::string>& romList = LoadRomList(
            []
            {
               VPXTableInfo tableInfo {};
               vpxApi->GetTableInfo(&tableInfo);
               return PluginStrings::PathFromNative(tableInfo.path);
            }());
         const ControllerDef* selected = nullptr;
         int bestScore = -1;
         for (const ControllerDef& controller : items)
         {
            const std::string_view gameNs = PinballPlugin::Controller::CtrlGetGameNamespace(controller.gameId);
            const std::string_view gameKey = PinballPlugin::Controller::CtrlGetGameKey(controller.gameId);
            if (gameKey.empty())
               continue;
            // Cab: skip B2S's unnamed-game placeholder (a random GUID registered before the
            // script supplies the real name) — selecting it would Init DOF against a name
            // matching no config row, open the output hardware, then tear it down and reopen
            // milliseconds later when the re-broadcast with the real name arrives. See
            // IsUnnamedB2SGameId above.
            if (gameNs == "b2s"sv && IsUnnamedB2SGameId(std::string(gameKey)))
               continue;
            const bool pinmame = gameNs == "pinmame"sv;
            const bool match = (!gameNs.empty() && RomListContains(romList, std::string(gameNs) + '_' + std::string(gameKey))) || RomListContains(romList, gameKey);
            const int score = (match ? 2 : 0) + (pinmame ? 1 : 0);
            if (score > bestScore)
            {
               bestScore = score;
               selected = &controller;
               if (score == 3)
                  break;
            }
         }
         items.clear();
         if (selected != nullptr)
            items.push_back(*selected);
      },
      // Deliberately NOT tearing dofThread down here (cab behaviour): DOFEventConsumer
      // copies its identity strings and borrows nothing from the item list, so it may
      // outlive a list change. SetupDOF destroys it iff the resolved ROM identity
      // actually changed — see the dedup comment there.
      []() { }, []() { SetupDOF(); });
   controllers->Subscribe();
}

MSGPI_EXPORT void MSGPIAPI DOFPluginUnload()
{
   controllers->Unsubscribe();
   controllers = nullptr;
   pDOF = nullptr;
   cachedRomList.clear();
   cachedRomListValid = false;

   msgApi = nullptr;
   vpxApi = nullptr;
}
