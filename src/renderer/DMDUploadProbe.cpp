// license:GPLv3+

#include "core/stdafx.h"
#include "utils/wintimer.h" // FrameProfiler (render-thread frame stats read at 1Hz)
#include "core/player.h" // PERF line: logic profiler + render device counters
#include "renderer/Renderer.h" // player.h only forward-declares Renderer; the PERF line walks m_renderer->m_renderDevice
#include "renderer/RenderDevice.h"

#include "DMDUploadProbe.h"

#include <thread>
#include <functional>
#include <cstdio>
#include <cstdlib>

namespace VPX::DMDProbe
{

std::atomic<bool> g_enabled { false };

// Cumulative counters, monotonic for the process lifetime. Relaxed ordering is
// deliberate: these are statistics, and the 1Hz reader tolerates being a few
// increments off; anything stronger would put fences on the paths we are
// trying to observe.
static std::atomic<uint32_t> s_ingestCount { 0 };
static std::atomic<uint64_t> s_ingestBytes { 0 };
static std::atomic<uint32_t> s_pluginIngestCount { 0 };
static std::atomic<uint32_t> s_stageCount { 0 };
static std::atomic<uint32_t> s_gpuUploadCount { 0 };

// Thread attribution. #3469's whole argument is WHICH thread does the DMD
// work, so record the first thread seen at each site and count hits from any
// other thread. A count, not a log: pre-#3469 the dmdutil worker was hitting
// texture state concurrently and a per-event log would have been its own
// perturbation. |1 guards against a hash that is legitimately 0.
static std::atomic<uint64_t> s_ingestFirstTid { 0 };
static std::atomic<uint32_t> s_ingestForeign { 0 };
static std::atomic<uint64_t> s_gpuFirstTid { 0 };
static std::atomic<uint32_t> s_gpuForeign { 0 };

// Per-rendered-frame upload bucketing, fed from RenderDevice::Flip on the
// render thread. This is the "upf" ratio: had we derived it from logic frames
// we would be assuming the 1:1 logic/render pacing we are trying to verify.
static std::atomic<uint64_t> s_renderTid { 0 };
static std::atomic<uint32_t> s_renderFlips { 0 };
static std::atomic<uint32_t> s_maxGpuPerFlip { 0 };

static void NoteThread(std::atomic<uint64_t>& first, std::atomic<uint32_t>& foreign)
{
   const uint64_t tid = std::hash<std::thread::id> {}(std::this_thread::get_id()) | 1ull;
   uint64_t expected = 0;
   if (first.load(std::memory_order_relaxed) == 0)
      first.compare_exchange_strong(expected, tid, std::memory_order_relaxed);
   if (first.load(std::memory_order_relaxed) != tid)
      foreign.fetch_add(1, std::memory_order_relaxed);
}

// Per-size attribution table. The upload streams on a cab are few (a handful
// of PUP videos, one or two DMD consumers, the odd label), so a small fixed
// open-addressed table with an overflow bucket covers reality; keys are only
// ever claimed, never removed, and counts are drained by the 1Hz reader.
// Sized so the 2026-08-02 workload (6 distinct sizes) fits three times over.
static constexpr size_t SZ_SLOTS = 16;
static std::atomic<uint64_t> s_szKey[SZ_SLOTS] {};   // (w<<24 | h<<4 | pixelSize), 0 = free
static std::atomic<uint32_t> s_szCount[SZ_SLOTS] {}; // drained (exchange 0) at window close
static std::atomic<uint32_t> s_szOverflow { 0 };

static void CountIngestSize(unsigned int width, unsigned int height, unsigned int pixelSize)
{
   const uint64_t key = (static_cast<uint64_t>(width) << 24) | (static_cast<uint64_t>(height) << 4) | pixelSize;
   for (size_t i = 0; i < SZ_SLOTS; i++)
   {
      uint64_t cur = s_szKey[i].load(std::memory_order_relaxed);
      if (cur == 0)
      {
         if (!s_szKey[i].compare_exchange_strong(cur, key, std::memory_order_relaxed))
         {
            if (cur != key) // lost the race to a different key: keep probing
               continue;
         }
         cur = key;
      }
      if (cur == key)
      {
         s_szCount[i].fetch_add(1, std::memory_order_relaxed);
         return;
      }
   }
   s_szOverflow.fetch_add(1, std::memory_order_relaxed);
}

void CountIngest(unsigned int width, unsigned int height, unsigned int pixelSize)
{
   s_ingestCount.fetch_add(1, std::memory_order_relaxed);
   s_ingestBytes.fetch_add(static_cast<uint64_t>(width) * height * pixelSize, std::memory_order_relaxed);
   CountIngestSize(width, height, pixelSize);
   NoteThread(s_ingestFirstTid, s_ingestForeign);
}

void CountPluginIngest() { s_pluginIngestCount.fetch_add(1, std::memory_order_relaxed); }

void CountStage() { s_stageCount.fetch_add(1, std::memory_order_relaxed); }

void CountGpuUpload()
{
   s_gpuUploadCount.fetch_add(1, std::memory_order_relaxed);
   NoteThread(s_gpuFirstTid, s_gpuForeign);
}

void CountRenderFlip()
{
   // Flip is only ever called by the (single) render thread, so plain statics
   // for the per-flip delta are safe; only the values the 1Hz logic-thread
   // reader consumes are atomics.
   static uint32_t s_gpuAtLastFlip = 0;
   s_renderTid.store(std::hash<std::thread::id> {}(std::this_thread::get_id()) | 1ull, std::memory_order_relaxed);
   const uint32_t now = s_gpuUploadCount.load(std::memory_order_relaxed);
   const uint32_t d = now - s_gpuAtLastFlip;
   s_gpuAtLastFlip = now;
   s_renderFlips.fetch_add(1, std::memory_order_relaxed);
   uint32_t cur = s_maxGpuPerFlip.load(std::memory_order_relaxed);
   while (d > cur && !s_maxGpuPerFlip.compare_exchange_weak(cur, d, std::memory_order_relaxed)) { }
}

void Init()
{
   // Strictly "1": this cab has a history of diagnostics left half-armed
   // firing later (DPMS), so anything other than an explicit opt-in is off.
   const char* const env = std::getenv("VPX_DMD_PROBE");
   const bool enable = (env != nullptr) && (env[0] == '1') && (env[1] == '\0');
   g_enabled.store(enable, std::memory_order_relaxed);
   if (enable)
   {
      PLOGI << "DMD upload probe ENABLED (VPX_DMD_PROBE=1). One 'DMDPROBE' line per ~1s: "
               "src = default display frameId advance (PinMAME integrated-content changes as seen by pullers), "
               "ingest = BaseTexture::Update calls (plugin = via plugin API, included), "
               "stage = Sampler::UpdateTexture, gpu = bgfx::updateTexture2D, "
               "upf = GPU uploads per rendered frame avg/peak + issuing thread, "
               "xthr = hits from a second thread, "
               "logic/srcf/quiet/render = frame times ms (srcf = frames where src advanced). "
               "A second 'DMDPROBE SZ' line breaks ingest down per WxHxBpp:rate:KB/s — "
               "a bucket at a video's native fps is content, a bucket locked to render fps is waste.";
   }
}

// ---------------------------------------------------------------------------
// Per-logic-frame aggregation. Logic thread only, so plain members. Window is
// closed on accumulated frame time, not a wall clock — no extra clock calls,
// and it reuses the numbers the profiler already produced.

namespace
{
   struct Window
   {
      unsigned int frames = 0;
      uint64_t frameUsSum = 0;
      unsigned int frameUsMax = 0;
      // Frames where the default display's frameId advanced vs not: this split
      // is the correlation the whole exercise exists for. Classifying by
      // "ingest > 0" would be useless — consumers ingest unconditionally at
      // render rate, so every frame with a visible DMD ingests.
      unsigned int srcFrames = 0;
      uint64_t srcFrameUsSum = 0;
      unsigned int srcFrameUsMax = 0;
      unsigned int quietFrames = 0;
      uint64_t quietFrameUsSum = 0;
      unsigned int quietFrameUsMax = 0;
      // Source id accounting. lumFrameId is monotonic, so a delta > 1 in one
      // poll means other consumers integrated frames between our polls — the
      // burst is visible even at 1-poll-per-frame resolution.
      unsigned int srcIdAdvance = 0;
      unsigned int srcIdAdvanceMaxPerFrame = 0;
      // Counter deltas.
      uint32_t ingest = 0;
      unsigned int ingestMaxPerFrame = 0;
      uint64_t ingestBytes = 0;
      uint32_t pluginIngest = 0;
      uint32_t stage = 0;
      uint32_t gpuUpload = 0;
   };

   Window s_win;
   bool s_haveLastId = false;
   unsigned int s_lastSrcFrameId = 0;
   uint32_t s_lastIngestCount = 0;
   uint64_t s_lastIngestBytes = 0;
   uint32_t s_lastPluginIngestCount = 0;
   uint32_t s_lastStageCount = 0;
   uint32_t s_lastGpuUploadCount = 0;
   uint32_t s_lastRenderFlips = 0;
}

void OnLogicFrame(const bool srcPresent, const unsigned int srcFrameId, const unsigned int logicFrameUs, const FrameProfiler* const renderProfiler)
{
   // Per-frame counter deltas. Ingest is fed from this same thread for the
   // core consumers, so the per-frame attribution is exact for them; plugin
   // threads (if any — xthr says) may land an increment in a neighbor frame,
   // which is fine at this resolution.
   const uint32_t ingestNow = s_ingestCount.load(std::memory_order_relaxed);
   const uint32_t ingestDelta = ingestNow - s_lastIngestCount;
   s_lastIngestCount = ingestNow;

   unsigned int idDelta = 0;
   if (srcPresent)
   {
      if (s_haveLastId)
      {
         // Monotonic counter; a decrease means the source restarted (new table
         // or ROM re-init) — count nothing rather than a bogus huge delta.
         idDelta = (srcFrameId >= s_lastSrcFrameId) ? (srcFrameId - s_lastSrcFrameId) : 0u;
      }
      s_lastSrcFrameId = srcFrameId;
      s_haveLastId = true;
   }
   else
      s_haveLastId = false;

   s_win.frames++;
   s_win.frameUsSum += logicFrameUs;
   if (logicFrameUs > s_win.frameUsMax)
      s_win.frameUsMax = logicFrameUs;
   s_win.srcIdAdvance += idDelta;
   if (idDelta > s_win.srcIdAdvanceMaxPerFrame)
      s_win.srcIdAdvanceMaxPerFrame = idDelta;
   s_win.ingest += ingestDelta;
   if (ingestDelta > s_win.ingestMaxPerFrame)
      s_win.ingestMaxPerFrame = ingestDelta;
   if (idDelta > 0)
   {
      s_win.srcFrames++;
      s_win.srcFrameUsSum += logicFrameUs;
      if (logicFrameUs > s_win.srcFrameUsMax)
         s_win.srcFrameUsMax = logicFrameUs;
   }
   else
   {
      s_win.quietFrames++;
      s_win.quietFrameUsSum += logicFrameUs;
      if (logicFrameUs > s_win.quietFrameUsMax)
         s_win.quietFrameUsMax = logicFrameUs;
   }

   if (s_win.frameUsSum < 1000000ull)
      return;

   // Close the window: pull the remaining cumulative deltas (these are fed
   // from other threads, so read once per second, not per frame).
   const uint64_t ingBytes = s_ingestBytes.load(std::memory_order_relaxed);
   s_win.ingestBytes = ingBytes - s_lastIngestBytes;
   s_lastIngestBytes = ingBytes;
   const uint32_t plug = s_pluginIngestCount.load(std::memory_order_relaxed);
   s_win.pluginIngest = plug - s_lastPluginIngestCount;
   s_lastPluginIngestCount = plug;
   const uint32_t stage = s_stageCount.load(std::memory_order_relaxed);
   s_win.stage = stage - s_lastStageCount;
   s_lastStageCount = stage;
   const uint32_t gpu = s_gpuUploadCount.load(std::memory_order_relaxed);
   s_win.gpuUpload = gpu - s_lastGpuUploadCount;
   s_lastGpuUploadCount = gpu;

   // Uploads per RENDERED frame — the number that decides push vs pull. The
   // peak is exchanged out so each window reports its own worst frame; the
   // render thread may race the exchange by one frame, which only moves that
   // frame's peak into the neighboring window.
   const uint32_t flips = s_renderFlips.load(std::memory_order_relaxed);
   const uint32_t flipsDelta = flips - s_lastRenderFlips;
   s_lastRenderFlips = flips;
   const uint32_t upfPeak = s_maxGpuPerFlip.exchange(0, std::memory_order_relaxed);
   const double upfAvg = flipsDelta ? static_cast<double>(s_win.gpuUpload) / flipsDelta : 0.;
   const uint64_t gpuTid = s_gpuFirstTid.load(std::memory_order_relaxed);
   const uint64_t renderTid = s_renderTid.load(std::memory_order_relaxed);
   const char* const gpuAt = (gpuTid == 0) ? "none" : (gpuTid == renderTid) ? "render" : "OTHER";

   const double winSec = static_cast<double>(s_win.frameUsSum) * 1e-6;
   const double perSec = 1.0 / winSec;

   // Render thread frame time over its own ~1s sliding window. The profiler's
   // ring buffer is written by the render thread while we read it here; the
   // entries are plain unsigned ints, so a torn read yields one bogus sample
   // in a statistic, which is acceptable for a diagnostic — do not "fix" this
   // with a lock on the render thread.
   const double renderAvgMs = renderProfiler ? renderProfiler->GetSlidingAvg(FrameProfiler::PROFILE_FRAME) * 1e-3 : 0.;
   const double renderMaxMs = renderProfiler ? static_cast<double>(renderProfiler->GetSlidingMax(FrameProfiler::PROFILE_FRAME)) * 1e-3 : 0.;

   const double avgMs = s_win.frames ? static_cast<double>(s_win.frameUsSum) / s_win.frames * 1e-3 : 0.;
   const double srcAvgMs = s_win.srcFrames ? static_cast<double>(s_win.srcFrameUsSum) / s_win.srcFrames * 1e-3 : 0.;
   const double quietAvgMs = s_win.quietFrames ? static_cast<double>(s_win.quietFrameUsSum) / s_win.quietFrames * 1e-3 : 0.;

   char line[640];
   snprintf(line, sizeof(line),
      "DMDPROBE %.2fs f=%u fps=%.1f | src id+%u chg%u max+%u/f | ingest %.0f/s max%u/f %.0fKB/s (plugin %.0f/s) | stage %.0f/s | gpu %.0f/s | upf avg%.2f peak%u @%s | xthr i%u g%u | "
      "logic avg%.2f max%.2fms | srcf avg%.2f max%.2f | quiet avg%.2f max%.2f | render avg%.2f max%.2f",
      winSec, s_win.frames, static_cast<double>(s_win.frames) * perSec,
      s_win.srcIdAdvance, s_win.srcFrames, s_win.srcIdAdvanceMaxPerFrame,
      static_cast<double>(s_win.ingest) * perSec, s_win.ingestMaxPerFrame, static_cast<double>(s_win.ingestBytes) * perSec / 1024.,
      static_cast<double>(s_win.pluginIngest) * perSec,
      static_cast<double>(s_win.stage) * perSec,
      static_cast<double>(s_win.gpuUpload) * perSec,
      upfAvg, upfPeak, gpuAt,
      s_ingestForeign.load(std::memory_order_relaxed), s_gpuForeign.load(std::memory_order_relaxed),
      avgMs, static_cast<double>(s_win.frameUsMax) * 1e-3,
      srcAvgMs, static_cast<double>(s_win.srcFrameUsMax) * 1e-3,
      quietAvgMs, static_cast<double>(s_win.quietFrameUsMax) * 1e-3,
      renderAvgMs, renderMaxMs);
   PLOGI << line;

   // Per-size breakdown: one "SZ" line per window listing every distinct
   // (width x height x bytes-per-pixel) ingested this second with its rate and
   // bandwidth share. This is what names the uploader: a 30/s bucket is a
   // video at its native cadence, a bucket locked to fps is a per-frame
   // re-upload, and the DMD shows up as 128x32x3 at exactly the chg rate.
   {
      struct SzRow { uint64_t key; uint32_t count; };
      SzRow rows[SZ_SLOTS];
      size_t n = 0;
      for (size_t i = 0; i < SZ_SLOTS; i++)
      {
         const uint64_t key = s_szKey[i].load(std::memory_order_relaxed);
         if (key == 0)
            continue;
         const uint32_t count = s_szCount[i].exchange(0, std::memory_order_relaxed);
         if (count > 0 && n < SZ_SLOTS)
            rows[n++] = { key, count };
      }
      const uint32_t szOver = s_szOverflow.exchange(0, std::memory_order_relaxed);
      if (n > 0 || szOver > 0)
      {
         for (size_t i = 1; i < n; i++) // insertion sort by count desc; n is tiny
         {
            const SzRow r = rows[i];
            size_t j = i;
            for (; j > 0 && rows[j - 1].count < r.count; j--)
               rows[j] = rows[j - 1];
            rows[j] = r;
         }
         char szLine[640];
         int pos = snprintf(szLine, sizeof(szLine), "DMDPROBE SZ");
         for (size_t i = 0; i < n && pos < static_cast<int>(sizeof(szLine)) - 48; i++)
         {
            const unsigned int w = static_cast<unsigned int>(rows[i].key >> 24);
            const unsigned int h = static_cast<unsigned int>((rows[i].key >> 4) & 0xFFFFFu);
            const unsigned int ps = static_cast<unsigned int>(rows[i].key & 0xFu);
            pos += snprintf(szLine + pos, sizeof(szLine) - pos, " %ux%ux%u:%.0f/s:%.0fKB/s",
               w, h, ps, static_cast<double>(rows[i].count) * perSec,
               static_cast<double>(rows[i].count) * w * h * ps * perSec / 1024.);
         }
         if (szOver > 0)
            pos += snprintf(szLine + pos, sizeof(szLine) - pos, " overflow:%u", szOver);
         PLOGI << szLine;
      }
   }

   // Frame cost breakdown at 1Hz — the A/B metric for BGFX submit-path work.
   // Logic side: prep = building the render frame (held under m_frameMutex, so
   // it serializes with the render thread's sub — every ms cut from either
   // widens the real frame budget). Render side: sub = VPX->BGFX encoding,
   // flip = bgfx::frame, waitsc = swapchain wait. draws/uni/st/tech are the
   // render device's last-frame counters; uni (ApplyUniform records) is the
   // one the uniform-value cache is expected to collapse. Sliding averages and
   // last-frame counters are written by their own threads and read torn-free
   // enough for a 1Hz diagnostic — same policy as the render stats above.
   if (g_pplayer != nullptr && g_pplayer->m_renderer != nullptr && renderProfiler != nullptr)
   {
      const RenderDevice* const rd = g_pplayer->m_renderer->m_renderDevice;
      const FrameProfiler& lp = g_pplayer->m_logicProfiler;
      char perfLine[512];
      snprintf(perfLine, sizeof(perfLine),
         "DMDPROBE PERF draws=%u uni=%u st=%u tech=%u | logic ms: prep %.2f/%.2f phys %.2f script %.2f misc %.2f sleep %.2f | "
         "render ms: sub %.2f/%.2f flip %.2f waitsc %.2f sleep %.2f",
         rd->Perf_GetNumDrawCalls(), rd->Perf_GetNumParameterChanges(), rd->Perf_GetNumStateChanges(), rd->Perf_GetNumTechniqueChanges(),
         static_cast<double>(lp.GetSlidingAvg(FrameProfiler::PROFILE_PREPARE_FRAME)) * 1e-3,
         static_cast<double>(lp.GetSlidingMax(FrameProfiler::PROFILE_PREPARE_FRAME)) * 1e-3,
         static_cast<double>(lp.GetSlidingAvg(FrameProfiler::PROFILE_PHYSICS)) * 1e-3,
         static_cast<double>(lp.GetSlidingAvg(FrameProfiler::PROFILE_SCRIPT)) * 1e-3,
         static_cast<double>(lp.GetSlidingAvg(FrameProfiler::PROFILE_MISC)) * 1e-3,
         static_cast<double>(lp.GetSlidingAvg(FrameProfiler::PROFILE_SLEEP)) * 1e-3,
         static_cast<double>(renderProfiler->GetSlidingAvg(FrameProfiler::PROFILE_RENDER_SUBMIT)) * 1e-3,
         static_cast<double>(renderProfiler->GetSlidingMax(FrameProfiler::PROFILE_RENDER_SUBMIT)) * 1e-3,
         static_cast<double>(renderProfiler->GetSlidingAvg(FrameProfiler::PROFILE_RENDER_FLIP)) * 1e-3,
         static_cast<double>(renderProfiler->GetSlidingAvg(FrameProfiler::PROFILE_RENDER_WAIT_SC)) * 1e-3,
         static_cast<double>(renderProfiler->GetSlidingAvg(FrameProfiler::PROFILE_RENDER_SLEEP)) * 1e-3);
      PLOGI << perfLine;
   }

   s_win = Window {};
}

}
