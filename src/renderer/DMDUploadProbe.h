// license:GPLv3+

#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>

// Measurement probe for vpinball#3675 ("DMD updates stutter the playfield on a
// 120Hz cabinet") vs. #3469 ("render FlexDMD on the main thread") vs. the
// coalescing proposed in #3676. The disagreement is about RATE vs THREADING:
// #3675 claims PinMAME emits a DMD frame per score event and each one costs a
// GPU texture upload that contends with the playfield; #3469 only moved DMD
// *rendering* off consumer threads. Nobody has measured the upload path since
// #3469 was picked up, so this probe counts it.
//
// What the code says the path is (so the counters below make sense):
//   - The display pipeline is PULL based. PinMAME (libpinmame) integrates its
//     PWM frame on the CALLER's thread inside GetRenderFrame and bumps a
//     monotonic lumFrameId only when the integrated image actually changed
//     (core_dmd_update_pwm in pinmame/src/wpc/core.c). Score-event bursts are
//     collapsed there: one pull sees at most one id increment.
//   - VPX consumers (flasher DMD/DISPLAY, textbox DMD, and plugins like
//     scoreview/b2s via VPXPluginAPIImpl::UpdateTexture) pull once per render
//     frame and call BaseTexture::Update UNCONDITIONALLY — there is no frameId
//     gate, so the CPU-side ingest (memcpy + alias clear + SetDirty) runs at
//     render rate even for a static image. That is the "ingest" counter.
//   - A dirty texture is staged by Sampler::UpdateTexture (bgfx::makeRef, no
//     copy) when it is bound ("stage"), and the actual bgfx::updateTexture2D
//     is issued from Sampler::GetCoreTexture on the render thread ("gpu").
//
// Design constraints, learned the hard way on this cab (see the images repo):
//   - Measurement must not perturb the measured thing (the x11grab capture
//     that starved itself to 12fps). Hot-path cost is one relaxed atomic add;
//     no locks, no clocks, no formatting. ONE log line per second, built on
//     the logic thread from counters the hot paths already fed.
//   - Off by default, and NOT an ini key: VPX rewrites VPinballX.ini on exit,
//     so an ini toggle can be silently lost or left armed. The switch is the
//     environment variable VPX_DMD_PROBE=1, read once at player start. With
//     it unset the probe is a single relaxed bool load per site.
//
// How to read the output (one line per ~1s window, prefix "DMDPROBE"):
//   f=120 fps=119.9        logic frames in the window and their rate
//   src id+31 chg31 max+2/f  default display (ctrl://default/display):
//                          total frameId advance, frames where it changed,
//                          peak advance in one frame. This is PinMAME's
//                          integrated-content change rate as seen by pullers —
//                          bursts beyond the total pull rate are collapsed
//                          upstream by design, which is exactly the finding.
//   ingest N/s maxM/f K KB/s (plugin P/s)
//                          BaseTexture::Update calls (all dynamic textures);
//                          "plugin" = the subset arriving through the plugin
//                          API (scoreview/b2s/pup), included in N.
//   stage N/s              Sampler::UpdateTexture (dirty texture staged)
//   gpu N/s                bgfx::updateTexture2D issued (render thread)
//   upf avgA peakB @T      THE decisive number: GPU uploads per RENDERED frame
//                          (avg over the window, peak in any single frame),
//                          measured render-side at Flip. ~1/DMD-visual and flat
//                          means uploads are frame-coalesced (pull model) and a
//                          scoring burst cannot multiply them; >>1 peaks mean a
//                          push model survives somewhere and work is being done
//                          that can never be displayed. @T reports whether the
//                          thread issuing bgfx::updateTexture2D is the render
//                          thread ("@render"), another thread ("@OTHER"), or
//                          none seen yet ("@none").
//   xthr iX gY             cumulative (not per-window) hits from a thread
//                          OTHER than the first one seen (i=ingest, g=gpu).
//                          Staying 0 means single-threaded, i.e. #3469's
//                          threading claim holds at this site.
//   logic avg/max          logic frame time in the window (ms)
//   srcf avg/max           ...restricted to frames where src frameId advanced
//   quiet avg/max          ...restricted to frames where it did not
//   render avg/max         render thread frame time (1s sliding, ms)
//
// Verdict mapping for #3675/#3676:
//   A (already coalesced): upf ≈ 1×(visible DMD visuals) and FLAT through
//     flipper volleys, src chg ~30-60/s — the pull model is in place, a burst
//     cannot multiply uploads, #3676 is unnecessary.
//   B (rate high, no harm): upf peaks >> 1 or src/ingest high, but
//     srcf ≈ quiet and render max stays flat — rate is real but harmless
//     because of threading; the fix (if any) is latest-wins coalescing of the
//     wasted uploads, not a timer.
//   C (#3469 insufficient): upf peaks and/or src bursts line up with srcf/render
//     max spikes — contention is real and frame-coalescing has a case.
//
// Queue note (code fact, not measured): all bgfx::updateTexture2D commands go
// into the SAME bgfx frame/submission stream as every view of every window
// (playfield + ancillary), executed in order on one graphics queue. Moving a
// producer to another thread moves who STAGES the data, never where the copy
// executes — so "off the render thread" cannot, by construction, take the
// upload off the playfield's GPU queue. Only its rate can change what the
// queue sees; that is what upf measures.

class FrameProfiler;

namespace VPX::DMDProbe
{
   extern std::atomic<bool> g_enabled;

   inline bool IsEnabled() { return g_enabled.load(std::memory_order_relaxed); }

   // Reads VPX_DMD_PROBE and logs a banner if enabling. Call once per player start.
   void Init();

   // Out-of-line counting; call only through the inline gates below.
   void CountIngest(unsigned int width, unsigned int height, unsigned int pixelSize);
   void CountPluginIngest();
   void CountStage();
   void CountGpuUpload();
   void CountRenderFlip();

   // Hot-path gates: a relaxed bool load when disabled, atomics when enabled.
   // Ingest takes the frame geometry, not pre-multiplied bytes: the 2026-08-02
   // cab measurement was mis-attributed for hours because the aggregate byte
   // rate admitted several size×rate factorizations (a 1080p surface at 100/s
   // and a 1.7Mpx surface at 120/s produce the same KB/s). The per-size table
   // in the "DMDPROBE SZ" line makes the decomposition an observation instead
   // of arithmetic.
   inline void OnIngest(unsigned int width, unsigned int height, unsigned int pixelSize) { if (IsEnabled()) CountIngest(width, height, pixelSize); }
   inline void OnPluginIngest()       { if (IsEnabled()) CountPluginIngest(); }
   inline void OnStage()              { if (IsEnabled()) CountStage(); }
   inline void OnGpuUpload()          { if (IsEnabled()) CountGpuUpload(); }
   // Once per rendered frame, from RenderDevice::Flip on the render thread:
   // closes the per-rendered-frame upload bucket (for the upf ratio) and
   // records the render thread's identity for the @T attribution.
   inline void OnRenderFrameFlip()    { if (IsEnabled()) CountRenderFlip(); }

   // Once per logic frame (Player::PrepareFrame, right after the profiler's
   // NewFrame so logicFrameUs is the just-completed frame). srcFrameId is the
   // default display's frameId from the caller's poll; renderProfiler is read
   // once per second for the render-thread frame stats.
   void OnLogicFrame(bool srcPresent, unsigned int srcFrameId, unsigned int logicFrameUs, const FrameProfiler* renderProfiler);
}
