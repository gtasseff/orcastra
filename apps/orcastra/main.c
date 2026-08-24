// orcastra — a real-time audio multi-tool for the FREE-WILi 2.
//
// Touch UI on the ST7796, 480x320. The home page is a 2x4 tile grid plus a
// transport strip; every tile opens a page, and BACK returns here:
//
//   FX         — the effects menu, 4x3 tiles. TAP toggles an effect, HOLD opens
//                its parameter page. Any combination runs at once; the pump
//                processes the whole chain in series, in the fixed order set out
//                at the audio pump below. Twelve are live (ORCATUNE, OCTAVE,
//                AUTO WAH, PITCH XY, OD, SHRED, BITCRUSH, EQ, FLANGER, GLITCH,
//                DELAY, REVERB); four more are written and disabled behind
//                FX_ENABLE_* in fx.h.
//   SAMPLER    — five PSRAM slots on the coloured hardware buttons (tap =
//                select, hold = record). Drag the pad to scrub playback rate
//                continuously, 1x reverse through stop to 1x forward; HOLD
//                latches the loop hands-free so you can back out to the FX menu
//                and process it. Runs through the chain.
//   SYNTH      — XY pad, 2.5 octaves on X, filter cutoff on Y; saw/sine/square.
//                With a note held the IMU becomes a modulation source.
//   DRUMS      — 16-step sequencer, five synthesised voices, four chainable
//                banks. Mixes pre- or post-chain (SEND TO FX).
//   THEREMIN   — pitch from the ambient light sensor (or the magnetometer),
//                volume from a touch strip.
//   KEYS       — chord keyboard: one octave x eight qualities, 4-voice
//                polyphony, FM e-piano or saw strings.
//   VIZ        — spectrum / scope. Also reachable from any page via PAGE, which
//                returns you to exactly where you were.
//   SETUP      — input profile, output route, output pad. Also the TUNER.
//
// Power-up defaults: IN: GUITAR, MIC IN source, 3.5 mm jack output.
//   INPUT      — analog gain profile: MIC HOT / MIC / LINE / LINE PAD
//                (NAU88C10 PGA gain reg 0x2D + +20 dB boost reg 0x2F — line
//                sources must run with the boost OFF or they clip the ADC)
//   OUTPUT     — onboard speaker or the 3.5 mm TRRS jack (safe during playback)
//   VOL        — output level via DAC volume reg 0x0B. The speaker is CAPPED at
//                -9 dB and high-passed at 900 Hz; see the volume policy section
//                for the power arithmetic and why those guards are not optional.
//
// The shipping build EXECUTES FROM PSRAM (target orcastra_psram) so this file's
// ~186 KB of code costs PSRAM rather than SRAM, leaving SRAM for .bss and the
// audio path. The BSP's PSRAM bootstrap handles that entry.
//
// MIC passthrough: TX and RX share ONE PIO state machine, so they are perfectly
// sample-locked. We run a 4-block TX ring (1024 frames) and write each completed
// capture block two slots ahead of the DMA reader — fixed phase forever, ~32 ms
// jitter margin, ~32-48 ms latency. The mono codec ADC (right I2S slot) is
// replicated to both output slots.
//
// Diagnostics over SEGGER RTT (fw rtt). Built entirely on the proven wilibsp BSP:
// NAU88C10 @ I2C1 addr 26, full-duplex I2S on PIO0 SM0, 48 kHz nominal
// (48,828 Hz actual, MCLK-locked — see FX_FS in fx.h).
#include "fw2.h"
#include "platform/diag.h"
#include "pico/stdlib.h"
#include "pico/bootrom.h"   // reset_usb_boot(): the BLUE-hold escape hatch
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "pico/multicore.h"
#include "synth.h"
#include "drums.h"
#include "viz.h"
#include "keys.h"
#include "agentio/agentio.h"
#include "input/uartkbd.h"
#include "input/app_recovery.h"
#include "sensors/bmi323.h"
#include "sensors/opt4001.h"
#include "sensors/bmm350.h"
#include "input/picpwr.h"
#include "fx.h"
#include "vox.h"
#include "voice_data.h"

// Big-endian RGB565 (the panel sends the high byte first).
static inline uint16_t rgb565_be(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t c = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8));
}

// ---- Layout (480x320 landscape) ----
// PRODUCTION-ENCLOSURE SAFE AREA, measured 2026-08-02 by sweeping a finger
// around the bezel of the real case and logging the touch envelope:
//     left edge   x stops at  24   <- the bezel; the one hard constraint
//     right edge  reaches    479   <- clear
//     top/bottom  reach 0 and 319  <- clear, away from the corners
//     corners     top-left (42,56) is worst; then (441,38) (443,286) (38,282)
// The bare dev board let a finger reach every pixel, so pre-enclosure layouts
// put controls at x=8 and x=4 -- physically unpressable now, and the user
// still sees the button, so a dead target reads as a broken device. Every
// touchable rect below therefore starts at SAFE_L, and nothing important sits
// in the top-left corner pocket.
// SAFE_L was 32, inferred from a bounding-box sweep that reported the left
// edge stopping at x=24. That was an over-correction and it left a visibly dead
// strip down every page. The 8x5 reach grid disproved it: the centres of B1,
// C1, D1 and E1 all sit at x=30 and all registered. A button is pressed in its
// MIDDLE, so one starting at x=10 has its centre near 60 and is easy. The real
// constraint was never the left edge -- it was the top-left CORNER (cell A1,
// the single cell that failed, and it registers as B1, i.e. the touch lands a
// row low). Keep that corner clear of anything critical; the edge itself is
// fine. BACK still lives clear of it, which was the change that mattered.
#define SAFE_L 10                      /* first comfortably touchable column */
#define SAFE_R 472                     /* one past the last                   */
#define SAFE_W (SAFE_R - SAFE_L)       /* 440 usable columns                  */
// Column helper for evenly spaced grids inside the safe area.
// Reach grid, 2026-08-03 (8x5 cells, tapped by thumb in a normal two-handed
// grip): 39 of 40 cells register. Only A1 -- the top-left cell -- fails, and it
// registers as B1, i.e. the touch lands about a row low there. So the right and
// bottom edges need NO inset (column 8 centres at x=450 and row E at y=288 are
// both comfortable), and the only genuinely unusable area is the top-left
// corner. This corrects the earlier bounding-box measurement, which was far too
// pessimistic about the right side.
//
// SAFE_R vs REACH_R: a BUTTON is pressed in its middle, so one ending at 472 is
// fine. A continuous STRIP is different -- its endpoint has to be touchable for
// the value at that end to be selectable at all.
#define REACH_R 450

// ---- THE INTERACTION BAND: every continuous control is DRAWN here ----------
//
// Superseding the older approach, which drew pads and strips nearly full-width
// (x 10..472) and then mapped the extremes to reachable positions inside them.
// That made drawn != usable: 1x playback on the turntable landed at about 80%
// of the visible pad, the outer ~40 px of every control did nothing, and a user
// reasonably expects the edge of a drawn control to be its extreme. Reported
// from the bench in production casing for the turntable, the SYNTH pad AND the
// X/Y pitch pad -- i.e. it was never one control's bug, it was the policy.
//
// So: bring the CONTROLS in instead of bending the MAPPING. Inside this band the
// mapping is 1:1 with only a small centroid allowance, so the drawn edge IS the
// extreme and the handle sits under the finger.
//
// 90 and 390 are not arbitrary: the 8x5 reach grid uses 60 px cells whose
// centres are 30, 90, 150 ... 450, and every cell except A1 registered. 90 and
// 390 are the second cell centres in from each edge, so they are measured-good
// as taps, which leaves margin for the harder case of DRAGGING to an extreme and
// holding it there while the other hand grips the case. Symmetric about 240, so
// a centred detent stays centred on the panel.
#define TRACK_L 90
#define TRACK_R 390
#define TRACK_W (TRACK_R - TRACK_L)          /* 300 */
#define TRACK_CX ((TRACK_L + TRACK_R) / 2)   /* 240, the panel centre */
// Centroid allowance only. The touch controller reports a centroid that lands
// short of the fingertip, so the outer few pixels of the band still saturate --
// but 12 px, not the 26-80 px the old scheme needed to dodge the bezel.
#define TRACK_PAD 12.0f
#define SAFE_COL(i, n, gap) (SAFE_L + (i) * ((SAFE_W + (gap)) / (n)))
#define SAFE_CW(n, gap)     (((SAFE_W + (gap)) / (n)) - (gap))

// ---- Touch -> normalised value, with reachable-end saturation --------------
// Every continuous control (parameter sliders, the XY pads, the turntable
// strip) needs the SAME two corrections. They were fixed one page at a time --
// the knob sliders got KTRK_PAD, then the theremin volume strip got its own
// copy -- and every page that had not been played yet still shipped the bug.
// Centralised here so a page cannot be forgotten again:
//
//   1. FULL SCALE AT REACH_R, NOT THE CONTROL'S DRAWN RIGHT EDGE. The pads are
//      462-470 px wide, i.e. they run to x=470..472, but the 8x5 reach grid
//      says a thumb is comfortable only to about x=450. The top of the range
//      was therefore not selectable at all. Reported from the bench as three
//      separate faults that are one fault: "cannot get to 100% playback speed
//      on the sampler", "cannot get the full range on the X/Y pitch pad",
//      "cannot get to the highest pitch on the synth page".
//   2. END SATURATION. Even well inside the reachable region a finger cannot
//      land on the literal first or last pixel, and the touch controller
//      reports a CENTROID that sits short of the fingertip. So the outer pad
//      at each end saturates to the extreme value instead of merely
//      approaching it -- the same trick KTRK_PAD plays on the knob sliders.
//
// Drawn geometry is deliberately left alone: the fill/cursor still spans the
// full track so the reading stays honest, and only the mapping saturates.
// TOUCH_PAD_X / TOUCH_PAD_Y / TOUCH_PAD_TT_* are GONE. They existed to bend
// the mapping inside a control drawn wider than a thumb can reach; the band
// above removes the need by bringing the control in instead. TRACK_PAD is all
// that remains, and it covers the touch centroid only.


static float touch_norm(float v, float lo, float hi,
                        float pad_lo, float pad_hi) {
    float span = (hi - pad_hi) - (lo + pad_lo);
    if (span < 1.0f) span = 1.0f;
    float t = (v - (lo + pad_lo)) / span;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t;
}

// Bipolar variant, for the two controls that have a PAINTED centre detent (the
// FX pad's octave axis and the turntable's rate axis). Rescaling those the
// naive way would slide zero ~10 px left of the drawn line, so instead each
// half is stretched independently: the detent stays exactly where it is drawn
// and both -1 and +1 become attainable. Returns -1..+1.
static float touch_norm_bipolar(float v, float lo, float ctr, float hi,
                                float pad_lo, float pad_hi) {
    if (v >= ctr) {
        float span = (hi - pad_hi) - ctr;
        if (span < 1.0f) span = 1.0f;
        float t = (v - ctr) / span;
        return t > 1.0f ? 1.0f : t;
    }
    float span = ctr - (lo + pad_lo);
    if (span < 1.0f) span = 1.0f;
    float t = (ctr - v) / span;
    if (t > 1.0f) t = 1.0f;
    return -t;
}

typedef struct { int x, y, w, h; } rect_t;
static const rect_t BTN_PLAY   = {  10, 270, 124,  44 };  // launcher transport
static const rect_t BTN_TUNERQ = { 362, 270, 110,  44 };  // quick-access tuner
                                                          // (it's not an effect)
static const rect_t BTN_IN   = {  20, 190, 218,  44 };  // SETUP: gain profile
static const rect_t BTN_SRC  = { 250,  90, 210,  44 };  // SETUP: input source
static const rect_t BTN_OUT  = { 250, 140, 210,  44 };  // SETUP: output route
static const rect_t BTN_VOL  = { 250, 190, 210,  44 };  // SETUP: volume
// y 236 h 42 (was 240 h 44) to clear the status strip. draw_status() fills
// y 282..306 starting at x=158, which is INSIDE this button's 20..238 span, so
// at y 240..284 the strip erased the button's bottom two rows and the yellow
// route text ("3.5MM>JACK") sat hard against it -- reported as the text
// clipping over the button's bottom-left. 236..278 leaves a 4 px gap. Moving the
// strip instead is not an option: it is shared with the launcher, where PLAY
// ends at 134 and TUNER starts at 362, so its x range is pinned.
static const rect_t BTN_APWR = {  20, 236, 218,  42 };  // SETUP: request the
                                                        // audio codec rail
// Launcher: the main page is a 2x4 tile grid (big walk-up touch targets,
// full names) + a bottom transport strip. The crowded top-row era is over.
static const rect_t BTN_GFX   = {  10,  48, 110, 104 };  // -> FX menu
static const rect_t BTN_VOX   = { 128,  48, 110, 104 };  // -> Sampler
static const rect_t BTN_SYN   = { 246,  48, 110, 104 };  // -> Synth
static const rect_t BTN_DRM   = { 364,  48, 108, 104 };  // -> Drums
static const rect_t BTN_THM   = {  10, 158, 110, 104 };  // -> Theremin
static const rect_t BTN_KEY   = { 128, 158, 110, 104 };  // -> Keys chord pad
static const rect_t BTN_VIZ   = { 246, 158, 110, 104 };  // -> Visualizer
static const rect_t BTN_SETUP = { 364, 158, 108, 104 };  // -> I/O setup page
// Drums page: transport + grid geometry.
static const rect_t BTN_D_PLAY  = { 372,   6, 100,  44 };
static const rect_t BTN_D_CLEAR = {  10, 244, 100,  40 };
// Bank tabs A..D + chain-length cycle (share the CLEAR/BPM row).
static const rect_t BTN_D_BANK[DRUM_BANKS] = {
    { 244, 244, 40, 40 }, { 288, 244, 40, 40 },
    { 332, 244, 40, 40 }, { 376, 244, 40, 40 },
};
static const rect_t BTN_D_LEN   = { 420, 244,  52,  40 };
static const rect_t BTN_D_FX    = { 142,   6,  92,  44 };  // SEND TO FX
// Drums and keys mix AFTER the FX chain by default (jam over a clean beat).
// These opt them in to the chain instead.
static bool drums_to_fx = false;
static bool keys_to_fx  = false;
#define DR_GX 58
#define DR_GY 56
#define DR_CW 26
#define DR_RH 34
// BACK sits on every sub page, and it used to live at {8,6} -- the exact
// top-left corner pocket, the single least reachable spot on the production
// unit (finger bottomed out at 42,56 there). Moved clear of both the bezel
// and the corner: the bulk of it now lands in easy reach.
static const rect_t BTN_BACK = {  32,   6, 104,  44 };  // sub pages, one level up
// Sample page: slot buttons mirroring the board's colored buttons, a control
// row (HOLD / input source / volume), then the turntable pad.
//
// FOUR slots, not five, since 2026-08-03. The fifth was RED, and holding RED
// is the power coprocessor's own power-off gesture -- it fires beneath
// anything this firmware can intercept, so a red "hold to record" slot could
// (and did, mid-demo) switch the unit off. There is no way to teach a walk-up
// user "tap this one, never hold it", so the slot is gone rather than
// documented. RED stays a TAP-only EQ band selector elsewhere; taps are safe.
// vox.h still allocates VOX_SLOTS==5 PSRAM buffers on purpose -- slot 4 is
// retained, hidden, as the load target for the flash sample library.
#define VOX_UI_SLOTS 4
static const rect_t BTN_V_SLOT[VOX_UI_SLOTS] = {
    {  10, 52, 110, 44 }, { 128, 52, 110, 44 },
    { 246, 52, 110, 44 }, { 364, 52, 108, 44 },
};
static const rect_t BTN_V_HOLD = {  10, 100, 148, 34 };
static const rect_t BTN_V_SRC  = { 166, 100, 148, 34 };
static const rect_t BTN_V_VOL  = { 324, 100, 148, 34 };
#define VPAD_X TRACK_L        /* turntable: drawn == usable */
#define VPAD_Y 140
#define VPAD_W TRACK_W
#define VPAD_H 118
#define VBAR_Y 262
#define VSTATUS_Y 276
// FX menu: 3 x 4 grid of effect buttons (tap = toggle, hold = page).
static const rect_t BTN_M_FX    = {  10,  52, 146,  60 };
static const rect_t BTN_M_OCT   = { 166,  52, 146,  60 };
static const rect_t BTN_M_GLI   = { 322,  52, 146,  60 };
static const rect_t BTN_M_SLAP  = {  10, 118, 146,  60 };
static const rect_t BTN_M_MET   = { 166, 118, 146,  60 };
static const rect_t BTN_M_PARA  = { 322, 118, 146,  60 };
static const rect_t BTN_M_OD    = {  10, 184, 146,  60 };
static const rect_t BTN_M_TREM  = { 166, 184, 146,  60 };
static const rect_t BTN_M_PHASE = { 322, 184, 146,  60 };
static const rect_t BTN_M_TUNE  = {  10, 250, 146,  60 };
static const rect_t BTN_M_TUNER = { 166, 250, 146,  60 };  // utility: tuner
static const rect_t BTN_M_EQ    = { 322, 250, 146,  60 };  // parametric EQ
// Keys page: instrument + sustain up top, chord-type column left third,
// one-octave piano (C..C) right two thirds.
static const rect_t BTN_K_INST = { 210,   6, 120,  44 };
static const rect_t BTN_K_SUS  = { 336,   6, 136,  44 };
static const rect_t BTN_K_FX   = { 142,   6,  62,  44 };  // SEND TO FX
// Piano shifted right so the chord column clears the bezel. Was KB_X 160 with
// chord buttons at x=4 w=74 -- a THIRD of every chord button was behind the
// bezel and unpressable. At 178 the piano still ends at 451 (7 x 39), inside
// the safe area, and the chord column gets 32..174 fully in reach.
#define KB_X  178
#define KB_Y  52
#define WK_W  39
#define WK_H  264
#define BK_W  26
#define BK_H  158
// Theremin page: mode toggle + hold latch up top, pitch/volume meter bars,
// gate strip along the bottom.
static const rect_t BTN_TH_MODE = { 142,   6,  78,  44 };
static const rect_t BTN_TH_CAL  = { 224,   6,  46,  44 };
static const rect_t BTN_TH_OCTM = { 274,   6,  50,  44 };
static const rect_t BTN_TH_OCTP = { 328,   6,  50,  44 };
static const rect_t BTN_TH_HOLD = { 382,   6,  90,  44 };
#define THM_BAR_X TRACK_L     /* theremin meters + gate strip */
#define THM_BAR_W TRACK_W
#define THM_BAR_H 36
#define THM_PIT_Y 92
#define THM_VOL_Y 168
#define THM_PAD_Y 232
// Synth page: wave selector row + XY pad + note hold latch.
static const rect_t BTN_S_HOLD = { 372,   6, 100,  44 };
static const rect_t BTN_S_SAW  = {  10,  52, 148,  44 };
static const rect_t BTN_S_SIN  = { 166,  52, 148,  44 };
static const rect_t BTN_S_SQR  = { 324,  52, 148,  44 };
// FULL WIDTH by request -- it simply looks better, and the edge notes were
// never the point. With the range widened below, the comfortably reachable
// middle (about x 90..390 of 10..472) still covers ~1.6 octaves, so more
// than a full octave is playable without stretching for the bezel.
#define SYN_PAD_X 10
#define SYN_PAD_Y 104
#define SYN_PAD_W 462
#define SYN_PAD_H 194
// EQ page: response plot + Q slider strip.
static const rect_t BTN_EQTOG  = { 372,   6, 100,  44 };
// FULL WIDTH by request. The band exists for controls whose EXTREMES must be
// touchable; the EQ plot is navigated with the coloured band buttons and the
// d-pad as well as by dragging, so losing 160 px of plot to guarantee edge
// reach was a bad trade -- the plot IS the information here.
#define EQ_PX 10
#define EQ_PY 48
#define EQ_PW 462
#define EQ_PH 192
// 276, was 268. The Q HANDLE is drawn 8 px above the track and 24 px tall, so
// at 268 it occupied y 260..284 while the readout above it occupies 246..264 --
// the handle bit 4 px out of the bottom of "YELLW 202Hz +0.5dB Q1.0", and
// eq_draw_qslider()'s own clear rect took the rest. Visible in the harness as
// the yellow handle cutting through the readout's letters. Latent before the
// full-width revert too (the banded handle still ranged x 96..384, under a
// readout starting at x=120), just less often. Budget below the plot: it ends
// at y=240 and the hint sits at 302, so 276 leaves 4 px under the readout and
// 10 px above the hint.
#define EQ_SLY 276              // Q slider track y
// The Q strip follows the PLOT, not the effect-page interaction band: this page
// is full width by request, and a 300 px strip under a 462 px plot looked wrong.
// Separate names so reverting EQ does not drag the effect pages with it.
#define EQ_SLX EQ_PX
#define EQ_SLW EQ_PW
#define EQ_DB_RANGE 14.0f       // plot +/- dB full scale
// Tuner page: 180-degree flat<->sharp gauge geometry.
#define TUN_CX 240
#define TUN_CY 272
#define TUN_RT 142                  // tick-mark radius
#define TUN_RN 128                  // needle length
// Orcatune page: toggle + key/scale/mode cycle buttons.
static const rect_t BTN_T_KEY   = {  10, 150, 148,  54 };
static const rect_t BTN_T_SCALE = { 166, 150, 148,  54 };
static const rect_t BTN_T_MODE  = { 324, 150, 148,  54 };
// Slap time slider: track maps 50..200 ms across its width.
#define SLIDER_X TRACK_L      /* single-slider effect pages */
// Moved up from 226 (strip 204) once the 300x84 ON/OFF button was removed from
// these pages: it had occupied y 80..164, and leaving the slider where it was
// would have left 158 px of a 320 px screen empty between the header and the
// track. Every one of the seven bespoke sliders is built purely from these
// defines -- track, handle, value text, end labels AND the touch band -- so
// moving the pair moves drawing and hit-testing together. Checked clear of the
// only other content in that band: the delay page's tab row ends at y=76 and
// the touch band now starts at 138.
#define SLIDER_Y 170
#define SLIDER_W TRACK_W
#define SLIDER_STRIP_Y 148   // full redraw strip (handle + track)
#define SLIDER_STRIP_H 46

// Integer touch mapping for the single-slider effect pages. Their tracks
// already END at REACH_R (SLIDER_X 20 + SLIDER_W 430 = 450), so correction 1
// above is already satisfied -- but correction 2 was missing, and it mattered:
// the naive form all eight pages used,
//     lo + (x - SLIDER_X) * (hi - lo) / SLIDER_W
// needs the LITERAL LAST PIXEL of the track to yield hi, so the top value of
// every one of these knobs was effectively unselectable. This is the same
// complaint the generic knob sliders already fixed with KTRK_PAD ("the bench
// could only get 1..9 out of a 0..10 knob") -- it was simply never carried
// across to the bespoke pages.
static int slider_val(int x, int lo, int hi) {
    float t = touch_norm((float)x, SLIDER_X, SLIDER_X + SLIDER_W,
                         TRACK_PAD, TRACK_PAD);
    int v = lo + (int)(t * (float)(hi - lo) + 0.5f);
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return v;
}
#define STATUS_Y 282   // transport strip, right of PLAY (launcher + setup)
// FX pad area (page 2): X = pitch (-1 oct left .. +1 oct right, per the
// user's (1,1)-(9,9) corner examples), Y = echo amount (dry bottom .. max top).
// FULL WIDTH by request. Same reasoning as the synth pad: the extremes here
// are the ENDS OF A RANGE, not named values a user must be able to select
// exactly, so widening the range absorbs the reach loss and nothing is lost.
// FX_PAD_OCTAVES below does that. Contrast the sampler strip and the theremin
// volume bar, which stay inside the band because "1x forward" and "full volume"
// ARE named values -- there is no equivalent of widening the range for those.
#define PAD_X 10              /* FX X/Y pitch pad */
#define PAD_Y 54
#define PAD_W 462
#define PAD_H 256

typedef enum { PAGE_MAIN, PAGE_FXMENU, PAGE_FX, PAGE_GLI, PAGE_SLAP,
               PAGE_MET, PAGE_PARA, PAGE_OD, PAGE_TREM, PAGE_PHASE,
               PAGE_TUNE, PAGE_TUNER, PAGE_EQ, PAGE_SYNTH, PAGE_DRUMS,
               PAGE_VOX, PAGE_THMN, PAGE_VIZ, PAGE_KEYS, PAGE_VERB,
               PAGE_KNOB,
               PAGE_CAL,
               PAGE_SETUP } page_t;
static volatile page_t page = PAGE_MAIN;   // UI writes, audio core reads

// UI state (file-scope so page-redraw functions can use it).
// Power-up defaults: guitar into the 3.5 mm jack, out to an amp on the jack.
// Shared UI-core/audio-core flags (core 0 writes, core 1 reads).
static volatile bool playing = false;
static volatile bool mic     = true;   // live input mode (false = TONE source)
static volatile bool use_pdm = false;  // live input: jack ADC / PDM array
static bool jack    = true;     // 3.5 mm jack out (false = speaker)
static volatile bool mic_armed = false; // mic TX ring armed (arms on 1st block)
static volatile bool spk_route = false; // true while output -> onboard speaker
static int  mic_slot  = 0;      // next ring slot to fill (audio core only)
// Audio-core -> UI mirrors (UI never touches audio data directly).
static volatile uint16_t pump_vu = 0;          // input peak, latest block
static volatile uint16_t pump_vu_out = 0;      // OUTPUT mix peak, latest block
static volatile uint16_t pump_vu_avg = 0;      // OUTPUT mix mean |sample| —
                                               // the LED meter runs on this
                                               // (peak pegged on transients)
                                               // (drums/synth/vox included, so
                                               // the LED meter follows what you
                                               // actually hear)
static volatile uint16_t pump_raw_l = 0, pump_raw_r = 0;
static volatile uint32_t pump_blocks = 0;      // blocks processed (lifetime)
static volatile uint32_t pump_busy_us = 0;     // cumulative audio-core work
                                               // time (load meter)
// Pop hunter (2026-07-23): timestamps (in blocks) of input transients, so
// the ~500 ms click's TRUE period can be measured instead of guessed.
static volatile uint32_t pop_blk[8];
static volatile uint32_t pop_n = 0;
static volatile uint32_t pop_maxd = 0;         // rolling max |Δsample|
// FX-chain input level, in DEVICE dBFS. Every clamp and threshold in fx.c is
// specified against this signal, and it cannot be inferred from a line-in
// capture of the jack (the PC's input gain and the output stage both sit in
// between — bench 2026-07-30 chased that mistake twice). The audio core only
// accumulates; core 0 converts and prints on the beat line.
static volatile uint32_t inlvl_pk_raw = 0;     // peak |sample| before the gate
static volatile uint32_t inlvl_pk = 0;         // peak |sample| after the gate
static volatile uint64_t inlvl_sumsq = 0;      // sum of squares, after the gate
static volatile uint32_t inlvl_n = 0;          // samples in the accumulation
// Output side, measured on the block as it goes to the DAC. The clamp count
// is the direct answer to "is this effect (or this stack of effects) at this
// level overflowing?" — every fx stage limits at +/-32700, and a stage that
// spends time on its limit is audible as fuzz. Inferring it from a line-in
// capture does not work: the capture sits ~6 dB below the device's own scale,
// so internal clipping can hide entirely.
static volatile uint32_t outlvl_pk = 0;        // peak |sample| at the DAC
static volatile uint32_t outlvl_clip = 0;      // samples sitting on the limit

// dBFS as a rounded integer. Reported alongside the raw sample counts so an
// exact figure can be recovered off-device without needing float formatting
// in the RTT path.
static int dbfs_i(uint32_t counts) {
    if (counts == 0) return -99;
    float d = 20.0f * log10f((float)counts / 32768.0f);
    return (int)(d < 0.0f ? d - 0.5f : d + 0.5f);
}
static int  in_idx  = 2;        // IN: GUITAR
static bool tuner_from_main = false;   // quick-access tuner: BACK -> launcher
static bool keys_reset_req = false;    // HOME left the keys page: its touch
                                       // branch resets its statics next poll
static int  vol_idx = 2;        // VOL 0DB (jack default; watch it on speaker)
static bool codec_ok = false;
// Effect toggles — any combination can be active at once; the pump runs the
// full chain in series and each effect is transparent when off.
static bool pad_on = false;   // FX pad: remembered x/y applied on/off
// Half-range of the FX pad's X axis, in octaves. 1.5 rather than 1.0 because
// the pad is drawn full width (x 10..472) while a thumb in the production case
// comfortably covers about 90..390. Through touch_norm_bipolar's 12 px end
// allowance that reaches |0.68| of the axis, so +/-1.5 puts the musically
// important +/-1 OCT right at the edge of comfortable reach and leaves the extra
// half-octave out in the corners you have to stretch for.
#define FX_PAD_OCTAVES 1.5f
static float pad_oct = 0.0f;  // FX pad remembered pitch (-1.5..+1.5 oct)
static float pad_amt = 0.0f;  // FX pad remembered echo amount (0..1)
static int  pad_px = -1, pad_py = -1;  // FX pad remembered cursor pixels
static bool oct_on = false;   // octave toggle state
static int  oct_mode = 0;     // 0 +1oct, 1 +2oct, 2 -1oct, 3 -2oct, 4 detune
static int  oct_mix_pct = 50; // octave MIX 0..100 % wet (50 = old fixed blend)
// 45, not 50: at exactly 50 the shifter is bit-transparent, so selecting DETUNE
// mode would appear to do NOTHING until you found the slider. 45 = -10 cents,
// which beats slowly against the dry -- bench verdict 2026-08-01 "a nice amount
// of detune... a really nice almost-chorus effect". Worth noting it lands the
// job the cut CHORUS was hired for.
static int  oct_det = 45;     // detune 0..100 -> -100..+100 cents (50 = in tune)
static bool gli_on = false;   // glitch toggle state
static int  gli_amt = 6;      // glitch burst FREQ (1..10)
static bool slap_on = false;  // slap toggle state
static int  dly_ms   = 120;   // DELAY time 40..330 ms
static int  dly_fb_amt  = 0;  // DELAY feedback 0..10 (0 = slapback)
static int  dly_mix_amt = 7;  // DELAY echo level 0..10
static int  dly_sel = 0;      // which knob the slider edits: 0 T, 1 F, 2 M
static bool met_on = false;   // metal toggle state
static int  met_amt = 30;     // metal drive (5..60)
static bool para_on = false;  // paranoid toggle state
static int  para_rng = 8;     // paranoid sweep RANGE (0..10)
static bool bc_on = false;    // bit crush toggle
static int  bc_amt = 7;       // crush AMOUNT: 7 is the bench sweet spot
#if FX_ENABLE_RINGMOD
static bool rm_on = false;    // ring mod toggle
static int  rm_amt = 5;       // ring FREQ 1..10
#endif
static bool fl_on = false;    // flanger toggle
// Bench picks 2026-08-01 ("a default for flanger sounds better with rate at 7
// and color at 6"). Both moves are consistent with the mechanism: RATE 4 is
// 0.29 Hz, slow enough to sit still, and COLOR 7 is feedback 0.595 against
// COLOR 6's 0.51 -- less feedback is literally less comb resonance, which is
// the thing that reads as "robotic".
static int  fl_amt = 7;       // flanger RATE 1..10 -> 0.05..10 Hz
static int  fl_mode = 0;      // flanger MODE 0 FLANGE, 1 FILTER MATRIX
static int  fl_col = 6;       // flanger COLOR 0..10 -> feedback 0..0.85
#if FX_ENABLE_VOWEL
static bool vw_on = false;    // vowel filter toggle
static int  vw_rate_amt = 4;  // vowel sweep RATE 1..10
static int  vw_pos_amt = 5;   // vowel POSITION 0..10 (MANUAL mode)
static int  vw_mode = 0;      // 0 = AUTO sweep, 1 = MANUAL position
#endif
#if FX_ENABLE_TREMOLO
static bool trem_on = false;  // tremolo toggle state
static int  trem_amt = 5;     // tremolo RATE 1..10 Hz (5 = the DSP default)
#endif
static bool verb_on = false;  // reverb toggle state (post-chain master fx)
// Bench picks 2026-08-01 after the mix went additive: MIX 9, DECAY 10,
// SHIMMER 10 -- "that sounded good and nice". MIX means something different
// now: full dry PLUS this much wet, so 9 is not the drowned-out 9 it was
// under the old crossfade. Measured +0.3 dB vs dry at MIX 10, zero clipping.
static int  verb_mix_amt = 9; // reverb MIX 0..10, wet ADDED over full dry
static int  verb_dec_amt = 10; // reverb DECAY 0..10 -> ~1.2..7.5 s RT60
// 6, down from 8: on a clean amp path the bench found "above that it sounded
// like a single-tone high frequency whine... with a hissy aura", and 6 was the
// verdict. RV_SH_LP was also lowered to stop the octave ladder piling up, so 8
// may be usable again -- worth a re-listen before changing this back.
static int  verb_sh_amt  = 10; // reverb SHIMMER 0..10 -> 0..0.6 (bench: 10;
                               // safe at every DECAY now that RV_SH_IN is fixed)
#if FX_ENABLE_RINGMOD
static int  rm_mix_amt   = 7; // ringmod MIX 0..10 (dry anchors chords)
#endif
static bool od_on = false;    // overdrive toggle state
static int  od_amt = 6;       // overdrive DRIVE (1..10); 4 was too polite
#if FX_ENABLE_PHASER
static bool ph_on = false;    // phaser toggle state
static int  ph_amt = 3;       // phaser RATE (1..10)
#endif
static bool tune_on = false;  // pitch-correction toggle state
static int  tune_key = 0;     // key root (0 = C .. 11 = B)
static int  tune_scale = 0;   // 0 = major, 1 = minor, 2 = chromatic
static bool tune_robot = true; // true = robotic hard-tune, false = natural
// Parametric EQ: UI-side band parameters (pushed to fx via fx_eq_band).
static bool  eq_on = false;
static float eq_f[5]  = { 80.0f, 250.0f, 800.0f, 2500.0f, 6000.0f };
static float eq_g[5]  = { 0, 0, 0, 0, 0 };
static float eq_qv[5] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
static int   eq_sel = 2;      // selected band (colored buttons / tap)
static bool  eq_dirty = false; // plot needs a (throttled) redraw

// Re-sync every effect's DSP state to the UI toggles/params (fx_reset wipes
// the DSP-side mixes, so call this right after any fx_reset).
static void fx_apply_all(void) {
    fx_set_params(pad_on ? pad_oct : 0.0f, pad_on ? pad_amt : 0.0f);
    fx_octave_mode(oct_mode);        // MIX before the toggle: fx_octave_set
    fx_octave_detune(oct_det);       // fades TO the mix value
    fx_octave_mix(oct_mix_pct);
    fx_octave_set(oct_on);
    fx_glitch_freq(gli_amt);
    fx_glitch_set(gli_on);
    fx_delay_set(slap_on);
    fx_delay_time_ms(dly_ms);
    fx_delay_fb(dly_fb_amt);
    fx_delay_mix(dly_mix_amt);
    fx_metal_set(met_on);
    fx_metal_drive(met_amt);
    fx_para_set(para_on);
    fx_para_range(para_rng);
    fx_od_set(od_on);
    fx_od_drive(od_amt);
    fx_verb_set(verb_on);
    fx_verb_mix(verb_mix_amt);
    fx_verb_decay(verb_dec_amt);
    fx_verb_shimmer(verb_sh_amt);
#if FX_ENABLE_TREMOLO
    fx_trem_set(trem_on);
    fx_trem_rate(trem_amt);
#endif
    fx_crush_set(bc_on);
    fx_crush_amount(bc_amt);
#if FX_ENABLE_RINGMOD
    fx_ring_set(rm_on);
    fx_ring_freq(rm_amt);
    fx_ring_mix(rm_mix_amt);
#endif
    fx_flange_set(fl_on);
    fx_flange_rate(fl_amt);
    fx_flange_color(fl_col);
    fx_flange_mode(fl_mode);
#if FX_ENABLE_VOWEL
    fx_vowel_rate(vw_rate_amt);
    fx_vowel_pos(vw_pos_amt);
    fx_vowel_mode(vw_mode);
    fx_vowel_set(vw_on);
#endif
#if FX_ENABLE_PHASER
    fx_phase_set(ph_on);
    fx_phase_rate(ph_amt);
#endif
    fx_tune_set(tune_on);
    fx_tune_key(tune_key);
    fx_tune_scale(tune_scale);
    fx_tune_mode(tune_robot);
    fx_eq_set(eq_on);
    for (int i = 0; i < 5; i++) fx_eq_band(i, eq_f[i], eq_g[i], eq_qv[i]);
}

// OBSOLETE (2026-07-17, kept as a no-op so call sites document their
// history): full-screen redraws used to starve the single-loop mic pump and
// slip the TX ring phase — "permanent fuzz" needing a re-arm. The audio
// pump now runs on CORE 1 and UI drawing can never stall it, so re-arming
// after redraws would only cause a needless 5-10 ms audio gap.
static void mic_ring_rephase(void) { }

static bool hit(const rect_t *r, uint16_t x, uint16_t y) {
    return x >= r->x && x < r->x + r->w && y >= r->y && y < r->y + r->h;
}

// Draw a button: filled rect + centered label (glyph cell = 6*scale x 8*scale).
static void draw_button(const rect_t *r, uint16_t bg, int scale, const char *label) {
    st7796_fill_rect(r->x, r->y, r->w, r->h, bg);
    int len = 0; while (label[len]) len++;
    int tx = r->x + (r->w - len * 6 * scale) / 2;
    int ty = r->y + (r->h - 8 * scale) / 2;
    st7796_draw_text(tx, ty, scale, rgb565_be(255, 255, 255), bg, label);
}

static void draw_status(uint16_t bg_screen, bool playing, bool mic, bool jack) {
    // Compact: lives between PLAY and TUNER on the transport strip.
    char line[24];
    const char *src = mic ? (use_pdm ? "PDM" : "3.5MM") : "TONE";
    const char *out = jack ? "JACK" : "SPKR";
    int n = 0;
    if (!playing) {
        for (const char *p = "MUTED"; *p; p++) line[n++] = *p;
    } else {
        for (const char *p = src; *p; p++) line[n++] = *p;
        line[n++] = '>';
        for (const char *p = out; *p; p++) line[n++] = *p;
    }
    line[n] = '\0';
    // PLAY/STOP now ends at x=152 (it moved right out of the bezel), and
    // TUNER starts at 362, so the status strip lives strictly between them.
    // At 136 this fill was painting over the button's right edge and the text
    // at 148 landed inside it.
    st7796_fill_rect(158, STATUS_Y, 200, 24, bg_screen);
    // A flat battery outranks the routing: when the coprocessor's low-voltage
    // cutoff fires it drops every rail -- display, audio and debug probe
    // together, with no graceful recovery. Bench experience 2026-07-30: the
    // cell reached 3.02 V and did exactly that.
    //
    // Thresholds are 3.6 V (warn) and 3.3 V (urgent), reportedly what the PIC
    // itself watches for, with shutdown near 3.0 V. UNVERIFIED -- second-hand
    // from a dev, not measured here, so treat the exact numbers as provisional.
    // Amber then red so there is warning before it is urgent.
    uartkbd_charger_t chg;
    if (uartkbd_charger(&chg) && chg.vbatt_mv && chg.vbatt_mv < 3600u) {
        // Digits written out rather than via fmt_int(), which is defined
        // further down the file. A single cell is always one digit of volts.
        char b[24]; int m = 0;
        for (const char *p = "BATT "; *p; p++) b[m++] = *p;
        b[m++] = (char)('0' + (chg.vbatt_mv / 1000u) % 10u);
        b[m++] = '.';
        b[m++] = (char)('0' + (chg.vbatt_mv % 1000u) / 100u);
        b[m++] = 'V';
        b[m] = '\0';
        uint16_t col = chg.vbatt_mv < 3300u ? rgb565_be(255, 60, 60)
                                            : rgb565_be(255, 170, 0);
        st7796_draw_text(168, STATUS_Y + 2, 2, col, bg_screen, b);
        return;
    }
    st7796_draw_text(168, STATUS_Y + 2, 2, rgb565_be(255, 255, 0), bg_screen, line);
}

// This unit's first WS2812 renders visibly hotter than its neighbors at the
// same RGB value (webcam-verified: identical frame data, LED0 blooms white
// while LED1 stays green). Compensate in the frame, not the driver.
static void vu_set_px(int i, rgb_t c) {
    if (i == 0) {
        c.r = (uint8_t)(c.r * 2 / 5);
        c.g = (uint8_t)(c.g * 2 / 5);
        c.b = (uint8_t)(c.b * 2 / 5);
    }
    ws2812_set_pixel((uint)i, c);
}

// The player sees SEVEN LEDs above the screen (user-verified on hardware);
// the WS2812 chain map claims 16 addresses. The meter renders only on the
// visible seven and forces the rest dark.
#define VU_LEDS 7

static void vu_led_meter(uint16_t avg, bool glitching) {
    // OUTPUT level on the visible LED row. Runs on the block MEAN (peak
    // pegged the bar on every pick transient), dB-scaled -45..-6 so real
    // playing moves through the range. The WHOLE bar takes one color by
    // level: green = moderate, yellow = hot, red = near clipping. Glitch
    // loops flash magenta.
    static int last = -2;                       // lit count, or -1 = glitch
    static float env = 0.0f;                    // lit-count envelope
    int lit;
    if (glitching) lit = -1;
    else {
        float target = 0.0f;
        if (avg >= 4) {
            float db = 20.0f * log10f((float)avg * (1.0f / 32768.0f));
            target = (db + 45.0f) * ((float)VU_LEDS / 39.0f);
            if (target < 0.0f) target = 0.0f;
            if (target > (float)VU_LEDS) target = (float)VU_LEDS;
        }
        // Fast attack, slow release: hits snap the bar up, it decays
        // visibly (~250 ms full-bar drain), grooves pump.
        if (target >= env) env = target;
        else { env -= 0.2f; if (env < target) env = target; }
        lit = (int)(env + 0.5f);
    }
    if (lit == last) return;
    last = lit;
    for (int i = 0; i < FW2_LED_COUNT; i++) {
        rgb_t c = { 0, 0, 0 };
        if (i < VU_LEDS && (glitching || i < lit)) {
            // positional zones on the visible row (user spec): leftmost
            // four green, next two yellow, the peak LED red.
            if (glitching)   c = (rgb_t){ 40, 0, 32 };
            else if (i < 4)  c = (rgb_t){ 0, 32, 0 };
            else if (i < 6)  c = (rgb_t){ 30, 24, 0 };
            else             c = (rgb_t){ 36, 0, 0 };
        }
        vu_set_px(i, c);
    }
    ws2812_show();
}

// ---- Codec gain staging (raw register writes; addr/encoding per the proven
// BSP driver: 7-bit reg + 9-bit value, addr byte = reg<<1 | val bit 8) ----
#define NAU_ADDR 26
static void nau_write(uint8_t reg, uint16_t val) {
    uint8_t buf[2] = { (uint8_t)((reg << 1) | ((val >> 8) & 1)), (uint8_t)val };
    if (i2c_write_blocking(i2c1, NAU_ADDR, buf, 2, false) != 2)
        DIAG("nau: write reg 0x%x FAILED\n", reg);
}

// Input profiles: PGA gain (reg 0x2D, 0.75 dB/step, 0x10 = 0 dB) and the
// +20 dB PGA boost (reg 0x2F bit 8). The BSP init default is MIC (boost on,
// PGA 0 dB) — correct for a headset electret, FAR too hot for line level.
typedef struct { const char *label; uint16_t pga; uint16_t boost; } in_profile_t;
static const in_profile_t IN_PROFILES[] = {
    { "MIC",      0x010, 0x100 },   // boost +20 dB, PGA  0 dB (BSP default)
    { "MIC HOT",  0x020, 0x100 },   // boost +20 dB, PGA +12 dB (quiet dynamic mic)
    { "GUITAR",   0x006, 0x100 },   // boost +20 dB, PGA -7.5 dB (+12.5 dB
                                        // total). Set from the input-level
                                        // report (2026-07-30): at +4.5 dB the
                                        // hardest strums pinned the ADC at
                                        // full scale and a normal open E
                                        // already sat at 0 dBFS, so every
                                        // nonlinear stage ran deep inside its
                                        // clamp — overdrive's output moved
                                        // 0.6 dB across 11.6 dB of input, i.e.
                                        // no touch response at all. -12 dB
                                        // puts the hardest strum near -6 dBFS
                                        // and normal playing near -12, just
                                        // under overdrive's -11.4 dBFS clamp,
                                        // so digging in is what breaks it up.
                                        // The hiss that motivated the +4.5 dB
                                        // was the headphone load on the jack
                                        // (~0.57 uF into 32 ohm = -19 dB
                                        // midband), not the front end: with
                                        // the rig cable pulled the input floor
                                        // measures -52..-60 dBFS and the gate
                                        // closes cleanly.
    { "BASS",     0x01C, 0x000 },   // no boost, PGA +9 dB — passive bass puts
                                        // out more low-frequency energy than guitar;
                                        // the +20 dB boost clips its lows into mud
    { "LINE LEVEL", 0x010, 0x000 },   // no boost,     PGA  0 dB (line level)
    { "LINE PAD",   0x000, 0x000 },   // no boost,     PGA -12 dB (hot line level)
};
#define IN_PROFILE_COUNT 6
static void apply_in_profile(int idx) {
    nau_write(0x2D, IN_PROFILES[idx].pga);
    nau_write(0x2F, IN_PROFILES[idx].boost);
    DIAG("input: %s (pga=0x%x boost=0x%x)\n", IN_PROFILES[idx].label,
         IN_PROFILES[idx].pga, IN_PROFILES[idx].boost);
}

// Output volume via DAC digital volume (reg 0x0B, 0.5 dB/step, 0xFF = 0 dB).
// Applies to speaker AND jack. Default -12 dB protects the onboard speaker.
typedef struct { const char *label; uint16_t dacvol; } vol_step_t;
// 0xFF = 0 dB, 0.5 dB per step down. -9 dB APPENDED rather than inserted in
// loudness order, so index 0 stays -12 dB: vol_clamp_quiet() forces vol_idx = 0
// as the safe speaker default and several call sites lean on that.
static const vol_step_t VOL_STEPS[] = {
    { "PAD 12DB", 0x0E7 },
    { "PAD 6DB",  0x0F3 },
    { "PAD OFF",  0x0FF },
    { "PAD 24DB", 0x0CF },
    { "PAD 9DB",  0x0ED },
};
#define VOL_STEP_COUNT 5

// Quietest -> loudest. SINGLE SOURCE OF TRUTH for the loudness ramp: the D-pad
// walks it, vol_cycle() walks it, and vol_rank() indexes into it for the
// overlay meter. It used to exist only as a local inside the D-pad handler while
// vol_cycle() hardcoded its own transitions, and the two drifted -- the D-pad
// let the speaker reach -6 dB while vol_cycle() and vol_clamp_quiet() both held
// it at -12 dB. One table, so they cannot disagree again.
static const int8_t VOL_ORDER[VOL_STEP_COUNT] = { 3, 0, 4, 1, 2 };
//                                    -24  -12  -9  -6   0 dB
#define VOL_ORDER_SPEAKER_MAX 2       // index of -9 dB: the speaker ceiling

// Position on the loudness ramp, 0 = quietest. Derived from VOL_ORDER rather
// than a parallel switch, which had to be edited in lockstep and silently fell
// through to the loudest rank for any value it did not name.
static int vol_rank(void) {
    for (int i = 0; i < VOL_STEP_COUNT; i++)
        if (VOL_ORDER[i] == vol_idx) return i;
    return 0;
}
static void apply_vol(int idx) {
    nau_write(0x0B, VOL_STEPS[idx].dacvol);
    DIAG("vol: %s (dacvol=0x%x)\n", VOL_STEPS[idx].label, VOL_STEPS[idx].dacvol);
}

// ---- Audio buffers (DMA read-ring: whole periods, power-of-two bytes, aligned) ----
// Tone + silence + mic TX rings (DMA read-ring: power-of-two, size-aligned).
// 1024 frames = 21 whole cycles of ~1001 Hz at 48,828 Hz — the DMA ring
// wraps seamlessly (frames must be a power of two; buffer aligned to size).
#define TONE_FRAMES 1024u
static uint32_t tone_buf[TONE_FRAMES] __attribute__((aligned(4096)));
// Silence ring: keeps TX fed while "stopped". CRITICAL: with an empty TX FIFO
// the SM stalls on autopull (fdebug TXSTALL, seen on hardware) — clocks stop
// and RX capture dies with it. TX must ALWAYS have a ring armed.
static uint32_t silence_buf[64] __attribute__((aligned(256)));
// Mic passthrough ring: 4 blocks = 1024 frames = 4096 bytes. The writer stays
// ~2 slots (~10.5 ms at 48.8 kHz) away from the DMA reader in both
// directions, so main-loop jitter can never tear a block the reader is
// consuming. Total round trip: ~5.2 ms capture + ~10.5 ms TX ≈ 16 ms.
#define MIC_SLOTS 4
static uint32_t mic_ring[MIC_SLOTS * AUDIO_CAPTURE_BLOCK_FRAMES] __attribute__((aligned(4096)));

// Swap the TX DMA to a different ring (always stop first: reconfiguring a
// running channel corrupts the transfer).
static void tx_play(const uint32_t *buf, uint frames) {
    audio_i2s_duplex_play_stop();
    audio_i2s_duplex_play_loop(buf, frames);
}

// Cross-core-safe TX retarget. While `playing && mic`, CORE 1 owns the TX
// DMA (it arms the mic ring itself); a UI-side tx_play racing that arm
// corrupts the transfer. Park the pump (clear `playing`, wait out one
// block), swap the ring, then set the new flags — pump re-arms/re-phases
// on resume. `new_mic`/`new_playing` are applied last, in that order.
static void tx_retarget(const uint32_t *buf, uint frames,
                        bool new_mic, bool new_playing) {
    playing = false;
    codec_nau88c10_dac_mute(true);   // soft ramp down: a hard ring swap is a
    sleep_ms(9);                     //   step discontinuity = audible click
    tx_play(buf, frames);
    mic_armed = false;
    mic = new_mic;
    playing = new_playing;
    sleep_ms(2);                     // new ring is clocking before unmute
    codec_nau88c10_dac_mute(false);  // soft ramp back up
}

// Copy one capture block into a mic_ring slot, replicating the mono ADC
// (right slot = low 16 bits, AUDIO_MIC_I2S_SLOT) onto both output slots.
static void mic_copy_slot(const uint32_t *blk, int slot) {
    uint32_t *dst = &mic_ring[slot * AUDIO_CAPTURE_BLOCK_FRAMES];
    for (int i = 0; i < AUDIO_CAPTURE_BLOCK_FRAMES; i++) {
        uint16_t s = (uint16_t)(blk[i] & 0xFFFF);
        dst[i] = ((uint32_t)s << 16) | s;
    }
}

// Copy one processed mono block into a mic_ring slot (both stereo slots).
static void mic_write_slot(const int16_t *mono, int slot) {
    uint32_t *dst = &mic_ring[slot * AUDIO_CAPTURE_BLOCK_FRAMES];
    for (int i = 0; i < AUDIO_CAPTURE_BLOCK_FRAMES; i++) {
        uint16_t s = (uint16_t)mono[i];
        dst[i] = ((uint32_t)s << 16) | s;
    }
}

// ---- PDM 4-mic array input (separate hardware from the codec jack ADC) ----
// The PDM stream is nominally 16000 Hz while the codec/TX side is MCLK-locked
// at ~16009 Hz (+0.06%), so the streams MUST be decoupled: a FIFO buffers
// PDM PCM and the codec capture stays the pacer. The consumer drains ~9
// samples/s faster than PDM produces; underruns repeat the last sample (one
// ~every 110 ms once the cushion is gone — inaudible on mic audio). A page-
// draw stall backlogs the FIFO instead of slipping the ring; a resync cap
// keeps latency bounded. The 4 mics are averaged to mono (up to +6 dB SNR
// for a centered source), DC-blocked (the raw stream is heavily DC-biased,
// see wilibsp docs/drivers/pdm.md), and digitally gained.
#define PDM_FIFO_N   4096u          // 256 ms of 16 kHz PCM (power of two)
#define PDM_FIFO_MASK (PDM_FIFO_N - 1u)
#define PDM_CUSHION  384u           // prime ~24 ms before playing
#define PDM_GAIN     8.0f           // digital gain after the 4-mic average
static int16_t  pdm_fifo[PDM_FIFO_N];
static volatile uint32_t pdm_w, pdm_r;  // FIFO write/read (audio core writes,
                                        // UI reads depth; reset while parked)
static bool     pdm_priming = true; // building the start-up cushion
static float    pdm_dc_y, pdm_dc_x; // persistent one-pole DC blocker ~50 Hz
static int16_t  pdm_hold;           // last popped sample (underrun pad)
static volatile unsigned pdm_vu;    // input peak of the last block (for LEDs)

// Drain stale capture + start fresh (call when the PDM source goes live).
static void pdm_input_reset(void) {
    static int16_t t[PDM_NUM_MICS][64];
    int16_t *dst[PDM_NUM_MICS] = { t[0], t[1], t[2], t[3] };
    while (pdm_capture_pull(dst, 64) > 0) ;
    pdm_w = pdm_r = 0;
    pdm_priming = true;
    pdm_dc_y = pdm_dc_x = 0.0f;
    pdm_hold = 0;
}

// Pull every fresh PDM sample, average the 4 mics, DC-block, gain, push.
// Called each main-loop pass (~2 ms) while the PDM source is live.
static void pdm_pump(void) {
    static int16_t t[PDM_NUM_MICS][64];
    int16_t *dst[PDM_NUM_MICS] = { t[0], t[1], t[2], t[3] };
    unsigned n;
    while ((n = pdm_capture_pull(dst, 64)) > 0) {
        for (unsigned i = 0; i < n; i++) {
            float x = 0.25f * (float)(t[0][i] + t[1][i] + t[2][i] + t[3][i]);
            float y = x - pdm_dc_x + 0.98f * pdm_dc_y;    // HP ~50 Hz
            pdm_dc_x = x; pdm_dc_y = y;
            float g = y * PDM_GAIN;
            if (g >  32700.0f) g =  32700.0f;
            if (g < -32700.0f) g = -32700.0f;
            if (pdm_w - pdm_r < PDM_FIFO_N)
                pdm_fifo[pdm_w++ & PDM_FIFO_MASK] = (int16_t)g;
        }
        if (n < 64) break;                                // drained
    }
}

// Pop one block for the TX pump; silence while priming, pad on underrun.
static void pdm_pop_block(int16_t *out, unsigned nreq) {
    uint32_t depth = pdm_w - pdm_r;
    if (pdm_priming) {
        if (depth >= PDM_CUSHION) pdm_priming = false;
        else { memset(out, 0, nreq * sizeof out[0]); pdm_vu = 0; return; }
    }
    if (depth > 4u * AUDIO_CAPTURE_BLOCK_FRAMES)          // post-stall backlog:
        pdm_r = pdm_w - PDM_CUSHION;                      // resync, cap latency
    // The PDM decimator still produces 16 kHz PCM (its PIO clocking is
    // independent of the codec rate); linear-interpolate up to the 48.8 kHz
    // chain rate (phase step 16000/48828 = 0.3277 input samples per output).
    static int16_t pdm_prev = 0;
    static float   pdm_phase = 1.0f;
    unsigned pk = 0;
    for (unsigned i = 0; i < nreq; i++) {
        pdm_phase += 16000.0f / (float)FX_FS;
        while (pdm_phase >= 1.0f && pdm_r != pdm_w) {
            pdm_prev = pdm_hold;
            pdm_hold = pdm_fifo[pdm_r++ & PDM_FIFO_MASK];
            pdm_phase -= 1.0f;
        }
        float fr = pdm_phase < 1.0f ? pdm_phase : 1.0f;
        int16_t s = (int16_t)((1.0f - fr) * (float)pdm_prev +
                              fr * (float)pdm_hold);
        out[i] = s;
        unsigned a = (unsigned)(s < 0 ? -s : s);
        if (a > pk) pk = a;
    }
    pdm_vu = pk;
}

// ---- Shared output-volume policy ----
// Loud DAC volume (-6 / 0 dB) is only safe out the jack. The onboard
// micro-speaker is rated 300 mW continuous / 500 mW ABSOLUTE MAX (8 ohm):
// a full-scale sine at 0 dB delivers ~0.7-1.1 W — 2-3x the max rating
// (this is how the first speaker died; it was errantly advertised at 2 W).
// The open PDM mics also feed back almost instantly at loud levels, so loud is
// allowed only for jack + a non-PDM input.
//
// "Allowed" is not "advisable". The jack is uncapped because it feeds amps and
// audio interfaces, which want line level -- but the same setting goes into
// headphones. Bench check 2026-08-24: in-ear monitors work fine and get loud,
// with -12 dB comfortable for listening. So nothing here should ever DEFAULT a
// jack route to a loud pad; make the user climb.
//
// THE SPEAKER CEILING IS -9 dB, raised from -12 dB. Worst case is a full-scale
// CONTINUOUS sine, which this app really does produce -- the 1 kHz tone, the
// theremin, and the synth with hold ON all sustain indefinitely -- so the
// derating is against the 300 mW continuous number, not the 500 mW peak:
//     0 dB    0.7-1.1 W     230-370% of rated   killed a speaker
//    -6 dB    175-275 mW     58- 92% of rated   inside spec, no margin left
//    -9 dB     88-139 mW     29- 46% of rated   <-- ceiling
//   -12 dB     44- 69 mW     15- 23% of rated   previous ceiling
// -9 dB is twice the acoustic power of -12 dB (a plainly audible step, roughly
// 1.4x perceived loudness) and still under half the continuous rating even at
// the pessimistic end of an ESTIMATED full-scale figure. -6 dB was rejected not
// because the datasheet forbids it but because 58-92% leaves nothing for that
// estimate being wrong, and it has already been wrong once on this bench.
// The 900 Hz speaker-protect HP keeps the excursion-heavy content out as well.
// A 900 Hz speaker-protect HP in the audio pump guards the cone as well
// (the driver's passband floor is ~1.7 kHz; bass = raw excursion).
static bool vol_loud_ok(void) { return jack && !use_pdm; }

// Pull the volume back to the speaker ceiling when the loud path goes away
// (jack unplugged, or the PDM mics selected). Tests the RANK against the ceiling
// rather than naming the two loud indices, which meant this function had to be
// edited every time the table gained a step -- and appending -9 dB would
// otherwise have left it silently correct for the wrong reason.
static void vol_clamp_quiet(void) {
    if (!vol_loud_ok() && vol_rank() > VOL_ORDER_SPEAKER_MAX) {
        vol_idx = VOL_ORDER[VOL_ORDER_SPEAKER_MAX];           // -> -9 dB
        apply_vol(vol_idx);
        DIAG("vol: clamped to -9 dB (%s)\n",
             use_pdm ? "pdm mics" : "speaker");
    }
}

// Advance one step up the loudness ramp, wrapping at the permitted ceiling.
// Walks VOL_ORDER so the on-screen sequence is monotonically louder; it used to
// step through the raw table order (-12, -6, 0, -24), which jumped from the
// loudest setting straight to the quietest.
static void vol_cycle(void) {
    int top = vol_loud_ok() ? VOL_STEP_COUNT - 1 : VOL_ORDER_SPEAKER_MAX;
    int oi = 0;
    for (int i = 0; i < VOL_STEP_COUNT; i++)
        if (VOL_ORDER[i] == vol_idx) oi = i;
    oi = (oi >= top) ? 0 : oi + 1;
    vol_idx = VOL_ORDER[oi];
    apply_vol(vol_idx);
}

// ---- Page rendering ----
#define C_BG     rgb565_be(0, 0, 40)
#define C_GREEN  rgb565_be(0, 120, 0)
#define C_RED    rgb565_be(150, 0, 0)
#define C_BLUE   rgb565_be(0, 90, 160)
#define C_GRAY   rgb565_be(70, 70, 90)
#define C_PURPLE rgb565_be(110, 40, 140)
#define C_PAD    rgb565_be(12, 12, 66)
#define C_LINE   rgb565_be(60, 60, 130)
#define C_CURSOR rgb565_be(255, 200, 0)
#define C_SYNBG  rgb565_be(16, 16, 40)         // synth pad background
#define C_WF_DIM    rgb565_be(0, 90, 110)      // sample waveform, unplayed
#define C_WF_LIT    rgb565_be(0, 230, 200)     // sample waveform, behind needle
#define C_WF_REC    rgb565_be(230, 60, 60)     // sample waveform, recording
#define C_WF_NEEDLE rgb565_be(255, 255, 255)   // playback needle

// Launcher tile: beveled color block, pictogram icon, centered label.
static void draw_tile(const rect_t *r, uint8_t cr, uint8_t cg, uint8_t cb,
                      const char *label, int icon) {
    uint16_t base = rgb565_be(cr, cg, cb);
    uint16_t lite = rgb565_be((uint8_t)(cr + (255 - cr) / 3),
                              (uint8_t)(cg + (255 - cg) / 3),
                              (uint8_t)(cb + (255 - cb) / 3));
    uint16_t dark = rgb565_be((uint8_t)(cr / 2), (uint8_t)(cg / 2),
                              (uint8_t)(cb / 2));
    uint16_t ink  = rgb565_be(235, 235, 245);
    st7796_fill_rect(r->x, r->y, r->w, r->h, base);
    st7796_fill_rect(r->x, r->y, r->w, 3, lite);              // bevel top
    st7796_fill_rect(r->x, r->y + r->h - 4, r->w, 4, dark);   // bevel bottom
    int cx = r->x + r->w / 2, iy = r->y + 16;
    switch (icon) {
    case 0:                                            // FX: 3 sliders
        for (int i = 0; i < 3; i++) {
            st7796_fill_rect(cx - 24, iy + 4 + i * 11, 48, 3, dark);
            int kx = cx - 24 + (i == 0 ? 8 : i == 1 ? 32 : 18);
            st7796_fill_rect(kx, iy + i * 11, 8, 11, ink);
        }
        break;
    case 1: {                                          // SAMPLER: waveform
        static const int8_t hb[7] = { 10, 20, 14, 28, 16, 24, 9 };
        for (int i = 0; i < 7; i++)
            st7796_fill_rect(cx - 27 + i * 8, iy + 32 - hb[i], 6, hb[i], ink);
        break;
    }
    case 2:                                            // SYNTH: square wave
        st7796_fill_rect(cx - 26, iy + 4, 14, 3, ink);
        st7796_fill_rect(cx - 12, iy + 4, 3, 22, ink);
        st7796_fill_rect(cx - 12, iy + 23, 14, 3, ink);
        st7796_fill_rect(cx + 2,  iy + 4, 3, 22, ink);
        st7796_fill_rect(cx + 2,  iy + 4, 14, 3, ink);
        st7796_fill_rect(cx + 16, iy + 4, 3, 22, ink);
        break;
    case 3:                                            // DRUMS: step grid
        for (int gy = 0; gy < 2; gy++)
            for (int gx = 0; gx < 4; gx++)
                st7796_fill_rect(cx - 28 + gx * 15, iy + 2 + gy * 15,
                                 11, 11, (gx + gy) % 2 ? dark : ink);
        break;
    case 4:                                            // THEREMIN: antenna
        st7796_fill_rect(cx + 6, iy, 4, 30, ink);
        st7796_fill_rect(cx - 22, iy + 4, 16, 3, ink);
        st7796_fill_rect(cx - 26, iy + 13, 20, 3, ink);
        st7796_fill_rect(cx - 30, iy + 22, 24, 3, ink);
        break;
    case 5:                                            // KEYS: mini piano
        st7796_fill_rect(cx - 24, iy + 2, 48, 28, ink);
        st7796_fill_rect(cx - 9, iy + 2, 3, 28, base);
        st7796_fill_rect(cx + 7, iy + 2, 3, 28, base);
        st7796_fill_rect(cx - 13, iy + 2, 9, 16, dark);
        st7796_fill_rect(cx + 3,  iy + 2, 9, 16, dark);
        break;
    case 6: {                                          // VIZ: rising bars
        static const int8_t vb[4] = { 10, 17, 24, 31 };
        for (int i = 0; i < 4; i++)
            st7796_fill_rect(cx - 24 + i * 13, iy + 32 - vb[i], 9, vb[i], ink);
        break;
    }
    default:                                           // SETUP: gear-ish
        st7796_fill_rect(cx - 9, iy + 6, 18, 18, ink);
        st7796_fill_rect(cx - 3, iy, 6, 30, ink);
        st7796_fill_rect(cx - 15, iy + 12, 30, 6, ink);
        st7796_fill_rect(cx - 4, iy + 11, 8, 8, base);
        break;
    }
    int lw = 0;
    for (const char *p = label; *p; p++) lw += 12;
    st7796_draw_text(cx - lw / 2, r->y + r->h - 26, 2, ink, base, label);
}

// Battery pill, launcher top-right. Everyone reads a phone battery icon
// without being taught, which is exactly why it was asked for. Percentage is
// an APPROXIMATION from resting cell voltage (3.50 V empty .. 4.15 V full,
// linear): a real fuel gauge needs coulomb counting, and this only has to
// answer "should I plug in before the demo?". A lightning bolt replaces the
// number while the charger is actually pushing current.
#define BAT_X 396
#define BAT_Y 8
#define BAT_W 60
#define BAT_H 22
static void draw_battery_pill(void) {
    uartkbd_charger_t chg;
    if (!uartkbd_charger(&chg) || !chg.vbatt_mv) return;   // no frame yet
    int pct = ((int)chg.vbatt_mv - 3500) * 100 / (4150 - 3500);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    bool charging = (chg.charge_status == UARTKBD_CHG_PRECHARGE ||
                     chg.charge_status == UARTKBD_CHG_FASTCHARGE);
    uint16_t ink  = rgb565_be(210, 210, 225);
    uint16_t fill = pct <= 15 ? rgb565_be(235, 60, 60)
                  : pct <= 35 ? rgb565_be(235, 170, 40)
                              : rgb565_be(70, 205, 110);
    // shell + terminal nub, then hollow out and fill to level
    st7796_fill_rect(BAT_X, BAT_Y, BAT_W, BAT_H, ink);
    st7796_fill_rect(BAT_X + BAT_W, BAT_Y + 6, 4, BAT_H - 12, ink);
    st7796_fill_rect(BAT_X + 2, BAT_Y + 2, BAT_W - 4, BAT_H - 4, C_BG);
    int fw = (BAT_W - 8) * pct / 100;
    if (fw > 0) st7796_fill_rect(BAT_X + 4, BAT_Y + 4, fw, BAT_H - 8, fill);
    if (charging) {
        // Two wedges reading as a bolt at this size; no glyph font needed.
        st7796_fill_rect(BAT_X + 26, BAT_Y + 4, 4, 8, rgb565_be(255, 255, 255));
        st7796_fill_rect(BAT_X + 30, BAT_Y + 10, 4, 8, rgb565_be(255, 255, 255));
    } else {
        char b[6]; int n = 0;
        if (pct == 100) { b[n++] = 'F'; b[n++] = 'U'; b[n++] = 'L'; b[n++] = 'L'; }
        else {
            if (pct >= 10) b[n++] = (char)('0' + pct / 10);
            b[n++] = (char)('0' + pct % 10);
            b[n++] = '%';
        }
        b[n] = '\0';
        int tx = BAT_X + (BAT_W - n * 6) / 2;
        st7796_draw_text(tx, BAT_Y + 7, 1, rgb565_be(10, 10, 30), fill, b);
    }
}

static void draw_main_page(void) {
    st7796_fill_screen(C_BG);
    // "Orcastra" is 8 chars at scale 2 (12 px/char) = 96 px, so x = 240 - 48
    // keeps it centred on the 480 px panel. The tagline below stays: it is the
    // theme's name too, and it is what the branding art says.
    st7796_draw_text(192, 8, 2, rgb565_be(255, 255, 255), C_BG, "Orcastra");
    draw_battery_pill();
    st7796_draw_text(183, 30, 1,
                     codec_ok ? rgb565_be(0, 220, 180) : rgb565_be(255, 0, 0),
                     C_BG, codec_ok ? "SET YOUR SOUND FREE" : "CODEC FAIL");
    draw_tile(&BTN_GFX,   110,  40, 140, "FX",       0);
    draw_tile(&BTN_VOX,     0, 115, 115, "SAMPLER",  1);
    draw_tile(&BTN_SYN,    30,  60, 165, "SYNTH",    2);
    draw_tile(&BTN_DRM,   165,  60,  30, "DRUMS",    3);
    draw_tile(&BTN_THM,    20, 115,  50, "THEREMIN", 4);
    draw_tile(&BTN_KEY,   135,  95,  20, "KEYS",     5);
    draw_tile(&BTN_VIZ,   150,  30, 110, "VIZ",      6);
    draw_tile(&BTN_SETUP,  70,  70,  90, "SETUP",    7);
    draw_button(&BTN_PLAY, playing ? C_RED : C_GREEN, 2,
                playing ? "STOP" : "PLAY");
    draw_button(&BTN_TUNERQ, rgb565_be(0, 110, 110), 2, "TUNER");
    draw_status(C_BG, playing, mic, jack);
}

// SETUP page: everything a demo audience never touches — input source,
// gain profile, output route, volume. Reached from the launcher tile.
static void draw_setup_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    st7796_draw_text(205, 18, 2, rgb565_be(255, 255, 255), C_BG, "SETUP");
    // Labels sit immediately LEFT of the buttons they name, right-aligned to end
    // at x=242 (8 px before BTN_SRC/BTN_OUT at x=250). They used to start at
    // x=20, which put 230 px of empty space between a label and its control and
    // read as two unrelated columns rather than a label-value pair.
    // Right-aligned by hand: scale 2 is 12 px/char, so "INPUT" (5) starts at
    // 242-60=182 and "OUTPUT" (6) at 242-72=170.
    st7796_draw_text(182, 104, 2, rgb565_be(150, 150, 160), C_BG, "INPUT");
    st7796_draw_text(170, 154, 2, rgb565_be(150, 150, 160), C_BG, "OUTPUT");
    if (mic && !use_pdm)     // input profile applies to the 3.5mm jack only
        draw_button(&BTN_IN, in_idx ? C_PURPLE : C_GRAY, 2,
                    IN_PROFILES[in_idx].label);
    else
        st7796_fill_rect(BTN_IN.x, BTN_IN.y, BTN_IN.w, BTN_IN.h, C_BG);
    draw_button(&BTN_SRC,  mic ? C_PURPLE : C_GRAY, 2,
                mic ? (use_pdm ? "IN: PDM MICS" : "IN: 3.5MM") : "IN: 1KHZ TONE");
    draw_button(&BTN_OUT,  jack ? C_BLUE : C_GRAY, 2,
                jack ? "OUT: 3.5MM" : "OUT: SPEAKER");
    draw_button(&BTN_VOL,  vol_idx == 2 ? C_RED : C_GRAY, 2,
                VOL_STEPS[vol_idx].label);
    {   // audio rail state, and the button that requests it
        uint32_t rails = 0;
        bool live = picpwr_rails(&rails);
        bool on = live && (rails & picpwr_zone_bit(PICPWR_ZONE_AUDIO));
        draw_button(&BTN_APWR, on ? C_GREEN : C_PURPLE, 2,
                    on ? "AUDIO PWR OK" : "AUDIO PWR REQ");
    }
    draw_status(C_BG, playing, mic, jack);
}

// Guitar FX menu: every implemented effect, one button each.
// TAP toggles the effect on/off (any combination may be active); HOLD opens
// the effect's settings page.
// MENU_N is the INDEX SPACE (every effect ever numbered, including the four
// disabled in fx.h). MENU_CELLS is how many tiles the grid actually draws.
// Keeping them separate is what lets effects be disabled without renumbering
// the index-keyed switches below -- and s_menu_rect stays MENU_N long so an
// omitted index keeps a zeroed rect, which hit() can never match (w == h == 0).
#define MENU_N 16
#define MENU_COLS 4
#define MENU_ROWS 3
#define MENU_CELLS (MENU_COLS * MENU_ROWS)
// Which menu INDEX sits in each grid cell, row-major, in SIGNAL-CHAIN order:
// front of chain, gain then EQ, then modulation, time and space.
// TWELVE tiles since 2026-08-01 (was sixteen). Twelve is exactly 4x3 so there
// is no empty cell to special-case, and the tiles grow 60 px -> 82 px.
// Missing indices 7 (TREMOLO), 8 (PHASER), 14 (RINGMOD), 15 (VOWEL) are
// disabled in fx.h, not deleted -- see the notes there to bring one back.
static const unsigned char MENU_GRID[MENU_CELLS] = {
     9,  1,  5,  0,     /* ORCATUNE  OCTAVE    AUTO WAH  PITCH XY */
     6,  4, 12, 11,     /* OD        SHRED     BITCRUSH  EQ       */
    13,  2,  3, 10,     /* FLANGER   GLITCH    DELAY     REVERB   */
};
static rect_t s_menu_rect[MENU_N];
static void menu_rects_init(void) {
    for (int k = 0; k < MENU_CELLS; k++) {
        int col = k % MENU_COLS, row = k / MENU_COLS;
        rect_t r = { 10 + col * 118, 52 + row * 88, 110, 82 };
        s_menu_rect[MENU_GRID[k]] = r;
    }
}
#define MENU_RECTS_AT(i) (&s_menu_rect[i])
// Labels in INDEX order (see MENU_GRID for where each one is drawn). Kept to
// 8 characters: at 12 px per char a 9-char label is 108 px in a 110 px button.
static const char *const MENU_LABELS[MENU_N] = {
    "PITCH XY", "OCTAVE", "GLITCH", "DELAY", "SHRED", "AUTO WAH",
    "OD", "TREMOLO", "PHASER", "ORCATUNE", "REVERB", "EQ",
    "BITCRUSH", "FLANGER", "RINGMOD", "VOWEL",
};

static bool menu_active(int i) {
    switch (i) {
    case 0: return pad_on;
    case 1: return oct_on;
    case 2: return gli_on;
    case 3: return slap_on;
    case 4: return met_on;
    case 5: return para_on;
    case 6: return od_on;
#if FX_ENABLE_TREMOLO
    case 7: return trem_on;
#endif
#if FX_ENABLE_PHASER
    case 8: return ph_on;
#endif
    case 9: return tune_on;
    case 10: return verb_on;
    case 12: return bc_on;
    case 13: return fl_on;
#if FX_ENABLE_RINGMOD
    case 14: return rm_on;
#endif
#if FX_ENABLE_VOWEL
    case 15: return vw_on;
#endif
    default: return eq_on;
    }
}

static void draw_menu_btn(int i) {
    draw_button(MENU_RECTS_AT(i), menu_active(i) ? C_GREEN : C_PURPLE, 2,
                MENU_LABELS[i]);
}

// Iterate the GRID, not the index space: a disabled effect's rect is zeroed and
// drawing it would put a stray label at the origin.
static int menu_btn_at(uint16_t x, uint16_t y) {
    for (int k = 0; k < MENU_CELLS; k++)
        if (hit(MENU_RECTS_AT(MENU_GRID[k]), x, y)) return MENU_GRID[k];
    return -1;
}

static void draw_fxmenu_page(void) {
    menu_rects_init();
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    st7796_draw_text(180, 10, 2, rgb565_be(255, 255, 255), C_BG, "FX");
    st7796_draw_text(150, 32, 1, rgb565_be(180, 180, 180), C_BG,
                     "TAP: ON/OFF   HOLD: SETTINGS");
    for (int k = 0; k < MENU_CELLS; k++) draw_menu_btn(MENU_GRID[k]);
}

// Tap on a menu button: flip that effect's toggle and repaint the button.
static void menu_toggle(int i) {
    switch (i) {
    case 0:
        pad_on = !pad_on;
        fx_set_params(pad_on ? pad_oct : 0.0f, pad_on ? pad_amt : 0.0f);
        DIAG("pad: %s\n", pad_on ? "ON (remembered x/y)" : "OFF");
        break;
    case 1:
        oct_on = !oct_on;
        fx_octave_set(oct_on);
        DIAG("octave: %s\n", oct_on ? "ON" : "OFF");
        break;
    case 2:
        gli_on = !gli_on;
        fx_glitch_set(gli_on);
        DIAG("glitch: %s\n", gli_on ? "ARMED" : "OFF");
        break;
    case 3:
        slap_on = !slap_on;
        fx_delay_set(slap_on);
        DIAG("delay: %s (%d ms fb %d mix %d)\n", slap_on ? "ON" : "OFF",
             dly_ms, dly_fb_amt, dly_mix_amt);
        break;
    case 4:
        met_on = !met_on;
        fx_metal_set(met_on);
        DIAG("metal: %s (drive %d)\n", met_on ? "ON" : "OFF", met_amt);
        break;
    case 5:
        para_on = !para_on;
        fx_para_set(para_on);
        DIAG("paranoid: %s (range %d)\n", para_on ? "ON" : "OFF", para_rng);
        break;
    case 6:
        od_on = !od_on;
        fx_od_set(od_on);
        DIAG("overdrive: %s (drive %d)\n", od_on ? "ON" : "OFF", od_amt);
        break;
#if FX_ENABLE_TREMOLO
    case 7:
        trem_on = !trem_on;
        fx_trem_set(trem_on);
        DIAG("tremolo: %s (rate %d)\n", trem_on ? "ON" : "OFF", trem_amt);
        break;
#endif
    case 12:
        bc_on = !bc_on;
        fx_crush_set(bc_on);
        DIAG("bitcrush: %s (amount %d)\n", bc_on ? "ON" : "OFF", bc_amt);
        break;
    case 13:
        fl_on = !fl_on;
        fx_flange_set(fl_on);
        DIAG("flanger: %s (rate %d)\n", fl_on ? "ON" : "OFF", fl_amt);
        break;
#if FX_ENABLE_RINGMOD
    case 14:
        rm_on = !rm_on;
        fx_ring_set(rm_on);
        DIAG("ringmod: %s (freq %d)\n", rm_on ? "ON" : "OFF", rm_amt);
        break;
#endif
#if FX_ENABLE_VOWEL
    case 15:
        vw_on = !vw_on;
        fx_vowel_set(vw_on);
        DIAG("vowel: %s\n", vw_on ? "ON" : "OFF");
        break;
#endif
#if FX_ENABLE_PHASER
    case 8:
        ph_on = !ph_on;
        fx_phase_set(ph_on);
        DIAG("phaser: %s (rate %d)\n", ph_on ? "ON" : "OFF", ph_amt);
        break;
#endif
    case 9:
        tune_on = !tune_on;
        fx_tune_set(tune_on);
        DIAG("orcatune: %s (key %d scale %d %s)\n", tune_on ? "ON" : "OFF",
             tune_key, tune_scale, tune_robot ? "robotic" : "natural");
        break;
    case 10:
        verb_on = !verb_on;
        fx_verb_set(verb_on);
        DIAG("reverb: %s (mix %d)\n", verb_on ? "ON" : "OFF", verb_mix_amt);
        break;
    default:
        eq_on = !eq_on;
        fx_eq_set(eq_on);
        DIAG("eq: %s\n", eq_on ? "ON" : "OFF");
        break;
    }
    draw_menu_btn(i);
}

static int fx_cur_x = -1, fx_cur_y = -1;   // cursor pixel pos (-1 = hidden)

static void fx_center_line(void) {
    st7796_fill_rect(PAD_X + PAD_W / 2 - 1, PAD_Y, 2, PAD_H, C_LINE);
}

// Move (or hide) the touch cursor square; keeps the center detent line alive.
static void fx_cursor(int nx, int ny) {
    if (fx_cur_x >= 0)
        st7796_fill_rect(fx_cur_x - 7, fx_cur_y - 7, 14, 14, C_PAD);
    fx_center_line();
    if (nx >= 0) {
        if (nx < PAD_X + 7) nx = PAD_X + 7;
        if (nx > PAD_X + PAD_W - 8) nx = PAD_X + PAD_W - 8;
        if (ny < PAD_Y + 7) ny = PAD_Y + 7;
        if (ny > PAD_Y + PAD_H - 8) ny = PAD_Y + PAD_H - 8;
        st7796_fill_rect(nx - 7, ny - 7, 14, 14, C_CURSOR);
    }
    fx_cur_x = nx; fx_cur_y = ny;
}

static char *fmt_int(char *p, int v) {   // minimal signed itoa, returns end ptr
    if (v < 0) { *p++ = '-'; v = -v; }
    char tmp[8]; int n = 0;
    do { tmp[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) *p++ = tmp[--n];
    return p;
}

// Top-strip readout: pitch in semitones + FX percent (redraw only on change).
static int fxro_semi = 999, fxro_fxp = -1;   // cache; reset on page redraw
static void fx_readout(int semi, int fxp) {
    if (semi == fxro_semi && fxp == fxro_fxp) return;
    fxro_semi = semi; fxro_fxp = fxp;
    char line[36], *p = line;
    const char *s1 = "PITCH "; while (*s1) *p++ = *s1++;
    if (semi > 0) *p++ = '+';
    p = fmt_int(p, semi);
    const char *s2 = " SEMI  FX "; while (*s2) *p++ = *s2++;
    p = fmt_int(p, fxp); *p++ = '%'; *p = '\0';
    // BACK now spans x 32..136; start clear of it.
    st7796_fill_rect(144, 6, 328, 40, C_BG);
    st7796_draw_text(148, 18, 2, rgb565_be(255, 255, 0), C_BG, line);
}

static void draw_fx_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    st7796_fill_rect(PAD_X - 2, PAD_Y - 2, PAD_W + 4, PAD_H + 4, C_LINE); // border
    st7796_fill_rect(PAD_X, PAD_Y, PAD_W, PAD_H, C_PAD);
    fx_cur_x = fx_cur_y = -1;
    fx_center_line();
    // Axis hints (drawn once; the cursor trail may eventually erase them).
    // 8 chars at 6 px = 48 px, so the right label needs a 54 px inset (it was
    // 44 for the 6-char "+1 OCT"); at the full-width pad these sit at x=16 and
    // x=418, far clear of the centred DRAG hint below.
    st7796_draw_text(PAD_X + 6,  PAD_Y + PAD_H - 14, 1, C_LINE, C_PAD, "-1.5 OCT");
    st7796_draw_text(PAD_X + PAD_W - 54, PAD_Y + PAD_H - 14, 1, C_LINE, C_PAD,
                     "+1.5 OCT");
    st7796_draw_text(PAD_X + 6,  PAD_Y + 6, 1, C_LINE, C_PAD, "FULL WET");
    // CENTRED between the two octave labels, not offset from the pad's midpoint.
    // At the old PAD_X + PAD_W/2 + 8 this hint started at x=248 and ran 138 px to
    // 386, straight over "+1 OCT" at 346..382 -- which is why "+1 OCT" was
    // INVISIBLE in the screenshot: it is drawn first and this overwrote it. The
    // collision appeared when the pad narrowed to the interaction band; at the
    // old 460 px width there was room for both.
    // 23 chars at scale 1 is 138 px, so centring on the panel puts it at
    // 240-69 = 171..309, leaving ~39 px to "-1 OCT" (ends 132) and ~37 px to
    // "+1 OCT" (starts 346).
    st7796_draw_text(240 - 69, PAD_Y + PAD_H - 14, 1, C_LINE, C_PAD,
                     "DRAG TO SET - IT STICKS");
    // Restore the remembered position (the pad keeps its setting on release
    // and across page visits; toggle it from the GUITAR FX menu).
    fxro_semi = 999; fxro_fxp = -1;              // bust the readout cache
    if (pad_px >= 0) fx_cursor(pad_px, pad_py);
    fx_readout((int)(pad_oct * 12.0f + (pad_oct >= 0 ? 0.5f : -0.5f)),
               (int)(pad_amt * 100.0f + 0.5f));
}

// OCTAVE had a bespoke on/off-only page until it grew MODE/MIX/DETUNE; it now
// runs on the generic knob page (KNOB_OCT) like the other multi-knob effects.

// ---- Shared header for the effect parameter pages -------------------------
// These pages each carried a 300x84 "EFFECT: ON/OFF" button parked across the
// middle of the screen. The generic KNOB_PAGES layout dropped its equivalent
// long ago, for reasons that apply verbatim here (see the note above KROW_Y0):
// the button is REDUNDANT -- tapping the tile on the FX grid toggles the
// effect, and HOLDING that tile is how you arrived on this page -- and it is
// actively harmful, because 84 px of vertical centre pushed the real controls
// out toward the bezel. It cost three separate bench complaints about
// mis-hitting parameters before it was removed from the knob pages, and then
// survived on nine bespoke ones.
//
// The STATE still has to be visible while you edit parameters, so it moves into
// the title line: "GLITCH - ON" in bright green, "GLITCH - OFF" in grey. Costs
// no layout space and there is nothing new to mis-hit.
//
// Left-aligned at FXHDR_X rather than centred so the string can change width
// without moving, and the repaint is bounded to FXHDR_W: the glitch page flashes
// a "<<LOOPING>>" indicator further right in this same row, and a full-width
// clear here would erase it.
#define FXHDR_X 150            /* clear of BACK, which ends at x=136       */
#define FXHDR_W 190            /* longest string is "OVERDRIVE - OFF", 180 */
static void draw_fx_header(const char *title, bool on) {
    st7796_fill_rect(FXHDR_X, 6, FXHDR_W, 40, C_BG);
    char line[32], *p = line;
    const char *t = title;
    while (*t && p < line + 24) *p++ = *t++;
    *p++ = ' '; *p++ = '-'; *p++ = ' ';
    const char *s = on ? "ON" : "OFF";
    while (*s) *p++ = *s++;
    *p = '\0';
    st7796_draw_text(FXHDR_X, 18, 2,
                     on ? rgb565_be(0, 230, 80) : rgb565_be(150, 150, 150),
                     C_BG, line);
}

static void draw_gli_toggle(void) { draw_fx_header("GLITCH", gli_on); }

// "LOOPING" flash while a fragment plays (called from the main loop).
static void gli_indicator(bool looping) {
    static int last = -1;
    if ((int)looping == last) return;
    last = (int)looping;
    // Moved right from x=270 to clear the shared header, which now occupies
    // FXHDR_X..FXHDR_X+FXHDR_W (150..340) in this same row.
    if (looping)
        st7796_draw_text(344, 18, 2, rgb565_be(255, 60, 220), C_BG, "<<LOOPING>>");
    else
        st7796_fill_rect(340, 14, 140, 24, C_BG);
}

// FREQ slider: how often the random takeovers fire (random within range).
static void draw_gli_slider(void) {
    char line[20], *p = line;
    const char *s = "FREQ: "; while (*s) *p++ = *s++;
    p = fmt_int(p, gli_amt); *p = '\0';
    st7796_fill_rect(0, SLIDER_STRIP_Y - 30, ST7796_W, 30 + SLIDER_STRIP_H, C_BG);
    st7796_draw_text(190, SLIDER_STRIP_Y - 26, 2, rgb565_be(255, 255, 0), C_BG, line);
    st7796_fill_rect(SLIDER_X, SLIDER_Y, SLIDER_W, 8, C_LINE);
    int hx = SLIDER_X + (gli_amt - 1) * SLIDER_W / 9;
    if (hx > SLIDER_X + SLIDER_W - 8) hx = SLIDER_X + SLIDER_W - 8;
    if (hx < SLIDER_X + 8) hx = SLIDER_X + 8;
    st7796_fill_rect(hx - 8, SLIDER_Y - 12, 16, 32, C_CURSOR);
    st7796_draw_text(SLIDER_X, SLIDER_Y + 26, 1, C_LINE, C_BG, "RARE");
    st7796_draw_text(SLIDER_X + SLIDER_W - 36, SLIDER_Y + 26, 1, C_LINE, C_BG, "OFTEN");
}

static void draw_gli_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    // title + on/off state are drawn together by draw_fx_header()
    draw_gli_toggle();
    draw_gli_slider();
    // 39 chars at scale 2 is 468 px; starting at x=30 overran the 480 px panel
    // and clipped the final "G" off "PLAYING". Start at 6 instead.
    st7796_draw_text(6, 290, 2, rgb565_be(180, 180, 180), C_BG,
                     "RANDOM STUTTER TAKEOVERS. KEEP PLAYING.");
}

static void draw_slap_toggle(void) { draw_fx_header("DELAY", slap_on); }

// Three knobs share one slider: the selector row picks which one it edits.
// UP/DOWN (or a tap on a tab) moves the selection, LEFT/RIGHT rides the value.
// Cheaper on screen space than three tracks, and it keeps every settings page
// looking the same.
// Above the toggle deliberately: draw_slap_slider() clears y 174..250 as one
// strip (SLIDER_STRIP_Y - 30 for SLIDER_STRIP_H + 30), so tabs placed in that
// band get wiped every time a value changes.
static const rect_t BTN_DLY_TAB[3] = {
    { 170, 44, 92, 32 }, { 272, 44, 92, 32 }, { 374, 44, 92, 32 },
};
static const char *const DLY_TAB_LABEL[3] = { "TIME", "FBK", "MIX" };

static void draw_dly_tabs(void) {
    for (int i = 0; i < 3; i++)
        draw_button(&BTN_DLY_TAB[i], i == dly_sel ? C_CURSOR : C_GRAY, 2,
                    DLY_TAB_LABEL[i]);
}

static void draw_slap_slider(void) {
    char line[24], *p = line;
    const char *s = DLY_TAB_LABEL[dly_sel];
    while (*s) *p++ = *s++;
    *p++ = ':'; *p++ = ' ';
    int val = dly_sel == 0 ? dly_ms : dly_sel == 1 ? dly_fb_amt : dly_mix_amt;
    p = fmt_int(p, val);
    if (dly_sel == 0) { *p++ = ' '; *p++ = 'M'; *p++ = 'S'; }
    *p = '\0';
    // Clear 22 px PAST the strip: unlike every other settings page, the
    // end-of-scale labels change here (ms for TIME, 0..10 for FBK/MIX) and
    // they sit at SLIDER_Y + 26, outside the normal strip. Without this the
    // old text shows through, e.g. "40 MS" under "0" reads as "00 MS".
    st7796_fill_rect(0, SLIDER_STRIP_Y - 30, ST7796_W,
                     30 + SLIDER_STRIP_H + 22, C_BG);
    st7796_draw_text(170, SLIDER_STRIP_Y - 26, 2, rgb565_be(255, 255, 0), C_BG, line);
    st7796_fill_rect(SLIDER_X, SLIDER_Y, SLIDER_W, 8, C_LINE);
    int hx = dly_sel == 0 ? SLIDER_X + (dly_ms - 40) * SLIDER_W / 290
                          : SLIDER_X + val * SLIDER_W / 10;
    if (hx > SLIDER_X + SLIDER_W - 8) hx = SLIDER_X + SLIDER_W - 8;
    if (hx < SLIDER_X + 8) hx = SLIDER_X + 8;
    st7796_fill_rect(hx - 8, SLIDER_Y - 12, 16, 32, C_CURSOR);
    if (dly_sel == 0) {
        st7796_draw_text(SLIDER_X, SLIDER_Y + 26, 1, C_LINE, C_BG, "40 MS");
        st7796_draw_text(SLIDER_X + SLIDER_W - 36, SLIDER_Y + 26, 1, C_LINE,
                         C_BG, "330 MS");
    } else {
        st7796_draw_text(SLIDER_X, SLIDER_Y + 26, 1, C_LINE, C_BG, "0");
        st7796_draw_text(SLIDER_X + SLIDER_W - 12, SLIDER_Y + 26, 1, C_LINE,
                         C_BG, "10");
    }
}

// Apply whichever knob the selector is on. Kept in one place so the touch
// handler and the D-pad path cannot drift apart.
static void dly_apply(void) {
    if (dly_sel == 0)      fx_delay_time_ms(dly_ms);
    else if (dly_sel == 1) fx_delay_fb(dly_fb_amt);
    else                   fx_delay_mix(dly_mix_amt);
}

static void draw_slap_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    // title + on/off state are drawn together by draw_fx_header()
    draw_slap_toggle();
    draw_dly_tabs();
    draw_slap_slider();
    st7796_draw_text(10, 290, 2, rgb565_be(180, 180, 180), C_BG,
                     "FBK 0 = SLAPBACK. UP/DOWN PICKS A KNOB.");
}

static void draw_met_toggle(void) { draw_fx_header("SHRED", met_on); }

static void draw_met_slider(void) {
    char line[20], *p = line;
    const char *s = "DRIVE: "; while (*s) *p++ = *s++;
    p = fmt_int(p, met_amt); *p = '\0';
    st7796_fill_rect(0, SLIDER_STRIP_Y - 30, ST7796_W, 30 + SLIDER_STRIP_H, C_BG);
    st7796_draw_text(190, SLIDER_STRIP_Y - 26, 2, rgb565_be(255, 255, 0), C_BG, line);
    st7796_fill_rect(SLIDER_X, SLIDER_Y, SLIDER_W, 8, C_LINE);            // track
    int hx = SLIDER_X + (met_amt - 5) * SLIDER_W / 55;
    if (hx > SLIDER_X + SLIDER_W - 8) hx = SLIDER_X + SLIDER_W - 8;
    if (hx < SLIDER_X + 8) hx = SLIDER_X + 8;
    st7796_fill_rect(hx - 8, SLIDER_Y - 12, 16, 32, C_CURSOR);
    st7796_draw_text(SLIDER_X, SLIDER_Y + 26, 1, C_LINE, C_BG, "CRUNCH");
    st7796_draw_text(SLIDER_X + SLIDER_W - 42, SLIDER_Y + 26, 1, C_LINE, C_BG, "MAYHEM");
}

static void draw_met_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    // title + on/off state are drawn together by draw_fx_header()
    draw_met_toggle();
    draw_met_slider();
    st7796_draw_text(30, 290, 2, rgb565_be(180, 180, 180), C_BG,
                     "SCOOPED HIGH-GAIN. PALM MUTE IT.");
}

static void draw_para_toggle(void) { draw_fx_header("AUTO WAH", para_on); }

// RANGE slider: sweep ceiling of the envelope filter (ride it like the pedal's
// foot-adjusted knob).
static void draw_para_slider(void) {
    char line[20], *p = line;
    const char *s = "RANGE: "; while (*s) *p++ = *s++;
    p = fmt_int(p, para_rng); *p = '\0';
    st7796_fill_rect(0, SLIDER_STRIP_Y - 30, ST7796_W, 30 + SLIDER_STRIP_H, C_BG);
    st7796_draw_text(190, SLIDER_STRIP_Y - 26, 2, rgb565_be(255, 255, 0), C_BG, line);
    st7796_fill_rect(SLIDER_X, SLIDER_Y, SLIDER_W, 8, C_LINE);            // track
    int hx = SLIDER_X + para_rng * SLIDER_W / 10;
    if (hx > SLIDER_X + SLIDER_W - 8) hx = SLIDER_X + SLIDER_W - 8;
    if (hx < SLIDER_X + 8) hx = SLIDER_X + 8;
    st7796_fill_rect(hx - 8, SLIDER_Y - 12, 16, 32, C_CURSOR);
    st7796_draw_text(SLIDER_X, SLIDER_Y + 26, 1, C_LINE, C_BG, "DARK");
    st7796_draw_text(SLIDER_X + SLIDER_W - 30, SLIDER_Y + 26, 1, C_LINE, C_BG, "OPEN");
}

static void draw_para_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    // title + on/off state are drawn together by draw_fx_header()
    draw_para_toggle();
    draw_para_slider();
    st7796_draw_text(30, 290, 2, rgb565_be(180, 180, 180), C_BG,
                     "ENVELOPE FILTER: PICK TO OPEN IT.");
}

static void draw_od_toggle(void) { draw_fx_header("OD", od_on); }

static void draw_od_slider(void) {
    char line[20], *p = line;
    const char *s = "DRIVE: "; while (*s) *p++ = *s++;
    p = fmt_int(p, od_amt); *p = '\0';
    st7796_fill_rect(0, SLIDER_STRIP_Y - 30, ST7796_W, 30 + SLIDER_STRIP_H, C_BG);
    st7796_draw_text(190, SLIDER_STRIP_Y - 26, 2, rgb565_be(255, 255, 0), C_BG, line);
    st7796_fill_rect(SLIDER_X, SLIDER_Y, SLIDER_W, 8, C_LINE);
    int hx = SLIDER_X + (od_amt - 1) * SLIDER_W / 9;
    if (hx > SLIDER_X + SLIDER_W - 8) hx = SLIDER_X + SLIDER_W - 8;
    if (hx < SLIDER_X + 8) hx = SLIDER_X + 8;
    st7796_fill_rect(hx - 8, SLIDER_Y - 12, 16, 32, C_CURSOR);
    st7796_draw_text(SLIDER_X, SLIDER_Y + 26, 1, C_LINE, C_BG, "WARM");
    st7796_draw_text(SLIDER_X + SLIDER_W - 42, SLIDER_Y + 26, 1, C_LINE, C_BG, "CRUNCH");
}

static void draw_od_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    // title + on/off state are drawn together by draw_fx_header()
    draw_od_toggle();
    draw_od_slider();
    st7796_draw_text(30, 290, 2, rgb565_be(180, 180, 180), C_BG,
                     "SMOOTH ASYMMETRIC-CLIP PEDAL.");
}

static void draw_verb_toggle(void) { draw_fx_header("REVERB", verb_on); }

// MIX runs 0..10 (0 = dry, 10 = 0.80 wet). Unlike the drive knobs this one
// starts at 0, so the slider maths differ from draw_od_slider's 1..10.
static void draw_verb_slider(void) {
    char line[20], *p = line;
    const char *s = "MIX: "; while (*s) *p++ = *s++;
    p = fmt_int(p, verb_mix_amt); *p = '\0';
    st7796_fill_rect(0, SLIDER_STRIP_Y - 30, ST7796_W, 30 + SLIDER_STRIP_H, C_BG);
    st7796_draw_text(200, SLIDER_STRIP_Y - 26, 2, rgb565_be(255, 255, 0), C_BG, line);
    st7796_fill_rect(SLIDER_X, SLIDER_Y, SLIDER_W, 8, C_LINE);
    int hx = SLIDER_X + verb_mix_amt * SLIDER_W / 10;
    if (hx > SLIDER_X + SLIDER_W - 8) hx = SLIDER_X + SLIDER_W - 8;
    if (hx < SLIDER_X + 8) hx = SLIDER_X + 8;
    st7796_fill_rect(hx - 8, SLIDER_Y - 12, 16, 32, C_CURSOR);
    st7796_draw_text(SLIDER_X, SLIDER_Y + 26, 1, C_LINE, C_BG, "DRY");
    st7796_draw_text(SLIDER_X + SLIDER_W - 30, SLIDER_Y + 26, 1, C_LINE, C_BG, "WET");
}

// ---- Generic one-knob settings page ---------------------------------------
// Three of the newest effects (bit crush, flanger, ring mod) are single-knob
// effects, and copy-pasting a fourth, fifth and sixth near-identical page is
// how this file got long in the first place. One descriptor table drives one
// draw function and one input handler instead. Dragging the slider updates
// continuously, which is the point: bit crush especially only shows itself
// when you SWEEP it while playing.
// Now up to THREE knobs per page, selected by a tab row (UP/DOWN or a tap),
// sharing one slider -- the pattern DELAY already used. Reverb needs MIX,
// DECAY and SHIMMER; ring mod needs FREQ and MIX. One table entry per effect.
typedef struct {
    const char *name, *lo, *hi;
    int   lo_v, hi_v;
    int  *value;
    void (*apply)(int);
    // Optional, both default to NULL for every knob that predates them:
    //   names — a value->label table, for a knob that picks a MODE rather than
    //           a magnitude (readout shows names[v], not the integer);
    //   unit  — suffix after the number, e.g. "%".
    const char *const *names;
    const char *unit;
} knob_t;

typedef struct {
    const char *title, *blurb;
    bool *on;
    void (*set)(int);
    int   n;                       // how many knobs (1..3)
    knob_t k[3];
} knob_page_t;

// KNOB_PAGES below is positional, so these members and that array's entries
// must be guarded in matching order.
enum { KNOB_CRUSH = 0, KNOB_FLANGE,
#if FX_ENABLE_RINGMOD
       KNOB_RING,
#endif
#if FX_ENABLE_VOWEL
       KNOB_VOWEL,
#endif
       KNOB_VERB, KNOB_OCT, KNOB_COUNT };
static int knob_cur = KNOB_CRUSH;  // which effect's page
static int knob_sel = 0;           // which knob on it

// MODE labels for the octave page. Kept short: the readout is drawn at scale 2
// and shares its line with the knob name.
static const char *const OCT_MODES[5] = {
    "+1 OCT", "+2 OCT", "-1 OCT", "-2 OCT", "DETUNE",
};

// FLANGER mode labels (BBD flanger: sweeping, or LFO frozen).
static const char *const FL_MODES[2] = { "FLANGE", "MATRIX" };

#if FX_ENABLE_VOWEL
// VOWEL mode labels.
static const char *const VW_MODES[2] = { "AUTO", "MANUAL" };
#endif

static const knob_page_t KNOB_PAGES[KNOB_COUNT] = {
    { "BITCRUSH", "7 IS THE 8-BIT CHIPTUNE SPOT. SET IT AND PLAY.",
      &bc_on, fx_crush_set, 1,
      { { "CRUSH", "12BIT 10X", "6BIT 24X", 1, 10, &bc_amt, fx_crush_amount } } },
    { "FLANGER", "ANALOG BBD MODEL. COLOR = RESONANCE. MATRIX FREEZES THE COMB.",
      &fl_on, fx_flange_set, 3,
      { { "MODE",  "FLANGE",  "MATRIX", 0, 1,  &fl_mode,  fx_flange_mode,
          FL_MODES, NULL },
        { "RATE",  "0.05 HZ", "10 HZ",  1, 10, &fl_amt,   fx_flange_rate },
        { "COLOR", "CLEAN",   "0.85",   0, 10, &fl_col,   fx_flange_color } } },
#if FX_ENABLE_RINGMOD
    { "RINGMOD", "1-3 IS TREMOLO. 7-10 IS ROBOT. MIX KEEPS CHORDS ALIVE.",
      &rm_on, fx_ring_set, 2,
      { { "FREQ", "1 HZ", "1200 HZ", 1, 10, &rm_amt, fx_ring_freq },
        { "MIX",  "DRY", "WET",      0, 10, &rm_mix_amt, fx_ring_mix } } },
#endif
#if FX_ENABLE_VOWEL
    { "VOWEL", "TWO FORMANTS: THE GUITAR TALKS. AUTO SWEEPS EE-EH-AH-OH-OO.",
      &vw_on, fx_vowel_set, 3,
      { { "MODE", "AUTO",    "MANUAL", 0, 1,  &vw_mode,     fx_vowel_mode,
          VW_MODES, NULL },
        { "RATE", "0.05 HZ", "4 HZ",   1, 10, &vw_rate_amt, fx_vowel_rate },
        { "POS",  "EE",      "OO",     0, 10, &vw_pos_amt,  fx_vowel_pos } } },
#endif
    { "REVERB", "8-COMB PLATE. SHIMMER PITCHES THE TAIL UP AN OCTAVE.",
      &verb_on, fx_verb_set, 3,
      { { "MIX",     "DRY",   "WET",   0, 10, &verb_mix_amt, fx_verb_mix },
        { "DECAY",   "1.2 S", "7.5 S", 0, 10, &verb_dec_amt, fx_verb_decay },
        { "SHIMMER", "OFF",   "GLIMMER", 0, 10, &verb_sh_amt, fx_verb_shimmer } } },
    { "OCTAVE", "MODE PICKS THE VOICE. DETUNE: 50% IS IN TUNE, ENDS ARE +/-1 SEMITONE.",
      &oct_on, fx_octave_set, 3,
      { { "MODE",   "+1 OCT",   "DETUNE",   0, 4,   &oct_mode,    fx_octave_mode,
          OCT_MODES, NULL },
        { "MIX",    "DRY",      "WET",      0, 100, &oct_mix_pct, fx_octave_mix,
          NULL, "%" },
        { "DETUNE", "-100 CENTS", "+100 CENTS", 0, 100, &oct_det, fx_octave_detune,
          NULL, "%" } } },
};

// Tab row for pages with more than one knob. Above the toggle, because the
// slider strip clears y 174..250 and would wipe anything placed in there.
static const rect_t BTN_KNOB_TAB[3] = {
    { 170, 44, 92, 32 }, { 272, 44, 92, 32 }, { 374, 44, 92, 32 },
};


// The knob pages never DREW this button -- draw_knob_page() has not called it
// since the layout was rewritten -- but the physical-button handler still did,
// which painted a phantom 300x84 button straight over the sliders. Now it
// repaints the shared header instead, so the knob pages finally show their
// on/off state too (they had none at all).
static void draw_knob_toggle(void) {
    const knob_page_t *k = &KNOB_PAGES[knob_cur];
    draw_fx_header(k->title, *k->on);
}

static void draw_knob_tabs(void) {
    const knob_page_t *p = &KNOB_PAGES[knob_cur];
    if (p->n < 2) return;                    // single-knob pages get no tabs
    for (int i = 0; i < p->n; i++)
        draw_button(&BTN_KNOB_TAB[i], i == knob_sel ? C_CURSOR : C_GRAY, 2,
                    p->k[i].name);
}

// ---- Parameter pages: every knob is its own slider, all visible at once ----
// The old layout was a tab row + ONE shared slider + a 300x84 "EFFECT: ON/OFF"
// button parked across the middle. Three complaints from the bench all trace to
// that one decision: DELAY's MIX was hard to hit, so was REVERB's SHIMMER, and
// reaching for DECAY toggled the effect off instead -- because the giant enable
// button occupied the centre of a page whose entire job is parameters, pushing
// the real controls out to the margins where the bezel gets them.
//
// The enable button is also redundant: tapping the tile on the FX grid already
// toggles the effect, and holding it is how you got here. So it is gone, and
// the reclaimed space goes to one full-width slider PER parameter. No tab row,
// no mode switching, nothing to mis-hit: what you see is what you can drag.
//
// Sliders rather than rotary knobs, chosen deliberately at the bench: knobs
// pack tighter and look more like gear, but a thumb hits a wide horizontal
// track far more accurately than it works an arc, and the value is readable at
// a glance without a pointer to interpret.
// 82, not 64: BACK occupies y 6..50, and a row at 64 puts its name/value line
// at y-24 = 40, straight through the button (reported from the device).
#define KROW_Y0   82          /* first slider row, clear of BACK */
#define KROW_DY   78          /* row pitch (3 rows fit above the blurb) */
#define KTRK_H    10          /* track thickness */
#define KTRK_X    TRACK_L
#define KTRK_W    TRACK_W   /* knob sliders: drawn == usable, see TRACK_L */
#define KGRAB_H   52          /* touch band height per row: generous on purpose */
// A thumb cannot reliably land on the literal first or last pixel of a track --
// the bench could only get 1..9 out of a 0..10 knob. So the outer KTRK_PAD of
// each end SATURATES to the extreme value: past that point you are already at
// lo or hi. The drawn fill still spans the whole track so the reading stays
// honest; only the touch mapping saturates early.
#define KTRK_PAD  50

// Track geometry for row i, and the touch band that drives it.
static rect_t knob_row_band(int i) {
    rect_t r = { KTRK_X, KROW_Y0 + i * KROW_DY - 12, KTRK_W, KGRAB_H };
    return r;
}

static void draw_knob_row(int i) {
    const knob_page_t *p = &KNOB_PAGES[knob_cur];
    const knob_t *k = &p->k[i];
    int y = KROW_Y0 + i * KROW_DY;
    // name + value, left and right of the same line above the track
    char line[40], *q = line;
    const char *t = k->name; while (*t) *q++ = *t++;
    *q++ = ':'; *q++ = ' ';
    if (k->names) {                       // a MODE knob: label, not a number
        const char *nm = k->names[*k->value];
        while (*nm) *q++ = *nm++;
    } else {
        q = fmt_int(q, *k->value);
        if (k->unit) { const char *u = k->unit; while (*u) *q++ = *u++; }
    }
    *q = '\0';
    // y-30, not y-24: at scale 2 this line is ~16 px tall, so at y-24 it
    // occupied y-24..y-8 while the HANDLE spans y-9..y+19 -- a 1-2 px collision
    // whenever the handle sat under the text, i.e. at LOW values. Reported from
    // the bench as sliders overlapping the parameter's descriptive text. y-30
    // leaves a 5 px gap. Checked against its neighbours: the row pitch is 78, so
    // the previous row's clear ends at y-36 and the next row's text starts at
    // y+48, both clear of this row's band; and row 0's text at y=52 clears the
    // page header, which ends at y=34.
    st7796_fill_rect(0, y - 32, ST7796_W, KROW_DY - 4, C_BG);
    st7796_draw_text(KTRK_X, y - 30, 2, rgb565_be(255, 255, 0), C_BG, line);
    // end labels, right-aligned on the right so they never run off
    // y+22, not y+18: the handle spans y-9..y+19, so at y+18 the right-hand
    // end label sat underneath it whenever the value was near maximum.
    st7796_draw_text(KTRK_X, y + 24, 1, C_LINE, C_BG, k->lo);
    int hw = 0; for (const char *c = k->hi; *c; c++) hw += 6;
    st7796_draw_text(KTRK_X + KTRK_W - hw, y + 24, 1, C_LINE, C_BG, k->hi);
    // track, filled up to the value, then the handle
    st7796_fill_rect(KTRK_X, y, KTRK_W, KTRK_H, C_LINE);
    int span = k->hi_v - k->lo_v;
    int fill = (*k->value - k->lo_v) * KTRK_W / (span ? span : 1);
    if (fill > 0) st7796_fill_rect(KTRK_X, y, fill, KTRK_H, C_CURSOR);
    int hx = KTRK_X + fill;
    if (hx > KTRK_X + KTRK_W - 9) hx = KTRK_X + KTRK_W - 9;
    if (hx < KTRK_X + 9) hx = KTRK_X + 9;
    st7796_fill_rect(hx - 9, y - 9, 18, KTRK_H + 18,
                     i == knob_sel ? rgb565_be(255, 255, 255) : C_CURSOR);
}

static void draw_knob_slider(void) {       // repaint the active row only
    draw_knob_row(knob_sel);
}

static void draw_knob_page(void) {
    const knob_page_t *k = &KNOB_PAGES[knob_cur];
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    draw_knob_toggle();                  // title + on/off state, shared header
    for (int i = 0; i < k->n; i++) draw_knob_row(i);
    st7796_draw_text(SAFE_L, 300, 1, rgb565_be(180, 180, 180), C_BG, k->blurb);
}

static void draw_verb_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    // title + on/off state are drawn together by draw_fx_header()
    draw_verb_toggle();
    draw_verb_slider();
    st7796_draw_text(10, 290, 2, rgb565_be(180, 180, 180), C_BG,
                     "PLATE, ~1.2 S TAIL. SITS ON THE WHOLE MIX.");
}

// TREMOLO's own page, back to driving TREMOLO. It was cut from the grid on
// 2026-07-30 to free a slot for RINGMOD -- never because it sounded bad -- and
// CHORUS then borrowed this page, which is why the internals were trem_* while
// the labels said CHORUS. CHORUS was cut on 2026-08-01 ("chorus does sound
// bad... get rid of it") and TREMOLO took its slot back: the DSP was still
// intact in fx.c, it costs no buffer at all (one LFO), and it keeps the grid at
// a full 4x4 so the D-pad cursor and hit-testing need no empty-cell handling.
// If a hole is preferred instead, this slot is the one to blank.
#if FX_ENABLE_TREMOLO
static void draw_trem_toggle(void) { draw_fx_header("TREMOLO", trem_on); }

static void draw_trem_slider(void) {
    char line[20], *p = line;
    const char *s = "RATE: "; while (*s) *p++ = *s++;
    p = fmt_int(p, trem_amt);
    *p++ = ' '; *p++ = 'H'; *p++ = 'Z'; *p = '\0';
    st7796_fill_rect(0, SLIDER_STRIP_Y - 30, ST7796_W, 30 + SLIDER_STRIP_H, C_BG);
    st7796_draw_text(180, SLIDER_STRIP_Y - 26, 2, rgb565_be(255, 255, 0), C_BG, line);
    st7796_fill_rect(SLIDER_X, SLIDER_Y, SLIDER_W, 8, C_LINE);
    int hx = SLIDER_X + (trem_amt - 1) * SLIDER_W / 9;
    if (hx > SLIDER_X + SLIDER_W - 8) hx = SLIDER_X + SLIDER_W - 8;
    if (hx < SLIDER_X + 8) hx = SLIDER_X + 8;
    st7796_fill_rect(hx - 8, SLIDER_Y - 12, 16, 32, C_CURSOR);
    st7796_draw_text(SLIDER_X, SLIDER_Y + 26, 1, C_LINE, C_BG, "SLOW");
    st7796_draw_text(SLIDER_X + SLIDER_W - 30, SLIDER_Y + 26, 1, C_LINE, C_BG, "FAST");
}

static void draw_trem_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    // title + on/off state are drawn together by draw_fx_header()
    draw_trem_toggle();
    draw_trem_slider();
    st7796_draw_text(30, 290, 2, rgb565_be(180, 180, 180), C_BG,
                     "SMOOTH VOLUME WOBBLE, 70% DEEP.");
}
#endif

#if FX_ENABLE_PHASER
static void draw_phase_toggle(void) { draw_fx_header("PHASER", ph_on); }

static void draw_phase_slider(void) {
    char line[20], *p = line;
    const char *s = "RATE: "; while (*s) *p++ = *s++;
    p = fmt_int(p, ph_amt); *p = '\0';
    st7796_fill_rect(0, SLIDER_STRIP_Y - 30, ST7796_W, 30 + SLIDER_STRIP_H, C_BG);
    st7796_draw_text(190, SLIDER_STRIP_Y - 26, 2, rgb565_be(255, 255, 0), C_BG, line);
    st7796_fill_rect(SLIDER_X, SLIDER_Y, SLIDER_W, 8, C_LINE);
    int hx = SLIDER_X + (ph_amt - 1) * SLIDER_W / 9;
    if (hx > SLIDER_X + SLIDER_W - 8) hx = SLIDER_X + SLIDER_W - 8;
    if (hx < SLIDER_X + 8) hx = SLIDER_X + 8;
    st7796_fill_rect(hx - 8, SLIDER_Y - 12, 16, 32, C_CURSOR);
    st7796_draw_text(SLIDER_X, SLIDER_Y + 26, 1, C_LINE, C_BG, "SLOW");
    st7796_draw_text(SLIDER_X + SLIDER_W - 30, SLIDER_Y + 26, 1, C_LINE, C_BG, "FAST");
}

static void draw_phase_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    // title + on/off state are drawn together by draw_fx_header()
    draw_phase_toggle();
    draw_phase_slider();
    st7796_draw_text(30, 290, 2, rgb565_be(180, 180, 180), C_BG,
                     "4-STAGE SWIRL, 50/50 MIX.");
}
#endif

// ---- The five coloured hardware buttons ----
// ONE table. There used to be two: this one, and a local copy inside
// eq_draw_readout() that read { "GREY", "YELLW", ... } -- truncated, and
// British-spelled. Nothing required the truncation (the readout is drawn at
// x=120 at 12 px/char, so its longest possible line, "YELLOW 12000Hz -12.0dB
// Q8.0" at 28 chars, ends at x=456 on a 480 px panel), so the EQ page simply
// disagreed with the sampler page about what the buttons are called. Two
// hand-maintained copies of one list is how that happens.
//
// VOX_SLOTS is 5 because there are five buttons; the sampler's slots and the
// EQ's bands are both just "one per coloured button".
static const char *const BTN_COLOR_NAMES[VOX_SLOTS] =
    { "GRAY", "YELLOW", "GREEN", "BLUE", "RED" };

static uint16_t vox_color(int i, bool dim) {
    uint8_t r, g, b;
    switch (i) {
    case 0:  r = 120; g = 120; b = 120; break;   // gray
    case 1:  r = 200; g = 180; b = 0;   break;   // yellow
    case 2:  r = 0;   g = 150; b = 0;   break;   // green
    case 3:  r = 0;   g = 90;  b = 190; break;   // blue
    default: r = 180; g = 0;   b = 0;   break;   // red
    }
    if (dim) { r >>= 2; g >>= 2; b >>= 2; }      // empty slot = dimmed
    return rgb565_be(r, g, b);
}

// Slot buttons: dimmed = empty, full color = recorded, white ring = selected.
// A short duration label sits on each recorded slot.
static void draw_vox_slot(int i) {
    const rect_t *r = &BTN_V_SLOT[i];
    bool sel = vox_selected() == i;
    uint16_t fill = vox_color(i, vox_len_ds(i) == 0);
    st7796_fill_rect(r->x, r->y, r->w, r->h,
                     sel ? rgb565_be(255, 255, 255) : C_BG);
    st7796_fill_rect(r->x + 3, r->y + 3, r->w - 6, r->h - 6, fill);
    unsigned ds = vox_len_ds(i);
    if (ds) {
        char line[10], *p = line;
        p = fmt_int(p, (int)(ds / 10));
        *p++ = '.'; p = fmt_int(p, (int)(ds % 10)); *p++ = 'S'; *p = '\0';
        st7796_draw_text(r->x + 8, r->y + r->h / 2 - 4, 1,
                         rgb565_be(255, 255, 255), fill, line);
    }
}

static void draw_vox_slots(void) {
    for (int i = 0; i < VOX_UI_SLOTS; i++) draw_vox_slot(i);
}

// Control row: HOLD latch + input source + output volume (so the sampler is
// self-contained — no trip back to the main page to change input/level).
static void draw_vox_hold(void) {
    draw_button(&BTN_V_HOLD, vox_held() ? C_GREEN : C_GRAY, 2,
                vox_held() ? "HOLD ON" : "HOLD");
}
static void draw_vox_src(void) {
    draw_button(&BTN_V_SRC, C_PURPLE, 2, use_pdm ? "IN: PDM MICS" : "IN: 3.5MM");
}
static void draw_vox_vol(void) {
    draw_button(&BTN_V_VOL, vol_idx == 2 ? C_RED : C_GRAY, 2,
                VOL_STEPS[vol_idx].label);
}

// Status strip: what the selected slot holds / recording progress.
static void draw_vox_status(void) {
    char line[40], *p = line;
    st7796_fill_rect(0, VSTATUS_Y, ST7796_W, 20, C_BG);
    uint16_t col = rgb565_be(255, 255, 0);
    if (!vox_ok()) {
        st7796_draw_text(20, VSTATUS_Y, 2, rgb565_be(255, 60, 60), C_BG,
                         "NO PSRAM - SAMPLER DISABLED");
        return;
    }
    if (vox_recording()) {
        const char *s = "REC "; while (*s) *p++ = *s++;
        const char *nm = BTN_COLOR_NAMES[vox_selected()]; while (*nm) *p++ = *nm++;
        *p++ = ' ';
        unsigned ds = vox_rec_ds();
        p = fmt_int(p, (int)(ds / 10)); *p++ = '.';
        p = fmt_int(p, (int)(ds % 10)); *p++ = 'S'; *p = '\0';
        col = rgb565_be(255, 60, 60);
    } else {
        const char *nm = BTN_COLOR_NAMES[vox_selected()]; while (*nm) *p++ = *nm++;
        *p++ = ' ';
        unsigned ds = vox_len_ds(vox_selected());
        if (ds) {
            p = fmt_int(p, (int)(ds / 10)); *p++ = '.';
            p = fmt_int(p, (int)(ds % 10)); *p++ = 'S';
            const char *s = " - TOUCH PAD TO PLAY"; while (*s) *p++ = *s++;
        } else {
            const char *s = "EMPTY - HOLD BUTTON TO REC"; while (*s) *p++ = *s++;
        }
        *p = '\0';
    }
    st7796_draw_text(20, VSTATUS_Y, 2, col, C_BG, line);
}

// Turntable pad crosshair (vertical + horizontal line through the touch).
static int vox_cur_x = -1, vox_cur_y = -1;

// ---- Sample waveform strip --------------------------------------------------
// The recorded sample lives INSIDE the turntable pad as a mirrored peak plot:
// dim columns while unplayed, a bright fill up to the needle while playing
// (the fill retreats when the platter spins in reverse), and red columns
// appearing left-to-right in real time while recording. Replaces the old
// progress bar under the pad.
#define VWF_TOP  (VPAD_Y + 20)
#define VWF_BOT  (VPAD_Y + VPAD_H - 20)
#define VWF_MIDY ((VWF_TOP + VWF_BOT) / 2)
#define VWF_HMAX ((VWF_BOT - VWF_TOP) / 2 - 1)
static uint8_t  vwf_amp[VPAD_W];     // per-column half-height, 0..VWF_HMAX
static int      vwf_have = 0;        // columns with valid data
static int      vwf_fill = 0;        // columns currently drawn bright
static int      vwf_needle = -1;     // needle column (-1 = hidden)
static bool     vwf_rec = false;     // recording: columns draw red
static unsigned vwf_rec_cap = 0;     // sample capacity (x axis while recording)

static void vwf_draw_col(int i, uint16_t color) {
    int x = VPAD_X + i;
    st7796_fill_rect(x, VWF_TOP, 1, VWF_BOT - VWF_TOP + 1, C_PAD);
    if (i == VPAD_W / 2 - 1 || i == VPAD_W / 2)          // keep the detent line
        st7796_fill_rect(x, VWF_TOP, 1, VWF_BOT - VWF_TOP + 1, C_LINE);
    int a = vwf_amp[i];
    if (i < vwf_have && a)
        st7796_fill_rect(x, VWF_MIDY - a, 1, 2 * a + 1, color);
}

static void vwf_restore_col(int i) {
    if (i < 0 || i >= VPAD_W) return;
    vwf_draw_col(i, vwf_rec ? C_WF_REC : (i < vwf_fill ? C_WF_LIT : C_WF_DIM));
}

static void vwf_draw_all(void) {
    for (int i = 0; i < VPAD_W; i++) vwf_restore_col(i);
    if (vwf_needle >= 0)
        st7796_fill_rect(VPAD_X + vwf_needle, VWF_TOP, 1,
                         VWF_BOT - VWF_TOP + 1, C_WF_NEEDLE);
}

static uint8_t vwf_seg_amp(const int16_t *d, uint32_t s0, uint32_t s1) {
    uint32_t stride = (s1 - s0 > 96u) ? (s1 - s0) / 96u : 1u;
    int pk = 0;
    for (uint32_t s = s0; s < s1; s += stride) {
        int v = d[s];
        if (v < 0) v = -v;
        if (v > pk) pk = v;
    }
    // sqrt lifts quiet detail: a -24 dB passage still reads as shape
    int a = (int)(sqrtf((float)pk * (1.0f / 32768.0f)) * (float)VWF_HMAX + 0.5f);
    return (uint8_t)(a > VWF_HMAX ? VWF_HMAX : a);
}

static void vwf_rescan(void) {
    unsigned len;
    const int16_t *d = vox_data(vox_selected(), &len);
    vwf_have = 0; vwf_fill = 0; vwf_needle = -1; vwf_rec = false;
    if (d && len >= (unsigned)VPAD_W) {
        for (int i = 0; i < VPAD_W; i++)
            vwf_amp[i] = vwf_seg_amp(d,
                             (uint32_t)((uint64_t)i * len / VPAD_W),
                             (uint32_t)((uint64_t)(i + 1) * len / VPAD_W));
        vwf_have = VPAD_W;
    }
}

static void vwf_rec_begin(void) {
    vwf_have = 0; vwf_fill = 0; vwf_needle = -1;
    vwf_rec = true; vwf_rec_cap = vox_capacity();
    vwf_draw_all();                  // clear the strip; columns arrive live
}

// While recording: reveal columns left-to-right in real time (x axis = full
// slot capacity; the strip is re-scanned to stretch on commit).
static void vwf_rec_update(void) {
    unsigned len;
    const int16_t *d = vox_data(vox_selected(), &len);
    if (!d || !vwf_rec_cap) return;
    int end = (int)((uint64_t)len * VPAD_W / vwf_rec_cap);
    if (end > VPAD_W) end = VPAD_W;
    for (int i = vwf_have; i < end; i++) {
        vwf_amp[i] = vwf_seg_amp(d,
                         (uint32_t)((uint64_t)i * vwf_rec_cap / VPAD_W),
                         (uint32_t)((uint64_t)(i + 1) * vwf_rec_cap / VPAD_W));
        vwf_have = i + 1;
        vwf_draw_col(i, C_WF_REC);
    }
}

// While playing: move the fill boundary and the needle to the platter
// position. Forward playback grows the bright region, reverse shrinks it;
// a loop-seam wrap redraws the whole strip once.
static void vwf_play_update(void) {
    if (!vwf_have || vwf_rec) return;
    int nc = (int)(vox_pos_frac() * (float)(vwf_have - 1) + 0.5f);
    if (nc == vwf_fill && nc == vwf_needle) return;
    int prev_needle = vwf_needle;
    vwf_needle = -1;                 // restores below use fill state only
    if (prev_needle >= 0 && prev_needle != nc) vwf_restore_col(prev_needle);
    int d = nc - vwf_fill;
    if (d > VPAD_W / 2 || d < -VPAD_W / 2) {       // loop-seam wrap
        vwf_fill = nc;
        vwf_draw_all();
    } else if (d > 0) {
        for (int i = vwf_fill; i < nc; i++) vwf_draw_col(i, C_WF_LIT);
        vwf_fill = nc;
    } else if (d < 0) {
        for (int i = nc; i < vwf_fill; i++) vwf_draw_col(i, C_WF_DIM);
        vwf_fill = nc;
    }
    st7796_fill_rect(VPAD_X + nc, VWF_TOP, 1, VWF_BOT - VWF_TOP + 1, C_WF_NEEDLE);
    vwf_needle = nc;
}

static void vox_pad_captions(void) {
    st7796_draw_text(VPAD_X + 6, VPAD_Y + VPAD_H - 14, 1, C_LINE, C_PAD,
                     "< 1X REVERSE");
    st7796_draw_text(VPAD_X + VPAD_W - 84, VPAD_Y + VPAD_H - 14, 1, C_LINE,
                     C_PAD, "1X FORWARD >");
    st7796_draw_text(VPAD_X + VPAD_W / 2 + 6, VPAD_Y + 6, 1, C_LINE, C_PAD,
                     "STOP");
}

// Rate cursor: a vertical line at the touch X (the pad's Y axis is
// decorative). Erasing restores the waveform columns underneath.
static void vox_cursor(int nx, int ny) {
    if (vox_cur_x >= 0)
        for (int i = vox_cur_x - 1 - VPAD_X; i <= vox_cur_x + 1 - VPAD_X; i++) {
            if (i < 0 || i >= VPAD_W) continue;
            st7796_fill_rect(VPAD_X + i, VPAD_Y, 1, VPAD_H, C_PAD);
            vwf_restore_col(i);
        }
    // ALWAYS repair the captions, not just on release. The erase above blanks
    // three full-height columns and vwf_restore_col() only puts the WAVEFORM
    // back, so every column the cursor crossed lost whatever caption pixels sat
    // in it. Repairing only when the cursor was removed (nx < 0) meant "STOP",
    // "< 1X REVERSE" and "1X FORWARD >" dissolved letter by letter while
    // dragging and only came back on release -- reported from the bench as
    // artifacting on the pad's background text.
    //
    // BEFORE the new cursor, deliberately. st7796_draw_text() paints a C_PAD
    // cell background behind each glyph, so repairing afterwards would punch
    // ~10 px gaps through the cursor line at the two caption rows. This way the
    // cursor is a solid line and the cost is a 3 px notch in whichever letter it
    // is crossing right now -- invisible under the finger that put it there.
    //
    // Three scale-1 draws, ~1.7k pixels, against the three full-height column
    // erases already happening here. Nowhere near invariant 13's redraw budget.
    vox_pad_captions();
    if (nx >= 0) {
        if (nx < VPAD_X + 2) nx = VPAD_X + 2;
        if (nx > VPAD_X + VPAD_W - 3) nx = VPAD_X + VPAD_W - 3;
        st7796_fill_rect(nx - 1, VPAD_Y, 3, VPAD_H, C_CURSOR);
    }
    vox_cur_x = nx; vox_cur_y = ny;
}

static void draw_vox_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    st7796_draw_text(184, 18, 2, rgb565_be(255, 255, 255), C_BG, "SAMPLER");
    draw_vox_slots();
    draw_vox_hold();
    draw_vox_src();
    draw_vox_vol();
    st7796_fill_rect(VPAD_X - 2, VPAD_Y - 2, VPAD_W + 4, VPAD_H + 4, C_LINE);
    st7796_fill_rect(VPAD_X, VPAD_Y, VPAD_W, VPAD_H, C_PAD);
    vox_cur_x = vox_cur_y = -1;
    vwf_rescan();
    vwf_draw_all();
    vox_pad_captions();
    draw_vox_status();
    // Was 81 chars: at scale 1 (6 px/char) from x=8 that is 494 px on a 480 px
    // panel, and it clipped to "...HANDS-FREE LO". 69 chars fits with margin.
    st7796_draw_text(8, 304, 1, rgb565_be(150, 150, 150), C_BG,
                     "SLOT: TAP=SELECT HOLD=REC   PAD: HOLD=PLAY DRAG=SPEED   BTN HOLD=LOOP");
}

// ---- Orcatune page ----
static const char *const TUNE_KEYS[12] =
    { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
static const char *const TUNE_SCALES[9] =
    { "MAJOR", "MINOR", "PENTA", "PENTA+4", "PENTA+7", "MAJ6", "TRIAD",
      "5TH", "CHROM" };

static void draw_tune_toggle(void) { draw_fx_header("ORCATUNE", tune_on); }

static void draw_tune_ctls(void) {
    char kb[12], *p = kb;
    const char *s = "KEY "; while (*s) *p++ = *s++;
    const char *k = TUNE_KEYS[tune_key]; while (*k) *p++ = *k++;
    *p = '\0';
    draw_button(&BTN_T_KEY,   C_BLUE, 2, kb);
    draw_button(&BTN_T_SCALE, C_BLUE, 2, TUNE_SCALES[tune_scale]);
    draw_button(&BTN_T_MODE,  tune_robot ? C_PURPLE : C_GRAY, 2,
                tune_robot ? "ROBOTIC" : "NATURAL");
}

static void draw_tune_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    // title + on/off state are drawn together by draw_fx_header()
    draw_tune_toggle();
    draw_tune_ctls();
    st7796_draw_text(30, 226, 2, rgb565_be(180, 180, 180), C_BG,
                     "SNAPS YOUR VOICE OR SONG TO THE");
    st7796_draw_text(30, 252, 2, rgb565_be(180, 180, 180), C_BG,
                     "NEAREST NOTE. ROBOTIC SNAPS HARD.");
    st7796_draw_text(30, 290, 2, rgb565_be(150, 150, 150), C_BG,
                     "SING STEADY NOTES. PDM MICS WORK.");
}

// ---- Tuner page: 180-degree flat<->sharp needle gauge ----
// Auto-detects the nearest chromatic note via the pitch engine's probe
// mode. Needle: -50 cents (far left, FLAT) .. +50 (far right, SHARP).
// LED strip: left half fills red when FLAT (tune UP), right half amber when
// SHARP (tune DOWN), whole strip flashes green when within +/-5 cents.
static float tuner_disp  = 0.0f;     // displayed cents (slewed)
static float tuner_drawn = -999.0f;  // cents angle of the needle on screen
static int   tuner_note  = -999;     // midi note shown (-999 none/redraw)
static bool  tuner_flash = false;    // green flash phase

// Draw (or erase, with C_BG) the needle at a given cents deflection.
static void tuner_needle(float cents, uint16_t color) {
    float a  = (90.0f - cents * 1.8f) * 0.0174533f;
    float dx = cosf(a), dy = sinf(a);
    for (int r = 18; r <= TUN_RN; r += 4)
        st7796_fill_rect(TUN_CX + (int)(dx * (float)r) - 1,
                         TUN_CY - (int)(dy * (float)r) - 1, 3, 3, color);
}

static void draw_tuner_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    st7796_draw_text(210, 18, 2, rgb565_be(255, 255, 255), C_BG, "TUNER");
    for (int c = -50; c <= 50; c += 10) {        // tick arc
        float a = (90.0f - (float)c * 1.8f) * 0.0174533f;
        int   s = (c == 0 || c == -50 || c == 50) ? 8 : 4;
        st7796_fill_rect(TUN_CX + (int)(cosf(a) * (float)TUN_RT) - s / 2,
                         TUN_CY - (int)(sinf(a) * (float)TUN_RT) - s / 2,
                         s, s, (c == 0) ? C_GREEN : C_LINE);
    }
    st7796_fill_rect(TUN_CX - 5, TUN_CY - 5, 10, 10, C_CURSOR);   // hub
    st7796_draw_text(40, 258, 2, rgb565_be(180, 180, 180), C_BG, "FLAT");
    st7796_draw_text(390, 258, 2, rgb565_be(180, 180, 180), C_BG, "SHARP");
    st7796_draw_text(60, 300, 1, rgb565_be(150, 150, 150), C_BG,
                     "LEDS: LEFT = TUNE UP   RIGHT = TUNE DOWN   GREEN = IN TUNE");
    tuner_disp = 0.0f; tuner_drawn = -999.0f; tuner_note = -999;
}

// LED guidance: direction + how far off (count), green flash in tune.
static void tuner_leds(bool have, float dev) {
    static int last = -12345;
    int state;
    if (!have) state = 0;
    else if (fabsf(dev) <= 5.0f) { tuner_flash = !tuner_flash;
                                   state = tuner_flash ? 1 : 2; }
    else {
        int mag = 1 + (int)(fabsf(dev) * (7.0f / 50.0f));
        if (mag > 8) mag = 8;
        state = (dev < 0.0f ? 100 : 200) + mag;
    }
    if (state == last) return;
    last = state;
    for (int i = 0; i < FW2_LED_COUNT; i++) {
        rgb_t c = { 0, 0, 0 };
        if (state == 1)        c = (rgb_t){ 0, 60, 0 };       // in tune (flash)
        else if (state >= 200) {                              // sharp: right
            if (i >= 8 && i < state - 200 + 8) c = (rgb_t){ 60, 30, 0 };
        } else if (state >= 100) {                            // flat: left
            if (i < 8 && i >= 8 - (state - 100)) c = (rgb_t){ 60, 0, 0 };
        }
        ws2812_set_pixel((uint)i, c);
    }
    ws2812_show();
}

// ~12 Hz refresh from the main loop while the tuner page is up.
static void tuner_update(void) {
    static const char *const TNN[12] =
        { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
    int hz10, cc;
    fx_tune_status(&hz10, &cc);
    bool have = hz10 > 0;
    int  note = tuner_note;
    float dev = 0.0f;                            // silent: relax to center
    if (have) {
        float midi = 69.0f + 12.0f *
                     log2f((float)hz10 * (0.1f / 440.0f));
        note = (int)floorf(midi + 0.5f);
        dev  = (midi - (float)note) * 100.0f;
        if (dev >  50.0f) dev =  50.0f;
        if (dev < -50.0f) dev = -50.0f;
    }
    tuner_disp += 0.35f * (dev - tuner_disp);
    bool intune = have && fabsf(tuner_disp) <= 5.0f;
    // Note readout (redraws on note change or signal gain/loss).
    int shown = have ? note : -998;
    if (shown != tuner_note) {
        tuner_note = shown;
        char nb[8], *p = nb;
        if (have) {
            const char *nm = TNN[((note % 12) + 12) % 12];
            while (*nm) *p++ = *nm++;
            p = fmt_int(p, note / 12 - 1);       // octave (midi 40 = E2)
        } else { *p++ = '-'; *p++ = '-'; }
        *p = '\0';
        st7796_fill_rect(150, 52, 180, 44, C_BG);
        int nlen = (int)(p - nb);
        st7796_draw_text(240 - nlen * 15, 54, 5,
                         rgb565_be(255, 255, 255), C_BG, nb);
    }
    // Cents readout beside the note.
    static int cents_shown = -999;
    int cnow = (int)(tuner_disp + (tuner_disp >= 0 ? 0.5f : -0.5f));
    if (have && cnow != cents_shown) {
        cents_shown = cnow;
        char cb[8], *p = cb;
        if (cnow >= 0) *p++ = '+';
        p = fmt_int(p, cnow); *p = '\0';
        st7796_fill_rect(340, 62, 80, 24, C_BG);
        st7796_draw_text(340, 62, 2,
                         intune ? C_GREEN : rgb565_be(255, 255, 0), C_BG, cb);
    } else if (!have && cents_shown != -999) {
        cents_shown = -999;
        st7796_fill_rect(340, 62, 80, 24, C_BG);
    }
    // Needle (erase old, draw new; ticks sit beyond the needle radius).
    if (fabsf(tuner_disp - tuner_drawn) > 1.0f) {
        if (tuner_drawn > -900.0f) tuner_needle(tuner_drawn, C_BG);
        tuner_needle(tuner_disp,
                     intune ? C_GREEN : rgb565_be(255, 255, 255));
        tuner_drawn = tuner_disp;
    }
    tuner_leds(have, tuner_disp);
    static uint32_t tdiag = 0;
    if (have && (tdiag++ & 0xF) == 0)
        DIAG("tuner: f=%d.%d Hz note=%d cents=%d\n",
             hz10 / 10, hz10 % 10, note, cnow);
}

// ---- Synth page: touch instrument ----
static void draw_syn_waves(void) {
    int w = synth_wave_get();
    draw_button(&BTN_S_SAW, w == SYNTH_SAW    ? C_GREEN : C_GRAY, 2, "SAW");
    draw_button(&BTN_S_SIN, w == SYNTH_SINE   ? C_GREEN : C_GRAY, 2, "SINE");
    draw_button(&BTN_S_SQR, w == SYNTH_SQUARE ? C_GREEN : C_GRAY, 2, "SQUARE");
}

static bool syn_hold_on = false;             // note latch (hands-free motion)
static void draw_syn_hold(void) {
    draw_button(&BTN_S_HOLD, syn_hold_on ? C_GREEN : C_GRAY, 2,
                syn_hold_on ? "HOLD ON" : "HOLD");
}

// ---- Live waveform trace ----------------------------------------------------
// One trace of the CURRENT waveform drawn across the pad: cycle count follows
// pitch (X) and amplitude follows filter openness (Y), so dragging visibly
// "plays" the shape — and each waveform button shows what its shape looks
// like. Incremental per-column erase/redraw: no flicker, no full clears.
static uint8_t syn_tr_t[SYN_PAD_W], syn_tr_b[SYN_PAD_W];  // segment extents
static bool    syn_tr_valid = false;
static float   syn_tr_x = 0.5f, syn_tr_y = 0.6f;          // last trace params
static bool    syn_gate_ui = false;   // mirrors synth_gate() for the display
static float   syn_tr_ph = 0.0f;      // scroll phase while a note plays
static float   syn_tr_wob = 1.0f;     // tremolo amp wobble while a note plays

static uint16_t syn_tr_color(void) {
    if (syn_gate_ui) return rgb565_be(70, 255, 90);        // playing: green
    switch (synth_wave_get()) {
    case SYNTH_SINE:   return rgb565_be(60, 200, 255);
    case SYNTH_SQUARE: return rgb565_be(90, 255, 130);
    default:           return rgb565_be(255, 150, 40);    // saw
    }
}

static void syn_trace_restore(int px0, int px1) {   // repair after cursor erase
    if (!syn_tr_valid) return;
    uint16_t c = syn_tr_color();
    for (int i = px0 - SYN_PAD_X; i <= px1 - SYN_PAD_X; i++) {
        if (i < 0 || i >= SYN_PAD_W) continue;
        st7796_fill_rect(SYN_PAD_X + i, SYN_PAD_Y + syn_tr_t[i], 1,
                         syn_tr_b[i] - syn_tr_t[i] + 1, c);
    }
}

static void synth_trace(float xn, float yn) {
    syn_tr_x = xn; syn_tr_y = yn;
    int w = synth_wave_get();
    uint16_t color = syn_tr_color();
    float cycles = 2.0f + xn * 10.0f;
    float amp = (0.12f + 0.80f * yn) * (float)(SYN_PAD_H / 2 - 10) * syn_tr_wob;
    int mid = SYN_PAD_Y + SYN_PAD_H / 2;
    int prev = mid;
    for (int i = 0; i < SYN_PAD_W; i++) {
        float ph = cycles * (float)i / (float)SYN_PAD_W + syn_tr_ph;
        float fr = ph - floorf(ph);
        float v = (w == SYNTH_SINE)   ? sinf(6.2831853f * fr)
                : (w == SYNTH_SQUARE) ? (fr < 0.5f ? 1.0f : -1.0f)
                                      : (2.0f * fr - 1.0f);
        int y = mid - (int)(v * amp);
        int t = (i && prev < y) ? prev : y;    // connect to previous column
        int b = (i && prev > y) ? prev : y;
        if (syn_tr_valid)
            st7796_fill_rect(SYN_PAD_X + i, SYN_PAD_Y + syn_tr_t[i], 1,
                             syn_tr_b[i] - syn_tr_t[i] + 1, C_SYNBG);
        st7796_fill_rect(SYN_PAD_X + i, t, 1, b - t + 1, color);
        syn_tr_t[i] = (uint8_t)(t - SYN_PAD_Y);
        syn_tr_b[i] = (uint8_t)(b - SYN_PAD_Y);
        prev = y;
    }
    syn_tr_valid = true;
}

static int syn_cx = -1, syn_cy = -1;         // pad cursor
static void syn_cursor(int x, int y) {
    if (syn_cx >= 0) {
        st7796_fill_rect(syn_cx - 6, syn_cy - 6, 12, 12, C_SYNBG);
        syn_trace_restore(syn_cx - 6, syn_cx + 5);
    }
    syn_cx = x; syn_cy = y;
    if (x >= 0) {
        if (syn_cx < SYN_PAD_X + 6) syn_cx = SYN_PAD_X + 6;
        if (syn_cx > SYN_PAD_X + SYN_PAD_W - 6) syn_cx = SYN_PAD_X + SYN_PAD_W - 6;
        if (syn_cy < SYN_PAD_Y + 6) syn_cy = SYN_PAD_Y + 6;
        if (syn_cy > SYN_PAD_Y + SYN_PAD_H - 6) syn_cy = SYN_PAD_Y + SYN_PAD_H - 6;
        st7796_fill_rect(syn_cx - 6, syn_cy - 6, 12, 12, C_CURSOR);
    }
}

static void draw_synth_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    st7796_draw_text(210, 18, 2, rgb565_be(255, 255, 255), C_BG, "SYNTH");
    draw_syn_hold();
    draw_syn_waves();
    st7796_fill_rect(SYN_PAD_X, SYN_PAD_Y, SYN_PAD_W, SYN_PAD_H, C_SYNBG);
    syn_cx = -1;
    syn_tr_valid = false;                    // pad was cleared
    synth_trace(syn_tr_x, syn_tr_y);         // show the shape immediately
    st7796_draw_text(30, 302, 1, rgb565_be(150, 150, 150), C_BG,
                     "TOUCH TO PLAY  X = PITCH (2.5 OCTAVES)  Y = FILTER OPEN/CLOSE");
    st7796_draw_text(30, 312, 1, rgb565_be(150, 150, 150), C_BG,
                     "HOLD A NOTE: SHAKE=RATTLE  BANK=TREMOLO  TIP=RESONANCE  SNAP TIP=DIVE");
}

// ---- Keys chord-pad page ----
// Latch a chord type on the left (radio), tap a piano key on the right:
// the chord strikes on that root. Chords ring across pages (like drums).
static const char *KEYS_LBL[KEYS_NCHORDS] = {
    "MAJ", "MIN", "DOM7", "MIN7", "MAJ7", "SUS4", "SUS2", "DIM",
};
static int keys_sel = 0;
static const int8_t KEYS_WH_SEMI[8] = { 0, 2, 4, 5, 7, 9, 11, 12 };
static const int8_t KEYS_BK_SEMI[5] = { 1, 3, 6, 8, 10 };
static const int8_t KEYS_BK_POS[5]  = { 1, 2, 4, 5, 6 };

static rect_t keys_chord_rect(int i) {
    // Two evenly spaced columns filling SAFE_L..KB_X (10..174).
    rect_t r = { (i & 1) ? 95 : SAFE_L, 52 + (i >> 1) * 67, 79, 62 };
    return r;
}
static void draw_keys_chordbtns(void) {
    for (int i = 0; i < KEYS_NCHORDS; i++) {
        rect_t r = keys_chord_rect(i);
        draw_button(&r, i == keys_sel ? C_PURPLE : C_GRAY, 2, KEYS_LBL[i]);
    }
}
static void draw_keys_inst(void) {
    draw_button(&BTN_K_INST, C_BLUE, 2,
                keys_instrument_get() == KEYS_EPIANO ? "E.PIANO" : "STRINGS");
}
static void draw_keys_sus(void) {
    draw_button(&BTN_K_SUS, keys_sustain_get() ? C_GREEN : C_GRAY, 2,
                "SUSTAIN");
}
static void draw_keys_piano(void) {
    uint16_t wht = rgb565_be(245, 245, 245);
    for (int i = 0; i < 8; i++) {
        st7796_fill_rect(KB_X + i * WK_W, KB_Y, WK_W - 1, WK_H, wht);
        char l[2] = { "CDEFGABC"[i], 0 };
        st7796_draw_text(KB_X + i * WK_W + 13, KB_Y + WK_H - 26, 2,
                         rgb565_be(40, 40, 60), wht, l);
    }
    for (int k = 0; k < 5; k++)
        st7796_fill_rect(KB_X + KEYS_BK_POS[k] * WK_W - BK_W / 2, KB_Y,
                         BK_W, BK_H, rgb565_be(15, 15, 20));
}
static void draw_keys_fx(void) {
    draw_button(&BTN_K_FX, keys_to_fx ? C_GREEN : C_GRAY, 2, "FX");
}

static void draw_keys_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    draw_keys_fx();
    draw_keys_inst();
    draw_keys_sus();
    draw_keys_chordbtns();
    draw_keys_piano();
}

// Which semitone (0..12) is under (x,y) on the piano, or -1. Black keys
// overlay the top of the whites, so they win inside their strip.
static int keys_semi_at(int x, int y) {
    if (x < KB_X || y < KB_Y) return -1;
    if (y < KB_Y + BK_H)
        for (int k = 0; k < 5; k++) {
            int bx = KB_X + KEYS_BK_POS[k] * WK_W - BK_W / 2;
            if (x >= bx && x < bx + BK_W) return KEYS_BK_SEMI[k];
        }
    int wi = (x - KB_X) / WK_W;
    if (wi > 7) wi = 7;
    return KEYS_WH_SEMI[wi];
}

// Horizontal span of a key, for the hold lock: while a finger owns a
// sounding key it keeps it as long as x stays inside this span — freeing
// the y axis for the slide-up tremolo without striking the key above.
static void keys_span_of(int semi, int *x0, int *x1) {
    for (int k = 0; k < 5; k++)
        if (KEYS_BK_SEMI[k] == semi) {
            *x0 = KB_X + KEYS_BK_POS[k] * WK_W - BK_W / 2;
            *x1 = *x0 + BK_W;
            return;
        }
    for (int wi = 0; wi < 8; wi++)
        if (KEYS_WH_SEMI[wi] == semi) {
            *x0 = KB_X + wi * WK_W;
            *x1 = *x0 + WK_W;
            return;
        }
    *x0 = *x1 = -1;
}

// ---- Theremin page ----
// Cable-free demo #2, two modes:
//   MAGNET: BMM350 field magnitude -> pitch (cube-law proximity of a small
//           magnet in the hand), light -> volume.
//   LIGHT:  light -> pitch (hand shadow over the sensor), volume set on the
//           touch strip (prop-free walk-up mode, the power-up default).
// Both sensor axes map in the log2 domain against self-tracking ranges
// (expand instantly, relax slowly) so room light / ambient field never need
// manual calibration. Gate = finger on the bottom strip; HOLD latches.
static bool  thm_mag_mode = false;
static bool  thm_hold_on  = false;
static bool  thm_gate_ui  = false;
static float thm_touch_vol = 0.8f;    // LIGHT mode: volume set on the strip
static float thm_pit_n = 0.0f, thm_vol_n = 0.0f;   // drawn bar values 0..1
static bool  opt_ok = false, mag_ok = false;       // probed at boot
static float thm_lg_lo, thm_lg_hi;    // light auto-cal range (log2 lux)
static float thm_lg_ema;              // smoothed light reading (poll + CAL)
static bool  thm_lg_manual = false;   // CAL pressed: range is user-anchored
static int   thm_cal_flash = 0;       // CAL button green-flash countdown
// Resting FIELD VECTOR baseline, in uT. Pitch is driven by |m - b|, not by
// |m|: the on-board speaker magnet puts ~1500 uT of static field on the sensor,
// so a hand magnet either reinforces or CANCELS it depending on orientation and
// |m| is not monotonic in distance. See the poll for the measurements.
static float thm_bx, thm_by, thm_bz;
static bool  thm_b_seeded;
static float thm_dev_hi;              // running max of |m - b| (uT), learned
static float thm_dev_ema;             // lightly smoothed deviation
static float thm_dev_prev;            // previous RAW deviation
static float thm_dev_jit;             // mean |change| of raw dev: hand tremor
                                      // (tens of uT) vs sensor noise (1-2 uT)
// Net drift over ~1 s: catches a SLOWLY moving magnet, which has little
// per-sample jitter but a real trend. Kept to 5 bytes on purpose -- this app
// sits just under a 32 KB SRAM alignment boundary, and an earlier version of
// this state (two floats + a uint16) pushed .bss across it, which cost 32 KB
// and overflowed RAM by 16604 bytes. Do not add statics here casually.
static float    thm_dev_lag;
static uint8_t  thm_lag_n;
// Deviation below this reads as zero pitch. Needed because cbrt is very steep
// near the origin: without a floor, a few tens of uT of residual baseline error
// shows as a third of the bar. Subtracted before normalising, so the bar leaves
// zero smoothly rather than jumping.
#define THM_DEV_FLOOR 60.0f
static int   thm_oct = 0;             // octave shift -2..+2 (window base 440)
static bool  thm_lg_seeded = false;
// MAGNET-mode CAL is two-step: 0 = next press learns the zero (magnet
// away), 1 = next press learns the peak (magnet at closest).
// Magnetometer telemetry, read over SWD rather than printed. Two blind fixes
// have not cured "close only reaches 40-50%, then wobbles", so the raw axes
// need looking at -- but a DIAG line's format string pushed .bss over a 32 KB
// alignment cliff in this copy-to-RAM build (link failed by 16568 bytes). Four
// floats cost 16 bytes and the host can sample them as fast as it likes.
// Prime suspect: a strong neodymium magnet railing one BMM350 axis, after which
// the magnitude stops tracking proximity and wanders with orientation.
volatile float dbg_mx, dbg_my, dbg_mz, dbg_mag, dbg_pit;
volatile float dbg_dev;   // |m - baseline| in uT: what drives pitch now

static void draw_thm_mode(void) {
    draw_button(&BTN_TH_MODE, thm_mag_mode ? C_BLUE : C_GRAY, 2,
                thm_mag_mode ? "MAGNET" : "LIGHT");
    // Label stays 3 chars: "CAL0"/"CAL1" was 48 px of glyphs in a 46 px button
    // and spilled into the neighbours (reported from the device). There is no
    // step to show any more -- CAL is a single re-zero in both modes.
    draw_button(&BTN_TH_CAL, C_GRAY, 2, "CAL");
}
static void draw_thm_hold(void) {
    draw_button(&BTN_TH_HOLD, thm_hold_on ? C_GREEN : C_GRAY, 2,
                thm_hold_on ? "HOLD ON" : "HOLD");
}
static void draw_thm_bar(int by, float v, uint16_t c) {
    int w = (int)(v * (float)(THM_BAR_W - 4));
    if (w < 0) w = 0;
    if (w > THM_BAR_W - 4) w = THM_BAR_W - 4;
    st7796_fill_rect(THM_BAR_X + 2, by + 2, w, THM_BAR_H - 4, c);
    st7796_fill_rect(THM_BAR_X + 2 + w, by + 2,
                     THM_BAR_W - 4 - w, THM_BAR_H - 4, C_SYNBG);
}
static void draw_thm_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    draw_thm_mode();      // also draws CAL, whose label carries the step
    draw_thm_hold();
    draw_button(&BTN_TH_OCTM, C_GRAY, 2, "OCT-");
    draw_button(&BTN_TH_OCTP, C_GRAY, 2, "OCT+");
    uint16_t wht = rgb565_be(230, 230, 240);
    if (thm_mag_mode) {
        // Nothing needs teaching now: the zero is a self-adapting field vector
        // and the range learns its own maximum on the first pass. All the player
        // needs to know is that the magnet goes at the bottom right, where the
        // sensor is, and that CAL is only there to re-zero on demand.
        st7796_draw_text(THM_BAR_X, 56, 1, rgb565_be(255, 200, 80), C_BG,
                         "MAGNET AT LOWER RIGHT.  CAL = RE-ZERO, MAGNET AWAY");
    }
    char pl[64];
    int fb = (int)(440.0f * exp2f((float)thm_oct));
    snprintf(pl, sizeof pl, "%s   %d-%d HZ",
             thm_mag_mode ? "PITCH  (MAGNET CLOSER = HIGHER)"
                          : "PITCH  (SHADE THE SENSOR = LOWER)", fb, fb * 2);
    st7796_draw_text(THM_BAR_X, THM_PIT_Y - 16, 1, wht, C_BG, pl);
    st7796_fill_rect(THM_BAR_X, THM_PIT_Y, THM_BAR_W, THM_BAR_H, C_LINE);
    st7796_draw_text(THM_BAR_X, THM_VOL_Y - 16, 1, wht, C_BG,
                     thm_mag_mode ? "VOLUME  (SLIDE ON THE STRIP BELOW)"
                                  : "VOLUME  (SLIDE ON THE STRIP BELOW)");
    st7796_fill_rect(THM_BAR_X, THM_VOL_Y, THM_BAR_W, THM_BAR_H, C_LINE);
    draw_thm_bar(THM_PIT_Y, thm_pit_n, C_CURSOR);
    draw_thm_bar(THM_VOL_Y, thm_vol_n, C_GREEN);
    st7796_fill_rect(0, THM_PAD_Y, 480, 320 - THM_PAD_Y, C_PAD);
    // Both modes now take volume from this strip, so the old magnet-mode
    // caption ("LIGHT = VOLUME") described behaviour that no longer exists.
    // 27 chars at 12 px = 324 px, centred in the usable span SAFE_L..REACH_R
    // so it cannot run past the full-volume marker drawn just below.
    st7796_draw_text(79, THM_PAD_Y + 36, 2, wht, C_PAD,
                     "LEFT = QUIET   RIGHT = LOUD");
    // No full-volume marker. A green band was drawn at REACH_R so the end of
    // the usable range would be visible rather than discovered by failing, but
    // on the device it just read as a distracting stripe across the pad and the
    // player found the end fine without it (reported from the bench).
    if (!opt_ok)
        st7796_draw_text(THM_BAR_X, 60, 1, rgb565_be(255, 80, 80), C_BG,
                         "LIGHT SENSOR ABSENT");
    if (thm_mag_mode && !mag_ok)
        st7796_draw_text(THM_BAR_X + 220, 60, 1, rgb565_be(255, 80, 80), C_BG,
                         "MAGNETOMETER ABSENT");
}

// ---- Parametric EQ page ----
// Handle colors MATCH the physical colored buttons (grey yellow green blue
// red = bands 1..5): press a button, grab that band.
static uint16_t eq_col(int b) {
    switch (b) {
    case 0:  return rgb565_be(170, 170, 170);
    case 1:  return rgb565_be(255, 210, 0);
    case 2:  return rgb565_be(0, 210, 90);
    case 3:  return rgb565_be(70, 130, 255);
    default: return rgb565_be(255, 70, 70);
    }
}

static float eq_col_freq(int col) {          // plot column -> Hz (log axis)
    return 40.0f * exp2f((float)col * (8.229f / (float)EQ_PW));
}                                            // 40 Hz * 2^8.229 = 12 kHz
static int eq_freq_col(float f) {
    return (int)(log2f(f / 40.0f) * ((float)EQ_PW / 8.229f));
}
static int eq_gain_y(float db) {
    return EQ_PY + EQ_PH / 2 - (int)(db * ((float)(EQ_PH / 2) / EQ_DB_RANGE));
}

// dB magnitude of one band at frequency f (RBJ peaking, closed form on the
// UI core — mirrors fx_eq_band's coefficients).
static float eq_band_db(int b, float f) {
    if (eq_g[b] > -0.05f && eq_g[b] < 0.05f) return 0.0f;
    float A  = exp2f(eq_g[b] * (1.0f / 12.041f));
    float w  = 6.2831853f * eq_f[b] / (float)FX_FS;
    float al = sinf(w) / (2.0f * eq_qv[b]);
    float b0 = 1.0f + al * A, b1 = -2.0f * cosf(w), b2 = 1.0f - al * A;
    float a0 = 1.0f + al / A, a2 = 1.0f - al / A;
    float ww = 6.2831853f * f / (float)FX_FS;
    float c1 = cosf(ww), s1 = sinf(ww);
    float c2 = 2.0f * c1 * c1 - 1.0f, s2 = 2.0f * s1 * c1;
    float nr = b0 + b1 * c1 + b2 * c2, ni = b1 * s1 + b2 * s2;
    float dr = a0 + b1 * c1 + a2 * c2, di = b1 * s1 + a2 * s2;
    float m2 = (nr * nr + ni * ni) / (dr * dr + di * di + 1e-12f);
    return 10.0f * log10f(m2 + 1e-12f);
}

static void eq_draw_handles(void) {
    for (int b = 0; b < 5; b++) {
        int x = EQ_PX + eq_freq_col(eq_f[b]);
        int y = eq_gain_y(eq_g[b]);
        if (b == eq_sel)
            st7796_fill_rect(x - 8, y - 8, 16, 16, rgb565_be(255, 255, 255));
        st7796_fill_rect(x - 5, y - 5, 10, 10, eq_col(b));
    }
}

// Full response plot: column sweep (4 px columns), grid + composite curve.
// ~30 ms of SPI — fine at a throttled cadence since audio owns core 1.
static void eq_draw_plot(void) {
    for (int col = 0; col < EQ_PW; col += 4) {
        int x = EQ_PX + col;
        st7796_fill_rect(x, EQ_PY, 4, EQ_PH, rgb565_be(14, 14, 34));
        // grid: 0 dB center + +/-6/12 dB + decade frequency lines
        st7796_fill_rect(x, eq_gain_y(0.0f), 4, 1, C_LINE);
        st7796_fill_rect(x, eq_gain_y(6.0f), 4, 1, rgb565_be(30, 30, 70));
        st7796_fill_rect(x, eq_gain_y(-6.0f), 4, 1, rgb565_be(30, 30, 70));
        float fc = eq_col_freq(col);
        float fn = eq_col_freq(col + 4);
        if ((fc < 100.0f && fn >= 100.0f) || (fc < 1000.0f && fn >= 1000.0f)
            || (fc < 10000.0f && fn >= 10000.0f))
            st7796_fill_rect(x + 2, EQ_PY, 1, EQ_PH, rgb565_be(30, 30, 70));
        float db = 0.0f;
        for (int b = 0; b < 5; b++) db += eq_band_db(b, fc);
        if (db >  EQ_DB_RANGE) db =  EQ_DB_RANGE;
        if (db < -EQ_DB_RANGE) db = -EQ_DB_RANGE;
        st7796_fill_rect(x, eq_gain_y(db) - 1, 4, 3, C_CURSOR);
    }
    eq_draw_handles();
}

static void eq_draw_readout(void) {
    char line[36], *p = line;
    const char *s = BTN_COLOR_NAMES[eq_sel]; while (*s) *p++ = *s++;
    *p++ = ' ';
    p = fmt_int(p, (int)(eq_f[eq_sel] + 0.5f));
    const char *hz = "Hz "; while (*hz) *p++ = *hz++;
    float gv = eq_g[eq_sel];
    if (gv >= 0.0f) *p++ = '+';
    else            { *p++ = '-'; gv = -gv; }
    p = fmt_int(p, (int)gv);
    *p++ = '.';
    p = fmt_int(p, ((int)(gv * 10.0f)) % 10);
    const char *db = "dB Q"; while (*db) *p++ = *db++;
    p = fmt_int(p, (int)eq_qv[eq_sel]);
    *p++ = '.'; p = fmt_int(p, ((int)(eq_qv[eq_sel] * 10.0f)) % 10);
    *p = '\0';
    st7796_fill_rect(0, 246, ST7796_W, 18, C_BG);
    st7796_draw_text(120, 246, 2, eq_col(eq_sel), C_BG, line);
}

static void eq_draw_qslider(void) {
    st7796_fill_rect(0, EQ_SLY - 8, ST7796_W, 30, C_BG);
    st7796_fill_rect(EQ_SLX, EQ_SLY, EQ_SLW, 6, C_LINE);
    // Q 0.4..8 log-mapped onto the track
    float t = log2f(eq_qv[eq_sel] / 0.4f) / 4.322f;      // /log2(20)
    // EQ_SL*, not SLIDER_*. The track, the hit-test and the label had all been
    // moved onto the EQ page's own full-width span while the HANDLE was still
    // positioned off the banded effect-page slider -- so the handle rendered
    // between x=96 and x=384 on a track drawn from 10 to 472: it never reached
    // either end and never sat under the finger that had just set it.
    int hx = EQ_SLX + (int)(t * (float)EQ_SLW);
    if (hx < EQ_SLX + 6) hx = EQ_SLX + 6;
    if (hx > EQ_SLX + EQ_SLW - 6) hx = EQ_SLX + EQ_SLW - 6;
    st7796_fill_rect(hx - 6, EQ_SLY - 8, 12, 24, eq_col(eq_sel));
    // Sits immediately left of the track it labels, 6 px at scale 1.
    st7796_draw_text(EQ_SLX - 8, EQ_SLY - 4, 1, C_LINE, C_BG, "Q");
}

static void draw_eq_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    st7796_draw_text(228, 18, 2, rgb565_be(255, 255, 255), C_BG, "EQ");
    draw_button(&BTN_EQTOG, eq_on ? C_GREEN : C_GRAY, 2, eq_on ? "ON" : "OFF");
    eq_draw_plot();
    eq_draw_readout();
    eq_draw_qslider();
    st7796_draw_text(30, 302, 1, rgb565_be(150, 150, 150), C_BG,
                     "DRAG POINT = FREQ+GAIN   SLIDER = Q   COLORED BUTTONS PICK THE BAND");
}

// ---- Drums page: 16-step sequencer grid ----
static const char *const DRUM_NAMES[DRUM_VOICES] =
    { "KICK", "SNARE", "HH CL", "HH OP", "CLAP" };

static uint16_t drum_col(int v) {
    switch (v) {
    case 0:  return rgb565_be(255, 80, 80);
    case 1:  return rgb565_be(255, 210, 0);
    case 2:  return rgb565_be(0, 210, 90);
    case 3:  return rgb565_be(70, 130, 255);
    default: return rgb565_be(200, 200, 200);
    }
}

static void drum_cell(int v, int st) {
    uint16_t c;
    if (drums_get(v, st)) c = drum_col(v);
    else c = (st % 4 == 0) ? rgb565_be(30, 30, 64) : rgb565_be(16, 16, 40);
    st7796_fill_rect(DR_GX + st * DR_CW, DR_GY + v * DR_RH, DR_CW - 2,
                     DR_RH - 4, c);
}

static void draw_drum_play(void) {
    draw_button(&BTN_D_PLAY, drums_playing() ? C_RED : C_GREEN, 2,
                drums_playing() ? "STOP" : "PLAY");
}

static void draw_drum_bpm(void) {
    char b[12], *p = b;
    p = fmt_int(p, drums_bpm_get());
    const char *t = " BPM";
    while (*t) *p++ = *t++;
    *p = 0;
    st7796_fill_rect(120, 252, 120, 20, C_BG);
    st7796_draw_text(130, 252, 2, rgb565_be(255, 255, 0), C_BG, b);
    st7796_fill_rect(0, 292, ST7796_W, 26, C_BG);
    st7796_fill_rect(SLIDER_X, 300, SLIDER_W, 6, C_LINE);
    int hx = SLIDER_X + (drums_bpm_get() - 60) * SLIDER_W / 124;
    if (hx < SLIDER_X + 6) hx = SLIDER_X + 6;
    if (hx > SLIDER_X + SLIDER_W - 6) hx = SLIDER_X + SLIDER_W - 6;
    st7796_fill_rect(hx - 6, 294, 12, 20, C_CURSOR);
}

// Bank tabs: PURPLE = the bank on the grid (edit), GREEN = the bank the
// chain is sounding right now, gray = in the chain, dark = past its length.
static void draw_drum_banks(void) {
    int g  = drums_playing() ? drums_step_now() : -1;
    int pb = g < 0 ? -1 : g >> 4;
    for (int i = 0; i < DRUM_BANKS; i++) {
        char lbl[2] = { (char)('A' + i), 0 };
        uint16_t bg;
        if (i == drums_bank_get())    bg = C_PURPLE;
        else if (i == pb)             bg = C_GREEN;
        else if (i < drums_len_get()) bg = C_GRAY;
        else                          bg = rgb565_be(34, 34, 48);
        draw_button(&BTN_D_BANK[i], bg, 2, lbl);
    }
    char ll[5] = { 'L', 'E', 'N', (char)('0' + drums_len_get()), 0 };
    draw_button(&BTN_D_LEN, C_GRAY, 2, ll);
}

// SEND TO FX: opt the kit into the effect chain instead of mixing it in
// clean afterwards. Off by default -- the usual case is jamming over a beat.
static void draw_drum_fx(void) {
    draw_button(&BTN_D_FX, drums_to_fx ? C_GREEN : C_GRAY, 2,
                drums_to_fx ? "FX ON" : "FX OFF");
}

static void draw_drums_page(void) {
    st7796_fill_screen(C_BG);
    draw_button(&BTN_BACK, C_GRAY, 2, "< BACK");
    // Centred in the gap between SEND-TO-FX (ends 234) and PLAY (starts 372).
    st7796_draw_text(273, 18, 2, rgb565_be(255, 255, 255), C_BG, "DRUMS");
    draw_drum_play();
    draw_drum_fx();
    for (int v = 0; v < DRUM_VOICES; v++) {
        st7796_draw_text(4, DR_GY + v * DR_RH + 10, 1, drum_col(v), C_BG,
                         DRUM_NAMES[v]);
        for (int st = 0; st < DRUM_STEPS; st++) drum_cell(v, st);
    }
    draw_button(&BTN_D_CLEAR, C_GRAY, 2, "CLEAR");
    draw_drum_banks();
    draw_drum_bpm();
}

// Playhead strip under the grid (called from the UI loop while playing).
static int drum_ph_drawn = -1;
static void draw_drum_playhead(int st) {
    if (drum_ph_drawn >= 0) {
        st7796_fill_rect(DR_GX + drum_ph_drawn * DR_CW, DR_GY + 5 * DR_RH,
                         DR_CW - 2, 6, C_BG);
        for (int v = 0; v < DRUM_VOICES; v++)    // un-highlight the old column
            drum_cell(v, drum_ph_drawn);
    }
    drum_ph_drawn = st;
    if (st >= 0) {
        st7796_fill_rect(DR_GX + st * DR_CW, DR_GY + 5 * DR_RH,
                         DR_CW - 2, 6, C_CURSOR);
        // Cells under the playhead flash white: THESE hits are sounding now.
        for (int v = 0; v < DRUM_VOICES; v++)
            if (drums_get(v, st))
                st7796_fill_rect(DR_GX + st * DR_CW, DR_GY + v * DR_RH,
                                 DR_CW - 2, DR_RH - 4,
                                 rgb565_be(255, 255, 255));
    }
}

// Long-press on a menu button: open that effect's settings page. Effect
// states are untouched — pages just show and edit them.
// Repaint an arbitrary page. Every page previously only knew how to draw
// itself from its own entry point, which meant nothing could return to
// "whatever was showing before". The PAGE button needs exactly that, and so
// would a preset recall or a screen blanker later.
//
// NOTE: this repaints chrome only. It deliberately does not re-run the
// live-arming or fx_reset() that some page entries do -- we are returning to
// a page that was already set up, not entering it fresh.
static void draw_cal_page(void);   // reach-calibration grid, defined below
static void page_repaint(page_t p) {
    switch (p) {
    case PAGE_FXMENU: draw_fxmenu_page(); break;
    case PAGE_FX:     draw_fx_page();     break;
    case PAGE_GLI:    draw_gli_page();    break;
    case PAGE_SLAP:   draw_slap_page();   break;
    case PAGE_MET:    draw_met_page();    break;
    case PAGE_PARA:   draw_para_page();   break;
    case PAGE_OD:     draw_od_page();     break;
#if FX_ENABLE_TREMOLO
    case PAGE_TREM:   draw_trem_page();   break;
#endif
#if FX_ENABLE_PHASER
    case PAGE_PHASE:  draw_phase_page();  break;
#endif
    case PAGE_TUNE:   draw_tune_page();   break;
    case PAGE_TUNER:  draw_tuner_page();  break;
    case PAGE_EQ:     draw_eq_page();     break;
    case PAGE_VERB:   draw_verb_page();   break;
    case PAGE_KNOB:   draw_knob_page();   break;
    case PAGE_SYNTH:  draw_synth_page();  break;
    case PAGE_DRUMS:  draw_drums_page();  break;
    case PAGE_VOX:    draw_vox_page();    break;
    case PAGE_THMN:   draw_thm_page();    break;
    case PAGE_KEYS:   draw_keys_page();   break;
    case PAGE_SETUP:  draw_setup_page();  break;
    case PAGE_CAL:    draw_cal_page();    break;
    case PAGE_VIZ:    viz_open();         break;
    default:          draw_main_page();   break;
    }
    mic_ring_rephase();          // every branch is a full-screen draw
}

// ---- Reach calibration grid ------------------------------------------------
// WHY THIS EXISTS: the first attempt at measuring the enclosure's usable area
// logged only a BOUNDING BOX (min/max x and y) from a finger tracing the
// bezel. That was the wrong instrument. A bounding box says "x reached 479",
// which is true -- somewhere, at some height, while deliberately tracing --
// and says nothing about whether the far right is reachable in a normal grip.
// The user could not reach the right side of the synth pad even though the
// box claimed 479. Reachability is a REGION, not a rectangle.
//
// So: tap every cell you can comfortably hit in your normal grip. Cells light
// up as they register and each one logs its centre. Whatever stays dark is
// out of reach, and the layout gets built from the lit set.
#define CAL_COLS 8
#define CAL_ROWS 5
#define CAL_CW (ST7796_W / CAL_COLS)
#define CAL_CH (320 / CAL_ROWS)
static uint8_t cal_hit[CAL_ROWS][CAL_COLS];

static void draw_cal_cell(int r, int c) {
    int x = c * CAL_CW, y = r * CAL_CH;
    uint16_t bg = cal_hit[r][c] ? rgb565_be(0, 150, 60) : rgb565_be(28, 28, 48);
    st7796_fill_rect(x + 1, y + 1, CAL_CW - 2, CAL_CH - 2, bg);
    char lbl[4]; int k = 0;
    lbl[k++] = (char)('A' + r);
    lbl[k++] = (char)('1' + c);
    lbl[k] = 0;
    st7796_draw_text(x + CAL_CW / 2 - 12, y + CAL_CH / 2 - 8, 2,
                     rgb565_be(235, 235, 245), bg, lbl);
}

static void draw_cal_page(void) {
    st7796_fill_screen(rgb565_be(0, 0, 0));
    for (int r = 0; r < CAL_ROWS; r++)
        for (int c = 0; c < CAL_COLS; c++) draw_cal_cell(r, c);
}

// Returns true if the touch was consumed.
static bool cal_touch(int x, int y) {
    int c = x / CAL_CW, r = y / CAL_CH;
    if (c < 0) c = 0; if (c >= CAL_COLS) c = CAL_COLS - 1;
    if (r < 0) r = 0; if (r >= CAL_ROWS) r = CAL_ROWS - 1;
    if (!cal_hit[r][c]) {
        cal_hit[r][c] = 1;
        draw_cal_cell(r, c);
        int lit = 0;
        for (int i = 0; i < CAL_ROWS; i++)
            for (int j = 0; j < CAL_COLS; j++) lit += cal_hit[i][j];
        DIAG("cal: %c%d hit at (%d,%d)  lit=%d/%d\n",
             'A' + r, c + 1, x, y, lit, CAL_ROWS * CAL_COLS);
    }
    return true;
}

// ---- Volume / mute overlay -------------------------------------------------
// The D-pad changes output level and CANCEL(X) mutes, but both were invisible
// unless you happened to be sitting on the SETUP page: family demo 2026-08-02,
// nobody realised the D-pad did anything at all. Reserving a permanent strip
// on every page is the obvious fix and the wrong one -- the enclosure bezel
// already costs us the left 32 px, and a rail would have to dodge the drum
// grid and the piano. This is a transient centred overlay instead, which
// self-erases by repainting the page under it, the way a TV does. Zero layout
// cost, identical behaviour on every page.
#define VOV_W 320
#define VOV_H 104
#define VOV_X ((480 - VOV_W) / 2)
#define VOV_Y ((320 - VOV_H) / 2)
#define VOV_HOLD_US 1500000
static absolute_time_t vov_t;
static bool vov_showing = false;


static void vol_overlay_show(void) {
    if (page == PAGE_VIZ) return;      // full-screen animation; leave it alone
    const uint16_t frame = rgb565_be(235, 235, 245);
    const uint16_t body  = rgb565_be(20, 20, 55);
    st7796_fill_rect(VOV_X, VOV_Y, VOV_W, VOV_H, frame);
    st7796_fill_rect(VOV_X + 3, VOV_Y + 3, VOV_W - 6, VOV_H - 6, body);
    if (!playing) {
        st7796_draw_text(VOV_X + 84, VOV_Y + 22, 4, rgb565_be(255, 90, 90),
                         body, "MUTED");
        st7796_draw_text(VOV_X + 60, VOV_Y + 68, 1, rgb565_be(180, 180, 200),
                         body, "PRESS X AGAIN TO UNMUTE");
    } else {
        int rank = vol_rank();
        // One segment per step, filled to the current rank and outlined beyond
        // it. Pitch is COMPUTED from VOL_STEP_COUNT: the hardcoded 70/60 pair
        // fitted exactly four segments in the 320 px panel, so adding -9 dB
        // would have run the fifth off the right-hand edge.
        const int sp = (VOV_W - 48) / VOL_STEP_COUNT;
        for (int i = 0; i < VOL_STEP_COUNT; i++) {
            int sx = VOV_X + 24 + i * sp, sy = VOV_Y + 20, sw = sp - 8, sh = 34;
            // Amber for the top step: past the speaker ceiling, jack only.
            uint16_t on = (i == VOL_STEP_COUNT - 1) ? rgb565_be(230, 170, 40)
                                                    : rgb565_be(70, 200, 110);
            if (i <= rank) st7796_fill_rect(sx, sy, sw, sh, on);
            else {
                st7796_fill_rect(sx, sy, sw, sh, rgb565_be(55, 55, 85));
                st7796_fill_rect(sx + 2, sy + 2, sw - 4, sh - 4, body);
            }
        }
        st7796_draw_text(VOV_X + 24, VOV_Y + 68, 2, rgb565_be(255, 255, 255),
                         body, VOL_STEPS[vol_idx].label);
        if (vol_rank() > VOL_ORDER_SPEAKER_MAX)
            st7796_draw_text(VOV_X + 168, VOV_Y + 70, 1,
                             rgb565_be(230, 170, 40), body, "JACK ONLY");
    }
    vov_t = get_absolute_time();
    vov_showing = true;
    // Same hazard as the erase path below: this overlay is 320x104 = 33k pixels,
    // which is well past the ~2 ms "button-sized redraw is safe" threshold in
    // invariant 13. Re-arm rather than assume only FULL-page draws matter.
    mic_ring_rephase();
}

// Called every main-loop pass: erase the overlay once its dwell expires.
static void vol_overlay_task(void) {
    if (!vov_showing) return;
    if (absolute_time_diff_us(vov_t, get_absolute_time()) < VOV_HOLD_US) return;
    vov_showing = false;
    page_repaint(page);
    // INVARIANT 13. page_repaint() is a full-page draw (~50 ms over SPI, about
    // three capture blocks), so the mic pump must be re-armed or the ring's
    // write/read phase slips and every block tears -- permanent fuzz that no
    // effect toggle clears. Reported from the bench as: pressing the d-pad
    // up/down on the DRUMS page glitches the screen and once stopped the beat
    // outright. Drums is the worst case because its grid repaint is the largest.
    mic_ring_rephase();
}

// PAGE button: jump to the visualizer from anywhere, press again to come back
// to exactly where you were. Non-destructive on purpose -- it changes no audio
// state, so it is safe to mash, which is not true of a bypass or preset button.
static page_t viz_prev_page = PAGE_MAIN;
static bool   viz_via_button = false;

static void viz_button_toggle(void) {
    if (page == PAGE_VIZ) {
        if (!viz_via_button) return;        // entered by tile: let BACK own it
        viz_via_button = false;
        page = viz_prev_page;
        page_repaint(page);
        DIAG("page: back to %d (PAGE button)\n", (int)page);
        return;
    }
    viz_prev_page = page;
    viz_via_button = true;
    page = PAGE_VIZ;
    viz_mode_set(2);                        // plasma: the one that draws a crowd
    viz_open();
    mic_ring_rephase();
    DIAG("page: viz (PAGE button, plasma)\n");
}

static void menu_enter_page(int i) {
    switch (i) {
    case 0:  page = PAGE_FX;    draw_fx_page();    DIAG("page: fx pad\n");    break;
    case 1:  knob_cur = KNOB_OCT;    knob_sel = 0; page = PAGE_KNOB;
             draw_knob_page(); DIAG("page: octave\n");   break;
    case 2:  page = PAGE_GLI;   draw_gli_page();   DIAG("page: glitch\n");    break;
    case 3:  page = PAGE_SLAP;  draw_slap_page();  DIAG("page: slap\n");      break;
    case 4:  page = PAGE_MET;   draw_met_page();   DIAG("page: metal\n");     break;
    case 5:  page = PAGE_PARA;  draw_para_page();  DIAG("page: paranoid\n");  break;
    case 6:  page = PAGE_OD;    draw_od_page();    DIAG("page: overdrive\n"); break;
#if FX_ENABLE_TREMOLO
    case 7:  page = PAGE_TREM;  draw_trem_page();  DIAG("page: tremolo\n");   break;
#endif
#if FX_ENABLE_PHASER
    case 8:  page = PAGE_PHASE; draw_phase_page(); DIAG("page: phaser\n");    break;
#endif
    case 9:  page = PAGE_TUNE;  draw_tune_page();  DIAG("page: orcatune\n");  break;
    case 10: knob_cur = KNOB_VERB;   knob_sel = 0; page = PAGE_KNOB;
             draw_knob_page(); DIAG("page: reverb\n");   break;
    // Slot 10 used to be a second TUNER entry. PAGE_TUNER is now reached only
    // from the launcher's quick-access button, which sets the page directly
    // (see BTN_TUNERQ) and uses tuner_from_main to route its BACK.
    case 12: knob_cur = KNOB_CRUSH;  knob_sel = 0;  page = PAGE_KNOB; draw_knob_page();
             DIAG("page: bitcrush\n"); break;
    case 13: knob_cur = KNOB_FLANGE;  knob_sel = 0; page = PAGE_KNOB; draw_knob_page();
             DIAG("page: flanger\n");  break;
#if FX_ENABLE_RINGMOD
    case 14: knob_cur = KNOB_RING;  knob_sel = 0;   page = PAGE_KNOB; draw_knob_page();
             DIAG("page: ringmod\n");  break;
#endif
#if FX_ENABLE_VOWEL
    case 15: knob_cur = KNOB_VOWEL; knob_sel = 0; page = PAGE_KNOB;
             draw_knob_page(); DIAG("page: vowel\n");     break;
#endif
    default:
        page = PAGE_EQ;
        draw_eq_page();
        DIAG("page: para eq\n");
        break;
    }
    mic_ring_rephase();          // every entry above is a full-screen draw
}

// ---- Physical-button navigation state ----
// FX menu D-pad: white-frame highlight, CENTER toggles, CENTER-hold opens
// the settings page. Every one of the sixteen has real controls now — OCTAVE
// was the last on/off-only effect and gained MODE/MIX/DETUNE.
static int  menu_cur = -1;                  // highlighted menu slot
static bool menu_center_dn = false;
static absolute_time_t menu_center_t;
static const uint8_t MENU_HAS_PAGE[MENU_N] = {
    1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1,
};
// VOX colored-button slots: tap = select, hold = record (touch parity).
// Set when a page is entered by a touch that would also land on that page's
// play surface; cleared once the finger comes up. See the KEYS tile handler.
static bool keys_need_release = false;
static int  kbtn_slot = -1;
static absolute_time_t kbtn_slot_t;

static void draw_menu_cursor(int i) {
    if (i < 0) return;
    const rect_t *r = MENU_RECTS_AT(i);
    uint16_t c = rgb565_be(255, 255, 255);
    st7796_fill_rect(r->x, r->y, r->w, 3, c);
    st7796_fill_rect(r->x, r->y + r->h - 3, r->w, 3, c);
    st7796_fill_rect(r->x, r->y, 3, r->h, c);
    st7796_fill_rect(r->x + r->w - 3, r->y, 3, r->h, c);
}

// Navigation happens in GRID space and is mapped back through MENU_GRID,
// because the index numbering deliberately does not match the visual order.
static void menu_cur_move(int dc, int dr) {
    int cell = 0;
    if (menu_cur >= 0)
        for (int k = 0; k < MENU_N; k++)
            if (MENU_GRID[k] == menu_cur) { cell = k; break; }
    int col = cell % MENU_COLS, row = cell / MENU_COLS;
    col += dc; row += dr;
    if (col < 0) col = 0;
    if (col > MENU_COLS - 1) col = MENU_COLS - 1;
    if (row < 0) row = 0;
    if (row > MENU_ROWS - 1) row = MENU_ROWS - 1;
    if (menu_cur >= 0) draw_menu_btn(menu_cur);  // erase old frame
    menu_cur = MENU_GRID[row * MENU_COLS + col];
    draw_menu_cursor(menu_cur);
}

// HOME: one level UP (bench feedback: a physical back). Effect settings
// pages return to the FX menu; everything else returns to the launcher —
// so from deep in an effect, HOME-HOME walks all the way home. Exit
// hygiene mirrors each page's BACK.
static void go_home(void) {
    switch (page) {
    case PAGE_FX: case PAGE_GLI: case PAGE_SLAP:
    case PAGE_MET: case PAGE_PARA: case PAGE_OD: case PAGE_TREM:
    case PAGE_PHASE: case PAGE_TUNE: case PAGE_EQ: case PAGE_VERB:
    case PAGE_KNOB:
        page = PAGE_FXMENU;
        draw_fxmenu_page();
        mic_ring_rephase();
        DIAG("page: fx menu (HOME)\n");
        return;
    case PAGE_TUNER:
        fx_tune_probe(0);
        ws2812_clear(); ws2812_show();
        if (!tuner_from_main) {          // entered from the FX menu
            page = PAGE_FXMENU;
            draw_fxmenu_page();
            mic_ring_rephase();
            DIAG("page: fx menu (HOME)\n");
            return;
        }
        tuner_from_main = false;
        break;
    case PAGE_SYNTH:
    case PAGE_THMN:
        synth_gate(0);
        synth_level(1.0f);
        syn_gate_ui = false;
        thm_gate_ui = false;
        break;
    case PAGE_KEYS:
        keys_release();
        keys_vibrato(0.0f);
        keys_reset_req = true;
        break;
    case PAGE_VOX:
        if (vox_recording()) vox_rec_stop();
        vox_play_release();
        kbtn_slot = -1;
        break;
    case PAGE_MAIN:
        return;
    default:
        break;
    }
    page = PAGE_MAIN;
    draw_main_page();
    mic_ring_rephase();
    DIAG("page: main (HOME)\n");
}

// D-pad + CENTER on effect settings pages: CENTER toggles the effect,
// LEFT/RIGHT rides its knob (UP/DOWN too where a page has a second axis
// — EQ gain, pitch scale/voice). Returns true when handled so the
// caller's global volume fallback stays out of the way.
static bool fx_page_btn(int btn) {
    int lr = (btn == UARTKBD_BTN_NAV_RIGHT) ? 1
           : (btn == UARTKBD_BTN_NAV_LEFT)  ? -1 : 0;
    int ud = (btn == UARTKBD_BTN_NAV_UP)    ? 1
           : (btn == UARTKBD_BTN_NAV_DOWN)  ? -1 : 0;
    bool c = (btn == UARTKBD_BTN_NAV_CENTER);
    switch (page) {
    case PAGE_GLI:
        if (c) { gli_on = !gli_on; fx_glitch_set(gli_on); draw_gli_toggle(); return true; }
        if (lr) {
            gli_amt += lr;
            if (gli_amt < 1) gli_amt = 1;
            if (gli_amt > 10) gli_amt = 10;
            fx_glitch_freq(gli_amt); draw_gli_slider(); return true;
        }
        return false;
    case PAGE_SLAP:
        if (c) { slap_on = !slap_on; fx_delay_set(slap_on);
                 draw_slap_toggle(); return true; }
        if (ud) {                        // pick which knob the slider edits
            dly_sel = (dly_sel - ud + 3) % 3;   // UP moves left along the row
            draw_dly_tabs();
            draw_slap_slider();
            return true;
        }
        if (lr) {
            if (dly_sel == 0) {
                dly_ms += lr * 10;
                if (dly_ms < 40)  dly_ms = 40;
                if (dly_ms > 330) dly_ms = 330;
            } else if (dly_sel == 1) {
                dly_fb_amt += lr;
                if (dly_fb_amt < 0)  dly_fb_amt = 0;
                if (dly_fb_amt > 10) dly_fb_amt = 10;
            } else {
                dly_mix_amt += lr;
                if (dly_mix_amt < 0)  dly_mix_amt = 0;
                if (dly_mix_amt > 10) dly_mix_amt = 10;
            }
            dly_apply();
            draw_slap_slider();
            return true;
        }
        return false;
    case PAGE_MET:
        if (c) { met_on = !met_on; fx_metal_set(met_on); draw_met_toggle(); return true; }
        if (lr) {
            met_amt += lr * 5;
            if (met_amt < 5) met_amt = 5;
            if (met_amt > 60) met_amt = 60;
            fx_metal_drive(met_amt); draw_met_slider(); return true;
        }
        return false;
    case PAGE_PARA:
        if (c) { para_on = !para_on; fx_para_set(para_on); draw_para_toggle(); return true; }
        if (lr) {
            para_rng += lr;
            if (para_rng < 0) para_rng = 0;
            if (para_rng > 10) para_rng = 10;
            fx_para_range(para_rng); draw_para_slider(); return true;
        }
        return false;
    case PAGE_OD:
        if (c) { od_on = !od_on; fx_od_set(od_on); draw_od_toggle(); return true; }
        if (lr) {
            od_amt += lr;
            if (od_amt < 1) od_amt = 1;
            if (od_amt > 10) od_amt = 10;
            fx_od_drive(od_amt); draw_od_slider(); return true;
        }
        return false;
    case PAGE_KNOB: {
        const knob_page_t *p = &KNOB_PAGES[knob_cur];
        if (c) { *p->on = !*p->on; p->set(*p->on);
                 draw_knob_toggle(); return true; }
        if (ud && p->n > 1) {              // pick which knob the slider edits
            knob_sel = (knob_sel - ud + p->n) % p->n;
            draw_knob_tabs();
            draw_knob_slider();
            return true;
        }
        if (lr) {
            const knob_t *k = &p->k[knob_sel];
            // Step ~1/20 of the range, min 1: a 0..10 knob still moves in ones,
            // but a 0..100 percentage does not need 100 button presses to cross.
            int span = k->hi_v - k->lo_v;
            int step = span > 20 ? span / 20 : 1;
            int v = *k->value + lr * step;
            if (v < k->lo_v) v = k->lo_v;
            if (v > k->hi_v) v = k->hi_v;
            if (v != *k->value) { *k->value = v; k->apply(v);
                                  draw_knob_slider(); }
            return true;
        }
        return false;
    }
    case PAGE_VERB:
        if (c) { verb_on = !verb_on; fx_verb_set(verb_on);
                 draw_verb_toggle(); return true; }
        if (lr) {
            verb_mix_amt += lr;
            if (verb_mix_amt < 0)  verb_mix_amt = 0;
            if (verb_mix_amt > 10) verb_mix_amt = 10;
            fx_verb_mix(verb_mix_amt); draw_verb_slider(); return true;
        }
        return false;
#if FX_ENABLE_TREMOLO
    case PAGE_TREM:
        if (c) { trem_on = !trem_on; fx_trem_set(trem_on);
                 draw_trem_toggle(); return true; }
        if (lr) {
            trem_amt += lr;
            if (trem_amt < 1) trem_amt = 1;
            if (trem_amt > 10) trem_amt = 10;
            fx_trem_rate(trem_amt); draw_trem_slider(); return true;
        }
        return false;
#endif
#if FX_ENABLE_PHASER
    case PAGE_PHASE:
        if (c) { ph_on = !ph_on; fx_phase_set(ph_on); draw_phase_toggle(); return true; }
        if (lr) {
            ph_amt += lr;
            if (ph_amt < 1) ph_amt = 1;
            if (ph_amt > 10) ph_amt = 10;
            fx_phase_rate(ph_amt); draw_phase_slider(); return true;
        }
        return false;
#endif
    case PAGE_TUNE:
        if (c) { tune_on = !tune_on; fx_tune_set(tune_on); draw_tune_toggle(); return true; }
        if (lr) {
            tune_key = (tune_key + (lr > 0 ? 1 : 11)) % 12;
            fx_tune_key(tune_key); draw_tune_ctls(); return true;
        }
        if (ud > 0) {
            tune_scale = (tune_scale + 1) % 9;
            fx_tune_scale(tune_scale); draw_tune_ctls(); return true;
        }
        if (ud < 0) {
            tune_robot = !tune_robot;
            fx_tune_mode(tune_robot); draw_tune_ctls(); return true;
        }
        return false;
    case PAGE_EQ:
        if (c) {
            eq_on = !eq_on;
            fx_eq_set(eq_on);
            draw_button(&BTN_EQTOG, eq_on ? C_GREEN : C_GRAY, 2,
                        eq_on ? "ON" : "OFF");
            return true;
        }
        if (lr) {           // selected band: freq, ~one semitone per press
            eq_f[eq_sel] *= (lr > 0) ? 1.09f : (1.0f / 1.09f);
            if (eq_f[eq_sel] < 40.0f) eq_f[eq_sel] = 40.0f;
            if (eq_f[eq_sel] > 16000.0f) eq_f[eq_sel] = 16000.0f;
            fx_eq_band(eq_sel, eq_f[eq_sel], eq_g[eq_sel], eq_qv[eq_sel]);
            eq_dirty = true;
            return true;
        }
        if (ud) {           // selected band: gain, 0.5 dB per press
            eq_g[eq_sel] += 0.5f * (float)ud;
            if (eq_g[eq_sel] < -12.0f) eq_g[eq_sel] = -12.0f;
            if (eq_g[eq_sel] > 12.0f) eq_g[eq_sel] = 12.0f;
            fx_eq_band(eq_sel, eq_f[eq_sel], eq_g[eq_sel], eq_qv[eq_sel]);
            eq_dirty = true;
            return true;
        }
        return false;
    default:
        return false;
    }
}

// ---- Core-1 audio pump ------------------------------------------------------
// The ENTIRE signal path runs here: capture block -> (PDM fill) -> sampler ->
// 10-effect chain -> mic TX ring. Core 0 owns everything else (display,
// touch, LEDs, RTT) and can stall for tens of ms without audio noticing.
// WHY: capture is a LOSSY ping-pong with a one-block deadline (5.2 ms at
// 48.8 kHz) and the TX ring phase-slips when the writer stalls — sharing a
// loop with UI drawing produced intermittent static/crackle/fuzz from day
// one (worked around with rephase calls until 2026-07-17). A dedicated core
// makes the deadline unconditional. RTT is NOT multicore-safe: this core
// never DIAGs — it exports meters via the pump_* mirrors.
static void audio_pump_block(void) {
    if (!audio_capture_block_ready()) return;
    const uint32_t *b = audio_capture_block();
    if (!b) return;
    uint32_t t_busy0 = time_us_32();   // load meter: time spent per block
    pump_blocks++;
    uint16_t vupk = vu_peak(b, AUDIO_CAPTURE_BLOCK_FRAMES, AUDIO_MIC_I2S_SLOT);
    if (use_pdm && playing && mic)
        vupk = (uint16_t)(pdm_vu > 65535u ? 65535u : pdm_vu);
    pump_vu    = vupk;
    pump_raw_l = vu_peak(b, AUDIO_CAPTURE_BLOCK_FRAMES, 0);
    pump_raw_r = vu_peak(b, AUDIO_CAPTURE_BLOCK_FRAMES, 1);
    // Pop hunter v2: track the largest sample-to-sample step per block and
    // export a rolling per-second maximum (pop_maxd) so the click's ADC
    // footprint is visible without threshold guessing; timestamp blocks
    // whose step exceeds pop_thresh (line-out recording measured the click
    // at ~1000 counts, so default 600).
    {
        static int16_t pd_prev = 0;
        int mx = 0;
        for (int i = 0; i < AUDIO_CAPTURE_BLOCK_FRAMES; i++) {
            int16_t s = (int16_t)(b[i] & 0xFFFF);
            int d = (int)s - (int)pd_prev;
            if (d < 0) d = -d;
            if (d > mx) mx = d;
            pd_prev = s;
        }
        if ((uint32_t)mx > pop_maxd) pop_maxd = (uint32_t)mx;
        if (mx > 600) { pop_blk[pop_n & 7u] = pump_blocks; pop_n++; }
    }
    if (!(playing && mic)) {           // tone/stopped: input peak
        pump_vu_out = vupk;
        pump_vu_avg = vupk >> 1;       // rough proxy; idle isn't the show
        pump_busy_us += time_us_32() - t_busy0;
        return;
    }
    // Extract mono, then run the WHOLE effect chain in series every block,
    // regardless of page. Each stage slews to bit-transparent when off, so
    // inactive effects cost only flops and toggling is click-free. Order:
    // pitch first, then drive/filter, then modulation, then time effects.
    // All stages are per-sample feed-forward: in-place is safe.
    static int16_t m_buf[AUDIO_CAPTURE_BLOCK_FRAMES];
    if (use_pdm) {
        // PDM array source: the codec block is only the pacer; audio comes
        // from the (rate-decoupled, upsampled) PDM FIFO.
        pdm_pop_block(m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    } else {
        for (int i = 0; i < AUDIO_CAPTURE_BLOCK_FRAMES; i++)
            m_buf[i] = (int16_t)(b[i] & 0xFFFF);
    }
    // Idle noise gate on the LIVE input: +24.5 dB of analog front-end gain
    // (GUITAR profile) makes the ADC floor audible on headphones between
    // notes. Below ~-60 dBFS the input fades out over ~300 ms — slow
    // enough that guitar decay tails ride it down instead of being cut.
    {
        static float g_env = 0.0f, g_gain = 1.0f;
        uint32_t pk_raw = 0, pk = 0;
        uint64_t sq = 0;
        for (int i = 0; i < AUDIO_CAPTURE_BLOCK_FRAMES; i++) {
            uint32_t araw = (uint32_t)(m_buf[i] < 0 ? -(int32_t)m_buf[i]
                                                    :  (int32_t)m_buf[i]);
            if (araw > pk_raw) pk_raw = araw;
            float ax = (float)araw * (1.0f / 32768.0f);
            g_env += (ax > g_env ? 0.02f : 0.0002f) * (ax - g_env);
            // ~-53 dBFS: raised from -60 after bench test — resting fingers
            // on the strings was enough to hold the gate open.
            float tgt = g_env > 0.0022f ? 1.0f : 0.0f;
            g_gain += (tgt > g_gain ? 0.004f : 0.00007f) * (tgt - g_gain);
            m_buf[i] = (int16_t)((float)m_buf[i] * g_gain);
            // Level of the signal the FX chain actually receives.
            uint32_t ag = (uint32_t)(m_buf[i] < 0 ? -(int32_t)m_buf[i]
                                                  :  (int32_t)m_buf[i]);
            if (ag > pk) pk = ag;
            sq += (uint64_t)ag * (uint64_t)ag;
        }
        if (pk_raw > inlvl_pk_raw) inlvl_pk_raw = pk_raw;
        if (pk > inlvl_pk)         inlvl_pk = pk;
        inlvl_sumsq += sq;
        inlvl_n     += AUDIO_CAPTURE_BLOCK_FRAMES;
    }
    // Synth replaces the live signal on its page (its output then runs the
    // whole FX chain — saw through the phaser/EQ/glitch is the point).
    // The theremin page plays the same voice (sine, level from its sensors).
    if (page == PAGE_SYNTH || page == PAGE_THMN)
        synth_process(m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    // Sampler replaces the live signal — input feeds the recorder, output is
    // turntable playback (still through the FX chain). Runs on the sample
    // page and anywhere while HOLD keeps a loop going.
    else if (page == PAGE_VOX || vox_playing())
        vox_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    // Opt-in pre-chain mix: with SEND TO FX on, the kit or the keys join the
    // signal HERE so the whole chain treats them like the live input. Mixed
    // before the corrector so even pitch correction sees them, which is the point
    // of asking for it. Default is off -- see the post-chain mix below.
    if (drums_active() && drums_to_fx)
        drums_process(m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    if (keys_active() && keys_to_fx)
        keys_process(m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    // Pitch correction leads: pitch detection wants the cleanest signal.
    fx_tune_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    fx_octave_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    fx_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    // Envelope filter runs BEFORE the gain stages: it tracks the raw
    // dynamics of the pickup, which is where a wah/envelope filter belongs in
    // a pedal chain. Behind a distortion its follower sees an already
    // compressed signal and barely sweeps.
    fx_para_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    // Gain stages, lowest to highest: OD then SHRED.
    fx_od_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    fx_metal_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    fx_crush_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    fx_eq_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);   // rack EQ:
#if FX_ENABLE_PHASER
    fx_phase_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES); // post-drive
#endif
#if FX_ENABLE_TREMOLO
    fx_trem_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
#endif
    fx_flange_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
#if FX_ENABLE_RINGMOD
    fx_ring_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
#endif
    fx_delay_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    fx_glitch_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
#if FX_ENABLE_VOWEL
    fx_vowel_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
#endif
    // Drums and keys mix POST-chain BY DEFAULT: the kit stays clean while the
    // live input (or synth) is effected, which is what you want when the point
    // is jamming over a beat. Each can opt IN to the chain instead -- see the
    // SEND TO FX buttons on their pages and the pre-chain mix above.
    if (drums_active() && !drums_to_fx)
        drums_process(m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    if (keys_active() && !keys_to_fx)
        keys_process(m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    // REVERB sits here, after the kit and the keys are mixed in, because it is
    // the one effect a player expects across the whole mix rather than on the
    // guitar alone. Everything above it is per-signal; this is a master send.
    fx_verb_process(m_buf, m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    // SPEAKER PROTECTION: the onboard micro-speaker is rated 300 mW
    // (500 mW absolute max, 8 ohm) with a passband of ~1.7-20 kHz — content
    // below self-resonance produces raw cone excursion (no acoustic load),
    // which both sounds broken and mechanically damages the driver. When
    // output routes to the speaker, strip the lows with a 2nd-order
    // Butterworth HP @ 900 Hz (jack path stays full-range). This works WITH
    // the -12 dB volume cap (~<70 mW continuous worst-case).
    if (spk_route) {
        static float hz1 = 0.0f, hz2 = 0.0f;
        for (int i = 0; i < AUDIO_CAPTURE_BLOCK_FRAMES; i++) {
            float x = (float)m_buf[i];
            float y = 0.921366f * x + hz1;
            hz1 = -1.842733f * x - (-1.836540f) * y + hz2;
            hz2 =  0.921366f * x - 0.848926f * y;
            if (y >  32700.0f) y =  32700.0f;
            if (y < -32700.0f) y = -32700.0f;
            m_buf[i] = (int16_t)y;
        }
    }
    {   // Output meter tap: the block as it goes to the DAC.
        uint16_t opk = 0;
        uint32_t oacc = 0;
        for (int i = 0; i < AUDIO_CAPTURE_BLOCK_FRAMES; i++) {
            int v = m_buf[i];
            if (v < 0) v = -v;
            if (v > opk) opk = (uint16_t)v;
            oacc += (uint32_t)v;
            if (v >= 32700) outlvl_clip++;   // sitting on an fx stage's limit
        }
        pump_vu_out = opk;
        pump_vu_avg = (uint16_t)(oacc / AUDIO_CAPTURE_BLOCK_FRAMES);
        if (opk > outlvl_pk) outlvl_pk = opk;
    }
    if (page == PAGE_VIZ)                    // visualizer eats the DAC mix
        viz_feed(m_buf, AUDIO_CAPTURE_BLOCK_FRAMES);
    if (!mic_armed) {
        // Arm: reader starts at slot 0 (silence); write ONE slot ahead. The
        // same-SM rate lock keeps the separation forever (~10.5 ms TX lead).
        memset(mic_ring, 0, sizeof mic_ring);
        mic_write_slot(m_buf, 1);
        tx_play(mic_ring, MIC_SLOTS * AUDIO_CAPTURE_BLOCK_FRAMES);
        mic_armed = true; mic_slot = 2;
    } else {
        mic_write_slot(m_buf, mic_slot);
        mic_slot = (mic_slot + 1) % MIC_SLOTS;
    }
    pump_busy_us += time_us_32() - t_busy0;
}

static void core1_pump(void) {
    for (;;) {
        if (use_pdm && playing) pdm_pump();  // keep the decimator drained
        audio_pump_block();
        tight_loop_contents();
    }
}
// Dedicated core-1 stack: the detector's staging buffers need more than the
// 2 KiB SDK default.
static uint32_t core1_stack[1024];           // 4 KiB
static uint32_t rails_changes;               // observed rail transitions


// ---- Boot-seeded demo sample: a synthesized dial-up sequence --------------
// WHY SYNTHESIZED rather than an embedded recording:
//   1. Sampler slots live in PSRAM, which is VOLATILE. A "factory" sample
//      cannot be stored in a slot at all -- it has to be regenerated every
//      boot, so there is no persistence to be had from shipping audio data.
//   2. This app links COPY-TO-RAM, so a plain const array would be pulled into
//      SRAM (the linker script says so explicitly: "segments not marked as
//      .flashdata are instead pulled into .data"). A few hundred KB of audio
//      would not fit in the ~16 KB free. It would need __in_flash() placement.
//   3. Provenance. Every tone here is a published telecom standard -- the
//      350+440 Hz dial tone, the DTMF pairs, the V.22bis 2225 Hz answer and the
//      V.25 2100 Hz answer tone -- so generating them is clean for a public
//      repo. Embedding somebody's recording of a modem is not.
// Structure and timings were MEASURED off a reference the bench picked (dial
// tone 352+445 for ~0.98 s; DTMF at 100 ms on / 100 ms off; carriers stepping
// 2225 -> 1183 -> 1850; then multi-tone textures; then the alternating
// high/low "ping pong" just before the noise). The bench's two corrections are
// baked in: the fuzz bed starts only AFTER dialling completes, and each carrier
// SWELLS ("brrrr" -> "RRRINNGGG") while the ping-pong answers each swell with a
// sharp low stab for contrast.
#define SM_SLOT 0                       /* GRAY */
static int16_t *sm_buf; static unsigned sm_n, sm_cap; static float sm_peak;
static uint32_t sm_rng = 0x1234567u;
static inline float sm_noise(void) {
    sm_rng ^= sm_rng << 13; sm_rng ^= sm_rng >> 17; sm_rng ^= sm_rng << 5;
    return (float)(int32_t)sm_rng * (1.0f / 2147483648.0f);
}
// Provisional scale: worst case pre-envelope amplitude is ~2.0 (a note plus its
// rasp partials), so 8000 keeps pass 1 clear of the int16 ceiling. Pass 2 then
// scales to the final target using the measured peak.
static void sm_put(float v) {
    if (sm_n >= sm_cap) return;
    float a = v < 0.0f ? -v : v;
    if (a > sm_peak) sm_peak = a;
    float s = v * 8000.0f;
    if (s >  32700.0f) s =  32700.0f;
    if (s < -32700.0f) s = -32700.0f;
    sm_buf[sm_n++] = (int16_t)s;
}
static void sm_gap(float dur) {
    unsigned n = (unsigned)(dur * (float)FX_FS);
    while (n--) sm_put(0.0f);
}
// Clean dual/multi tone (dial tone and DTMF: no buzz, no swell).
static void sm_clean(float dur, const float *f, int nf, float gain) {
    unsigned n = (unsigned)(dur * (float)FX_FS);
    unsigned k = (unsigned)(0.004f * (float)FX_FS);
    for (unsigned i = 0; i < n; i++) {
        float t = (float)i / (float)FX_FS, y = 0.0f;
        for (int j = 0; j < nf; j++) y += sinf(6.2831853f * f[j] * t);
        y /= (float)nf;
        if (k && i < k)         y *= 0.5f - 0.5f * cosf(3.14159265f * (float)i / (float)k);
        if (k && i + k >= n)    y *= 0.5f - 0.5f * cosf(3.14159265f * (float)(n - i) / (float)k);
        sm_put(y * gain);
    }
}
// A carrier note. sharp==0: swells from 10% to full while the amplitude flutter
// and the raspy partials die away (brrrr -> RING). sharp==1: 1.5 ms attack and
// an exponential decay -- the low stab that answers a swell.
static void sm_note(float dur, const float *f, int nf, int sharp, float gain,
                    float buzz, float am, float rasp, float tau) {
    unsigned n = (unsigned)(dur * (float)FX_FS);
    for (unsigned i = 0; i < n; i++) {
        float t = (float)i / (float)FX_FS;
        float p = t / dur;
        float inv = 1.0f - p;
        float m = sharp ? rasp * expf(-t / (tau * 0.6f))
                        : rasp * inv * inv * (inv > 0.0f ? 1.0f : 0.0f);
        if (!sharp) m = rasp * inv * inv;      /* (1-p)^2, close to the ^1.5 used in design */
        float y = 0.0f;
        for (int j = 0; j < nf; j++) {
            float w = 6.2831853f * f[j] * t;
            y += sinf(w);
            if (m > 0.02f)                      /* skip the partials once they fade */
                y += m * (0.5f * sinf(2.0f * w) + 0.33f * sinf(3.0f * w)
                        + 0.25f * sinf(4.0f * w));
        }
        y /= (float)nf;
        if (!sharp) {
            float d = buzz * inv * inv;                       /* the brrrr */
            y *= 1.0f - d * (0.5f + 0.5f * sinf(6.2831853f * am * t));
            y *= 0.10f + 0.90f * p * p;                       /* steep swell */
        } else {
            float atk = t / 0.0015f; if (atk > 1.0f) atk = 1.0f;
            y *= atk * expf(-t / tau);
        }
        sm_put(y * gain);
    }
}
static void seed_modem_sample(void) {
    sm_buf = vox_seed_begin(SM_SLOT, &sm_cap);
    if (!sm_buf) return;
    sm_n = 0; sm_peak = 0.0f;
    absolute_time_t t0 = get_absolute_time();
    static const float DIAL[2] = { 350.0f, 440.0f };
    /* DTMF row+col pairs for 7-0-2-3-4-0-0 (measured off the reference) */
    static const float DG[7][2] = {
        { 852.0f, 1209.0f }, { 941.0f, 1336.0f }, { 697.0f, 1336.0f },
        { 697.0f, 1477.0f }, { 770.0f, 1209.0f }, { 941.0f, 1336.0f },
        { 941.0f, 1336.0f },
    };
    static const float C1[1] = { 2225.0f };          /* V.22bis answer      */
    static const float C2[1] = { 1183.0f };          /* originate carrier   */
    static const float C3[1] = { 1850.0f };
    static const float T1[3] = { 2100.0f, 1100.0f, 950.0f };
    static const float T2[4] = { 1850.0f, 1700.0f, 1600.0f, 1075.0f };
    static const float PING[2] = { 2400.0f, 1800.0f };
    static const float PONG[3] = { 300.0f, 450.0f, 600.0f };
    static const float TAIL[3] = { 2400.0f, 1250.0f, 1000.0f };
    sm_clean(0.85f, DIAL, 2, 0.55f);
    sm_gap(0.10f);
    for (int d = 0; d < 7; d++) { sm_clean(0.10f, DG[d], 2, 0.95f); sm_gap(0.10f); }
    unsigned dial_done = sm_n;              /* fuzz starts HERE, not before */
    sm_gap(0.30f);
    sm_note(0.45f, C1, 1, 0, 0.85f, 0.85f, 45.0f, 0.55f, 0.07f);
    sm_note(0.55f, C2, 1, 0, 0.85f, 0.85f, 38.0f, 0.55f, 0.07f);
    sm_note(0.75f, C3, 1, 0, 0.90f, 0.85f, 52.0f, 0.55f, 0.07f);
    sm_note(0.80f, T1, 3, 0, 0.95f, 0.70f, 41.0f, 0.55f, 0.07f);
    sm_note(0.80f, T2, 4, 0, 0.95f, 0.70f, 60.0f, 0.55f, 0.07f);
    for (int i = 0; i < 3; i++) {           /* PING swells, PONG stabs */
        sm_note(0.40f, PING, 2, 0, 1.00f, 0.85f, 70.0f, 0.60f, 0.07f);
        sm_note(0.22f, PONG, 3, 1, 1.00f, 0.00f,  0.0f, 0.90f, 0.055f);
        sm_gap(0.06f);
    }
    sm_note(0.35f, TAIL, 3, 0, 0.90f, 0.60f, 48.0f, 0.55f, 0.07f);
    // Pass 2: add the band-limited fuzz bed after dialling, and normalise.
    // Bed is a 300 Hz..3 kHz band of noise (two one-poles differenced), eased
    // in so it does not click on.
    float lo = 0.0f, hi = 0.0f, ease = 0.0f;
    float g = (sm_peak > 0.0001f) ? (0.55f / sm_peak) : 1.0f;
    for (unsigned i = 0; i < sm_n; i++) {
        float nz = sm_noise();
        lo += 0.30f  * (nz - lo);
        hi += 0.021f * (nz - hi);
        float tgt = (i >= dial_done) ? 1.0f : 0.0f;
        ease += 0.0010f * (tgt - ease);
        float v = (float)sm_buf[i] * (1.0f / 8000.0f);
        v = (v + 0.10f * (lo - hi) * ease) * g;
        float s = v * 32767.0f;
        if (s >  32700.0f) s =  32700.0f;
        if (s < -32700.0f) s = -32700.0f;
        sm_buf[i] = (int16_t)s;
    }
    vox_seed_commit(SM_SLOT, sm_n);
    DIAG("vox: seeded slot %d with %u samples (%u.%u s) in %lld us\n",
         SM_SLOT, sm_n, sm_n / 48828u, (sm_n * 10u / 48828u) % 10u,
         absolute_time_diff_us(t0, get_absolute_time()));
}


// ---- Boot-seeded demo sample 2: robotic singing voice ---------------------
// Unlike the modem sequence above (synthesised on the device), this one is a
// pre-rendered waveform living in FLASH -- see voice_data.h for the reasoning.
// Short version: a formant synthesiser in C would cost 2-3 KB of code, this app
// links copy-to-RAM so code eats SRAM, and only ~3 KB remained. The loader
// below costs a few hundred bytes and reproduces the approved render exactly.
// Melody is public domain (1892). Upsamples 16 kHz -> FX_FS on the way in.
// ---- Power-zone policy -----------------------------------------------------
// The demo's rail request is (live rails | NEEDED) & ~FORBIDDEN.
//
// History: this used to be a blanket all-rails request, chosen because a
// stale or misparsed status frame echoed back as a mask once dropped the
// display+USB rails mid-session (bench 2026-07-29). ORing the live rails
// into the request keeps that anti-drop property. What changed: production
// power-management firmware ships the radio and compute-module rails OFF by
// default, and a blanket request would energize them uninvited. A blank
// ESP32 reset-storms and browns out the whole USB tree including the debug
// probe (dev bench lost hours to exactly this), and the compute module is a
// full Linux computer this demo has no business booting.
#define DEMO_ZONES_NEEDED   (picpwr_zone_bit(PICPWR_ZONE_SENSORS)     | \
                             picpwr_zone_bit(PICPWR_ZONE_DISPLAY)     | \
                             picpwr_zone_bit(PICPWR_ZONE_AUDIO)       | \
                             picpwr_zone_bit(PICPWR_ZONE_SDCARD)      | \
                             picpwr_zone_bit(PICPWR_ZONE_USB_HUB)     | \
                             picpwr_zone_bit(PICPWR_ZONE_STATUS_LED)  | \
                             picpwr_zone_bit(PICPWR_ZONE_RGB_LEDS)    | \
                             picpwr_zone_bit(PICPWR_ZONE_USB_SERIAL)  | \
                             picpwr_zone_bit(PICPWR_ZONE_DEBUG_PROBE))
#define DEMO_ZONES_FORBIDDEN (picpwr_zone_bit(PICPWR_ZONE_WIFI_BT)    | \
                             picpwr_zone_bit(PICPWR_ZONE_COMPUTE))
// Sleep mask is applied wholesale on sleep entry, and sleep can be entered
// by the coprocessor's own button handling at any moment — a zeroed sleep
// mask would cut the USB hub and debug probe on the next sleep and lock a
// sealed unit out of its own recovery paths. Hold exactly those through
// sleep: nothing the demo needs, everything a rescue needs.
#define DEMO_ZONES_SLEEP    (picpwr_zone_bit(PICPWR_ZONE_USB_HUB)     | \
                             picpwr_zone_bit(PICPWR_ZONE_USB_SERIAL)  | \
                             picpwr_zone_bit(PICPWR_ZONE_STATUS_LED)  | \
                             picpwr_zone_bit(PICPWR_ZONE_DEBUG_PROBE))

#define VOICE_SLOT 1                    /* YELLOW */
static void seed_voice_sample(void) {
    unsigned cap = 0;
    int16_t *d = vox_seed_begin(VOICE_SLOT, &cap);
    if (!d) return;
    absolute_time_t t0 = get_absolute_time();
    /* linear interpolation; step < 1 so we are upsampling */
    float step = (float)voice_data_rate / (float)FX_FS;
    float pos = 0.0f;
    unsigned n = 0;
    while (n < cap) {
        unsigned i = (unsigned)pos;
        if (i + 1 >= voice_data_len) break;
        float fr = pos - (float)i;
        float a = (float)voice_data[i];
        float b = (float)voice_data[i + 1];
        float v = (a + (b - a) * fr) * (32767.0f / 127.0f);
        if (v >  32700.0f) v =  32700.0f;
        if (v < -32700.0f) v = -32700.0f;
        d[n++] = (int16_t)v;
        pos += step;
    }
    vox_seed_commit(VOICE_SLOT, n);
    DIAG("vox: seeded slot %d (voice) %u samples in %lld us\n",
         VOICE_SLOT, n, absolute_time_diff_us(t0, get_absolute_time()));
}

// SD-launch boot trace (_sd target only): paints a step marker top-right as
// each init stage completes, so a freeze on a menu-launched unit is diagnosable
// from the screen alone -- no debugger needed on someone else's device. The
// last visible marker names the stage that hung. Steps: 1 display, 2 touch,
// 3 PDM, 4 LEDs, 5 codec, 6 I2S armed, 7 vox/UI, 8 sample seeds, 9 agentio,
// A sensors, B core1 pump.
// Gated SEPARATELY from the sd_boot_stage breadcrumb (see near the includes).
// The two are different instruments and only one of them should ship:
//   - the breadcrumb is invisible, costs one store per milestone, survives
//     LOCKUP and is what tools/read_boot_stage.py reads. It stays ON.
//   - these letters are for a human watching a screen that may never finish
//     booting. They served their purpose; a shipped app should not paint a
//     debug glyph over its own title bar.
#if defined(ORCASTRA_SD_BOOT_MARKERS) && ORCASTRA_SD_BOOT_MARKERS
static void sd_bt(char c) {
    char s[2] = { c, 0 };
    st7796_draw_text(452, 4, 3, 0xFFFF, 0x0000, s);
}
#else
#define sd_bt(c) ((void)0)
#endif

int main(void) {
    // Clocks, vreg and the QMI re-time are already done by the BSP PSRAM
    // bootstrap before main() is entered; board_init() detects that and
    // brings up the inherited peripherals only.
    board_init();
    DIAG("\n=== orcastra: tone + mic + FX pad boot ===\n");
#if defined(ORCASTRA_SD_BOOT_MARKERS) && ORCASTRA_SD_BOOT_MARKERS
    // Marker build only: bring the panel up FIRST (display rail is on in the PIC
    // defaults, and SPI is immune to the rail-walk I2C glitch that motivated
    // rails-first ordering) so markers can see inside the picpwr block below.
    // NOT done in a shipping build -- it reorders display init ahead of the
    // power sequence purely so a human can watch letters appear.
    st7796_init(); st7796_fill_screen(0x0000); board_backlight_set(1); sd_bt('0');
#endif

    // Coprocessor link + power rails FIRST, before anything that lives on
    // a switched rail is initialized. A rail request runs a ~1 s walk on
    // the coprocessor, and a rail coming up can glitch the shared I2C bus
    // (bench 2026-07-29: codec rail-up mid-boot wedged SDA — codec NAKs
    // then bus timeouts, audio engine then blocks on missing I2S clocks).
    // Requesting first means peripherals initialize onto stable rails.
    // Performs uartkbd_init() itself (input/app_recovery.h). Do not also
    // call uartkbd_init(): it claims a DMA channel and is not idempotent.
    // It belongs HERE rather than later because the power-rail block below
    // needs the keyboard link for picpwr status frames, and st7796_init()
    // does not run until after that block.
    //
    // The rail waits below pump uartkbd_task(), NOT fw2_app_recovery_task():
    // recovery reboots to the display loader, and doing that before the panel
    // is up gives the user a blank screen with no indication of why. Recovery
    // is armed from the main loop onward, once there is a UI to leave.
    fw2_app_recovery_init();
    DIAG("uartkbd: init (buttons live, HOME recovery armed)\n");
    sd_bt('P');
    {
        // Status frames arrive ~1/s; wait up to 3 s for the first one so
        // a warm boot (rails already up) can prove it and skip the send —
        // every needless send costs a ~1 s rail walk.
        // REGISTER THE RAILS FIRST, THEN PUMP. Order matters and getting it
        // backwards cost two attempts.
        //
        // picpwr rides the uartkbd transport (picpwr.c calls
        // uartkbd_status_raw()/uartkbd_frames()), so nothing is known until a
        // PIC status frame has been parsed -- and picpwr_keep_awake() returns
        // FALSE while "stable live state is unavailable". Our first version
        // waited for rails, THEN called keep_awake once, and on a cold boot it
        // returned false and we gave up: the log showed rails=0x000000 and no
        // "after request" line at all.
        //
        // keep_awake also REMEMBERS the request ("picpwr_task() re-asserts them
        // if they later read as off"), so calling it up front registers the
        // intent and picpwr_task() applies it as soon as frames arrive. That is
        // the shape the BSP's own docs/drivers/power.md prescribes:
        // keep_awake once, then poll.
        uint32_t rails = 0;
        bool want_audio = false;
        picpwr_keep_awake(DEMO_ZONES_NEEDED);   // may return false; intent sticks
        for (int i = 0; i < 200; i++) {         // up to 5 s: frame + ~1 s walk
            picpwr_task();
            uartkbd_task();
            sleep_ms(25);
            if (picpwr_rails(&rails)) {
                if (rails & picpwr_zone_bit(PICPWR_ZONE_AUDIO)) { want_audio = true; break; }
                // Live state is known now, so a request can actually be sent.
                // Cheap to repeat: additive, rate-limited by the driver.
                picpwr_keep_awake(DEMO_ZONES_NEEDED);
            }
        }
        DIAG("picpwr: link=%s rails=0x%06x audio=%s\n",
             picpwr_rails(&rails) ? "up" : "NO STATUS FRAME",
             (unsigned)rails, want_audio ? "ON" : "OFF");
        DIAG("picpwr: rails=0x%06x audio=%s can=%s\n", (unsigned)rails,
             (rails & picpwr_zone_bit(PICPWR_ZONE_AUDIO)) ? "on" : "OFF",
             (rails & picpwr_zone_bit(PICPWR_ZONE_CAN)) ? "on" : "off");
        sd_bt('Q');
        if (!(rails & picpwr_zone_bit(PICPWR_ZONE_AUDIO))) {
            // picpwr_keep_awake() rather than a hand-built picpwr_send(): the
            // BSP header calls it "the primary entry point for applications",
            // it is ADDITIVE (no rail that currently reads as powered is ever
            // cleared), it seeds from LIVE rail state requiring two agreeing
            // reads, and picpwr_task() re-asserts the rails later if the device
            // drops them on its own (sleep/wake, USB attach, watchdog).
            //
            // This also retires the invariant-18 hazard at the source. The old
            // code sent an ABSOLUTE mask seeded from `rails`, so with rails
            // reading 0x000000 -- exactly what happened -- the frame it sent
            // asked for NEEDED and implicitly cleared everything else. That is
            // the same shape as the per-zone "Set Zone" that turned every rail
            // off earlier in this project. DEMO_ZONES_FORBIDDEN is no longer
            // needed for the request path either: additive means we only ever
            // name what we want, so radios are never switched on by accident.
            if (picpwr_keep_awake(DEMO_ZONES_NEEDED)) {
                // Ride out the ~1 s rail walk, pumping the link so the status
                // frame that proves it can actually be received. Exit as soon
                // as audio reads on rather than always burning the full 3 s.
                for (int i = 0; i < 120; i++) {
                    picpwr_task();
                    uartkbd_task();
                    sleep_ms(25);
                    if (picpwr_rails(&rails) &&
                        (rails & picpwr_zone_bit(PICPWR_ZONE_AUDIO))) break;
                }
                sd_bt('R');
                picpwr_rails(&rails);
                DIAG("picpwr: after request audio=%s\n",
                     (rails & picpwr_zone_bit(PICPWR_ZONE_AUDIO)) ? "ON"
                                                                  : "still off");
                // The rail coming up can glitch devices on the shared
                // I2C bus mid-transition; clear any wedge before the
                // bus's clients are initialized. Then re-run the I/O
                // expander config: on a rails-cold boot it was unpowered
                // when board_init() first configured it (init NAK) and
                // has only its power-on defaults.
                board_i2c1_init();
                sleep_ms(100);
                sd_bt('S');
                ioexp_init();
                sd_bt('T');
            }
        }
        // Deliberately NOT calling picpwr_keep_awake() any more. It registered
        // the audio rail so picpwr_task() would re-assert it if a status frame
        // ever showed it off — but every re-assert costs a full rail walk, and
        // a walk that touches the debug probe's zone drops the probe off USB.
        // On 2026-07-30 that cost hours: the probe died within seconds of any
        // run that injected BUTTON events (touch-only runs were fine all day),
        // which fits injection perturbing the uartkbd frame stream that also
        // carries rail telemetry. The boot block above already proves the audio
        // rail up, and the rail-change monitor in the main loop logs any real
        // drop, so nothing is silently lost by not auto-re-asserting.
        // With s_desired left at 0, picpwr_task() early-returns and is free.
    }

    st7796_init();
    st7796_fill_screen(C_BG);
    board_backlight_set(1);
    sd_bt('1');

    bool touch_ok = ft6336_init();
    sd_bt('2');
    DIAG("touch: ft6336 init %s\n", touch_ok ? "ok" : "FAILED");

    // pio1 order matters (invariant 9): PDM first (claims its SM + program),
    // THEN the LED strip on a freshly-claimed SM. pdm_capture_init also
    // gates the mic power rail on (+50 ms settle) and starts the
    // free-running capture DMA.
#define BISECT_NO_PDM 0    // TEMP: click bisection build A
#if !BISECT_NO_PDM
    pdm_capture_init();
#else
    DIAG("BISECT: pdm capture DISABLED\n");
#endif
    sd_bt('3');
    ws2812_init(pio1, pio_claim_unused_sm(pio1, true), PIN_LED_DATA);
    ws2812_clear(); ws2812_show();
    sd_bt('4');

    codec_nau88c10_init();                       // driver defaults to 16 kHz...
#if !BISECT_16K
    nau_write(0x07, 0x0000);                     // ...override SMPLR -> 48 kHz
    DIAG("codec: SMPLR override -> 48 kHz filters\n");
#endif
    codec_ok = codec_nau88c10_input_ok();
    if (codec_ok) DIAG("codec: input path ready\n");
    sd_bt('5');
    // NOTE: codec_nau88c10_dump() wedges this app mid-scan (blocking I2C read
    // around reg 0x1E) — do not reinstate. Init values are known from source.

    // Bake the ~1 kHz tone ring: 21 whole cycles over 1024 frames so the
    // ring seam is phase-continuous (f = 21 * FX_FS / 1024 = 1001.4 Hz).
    {
        int16_t mono[64]; float ph = 0.0f;
        float ftone = 21.0f * (float)FX_FS / (float)TONE_FRAMES;
        for (uint32_t c = 0; c < TONE_FRAMES; c += 64) {
            tone_gen_fill(mono, 64, ftone, (float)FX_FS, &ph);
            for (int i = 0; i < 64; i++) {
                uint16_t s = (uint16_t)mono[i];
                tone_buf[c + i] = ((uint32_t)s << 16) | s;   // L and R slots
            }
        }
    }

    codec_nau88c10_dac_mute(false);   // init leaves the DAC soft-muted
    audio_i2s_duplex_init(48000);     // ACTUAL rate 48,828 Hz (integer MCLK
                                      // divide — see FX_FS in fx.h)
    // REQUIRED even for playback-only apps: the duplex PIO SM autopushes ADC
    // bits into the RX FIFO; if nothing drains it, the FIFO fills in ~250 us
    // and the autopush STALLS the whole SM (clocks + DAC die -> just a pop).
    // audio_capture_start()'s ping-pong DMA drains RX forever, zero CPU.
    audio_capture_start();
    // Arm the silence ring immediately: TX-empty ALSO stalls the SM (autopull
    // with an empty FIFO freezes the I2S clocks -> capture dies too; confirmed
    // on hardware via fdebug TXSTALL). TX is never left unarmed from here on.
    audio_i2s_duplex_play_loop(silence_buf, 64);
    DIAG("tx: silence ring armed at boot\n");
    sd_bt('6');

    // Power-up defaults: guitar in, full DAC volume, 3.5 mm jack out.
    apply_in_profile(in_idx);
    apply_vol(vol_idx);
    if (jack) {
        codec_nau88c10_set_output(CODEC_OUT_HEADPHONE);
        DIAG("route: output -> 3.5mm jack (default)\n");
    }
    fx_reset();
    fx_apply_all();
    vox_init();          // PSRAM sample slots (after board_init's final clocks)
    draw_main_page();
    DIAG("ui: ready\n");
    sd_bt('7');

    bool was_down = false;            // edge-detect so a hold = one press
    // Menu tap-vs-hold tracking: press start time + which button, and a
    // swallow-until-release flag so a long-press's finger doesn't leak a
    // phantom touch into the page it just opened.
    int  menu_press_btn = -1;
    absolute_time_t menu_press_t = get_absolute_time();
    bool ignore_until_up = false;
    // VOX page press tracking: which slot button is held (tap = select,
    // hold >= 600 ms = record until release) or whether the pad is held.
    int  vox_press_slot = -1;
    absolute_time_t vox_press_t = get_absolute_time();
    bool vox_in_pad = false;
    // Seed a starter beat so DRUMS demos instantly. This was four-on-the-floor
    // with a backbeat and eighth hats -- correct, and dull: bench verdict was
    // "it is kind of a dull beat so if we are going to have something preset
    // let's make it weird(ish)". This is a broken/syncopated pattern instead:
    // the kick lands off the grid, the snare keeps a displaced backbeat, and
    // the hats run sixteenths with GAPS rather than a metronomic eighth pulse.
    // Steps are 0-indexed, 16 per bank.
    // Refined at the bench 2026-08-01: the player edited it on the panel and
    // this is that pattern, read back off the screen pixel-by-pixel (the beat
    // was STOPPED first -- a running playhead paints a column white and makes
    // the grid ambiguous to read).
    //          step: 1 2 3 4 5 6 7 8 9 . . . . . . .   (shown 1-indexed)
    //   kick        : X . . . . . X . . . X . . . . .
    //   snare       : . . . . X . . . . . . . X . . .
    //   closed hat  : X . X X . X X . X . X X . X X .
    //   open hat    : . . . . . . . X . . . . . . . X
    //   clap        : . . . . . . . . . . . . X . . .
    static const uint16_t SEED_KICK  = 0x0441;  /* 0,6,10   */
    static const uint16_t SEED_SNARE = 0x1010;  /* 4,12     */
    static const uint16_t SEED_HHCL  = 0x6D6D;  /* 0,2,3,5,6,8,10,11,13,14 */
    static const uint16_t SEED_HHOP  = 0x8080;  /* 7,15     */
    static const uint16_t SEED_CLAP  = 0x1000;  /* 12       */
    for (int st = 0; st < 16; st++) {
        if ((SEED_KICK  >> st) & 1u) drums_toggle(0, st);
        if ((SEED_SNARE >> st) & 1u) drums_toggle(1, st);
        if ((SEED_HHCL  >> st) & 1u) drums_toggle(2, st);
        if ((SEED_HHOP  >> st) & 1u) drums_toggle(3, st);
        if ((SEED_CLAP  >> st) & 1u) drums_toggle(4, st);
    }
    seed_modem_sample();   // demo sample in slot 0 (see above)
    seed_voice_sample();   // demo sample in slot 1 (see above)
    sd_bt('8');

    // (the coprocessor link is brought up earlier — the power-zone request
    // before codec init needs its status frames)

    // Agent E2E harness: injected touch/buttons + PSRAM shadow screen capture
    // (init AFTER psram_init/vox_init; captures only see draws made after
    // this call, so the pages get redrawn on entry anyway).
    agentio_init();
    DIAG("agentio: init (fw touch/press/screenshot live)\n");
    draw_main_page();   // repaint so the agentio shadow sees the first screen
    sd_bt('9');

    // IMU (BMI323, I2C1): motion control for the synth page.
    bool imu_ok = bmi323_init();
    DIAG("bmi323: %s\n", imu_ok ? "init ok (synth motion live)" : "ABSENT");
    // Theremin sensors (same I2C1 bus): light = always used, mag = optional.
    opt_ok = opt4001_init();
    if (opt_ok) {
        // Pitch-rate light updates: rewrite CONFIG with CONVERSION_TIME=
        // 25 ms (init's default 100 ms steps audibly under a glide).
        // Same auto-range/continuous/latch bits as the driver's init.
        uint8_t cfg[3] = { 0x0A, 0x31, 0xB8 };
        i2c_write_timeout_us(i2c1, 0x45, cfg, 3, false, 2000);
    }
    mag_ok = bmm350_init();
    DIAG("theremin: light %s (25ms conv), mag %s\n",
         opt_ok ? "ok" : "ABSENT", mag_ok ? "ok" : "ABSENT");
    sd_bt('A');

#define BISECT_NO_CORE1 0  // TEMP: don't even launch the audio pump — pure
                           // DMA/PIO/codec hardware runs the tone
#if !BISECT_NO_CORE1
    // Audio gets its own core from here on: UI stalls can never again tear
    // capture blocks or slip the TX ring (the day-one static/fuzz bug).
    multicore_launch_core1_with_stack(core1_pump, core1_stack,
                                      sizeof core1_stack);
    DIAG("core1: audio pump launched (signal has its own core)\n");
    sd_bt('B');
#else
    DIAG("BISECT: core1 pump NOT launched - zero code in the audio path\n");
#endif

    uint32_t loop_n = 0;
    absolute_time_t t_beat = get_absolute_time();
    for (;;) {
        // 1 Hz heartbeat: UI loop liveness + core-1 block rate + PIO stalls,
        // plus the meter/telemetry prints that used to live in the pump
        // (RTT is core-0-only).
        if (absolute_time_diff_us(t_beat, get_absolute_time()) > 1000000) {
            t_beat = get_absolute_time();
            static uint32_t last_pb = 0;
            uint32_t pb = pump_blocks;
            DIAG("beat: loops=%u cap_blocks=%u fdebug=%x railschg=%u\n",
                 (unsigned)loop_n, (unsigned)(pb - last_pb),
                 (unsigned)pio0->fdebug, (unsigned)rails_changes);
            {   // FX-chain input level, the reference every fx.c threshold
                // is specified against. Snapshot and clear so each report
                // covers one interval.
                uint32_t n = inlvl_n, pkr = inlvl_pk_raw, pk = inlvl_pk;
                uint64_t sq = inlvl_sumsq;
                inlvl_n = 0; inlvl_pk_raw = 0; inlvl_pk = 0; inlvl_sumsq = 0;
                if (n) {
                    uint32_t rc = (uint32_t)sqrtf((float)((double)sq
                                                          / (double)n));
                    uint32_t opk = outlvl_pk, oclip = outlvl_clip;
                    outlvl_pk = 0; outlvl_clip = 0;
                    DIAG("inlvl: raw_pk=%d pk=%d rms=%d dBFS  "
                         "counts raw_pk=%u pk=%u rms=%u n=%u\n",
                         dbfs_i(pkr), dbfs_i(pk), dbfs_i(rc),
                         (unsigned)pkr, (unsigned)pk, (unsigned)rc,
                         (unsigned)n);
                    DIAG("outlvl: pk=%d dBFS count=%u clipped=%u\n",
                         dbfs_i(opk), (unsigned)opk, (unsigned)oclip);
                    // Internal saturation, which the line above CANNOT see:
                    // every stage clamps independently and the output counter
                    // sits downstream of the feedback lines.
                    uint32_t cm = 0, cc = 0;
                    fx_clip_report(&cm, &cc);
                    if (cc)
                        DIAG("fxclip: mask=0x%02x samples=%u (bit0 delay, "
                             "1 combs, 2 allpass, 3 shimmer, 4 flanger)\n",
                             (unsigned)cm, (unsigned)cc);
                    // Battery telemetry. The coprocessor has reported this in
                    // the button frames all along and nothing looked at it --
                    // which is how a bench session ran the cell from 4.11 V
                    // down to 3.02 V unnoticed on 2026-07-30, until the PIC's
                    // low-voltage cutoff dropped every rail and took the
                    // display, the audio and the debug probe with it. A demo
                    // dying this way in front of an audience has no graceful
                    // recovery, so it is now visible.
                    uartkbd_charger_t chg;
                    if (uartkbd_charger(&chg)) {
                        DIAG("power: vbatt=%umV vbus=%umV vsys=%umV "
                             "chg=%u vbus_st=%u\n",
                             (unsigned)chg.vbatt_mv, (unsigned)chg.vbus_mv,
                             (unsigned)chg.vsys_mv,
                             (unsigned)chg.charge_status,
                             (unsigned)chg.vbus_status);
                    }
                }
            }
            last_pb = pb;
            pio0->fdebug = 0xFFFFFFFF;   // clear sticky stall flags
            loop_n = 0;
            // Charger/keyboard telemetry: frame arrival rate + power state
            // (diagnosing a periodic speaker pop 2026-07-23).
            {
                uartkbd_charger_t chg;
                bool have = uartkbd_charger(&chg);
                DIAG("kbd: frames=%u err=%u chg=%d vbus=%d fault=%d\n",
                     (unsigned)uartkbd_frames(), (unsigned)uartkbd_errors(),
                     have ? (int)chg.charge_status : -1,
                     have ? (int)chg.vbus_status : -1,
                     have ? (int)chg.fault : -1);
            }
            // Pop hunter report: per-second max step + transient timestamps
            // (consecutive block deltas x 5.24 ms = the pop's true period).
            {
                static uint32_t pop_seen = 0;
                uint32_t pn = pop_n;
                uint32_t a = pn ? pop_blk[(pn - 1) & 7u] : 0;
                uint32_t b2 = pn >= 2 ? pop_blk[(pn - 2) & 7u] : 0;
                uint32_t c = pn >= 3 ? pop_blk[(pn - 3) & 7u] : 0;
                DIAG("pop: maxd=%u n=%u d1=%u d2=%u (x5.24ms)\n",
                     (unsigned)pop_maxd, (unsigned)pn,
                     (unsigned)(a - b2), (unsigned)(b2 - c));
                pop_maxd = 0;
                pop_seen = pn;
            }
            if (playing && mic) {
                if (use_pdm)
                    DIAG("pdm: depth=%u peak=%u\n",
                         (unsigned)(pdm_w - pdm_r), (unsigned)pump_vu);
                else
                    DIAG("cap: rawL=%u rawR=%u\n",
                         (unsigned)pump_raw_l, (unsigned)pump_raw_r);
                if (tune_on) {
                    int hz10, cents;
                    fx_tune_status(&hz10, &cents);
                    DIAG("tune: f=%d.%d Hz corr=%d cents\n",
                         hz10 / 10, hz10 % 10, cents);
                }
            }
        }
        loop_n++;
        // (PDM pumping moved to core 1 with the rest of the audio path.)
        // While a synth note sounds, the trace comes alive: green, scrolling,
        // with a tremolo amp wobble (~12 Hz redraws; audio is on core 1).
        if (page == PAGE_SYNTH && syn_gate_ui) {
            static absolute_time_t t_st;
            if (absolute_time_diff_us(t_st, get_absolute_time()) > 80000) {
                t_st = get_absolute_time();
                syn_tr_ph += 0.21f;
                syn_tr_wob = 1.0f + 0.22f * sinf(syn_tr_ph * 2.3f);
                syn_cursor(-1, -1);
                synth_trace(syn_tr_x, syn_tr_y);
            }
        }
        // ~40 Hz waveform needle / record-strip refresh on the sampler page.
        if (page == PAGE_VOX && (vox_playing() || vox_recording())) {
            static absolute_time_t t_bar;
            if (absolute_time_diff_us(t_bar, get_absolute_time()) > 25000) {
                t_bar = get_absolute_time();
                if (vox_recording()) vwf_rec_update();
                else                 vwf_play_update();
            }
        }
        // ~12 Hz needle/LED refresh while the tuner page is up.
        if (page == PAGE_TUNER) {
            static absolute_time_t t_tun;
            if (absolute_time_diff_us(t_tun, get_absolute_time()) > 80000) {
                t_tun = get_absolute_time();
                tuner_update();
            }
        }
        // ~50 Hz IMU poll while the synth page is up: shake envelope (fast
        // attack, ~1.2 s release = "fades off when still"), bank/tip tilt,
        // and gyro-Z twist -> pitch dive. Axis mapping is a first guess from
        // the board orientation; swap signs/axes here if the hardware test
        // says otherwise.
        if (page == PAGE_SYNTH && imu_ok) {
            static absolute_time_t t_imu;
            static float shake_env = 0.0f;
            if (absolute_time_diff_us(t_imu, get_absolute_time()) > 20000) {
                t_imu = get_absolute_time();
                bmi323_reading_t r;
                if (bmi323_read(&r) && r.valid) {
                    float m = sqrtf(r.ax * r.ax + r.ay * r.ay + r.az * r.az);
                    float sh = m - 1.0f;
                    if (sh < 0.0f) sh = -sh;
                    sh *= 1.6f;                       // ~0.6 g shake = full
                    if (sh > 1.0f) sh = 1.0f;
                    if (sh > shake_env) shake_env += 0.5f * (sh - shake_env);
                    else                shake_env *= 0.965f;   // ~1.2 s fade
                    // Pitch dive: calibration (2026-07-24) showed the natural
                    // "dive" gesture is a sharp TIP of the far edge =
                    // rotation on gx (not the flat gz twist originally
                    // guessed). 60 dps dead-zone keeps slow resonance-tips
                    // from bending; beyond it, snap speed = bend depth.
                    float wg = r.gx;
                    if (wg > -60.0f && wg < 60.0f) wg = 0.0f;
                    else wg -= (wg > 0.0f ? 60.0f : -60.0f);
                    synth_motion(shake_env, r.ax, r.ay, wg * (1.0f / 300.0f));
                    static uint32_t imu_n = 0;
                    if ((imu_n++ & 0x1F) == 0)   // full 6-axis, ~0.64 s cadence
                        DIAG("imu6: a=%d,%d,%d g=%d,%d,%d shk=%d\n",
                             (int)(r.ax * 100), (int)(r.ay * 100),
                             (int)(r.az * 100), (int)r.gx, (int)r.gy,
                             (int)r.gz, (int)(shake_env * 100));
                }
            }
        }
        // ~25 Hz sensor poll while the theremin page is up. Log2-domain
        // auto-cal: ranges expand instantly to include what they see and
        // relax toward the current reading slowly (~40 s), so covering the
        // sensor / bringing the magnet in defines the playable span.
        if (page == PAGE_THMN) {
            static absolute_time_t t_thm;
            static uint32_t thm_n = 0;
            if (absolute_time_diff_us(t_thm, get_absolute_time()) > 10000) {
                t_thm = get_absolute_time();         // 100 Hz tick (mag rate;
                float light_n = -1.0f;               //  light every 4th tick)
                static float lux = 0.0f;
                if (opt_ok && (thm_n & 3) == 0 && opt4001_read(&lux)) {
                    float lg = log2f(lux + 1.0f);
                    if (!thm_lg_seeded) {
                        thm_lg_ema = lg;
                        thm_lg_lo = lg - 0.3f; thm_lg_hi = lg + 0.3f;
                        thm_lg_seeded = true;
                    }
                    thm_lg_ema += 0.30f * (lg - thm_lg_ema);   // ~130 ms
                    lg = thm_lg_ema;
                    if (!thm_lg_manual) {    // CAL anchors the range; auto
                        if (lg < thm_lg_lo) thm_lg_lo = lg;     // otherwise
                        if (lg > thm_lg_hi) thm_lg_hi = lg;
                        // Range relax is glacial (~7 min): the note must
                        // not drift under a held hand.
                        thm_lg_lo += 0.0001f * (lg - thm_lg_lo);
                        thm_lg_hi += 0.0001f * (lg - thm_lg_hi);
                    }
                    float span = thm_lg_hi - thm_lg_lo;
                    if (span < 0.5f) span = 0.5f;    // uncalibrated: stay tame
                    light_n = (lg - thm_lg_lo) / span;
                    if (light_n < 0.0f) light_n = 0.0f;
                    if (light_n > 1.0f) light_n = 1.0f;
                }
                float mag_n = -1.0f;
                bmm350_reading_t mr;
                if (thm_mag_mode && mag_ok && bmm350_read(&mr) && mr.valid
                        && mr.magnitude > 0.0f) {
                    dbg_mx = mr.mx; dbg_my = mr.my; dbg_mz = mr.mz;
                    dbg_mag = mr.magnitude;

                    // PITCH COMES FROM THE DEVIATION FROM THE RESTING FIELD
                    // VECTOR, not from |m|. This is the whole fix, and it is
                    // measurement-driven (production unit, 2026-08-03, BMM350
                    // sampled over SWD during a 60 s magnet sweep):
                    //
                    //   There is a LARGE STATIC INTERNAL FIELD, ~1500 uT at the
                    //   sensor -- about 30x Earth's -- from the on-board speaker
                    //   magnet, which sits at the bottom right where the response
                    //   is strongest. A hand magnet adds to that VECTORIALLY, so
                    //   |m| is NOT monotonic in distance: across one sweep the
                    //   magnitude ranged 715..7055 uT. Held one way the magnet
                    //   reinforces the internal field (7055); flipped, it CANCELS
                    //   it (715, i.e. BELOW resting).
                    //
                    //   Two symptoms followed, both reported from the bench and
                    //   both reproduced exactly by arithmetic:
                    //     - "sometimes 100%, sometimes 60% at the same distance"
                    //       -- orientation, not distance.
                    //     - "stuck at ~40% with the magnet far away" -- thm_mg_lo
                    //       latched onto the 715 dip, leaving resting field about
                    //       1.0 in log2 ABOVE the zero. The recovery gate only
                    //       ran within 0.35, so it was permanently blocked and
                    //       the bar parked at 34..44%.
                    //
                    // Subtracting a slowly-adapting baseline VECTOR removes the
                    // internal field entirely: |m - b| is ~0 with no magnet
                    // whatever the orientation, and grows with proximity either
                    // way up. Cancellation stops mattering because a cancelling
                    // magnet is still a large DEVIATION.
                    float bdx = mr.mx - thm_bx;
                    float bdy = mr.my - thm_by;
                    float bdz = mr.mz - thm_bz;
                    float dev = sqrtf(bdx * bdx + bdy * bdy + bdz * bdz);
                    dbg_dev = dev;
                    if (!thm_b_seeded) {
                        thm_bx = mr.mx; thm_by = mr.my; thm_bz = mr.mz;
                        thm_b_seeded = true;
                        // Seed below the measured peak deviation (~4800 uT) so
                        // the running max below can actually climb to it. Same
                        // trap as the old log2 seed: a seed above anything
                        // reachable freezes the span forever.
                        thm_dev_hi = 300.0f;
                        dev = 0.0f;
                    }
                    // Jitter of the RAW deviation, before smoothing -- this is
                    // what tells a hand from a baseline offset (see the adapt
                    // below). Measured on the raw value because the EMA would
                    // hide exactly the tremor we are looking for.
                    thm_dev_jit += 0.05f * (fabsf(dev - thm_dev_prev)
                                            - thm_dev_jit);
                    thm_dev_prev = dev;
                    // Net drift over ~1 s at the ~100 Hz poll. Distinguishes a
                    // SLOWLY MOVING magnet (small per-sample jitter but a real
                    // trend) from a static baseline error (neither).
                    if (++thm_lag_n >= 100) { thm_lag_n = 0; thm_dev_lag = dev; }
                    float trend = fabsf(dev - thm_dev_lag);
                    thm_dev_ema += 0.50f * (dev - thm_dev_ema); // keep vibrato
                    dev = thm_dev_ema;
                    if (dev > thm_dev_hi) thm_dev_hi = dev;    // learn the peak

                    // Baseline follows the ambient field ONLY while no magnet is
                    // near (deviation small). That is what makes the hard-iron
                    // drift self-correct -- resting field wandered 1305..1606 uT
                    // across one sweep -- without eroding the zero while a note
                    // is being held, which an unconditional adapt would do.
                    // Two rates, and the gate is ABSOLUTE in uT -- not a fraction
                    // of thm_dev_hi. Bench 2026-08-03: a relative gate of
                    // 0.10*dev_hi opened at ~480 uT, which is still well inside
                    // the magnet's influence, so on the way out the baseline
                    // absorbed part of the MAGNET's own field. Once the magnet
                    // was gone |m-b| then sat ~2000 uT -- above the gate -- so the
                    // adapt could not run and it locked. Reported as "brought it
                    // in for a sweep and it stuck at about 75%".
                    //
                    // BEWARE THE CUBE ROOT when picking these thresholds. Because
                    // pitch is cbrt(dev/dev_hi), small deviations are NOT small on
                    // the bar: at dev_hi ~4800, dev = 150 uT is already cbrt(0.031)
                    // = 31% of the pitch range. A first attempt fast-adapted below
                    // 150 uT and therefore ate the bottom THIRD of playable travel
                    // -- reported as a note held far away sagging on its own, and
                    // as slow deliberate movement not matching the pitch. The fast
                    // band must sit entirely below DEV_FLOOR, i.e. inside the dead
                    // zone where the bar reads zero anyway.
                    if (dev < 40.0f) {
                        thm_bx += 0.05f * (mr.mx - thm_bx);
                        thm_by += 0.05f * (mr.my - thm_by);
                        thm_bz += 0.05f * (mr.mz - thm_bz);
                    } else if (thm_dev_jit < 3.0f && trend < 8.0f) {
                        // A LARGE BUT TRULY STATIC deviation is not a hand.
                        //
                        // "Magnet held still" and "baseline is wrong" are identical
                        // in magnitude but not in movement: a magnet in a hand
                        // carries tremor worth tens of uT, a baseline offset only
                        // the sensor's 1-2 uT noise floor.
                        //
                        // JITTER ALONE IS NOT ENOUGH: a slowly moving magnet also
                        // has tiny per-sample change, so jitter-only misread slow
                        // deliberate movement as baseline and zeroed the pitch out
                        // from under the player. So also require no NET DRIFT over
                        // the last second -- a slow move accumulates one, a dead
                        // baseline does not. Both conditions together mean nothing
                        // is happening, and only then is it safe to clear fast.
                        thm_bx += 0.05f * (mr.mx - thm_bx);
                        thm_by += 0.05f * (mr.my - thm_by);
                        thm_bz += 0.05f * (mr.mz - thm_bz);
                    } else {
                        // Moving hand at a real distance: hold the zero, so a
                        // sustained note sustains. Only a very slow leak, as a
                        // last-resort guarantee that nothing locks forever.
                        thm_bx += 0.0004f * (mr.mx - thm_bx);
                        thm_by += 0.0004f * (mr.my - thm_by);
                        thm_bz += 0.0004f * (mr.mz - thm_bz);
                    }
                    // Let the learned range relax too, so one unusually close
                    // pass cannot compress the rest of the session's travel.
                    // Floored at the seed so it never collapses to nothing.
                    thm_dev_hi *= 0.99990f;
                    if (thm_dev_hi < 300.0f) thm_dev_hi = 300.0f;

                    // Distance-linear feel: dipole field ~ 1/d^3, so cbrt of the
                    // normalised deviation is ~linear in 1/distance.
                    //
                    // Subtract THM_DEV_FLOOR first. cbrt is near-vertical at the
                    // origin, so without a floor a small residual baseline error
                    // reads as a large pitch offset, and the bottom of the range
                    // is unusably twitchy. With it, everything below the floor is
                    // a true zero and the bar rises smoothly out of it.
                    float top = thm_dev_hi - THM_DEV_FLOOR;
                    if (top < 1.0f) top = 1.0f;
                    float r = (dev - THM_DEV_FLOOR) / top;
                    if (r < 0.0f) r = 0.0f;
                    if (r > 1.0f) r = 1.0f;
                    // r^(1/3) via exp2/log2 rather than cbrtf. cbrtf pulls a
                    // fresh libm routine into .text, and because this app is
                    // pico_set_binary_type(copy_to_ram) that displaces .bss and
                    // overflowed RAM by 16.6 KB. exp2f/log2f are already linked
                    // for the light-sensor path, so this costs nothing.
                    mag_n = (r > 0.0f) ? exp2f(log2f(r) * (1.0f / 3.0f)) : 0.0f;
                    // Deadband: sensor noise is a couple of uT, but the baseline
                    // adapt leaves a little residue. Without this the bar idles
                    // a few percent up and the tone never truly rests.
                    if (mag_n < 0.04f) mag_n = 0.0f;
                }
                // MAGNET mode is two-handed: the tone auto-gates (a real
                // theremin always sounds — silence = cover the light).
                if (thm_mag_mode) synth_gate(1);
                float pit = thm_mag_mode ? mag_n : light_n;
                // Volume is the touch strip in both modes (see the strip
                // handler): magnet mode no longer reads the light sensor at all.
                float vol = thm_touch_vol;
                if (pit >= 0.0f) {
                    // One-octave window (fine hand control, vibrato-able),
                    // shifted by OCT-/OCT+. Base 440 Hz sits above the
                    // speaker route's 900 Hz protection HP by mid-window.
                    synth_freq(440.0f * exp2f((float)thm_oct + pit));
                    thm_pit_n = pit;
                }
                if (vol >= 0.0f) {
                    // Linear, not squared. synth.c's output scale just dropped
                    // 6 dB to stop the synth page clipping, and the theremin
                    // rides the same path -- squaring on top of that would have
                    // made an instrument already reported as "underwhelming"
                    // measurably worse. At the ~60% sensor ceiling seen on the
                    // production unit this lands within 1.6 dB of where it was,
                    // and it doubles the response at low levels. The taper is
                    // a feel decision: revisit it at the bench with the magnet.
                    synth_level(vol);
                    thm_vol_n = vol;
                }
                if (thm_cal_flash > 0 && --thm_cal_flash == 0)
                    draw_button(&BTN_TH_CAL, C_GRAY, 2, "CAL");
                if ((thm_n & 7) == 0) {              // ~12 Hz bar redraw
                    draw_thm_bar(THM_PIT_Y, thm_pit_n, C_CURSOR);
                    draw_thm_bar(THM_VOL_Y, thm_vol_n, C_GREEN);
                }
                if ((thm_n++ & 0x7F) == 0)           // ~1.3 s diag cadence
                    DIAG("thm: %s pit=%d vol=%d lux=%d oct=%d\n",
                         thm_mag_mode ? "MAG" : "LIGHT",
                         (int)(thm_pit_n * 100), (int)(thm_vol_n * 100),
                         (int)lux, thm_oct);
            }
        }
        // Visualizer: one full frame per loop pass (~25 fps, DMA-flush
        // bound). Touch/keyboard polling rides between frames.
        if (page == PAGE_VIZ) viz_frame();
        // Drum playhead chase (redraws only on step change). The global
        // step spans the whole chain; the column strip only shows when the
        // sounding bank is the one on the grid, and the tabs re-light when
        // the chain crosses a bar line.
        //
        // HELD while the volume overlay is up. The overlay is an opaque
        // 320x104 panel drawn OVER the grid; the chase below does not know it
        // exists and happily repaints bank tabs and playhead columns straight
        // through it, ~16 times a second. From the bench: d-pad up/down while
        // the beat runs makes the screen "look glitched out and weird", and it
        // only comes right when the overlay expires and page_repaint() runs.
        // Suppressing the chase is the correct fix rather than reordering the
        // draws, because the overlay is deliberately modal-looking -- a
        // playhead crawling across it would be wrong even if it were tidy.
        //
        // Nothing is lost by skipping: draw_drum_playhead() erases its previous
        // column via drum_cell(), so it repairs whatever it finds and a stale
        // drum_ph_drawn across the gap is self-correcting. The audio sequencer
        // is untouched -- this is paint only, so the beat keeps its timing.
        if (page == PAGE_DRUMS && drums_playing() && !vov_showing) {
            int g  = drums_step_now();
            int pb = g < 0 ? -1 : g >> 4;
            static int drum_pb_drawn = -1;
            if (pb != drum_pb_drawn) {
                drum_pb_drawn = pb;
                draw_drum_banks();
            }
            int col = (pb == drums_bank_get()) ? (g & 15) : -1;
            if (col != drum_ph_drawn) draw_drum_playhead(col);
        }
        // Colored buttons: poll the keyboard coprocessor; on the EQ page
        // GREY/YELLOW/GREEN/BLUE/RED grab bands 1..5 (colors match the
        // on-screen handles). Other pages just drain the event queue.
        // This also services HOME recovery, which reboots to the display
        // loader after a 5 s hold; it must run on every main-loop pass.
        fw2_app_recovery_task();
        // ---- Escape hatch: hold BLUE through the first 8 s after boot ----
        // Drops the display CPU into its USB (BOOTSEL) bootloader so this
        // app can ALWAYS be reflashed by drag-drop, with no debug probe and
        // no stock firmware present. Production units have no hardware
        // BOOTSEL path for the display CPU (red is wired to MAIN only) and
        // the stock blue-long-press lives in the firmware this app replaces,
        // so without this block, flashing orcastra onto a production display
        // CPU would be a one-way door. Polls uartkbd_buttons() STATE rather
        // than DOWN events so a button held since power-on still registers
        // even though its edge predates uartkbd_init(). Requires a 1500 ms
        // continuous hold inside the window: a stray EQ-band tap can't trip
        // it. BLUE deliberately mirrors the stock display-bootloader button.
        {
            static bool     hatch_armed = true;
            static uint32_t hatch_held_ms = 0;
            static uint32_t hatch_last_ms = 0;
            static uint32_t hatch_t0 = 0;
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            // Window opens at LOOP entry, not chip boot: a rails-cold boot
            // spends up to ~7 s in init (status-frame wait + rail walk)
            // before this loop runs, and a boot-anchored window would close
            // before the first button poll ever happened.
            if (hatch_t0 == 0) hatch_t0 = now_ms;
            if (hatch_armed && now_ms - hatch_t0 > 8000) {
                hatch_armed = false;   // window over; button is EQ's again
            } else if (hatch_armed) {
                bool blue = (uartkbd_buttons() >> UARTKBD_BTN_BLUE) & 1u;
                uint32_t dt = now_ms - hatch_last_ms;
                hatch_last_ms = now_ms;
                hatch_held_ms = blue ? hatch_held_ms + (dt < 100 ? dt : 0)
                                     : 0;
                if (hatch_held_ms >= 1500) {
                    DIAG("hatch: BLUE held %u ms -> USB bootloader\n",
                         (unsigned)hatch_held_ms);
                    st7796_fill_rect(0, 0, 480, 320, C_BG);
                    st7796_draw_text(114, 150, 3, rgb565_be(255, 255, 255),
                                     C_BG, "USB BOOTLOADER");
                    sleep_ms(150);           // let the panel flush finish
                    reset_usb_boot(0, 0);    // no return
                }
            }
        }
        picpwr_task();           // re-assert kept rails if one reads off
        // Rail-change monitor: log every observed transition so autonomous
        // drops (USB detach, sleep) and re-asserts are visible over RTT.
        {
            static uint32_t rails_seen = 0xFFFFFFFF;
            uint32_t r;
            if (picpwr_rails(&r) && r != rails_seen) {
                if (rails_seen != 0xFFFFFFFF) rails_changes++;
                DIAG("picpwr: rails %s0x%06x audio=%s\n",
                     rails_seen == 0xFFFFFFFF ? "" : "CHANGED -> ",
                     (unsigned)r,
                     (r & picpwr_zone_bit(PICPWR_ZONE_AUDIO)) ? "on" : "OFF");
                rails_seen = r;
            }
        }
        vol_overlay_task();      // erase the volume/mute popup when it expires
        agentio_task();          // service injected input / capture requests
        {
            uartkbd_event_t kev;
            while (uartkbd_next_event(&kev)) {
                // Every event logged: mapping verification against the
                // fresh PIC16 firmware (2026-07-29: all 14 verified).
                DIAG("btn: %d %s\n", (int)kev.btn,
                     kev.pressed ? "DOWN" : "UP");
                // The D-pad's centre is stiff on production units, so teams
                // are moving to the checkmark instead. Alias OK onto
                // NAV_CENTER once, here, rather than at each of the sites
                // that handle centre (FX-menu select and hold-for-settings,
                // and every effect settings page's on/off). Both buttons work
                // everywhere; OK is not otherwise bound.
                if (kev.btn == UARTKBD_BTN_OK)
                    kev.btn = UARTKBD_BTN_NAV_CENTER;
                // Physical controls: HOME = launcher from anywhere,
                // CANCEL = master mute, D-pad = page navigation where a
                // page claims it (FX menu), else UP/DOWN = volume. VOX
                // colored buttons = slot select / hold-record.
                if (kev.pressed && kev.btn == UARTKBD_BTN_GREY &&
                    page == PAGE_MAIN) {
                    // Reach-calibration grid. Entered from a PHYSICAL button on
                    // purpose: the whole point is to find out which parts of the
                    // screen a finger can get to, so the way in must not depend
                    // on reaching any particular pixel. HOME exits.
                    for (int r = 0; r < CAL_ROWS; r++)
                        for (int c = 0; c < CAL_COLS; c++) cal_hit[r][c] = 0;
                    page = PAGE_CAL;
                    draw_cal_page();
                    DIAG("cal: grid open (%dx%d) - tap every cell you can reach\n",
                         CAL_COLS, CAL_ROWS);
                } else if (kev.pressed && kev.btn == UARTKBD_BTN_PAGE) {
                    viz_button_toggle();
                } else if (kev.pressed && kev.btn == UARTKBD_BTN_HOME) {
                    go_home();
                } else if (kev.pressed && kev.btn == UARTKBD_BTN_CANCEL) {
                    if (!playing) {
                        if (use_pdm) pdm_input_reset();
                        if (mic) { mic_armed = false; playing = true; }
                        else tx_retarget(tone_buf, TONE_FRAMES, false, true);
                    } else {
                        tx_retarget(silence_buf, 64, mic, false);
                    }
                    if (page == PAGE_MAIN) {
                        draw_button(&BTN_PLAY, playing ? C_RED : C_GREEN, 2,
                                    playing ? "STOP" : "PLAY");
                        draw_status(C_BG, playing, mic, jack);
                    }
                    vol_overlay_show();   // mute state, visible on every page
                    DIAG("btn: CANCEL -> %s\n", playing ? "LIVE" : "MUTED");
                } else if (page == PAGE_FXMENU &&
                           kev.btn >= UARTKBD_BTN_NAV_CENTER &&
                           kev.btn <= UARTKBD_BTN_NAV_RIGHT) {
                    if (kev.pressed) {
                        switch (kev.btn) {
                        case UARTKBD_BTN_NAV_UP:    menu_cur_move(0, -1); break;
                        case UARTKBD_BTN_NAV_DOWN:  menu_cur_move(0, 1);  break;
                        case UARTKBD_BTN_NAV_LEFT:  menu_cur_move(-1, 0); break;
                        case UARTKBD_BTN_NAV_RIGHT: menu_cur_move(1, 0);  break;
                        default:                    // CENTER down
                            if (menu_cur < 0) {
                                menu_cur_move(0, 0);
                            } else {
                                menu_center_dn = true;
                                menu_center_t = get_absolute_time();
                            }
                            break;
                        }
                    } else if (kev.btn == UARTKBD_BTN_NAV_CENTER &&
                               menu_center_dn) {
                        menu_center_dn = false;      // short press = toggle
                        if (menu_cur >= 0) {
                            menu_toggle(menu_cur);
                            draw_menu_cursor(menu_cur);
                        }
                    }
                } else if (page == PAGE_VOX && kev.btn < UARTKBD_BTN_RED) {
                    // "< RED" deliberately: GREY/YELLOW/GREEN/BLUE only.
                    // Holding RED is the power coprocessor's power-off
                    // gesture, below anything this firmware can see, so RED
                    // must never invite a hold. See the BTN_V_SLOT note.
                    if (kev.pressed) {
                        kbtn_slot = (int)kev.btn;
                        kbtn_slot_t = get_absolute_time();
                    } else if ((int)kev.btn == kbtn_slot) {
                        if (vox_recording()) {
                            vox_rec_stop();
                            DIAG("vox: REC %s done (%u.%us) [button]\n",
                                 BTN_COLOR_NAMES[kbtn_slot],
                                 vox_len_ds(kbtn_slot) / 10u,
                                 vox_len_ds(kbtn_slot) % 10u);
                        } else {
                            vox_select(kbtn_slot);
                            DIAG("vox: slot %s selected [button]\n",
                                 BTN_COLOR_NAMES[kbtn_slot]);
                        }
                        draw_vox_slots();
                        draw_vox_status();
                        vwf_rescan();
                        vwf_draw_all();
                        kbtn_slot = -1;
                    }
                } else if (kev.pressed &&
                           kev.btn >= UARTKBD_BTN_NAV_CENTER &&
                           kev.btn <= UARTKBD_BTN_NAV_RIGHT &&
                           fx_page_btn((int)kev.btn)) {
                    // handled by the effect settings page (toggle/knob)
                } else if (kev.pressed &&
                           (kev.btn == UARTKBD_BTN_NAV_UP ||
                            kev.btn == UARTKBD_BTN_NAV_DOWN)) {
                    // Shares VOL_ORDER with vol_cycle(). The old local copy
                    // capped with `if (ORDER[oi] == 2 && !vol_loud_ok())
                    // oi = 2;` -- which resolved to ORDER[2] == 1 == -6 dB, so
                    // the D-pad handed the speaker a setting that vol_cycle()
                    // and vol_clamp_quiet() both refused. That is the "D-pad up
                    // gets me louder than the settings menu allows" report.
                    int top = vol_loud_ok() ? VOL_STEP_COUNT - 1
                                            : VOL_ORDER_SPEAKER_MAX;
                    int oi = 0;
                    for (int i = 0; i < VOL_STEP_COUNT; i++)
                        if (VOL_ORDER[i] == vol_idx) oi = i;
                    oi += (kev.btn == UARTKBD_BTN_NAV_UP) ? 1 : -1;
                    if (oi < 0) oi = 0;
                    if (oi > top) oi = top;      // clamps, does not wrap
                    vol_idx = VOL_ORDER[oi];
                    apply_vol(vol_idx);
                    if (page == PAGE_SETUP)
                        draw_button(&BTN_VOL, vol_idx == 2 ? C_RED : C_GRAY,
                                    2, VOL_STEPS[vol_idx].label);
                    vol_overlay_show();   // otherwise this change is invisible
                    DIAG("btn: vol -> %s\n", VOL_STEPS[vol_idx].label);
                }
                if (page == PAGE_EQ && kev.pressed &&
                    kev.btn <= UARTKBD_BTN_RED) {
                    eq_sel = (int)kev.btn;      // GREY=0 .. RED=4 = band order
                    eq_draw_readout();
                    eq_draw_qslider();
                    eq_dirty = true;            // re-highlight the handle
                    DIAG("eq: band %d via button\n", eq_sel + 1);
                }
            }
        }
        // Physical-button holds. CENTER held on the FX menu opens the
        // highlighted effect's settings — unless it has none (OCTAVE).
        if (menu_center_dn && page == PAGE_FXMENU && menu_cur >= 0 &&
            absolute_time_diff_us(menu_center_t, get_absolute_time())
                > 600000) {
            menu_center_dn = false;
            if (MENU_HAS_PAGE[menu_cur])
                menu_enter_page(menu_cur);
        }
        // Colored button held on the sampler = record until release
        // (same 600 ms threshold as the on-screen slots).
        if (kbtn_slot >= 0 && page != PAGE_VOX) kbtn_slot = -1;
        if (kbtn_slot >= 0 && !vox_recording() &&
            absolute_time_diff_us(kbtn_slot_t, get_absolute_time())
                > 600000) {
            vox_rec_start(kbtn_slot);
            draw_vox_slots();
            draw_vox_status();
            vwf_rec_begin();
            DIAG("vox: REC %s... [button]\n", BTN_COLOR_NAMES[kbtn_slot]);
        }
        // Throttled EQ plot redraw while dragging (~8 Hz; audio is on core 1
        // so this can take its time).
        if (page == PAGE_EQ && eq_dirty) {
            static absolute_time_t t_eq;
            if (absolute_time_diff_us(t_eq, get_absolute_time()) > 120000) {
                t_eq = get_absolute_time();
                eq_dirty = false;
                eq_draw_plot();
                eq_draw_readout();   // freq/gain readout follows the drag
            }
        }
        uint16_t x, y, x2 = 0, y2 = 0;
        int ntouch = ft6336_poll2(&x, &y, &x2, &y2);
        bool down  = ntouch >= 1;
        bool down2 = ntouch >= 2;    // second finger (keys page uses it)
        // Touch-envelope tracker for the enclosure calibration sweep: the
        // production case bezels off screen edges the bare dev board could
        // reach. Tracks the extreme coordinates a finger actually produces
        // and reports once per second over RTT while a touch is active.
        // Harmless to leave in: one DIAG/s during touches, silent otherwise.
        {
            static uint16_t env_x0 = 0xFFFF, env_y0 = 0xFFFF, env_x1, env_y1;
            static absolute_time_t t_env;
            if (down) {
                if (x < env_x0) env_x0 = x;
                if (x > env_x1) env_x1 = x;
                if (y < env_y0) env_y0 = y;
                if (y > env_y1) env_y1 = y;
                if (absolute_time_diff_us(t_env, get_absolute_time())
                        > 1000000) {
                    t_env = get_absolute_time();
                    DIAG("touchenv: x=%u..%u y=%u..%u now=(%u,%u)\n",
                         env_x0, env_x1, env_y0, env_y1, x, y);
                }
            }
        }
        // Up-debounce: the FT6336 drops frames mid-press (worse on light
        // stationary holds — a held piano key read as release+re-press and
        // "mellotron looped"). Hold the last contact before believing an up;
        // real releases just land late, which no interaction here can perceive.
        //
        // 90 ms was tuned on the BARE dev board. The production panel — same
        // controller, different stack-up behind the enclosure — drops frames
        // for longer, and 90 ms was short enough that a single sustained key
        // press re-struck the voice several times (family demo 2026-08-02:
        // "a single press and hold fired off multiple press events and started
        // the sample over"). 190 ms covers the production dropouts. The cost
        // is release latency, and nothing here is latency-critical on RELEASE:
        // a key with sustain, a button, or the scratch platter coasting an
        // extra fifth of a second are all imperceptible. Press edges are
        // untouched, so attack timing — the part you can hear — is unchanged.
        {
            static uint16_t lx, ly;
            static absolute_time_t t_last_dn;
            if (down) {
                lx = x; ly = y;
                t_last_dn = get_absolute_time();
            } else if (was_down && absolute_time_diff_us(
                           t_last_dn, get_absolute_time()) < 190000) {
                down = true; x = lx; y = ly;
            }
        }

        if (ignore_until_up) {
            // Swallow the touch that long-pressed into a page; resume once
            // the finger lifts.
            if (!down) ignore_until_up = false;
        } else if (page == PAGE_FX) {
            // Continuous XY tracking: X = pitch, Y = FX amount. The position
            // STICKS on release (toggle the pad on/off from the FX menu).
            if (down && !hit(&BTN_BACK, x, y)) {
                // X: bipolar about the PAINTED centre line (PAD_X + PAD_W/2).
                // The pad's own edges are past a comfortable thumb by design --
                // FX_PAD_OCTAVES widens the range instead of shrinking the pad,
                // so +/-1 OCT lands inside easy reach and the corners hold the
                // extra half-octave.
                // Normalised -1..+1 first, so the centre detent below keeps
                // its width in SCREEN pixels regardless of FX_PAD_OCTAVES.
                float oct = touch_norm_bipolar((float)x, PAD_X,
                                               PAD_X + PAD_W / 2,
                                               PAD_X + PAD_W,
                                               TRACK_PAD, TRACK_PAD);
                // Y needs no saturation: both ends are already overshootable
                // (the pad spans y 54..310 inside a 0..319 screen, so a finger
                // can travel past either end and simply clamp).
                float fx_y = ((float)y - PAD_Y) / (float)PAD_H;
                if (fx_y < 0) fx_y = 0; if (fx_y > 1) fx_y = 1;
                if (oct > -0.07f && oct < 0.07f) oct = 0.0f;   // center detent
                oct *= FX_PAD_OCTAVES;                         // -> octaves
                float amt = 1.0f - fx_y;                       // top = max FX
                if (amt < 0.05f) amt = 0.0f;
                pad_oct = oct; pad_amt = amt;
                pad_on = true;                    // dragging = auditioning it
                fx_set_params(pad_oct, pad_amt);
                fx_cursor((int)x, (int)y);
                pad_px = fx_cur_x; pad_py = fx_cur_y;   // clamped position
                fx_readout((int)(oct * 12.0f + (oct >= 0 ? 0.5f : -0.5f)),
                           (int)(amt * 100.0f + 0.5f));
            }
            if (down && !was_down && hit(&BTN_BACK, x, y)) {
                page = PAGE_FXMENU;
                draw_fxmenu_page();
                mic_ring_rephase();          // full-screen draw starved pump
                DIAG("page: fx menu (pad %s)\n", pad_on ? "held ON" : "off");
            }
        } else if (page == PAGE_GLI) {
            gli_indicator(fx_glitch_active() != 0);
            if (down) {
                if ((int)y >= SLIDER_STRIP_Y - 10 &&
                    (int)y <= SLIDER_STRIP_Y + SLIDER_STRIP_H + 10) {
                    int amt = slider_val((int)x, 1, 10);
                    if (amt != gli_amt) {
                        gli_amt = amt;
                        fx_glitch_freq(gli_amt);
                        draw_gli_slider();
                    }
                } else if (!was_down && hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;      // effect keeps its state
                    draw_fxmenu_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: fx menu\n");
                }
            }
        } else if (page == PAGE_SLAP) {
            if (down) {
                // Continuous slider drag (wide touch zone around the track).
                if ((int)y >= SLIDER_STRIP_Y - 10 &&
                    (int)y <= SLIDER_STRIP_Y + SLIDER_STRIP_H + 10) {
                    int changed = 0;
                    if (dly_sel == 0) {
                        int ms = slider_val((int)x, 40, 330);
                        if (ms != dly_ms) { dly_ms = ms; changed = 1; }
                    } else {
                        int v = slider_val((int)x, 0, 10);
                        if (dly_sel == 1 && v != dly_fb_amt) {
                            dly_fb_amt = v; changed = 1;
                        } else if (dly_sel == 2 && v != dly_mix_amt) {
                            dly_mix_amt = v; changed = 1;
                        }
                    }
                    if (changed) { dly_apply(); draw_slap_slider(); }
                } else if (!was_down && (hit(&BTN_DLY_TAB[0], x, y) ||
                                         hit(&BTN_DLY_TAB[1], x, y) ||
                                         hit(&BTN_DLY_TAB[2], x, y))) {
                    dly_sel = hit(&BTN_DLY_TAB[0], x, y) ? 0
                            : hit(&BTN_DLY_TAB[1], x, y) ? 1 : 2;
                    draw_dly_tabs();
                    draw_slap_slider();
                } else if (!was_down && hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;      // effect keeps its state
                    draw_fxmenu_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: fx menu\n");
                }
            }
        } else if (page == PAGE_MET) {
            if (down) {
                if ((int)y >= SLIDER_STRIP_Y - 10 &&
                    (int)y <= SLIDER_STRIP_Y + SLIDER_STRIP_H + 10) {
                    int amt = slider_val((int)x, 5, 60);
                    if (amt < 5)  amt = 5;
                    if (amt > 60) amt = 60;
                    if (amt != met_amt) {
                        met_amt = amt;
                        fx_metal_drive(met_amt);
                        draw_met_slider();
                    }
                } else if (!was_down && hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;      // effect keeps its state
                    draw_fxmenu_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: fx menu\n");
                }
            }
        } else if (page == PAGE_PARA) {
            if (down) {
                // Continuous RANGE drag (the pedal's foot-ridden knob).
                if ((int)y >= SLIDER_STRIP_Y - 10 &&
                    (int)y <= SLIDER_STRIP_Y + SLIDER_STRIP_H + 10) {
                    int rng = slider_val((int)x, 0, 10);
                    if (rng < 0)  rng = 0;
                    if (rng > 10) rng = 10;
                    if (rng != para_rng) {
                        para_rng = rng;
                        fx_para_range(para_rng);
                        draw_para_slider();
                    }
                } else if (!was_down && hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;      // effect keeps its state
                    draw_fxmenu_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: fx menu\n");
                }
            }
        } else if (page == PAGE_KNOB) {
            const knob_page_t *p = &KNOB_PAGES[knob_cur];
            // Every parameter has its own row now, so a press picks the row it
            // landed in and then drags THAT value. Once a drag owns a row it
            // keeps it until release, so sliding slightly up or down mid-sweep
            // cannot jump to the neighbouring parameter -- the same class of
            // problem as the keys page stealing a black key mid-slide.
            static int krow_drag = -1;
            if (down) {
                if (!was_down) {
                    krow_drag = -1;
                    for (int i = 0; i < p->n; i++) {
                        rect_t b = knob_row_band(i);
                        if (hit(&b, x, y)) { krow_drag = i; break; }
                    }
                    if (krow_drag >= 0 && krow_drag != knob_sel) {
                        int prev = knob_sel;
                        knob_sel = krow_drag;
                        draw_knob_row(prev);       // un-highlight the old handle
                        draw_knob_row(knob_sel);
                    }
                }
                if (krow_drag >= 0) {
                    const knob_t *k = &p->k[krow_drag];
                    // Continuous while dragging: this IS the sweep gesture.
                    // ROUND, do not truncate. Truncating broke every two-value
                    // knob outright: with span == 1, (x - X) * 1 / W is 0 for
                    // every pixel except the rightmost one, so FLANGER's
                    // FLANGE/MATRIX could not be dragged off FLANGE at all
                    // (bench-reported 2026-08-01: "I can not slide to matrix").
                    // Rounding also centres each detent and lets a 0..10 knob
                    // actually reach 10 at the right-hand end.
                    int span = k->hi_v - k->lo_v;
                    int ew   = KTRK_W - 2 * KTRK_PAD;   // saturating width
                    if (ew < 1) ew = 1;
                    int ex   = (int)x - (KTRK_X + KTRK_PAD);
                    if (ex < 0)  ex = 0;
                    if (ex > ew) ex = ew;
                    int v = k->lo_v + (ex * span * 2 + ew) / (2 * ew);
                    if (v < k->lo_v) v = k->lo_v;
                    if (v > k->hi_v) v = k->hi_v;
                    if (v != *k->value) {
                        *k->value = v; k->apply(v); draw_knob_row(krow_drag);
                    }
                } else if (!was_down && hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;
                    draw_fxmenu_page();
                    mic_ring_rephase();
                }
            } else if (was_down) {
                krow_drag = -1;
            }
        } else if (page == PAGE_VERB) {
            if (down) {
                if ((int)y >= SLIDER_STRIP_Y - 10 &&
                    (int)y <= SLIDER_STRIP_Y + SLIDER_STRIP_H + 10) {
                    // 0..10, unlike the drive knobs' 1..10 — fully dry is a
                    // legitimate setting for a reverb.
                    int amt = slider_val((int)x, 0, 10);
                    if (amt < 0)  amt = 0;
                    if (amt > 10) amt = 10;
                    if (amt != verb_mix_amt) {
                        verb_mix_amt = amt;
                        fx_verb_mix(verb_mix_amt);
                        draw_verb_slider();
                    }
                } else if (!was_down && hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;      // effect keeps its state
                    draw_fxmenu_page();
                    mic_ring_rephase();
                }
            }
        } else if (page == PAGE_OD) {
            if (down) {
                if ((int)y >= SLIDER_STRIP_Y - 10 &&
                    (int)y <= SLIDER_STRIP_Y + SLIDER_STRIP_H + 10) {
                    int amt = slider_val((int)x, 1, 10);
                    if (amt != od_amt) {
                        od_amt = amt;
                        fx_od_drive(od_amt);
                        draw_od_slider();
                    }
                } else if (!was_down && hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;      // effect keeps its state
                    draw_fxmenu_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: fx menu\n");
                }
            }
#if FX_ENABLE_TREMOLO
        } else if (page == PAGE_TREM) {
            if (down) {
                if ((int)y >= SLIDER_STRIP_Y - 10 &&
                    (int)y <= SLIDER_STRIP_Y + SLIDER_STRIP_H + 10) {
                    int amt = slider_val((int)x, 1, 10);
                    if (amt != trem_amt) {
                        trem_amt = amt;
                        fx_trem_rate(trem_amt);
                        draw_trem_slider();
                    }
                } else if (!was_down && hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;      // effect keeps its state
                    draw_fxmenu_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: fx menu\n");
                }
            }
#endif
#if FX_ENABLE_PHASER
        } else if (page == PAGE_PHASE) {
            if (down) {
                if ((int)y >= SLIDER_STRIP_Y - 10 &&
                    (int)y <= SLIDER_STRIP_Y + SLIDER_STRIP_H + 10) {
                    int amt = slider_val((int)x, 1, 10);
                    if (amt != ph_amt) {
                        ph_amt = amt;
                        fx_phase_rate(ph_amt);
                        draw_phase_slider();
                    }
                } else if (!was_down && hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;      // effect keeps its state
                    draw_fxmenu_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: fx menu\n");
                }
            }
#endif
        } else if (page == PAGE_TUNER) {
            if (down && !was_down && hit(&BTN_BACK, x, y)) {
                fx_tune_probe(0);
                ws2812_clear(); ws2812_show();   // hand the strip back to VU
                if (tuner_from_main) {           // quick-access entry
                    tuner_from_main = false;
                    page = PAGE_MAIN;
                    draw_main_page();
                    mic_ring_rephase();
                    DIAG("page: main\n");
                } else {
                    page = PAGE_FXMENU;
                    draw_fxmenu_page();
                    mic_ring_rephase();  // full-screen draw starved pump
                    DIAG("page: fx menu\n");
                }
            }
        } else if (page == PAGE_SYNTH) {
            if (down) {
                if (!was_down && hit(&BTN_BACK, x, y)) {
                    synth_gate(0);
                    syn_gate_ui = false; syn_tr_wob = 1.0f;
                    syn_cursor(-1, -1);
                    page = PAGE_MAIN;
                    draw_main_page();
                    DIAG("page: main\n");
                } else if (!was_down && hit(&BTN_S_SAW, x, y)) {
                    synth_wave(SYNTH_SAW);    draw_syn_waves();
                    synth_trace(syn_tr_x, syn_tr_y);
                } else if (!was_down && hit(&BTN_S_SIN, x, y)) {
                    synth_wave(SYNTH_SINE);   draw_syn_waves();
                    synth_trace(syn_tr_x, syn_tr_y);
                } else if (!was_down && hit(&BTN_S_SQR, x, y)) {
                    synth_wave(SYNTH_SQUARE); draw_syn_waves();
                    synth_trace(syn_tr_x, syn_tr_y);
                } else if (!was_down && hit(&BTN_S_HOLD, x, y)) {
                    syn_hold_on = !syn_hold_on;
                    draw_syn_hold();
                    if (!syn_hold_on) {
                        synth_gate(0);           // unlatch: note stops now
                        if (syn_gate_ui) {
                            syn_gate_ui = false;
                            syn_tr_wob = 1.0f;
                            syn_cursor(-1, -1);
                            synth_trace(syn_tr_x, syn_tr_y);
                        }
                    }
                    DIAG("synth: hold %s\n",
                         syn_hold_on ? "ON (hands-free)" : "OFF");
                } else if ((int)y >= SYN_PAD_Y) {
                    // X is PITCH (110 Hz * 2^2.5x), Y is filter cutoff.
                    // The pad is full width, so the sweep's extremes DO sit
                    // past a comfortable thumb; synth_xy() widens the range to
                    // 2.5 octaves so the reachable middle still carries more
                    // than one, rather than shrinking the pad to guarantee the
                    // corners. See the note there for the reach arithmetic.
                    float px = touch_norm((float)x, SYN_PAD_X,
                                          SYN_PAD_X + SYN_PAD_W,
                                          TRACK_PAD, TRACK_PAD);
                    // Y: saturate the TOP end only. The wave-selector row sits
                    // directly above the pad, so this branch is gated on
                    // y >= SYN_PAD_Y and a finger CANNOT overshoot upward --
                    // maximum cutoff previously required landing on the pad's
                    // single first row. The bottom needs nothing: the pad ends
                    // at y=298 with 21 px of screen below it to overshoot into.
                    float py = 1.0f - touch_norm((float)y, SYN_PAD_Y,
                                                 SYN_PAD_Y + SYN_PAD_H,
                                                 TRACK_PAD, 0.0f);
                    synth_xy(px, py);
                    synth_gate(1);
                    syn_gate_ui = true;
                    // The trace follows the finger (throttled ~11 Hz; a full
                    // redraw is ~930 one-column rects).
                    static absolute_time_t t_tr;
                    if (absolute_time_diff_us(t_tr, get_absolute_time())
                            > 90000) {
                        t_tr = get_absolute_time();
                        syn_cursor(-1, -1);  // lift cursor before the redraw
                        synth_trace(px, py);
                    }
                    syn_cursor((int)x, (int)y);
                }
            } else if (was_down) {
                if (!syn_hold_on) {
                    synth_gate(0);                 // finger up (unless held)
                    if (syn_gate_ui) {
                        syn_gate_ui = false;       // trace back to idle color
                        syn_tr_wob = 1.0f;
                        syn_cursor(-1, -1);
                        synth_trace(syn_tr_x, syn_tr_y);
                    }
                }
            }
        } else if (page == PAGE_THMN) {
            if (down) {
                if (!was_down && hit(&BTN_BACK, x, y)) {
                    synth_gate(0);
                    synth_level(1.0f);       // don't haunt the synth page
                    thm_gate_ui = false;
                    page = PAGE_MAIN;
                    draw_main_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: main\n");
                } else if (!was_down && hit(&BTN_TH_MODE, x, y)) {
                    thm_mag_mode = !thm_mag_mode;
                    // Leaving MAGNET mode ends its auto-gate (the poll
                    // re-gates it each tick while the mode is on).
                    if (!thm_mag_mode && !thm_hold_on && !thm_gate_ui)
                        synth_gate(0);
                    // Mag mode gets a snappier glide: at 100 Hz updates it
                    // can carry a wrist vibrato; light stays syrupy.
                    synth_glide(thm_mag_mode ? 0.0015f : 0.0006f);
                    draw_thm_page();         // labels change per mode
                    DIAG("thm: mode %s\n", thm_mag_mode ? "MAGNET" : "LIGHT");
                } else if (!was_down && hit(&BTN_TH_CAL, x, y) && thm_mag_mode) {
                    // MAGNET mode: CAL is now a single RE-ZERO.
                    //
                    // The old two-step teach (peak, then zero) existed because
                    // pitch came from |m| against a learned log2 span, and that
                    // span could not be discovered automatically. Pitch now comes
                    // from |m - b| against a running max, both of which learn
                    // themselves, so there is nothing left to teach -- and a
                    // two-step control that must be performed in a specific order
                    // with a magnet in the other hand was the worst part of this
                    // page to explain.
                    //
                    // What remains genuinely useful is snapping the baseline to
                    // "right now, nothing near", for when the resting field has
                    // shifted and the player would rather not wait for the slow
                    // adapt. Refuse it while a magnet is clearly present, so a
                    // stray press cannot zero the instrument mid-note.
                    if (thm_b_seeded && thm_dev_ema < 0.25f * thm_dev_hi) {
                        thm_bx = dbg_mx; thm_by = dbg_my; thm_bz = dbg_mz;
                        thm_dev_ema = 0.0f;
                        DIAG("thm: magCAL re-zero b=(%d,%d,%d) devhi=%d uT\n",
                             (int)dbg_mx, (int)dbg_my, (int)dbg_mz,
                             (int)thm_dev_hi);
                    } else {
                        DIAG("thm: magCAL refused (magnet near: dev=%d of %d)\n",
                             (int)thm_dev_ema, (int)thm_dev_hi);
                    }
                    thm_cal_flash = 60;
                    draw_thm_page();          // prompt changes with the step
                } else if (!was_down && hit(&BTN_TH_CAL, x, y)) {
                    // Anchor BOTH sensors to "right now": current light =
                    // 100% (the 3.5 octaves of shadow below it = the
                    // sweep; freezes light auto-cal until page re-entry),
                    // and current field = the no-magnet 0% baseline —
                    // instant fix when the ambient field jumps (an amp's
                    // speaker magnet arriving on the bench) instead of
                    // waiting out the slow auto-baseline. Keeps the taught
                    // magnet-contact peak by carrying the span across.
                    if (thm_lg_seeded) {
                        thm_lg_manual = true;
                        thm_lg_hi = thm_lg_ema;
                        thm_lg_lo = thm_lg_ema - 3.5f;
                    }
                    // LIGHT-mode CAL also re-zeros the magnet baseline, since
                    // "nothing near" is being asserted for both sensors at once.
                    if (thm_b_seeded) {
                        thm_bx = dbg_mx; thm_by = dbg_my; thm_bz = dbg_mz;
                        thm_dev_ema = 0.0f;
                    }
                    thm_cal_flash = 60;              // green for ~0.6 s
                    draw_button(&BTN_TH_CAL, C_GREEN, 2, "CAL");
                    DIAG("thm: CAL anchor (lux ~%d)\n", (int)exp2f(thm_lg_ema));
                } else if (!was_down && hit(&BTN_TH_OCTM, x, y)) {
                    if (thm_oct > -2) thm_oct--;
                    draw_thm_page();
                    DIAG("thm: oct %d\n", thm_oct);
                } else if (!was_down && hit(&BTN_TH_OCTP, x, y)) {
                    if (thm_oct < 2) thm_oct++;
                    draw_thm_page();
                    DIAG("thm: oct %d\n", thm_oct);
                } else if (!was_down && hit(&BTN_TH_HOLD, x, y)) {
                    thm_hold_on = !thm_hold_on;
                    draw_thm_hold();
                    if (!thm_hold_on && !thm_gate_ui) synth_gate(0);
                    DIAG("thm: hold %s\n", thm_hold_on ? "ON" : "OFF");
                } else if ((int)y >= THM_PAD_Y) {
                    // Strip X sets volume in BOTH modes now. In MAGNET mode it
                    // used to be the light sensor, which meant a passing shadow
                    // or a hand over the case produced random bursts and
                    // silences -- the reason magnet mode was unusable. Magnet
                    // mode is now magnet-only: pitch from the field, volume
                    // from where your other finger sits on this strip.
                    // Mapped across the reachable width, not 10..470: full
                    // scale at REACH_R with saturation at both ends. This was
                    // the first page to get the fix and it used to carry its
                    // own copy of the arithmetic; touch_norm() is that same
                    // formula, now shared by every continuous control so the
                    // remaining pages stop shipping the bug one at a time.
                    thm_touch_vol = touch_norm((float)x, THM_BAR_X,
                                               THM_BAR_X + THM_BAR_W,
                                               TRACK_PAD, TRACK_PAD);
                    synth_gate(1);
                    thm_gate_ui = true;
                }
            } else if (was_down) {
                if (thm_gate_ui) {
                    thm_gate_ui = false;
                    if (!thm_hold_on) synth_gate(0);
                }
            }
        } else if (page == PAGE_CAL) {
            if (down && !was_down) cal_touch((int)x, (int)y);
        } else if (page == PAGE_VIZ) {
            if (down && !was_down && viz_touch((int)x, (int)y)) {
                page = PAGE_MAIN;
                draw_main_page();
                mic_ring_rephase();          // full-screen draw starved pump
                DIAG("page: main\n");
            }
        } else if (page == PAGE_KEYS) {
            // Two-point note logic: the set of touched keys is rebuilt
            // every poll from both touch points (the FT6336 slots swap when
            // a finger lifts, so track semis as a SET, never by slot).
            // Newest key wins the voice; when it lifts, the still-held key
            // re-strikes ("hold C, tap F above, release F -> C returns").
            static int snd = -1;             // sounding semi
            static int prev_a = -1, prev_b = -1;   // last poll's touched set
            static int snd_x0 = -1, snd_x1 = -1;   // held key's x span
            static int snd_y0 = 0;                 // press y (tremolo ref)
            static bool snd_vlock = false;         // vertical-slide note lock
            // Swallow the carry-over touch that opened this page (see the KEYS
            // tile handler) until the finger actually lifts.
            if (keys_need_release) {
                if (!down && !down2) keys_need_release = false;
                else { down = down2 = false; }
            }
            int s1 = down  ? keys_semi_at((int)x,  (int)y)  : -1;
            int s2 = down2 ? keys_semi_at((int)x2, (int)y2) : -1;
            // Hold lock: the point owning the sounding key keeps it while
            // its x stays in the key's span (y is free for the tremolo).
            //
            // VERTICAL-SLIDE LOCK (family demo 2026-08-02): sliding up a white
            // key for tremolo would jump to a BLACK key the moment the finger
            // grazed one, because the x-span test above fails as soon as x
            // leaves the white key. A deliberate vertical drag is unambiguous
            // intent -- nobody slides up a key in order to change note -- so
            // once this gesture has travelled far enough vertically we latch
            // the note and stop consulting x at all until release. 18 px is
            // past the tremolo's own 8 px dead zone but well short of the
            // 90 px full-depth throw, so the lock is in place before the
            // vibrato is even audible.
            if (snd >= 0 && snd_x0 >= 0) {
                int oy_now = down ? (int)y : (down2 ? (int)y2 : -1);
                if (!snd_vlock && oy_now >= 0 && snd_y0 - oy_now >= 18)
                    snd_vlock = true;
                if (snd_vlock) {
                    // x is ignored: whichever point is still down keeps the note
                    if (down)       s1 = snd;
                    else if (down2) s2 = snd;
                } else if (down && (int)x >= snd_x0 && (int)x < snd_x1
                    && (int)y >= KB_Y) s1 = snd;
                else if (down2 && (int)x2 >= snd_x0 && (int)x2 < snd_x1
                         && (int)y2 >= KB_Y) s2 = snd;
            }
            if (!down && !down2) snd_vlock = false;   // gesture over
            // Chrome/chord buttons: point-1 press edges only.
            if (down && !was_down && s1 < 0) {
                if (hit(&BTN_BACK, x, y)) {
                    keys_release();
                    snd = -1; prev_a = prev_b = -1;
                    keys_vibrato(0.0f);
                    page = PAGE_MAIN;        // tail rings out (like drums)
                    draw_main_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: main (keys ring out)\n");
                } else if (hit(&BTN_K_INST, x, y)) {
                    keys_instrument(keys_instrument_get() == KEYS_EPIANO
                                    ? KEYS_STRINGS : KEYS_EPIANO);
                    draw_keys_inst();
                    DIAG("keys: inst %s\n",
                         keys_instrument_get() == KEYS_EPIANO
                         ? "E.PIANO" : "STRINGS");
                } else if (hit(&BTN_K_FX, x, y)) {
                    keys_to_fx = !keys_to_fx;
                    draw_keys_fx();
                    DIAG("keys: send to fx %s\n", keys_to_fx ? "ON" : "OFF");
                } else if (hit(&BTN_K_SUS, x, y)) {
                    keys_sustain(!keys_sustain_get());
                    draw_keys_sus();
                    DIAG("keys: sustain %s\n",
                         keys_sustain_get() ? "ON" : "OFF");
                } else if ((int)x < KB_X && (int)y >= 52) {
                    for (int i = 0; i < KEYS_NCHORDS; i++) {
                        rect_t r = keys_chord_rect(i);
                        if (hit(&r, x, y)) {
                            keys_sel = i;
                            draw_keys_chordbtns();
                            DIAG("keys: chord %s\n", KEYS_LBL[i]);
                            break;
                        }
                    }
                }
            }
            if (page == PAGE_KEYS) {         // (BACK may have left the page)
                // 1. arrivals: any touched semi not in last poll's set
                //    strikes and takes the voice (also = glissando drag).
                int struck_y = -1;
                if (s1 >= 0 && s1 != prev_a && s1 != prev_b && s1 != snd) {
                    keys_chord(keys_sel, s1);
                    snd = s1;
                    struck_y = (int)y;
                } else if (s2 >= 0 && s2 != prev_a && s2 != prev_b
                           && s2 != snd) {
                    keys_chord(keys_sel, s2);
                    snd = s2;
                    struck_y = (int)y2;
                }
                // 2. the sounding key left the set: fall back to the other
                //    held key (re-strike) or enter release.
                if (snd >= 0 && snd != s1 && snd != s2) {
                    int other = (s1 >= 0) ? s1 : s2;
                    if (other >= 0) {
                        keys_chord(keys_sel, other);
                        snd = other;
                        struck_y = (other == s1) ? (int)y : (int)y2;
                    } else {
                        keys_release();
                        snd = -1;
                        snd_x0 = snd_x1 = -1;
                    }
                }
                if (struck_y >= 0) {         // new strike: capture the key
                    keys_span_of(snd, &snd_x0, &snd_x1);
                    snd_y0 = struck_y;
                    snd_vlock = false;       // a real strike starts fresh
                    // One line per attack. A single sustained press must
                    // produce exactly ONE of these -- that is the whole test
                    // for the retrigger bug, and it is far sharper than
                    // counting onsets in a recording.
                    DIAG("keys: strike semi=%d\n", snd);
                }
                prev_a = s1; prev_b = s2;
                // 3. slide-up tremolo: dragging the holding finger UP the
                //    key deepens the vibrato; sliding back down fades it.
                int oy = (s1 == snd && snd >= 0) ? (int)y
                       : (s2 == snd && snd >= 0) ? (int)y2 : -1;
                if (oy >= 0) {
                    float d = ((float)(snd_y0 - oy) - 8.0f) * (1.0f / 90.0f);
                    keys_vibrato(d);         // clamps 0..1 itself
                } else {
                    keys_vibrato(0.0f);
                }
            }
        } else if (page == PAGE_EQ) {
            static bool eq_drag = false;
            static bool eq_qdrag = false;
            if (down && !was_down) {
                eq_drag = eq_qdrag = false;
                if (hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;      // EQ keeps running
                    draw_fxmenu_page();
                    DIAG("page: fx menu\n");
                } else if (hit(&BTN_EQTOG, x, y)) {
                    eq_on = !eq_on;
                    fx_eq_set(eq_on);
                    draw_button(&BTN_EQTOG, eq_on ? C_GREEN : C_GRAY, 2,
                                eq_on ? "ON" : "OFF");
                    DIAG("eq: %s\n", eq_on ? "ON" : "OFF");
                } else if ((int)y >= EQ_SLY - 18) {
                    eq_qdrag = true;         // Q slider strip
                } else if ((int)y >= EQ_PY - 10) {
                    // Select the nearest handle, then drag freq+gain.
                    int best = eq_sel, bd = 1 << 30;
                    for (int b = 0; b < 5; b++) {
                        int dx = (int)x - (EQ_PX + eq_freq_col(eq_f[b]));
                        int dy = (int)y - eq_gain_y(eq_g[b]);
                        int d = dx * dx + dy * dy;
                        if (d < bd) { bd = d; best = b; }
                    }
                    if (best != eq_sel) { eq_sel = best; eq_draw_readout();
                                          eq_draw_qslider(); }
                    eq_drag = true;
                }
            }
            if (down && eq_drag && (int)y >= EQ_PY - 10 && (int)y < EQ_SLY - 18) {
                int col = (int)x - EQ_PX;
                if (col < 0) col = 0;
                if (col >= EQ_PW) col = EQ_PW - 1;
                float db = ((float)(EQ_PY + EQ_PH / 2) - (float)y)
                         * (EQ_DB_RANGE / (float)(EQ_PH / 2));
                if (db >  12.0f) db =  12.0f;
                if (db < -12.0f) db = -12.0f;
                eq_f[eq_sel] = eq_col_freq(col);
                eq_g[eq_sel] = db;
                fx_eq_band(eq_sel, eq_f[eq_sel], eq_g[eq_sel], eq_qv[eq_sel]);
                eq_dirty = true;             // plot redraw is throttled
            } else if (down && eq_qdrag) {
                // End-saturated like every other strip: Q=8 sat at the top of
                // the track needed the literal last pixel before.
                float t = touch_norm((float)x, EQ_SLX, EQ_SLX + EQ_SLW,
                                     TRACK_PAD, TRACK_PAD);
                eq_qv[eq_sel] = 0.4f * exp2f(t * 4.322f);   // 0.4..8 log
                fx_eq_band(eq_sel, eq_f[eq_sel], eq_g[eq_sel], eq_qv[eq_sel]);
                eq_draw_qslider();
                eq_draw_readout();
                eq_dirty = true;
            }
        } else if (page == PAGE_TUNE) {
            if (down && !was_down) {
                if (hit(&BTN_T_KEY, x, y)) {
                    tune_key = (tune_key + 1) % 12;
                    fx_tune_key(tune_key);
                    draw_tune_ctls();
                    DIAG("orcatune: key %s\n", TUNE_KEYS[tune_key]);
                } else if (hit(&BTN_T_SCALE, x, y)) {
                    tune_scale = (tune_scale + 1) % 9;
                    fx_tune_scale(tune_scale);
                    draw_tune_ctls();
                    DIAG("orcatune: scale %s\n", TUNE_SCALES[tune_scale]);
                } else if (hit(&BTN_T_MODE, x, y)) {
                    tune_robot = !tune_robot;
                    fx_tune_mode(tune_robot);
                    draw_tune_ctls();
                    DIAG("orcatune: %s\n", tune_robot ? "robotic" : "natural");
                } else if (hit(&BTN_BACK, x, y)) {
                    page = PAGE_FXMENU;      // effect keeps its state
                    draw_fxmenu_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: fx menu\n");
                }
            }
        } else if (page == PAGE_DRUMS) {
            static bool dr_bpm_drag = false;
            static int dr_tab_press = -1;          // tab held since t0:
            static absolute_time_t dr_tab_t0;      // tap=select, hold=copy
            if (down && !was_down) {
                dr_bpm_drag = false;
                dr_tab_press = -1;
                if (hit(&BTN_BACK, x, y)) {
                    page = PAGE_MAIN;          // beat keeps playing
                    draw_main_page();
                    DIAG("page: main (drums %s)\n",
                         drums_playing() ? "still playing" : "stopped");
                } else if (hit(&BTN_D_FX, x, y)) {
                    drums_to_fx = !drums_to_fx;
                    draw_drum_fx();
                    DIAG("drums: send to fx %s\n", drums_to_fx ? "ON" : "OFF");
                } else if (hit(&BTN_D_PLAY, x, y)) {
                    drums_play(!drums_playing());
                    if (!drums_playing()) {
                        draw_drum_playhead(-1);
                        draw_drum_banks();       // drop the green tab
                    }
                    draw_drum_play();
                    DIAG("drums: %s (%d bpm)\n",
                         drums_playing() ? "PLAY" : "STOP", drums_bpm_get());
                } else if (hit(&BTN_D_CLEAR, x, y)) {
                    drums_clear();
                    for (int v = 0; v < DRUM_VOICES; v++)
                        for (int st = 0; st < DRUM_STEPS; st++)
                            drum_cell(v, st);
                    DIAG("drums: cleared bank %c\n", 'A' + drums_bank_get());
                } else if (hit(&BTN_D_LEN, x, y)) {
                    drums_len_set(drums_len_get() % DRUM_BANKS + 1);
                    draw_drum_banks();
                    DIAG("drums: chain len %d\n", drums_len_get());
                } else if ((int)y >= 244 && (int)y < 284 && (int)x >= 244
                           && (int)x < 416) {
                    // Bank tab A..D: acted on at RELEASE — tap selects,
                    // hold >= 600 ms copies the current bank into it.
                    dr_tab_press = ((int)x - 244) / 44;
                    dr_tab_t0 = get_absolute_time();
                } else if ((int)y >= DR_GY && (int)y < DR_GY + 5 * DR_RH &&
                           (int)x >= DR_GX) {
                    int st = ((int)x - DR_GX) / DR_CW;
                    int v = ((int)y - DR_GY) / DR_RH;
                    if (st < DRUM_STEPS && v < DRUM_VOICES) {
                        drums_toggle(v, st);
                        drum_cell(v, st);
                    }
                } else if ((int)y >= 288) {
                    dr_bpm_drag = true;
                }
            }
            if (down && dr_bpm_drag && (int)y >= 288) {
                // Was unclamped as well as unsaturated: x below SLIDER_X gave
                // 58 BPM and below, under the intended 60 floor.
                int b = slider_val((int)x, 60, 184);
                if (b != drums_bpm_get()) {
                    drums_bpm_set(b);
                    draw_drum_bpm();
                }
            }
            if (!down && was_down && dr_tab_press >= 0) {
                int b   = dr_tab_press;
                int cur = drums_bank_get();
                dr_tab_press = -1;
                bool held = absolute_time_diff_us(
                                dr_tab_t0, get_absolute_time()) >= 600000;
                if (held && b != cur) {
                    drums_copy(cur, b);
                    // A copy you can't hear is a trap: grow the chain to
                    // reach the copy's bank.
                    if (b >= drums_len_get()) drums_len_set(b + 1);
                    DIAG("drums: copied bank %c -> %c (len %d)\n",
                         'A' + cur, 'A' + b, drums_len_get());
                }
                if (b != cur) {
                    draw_drum_playhead(-1);    // strip off the old bank
                    drums_bank_set(b);
                    for (int v = 0; v < DRUM_VOICES; v++)
                        for (int st = 0; st < DRUM_STEPS; st++)
                            drum_cell(v, st);
                    DIAG("drums: edit bank %c\n", 'A' + b);
                }
                draw_drum_banks();
            }
        } else if (page == PAGE_VOX) {
            // Presses are classified at the down edge and stay sticky until
            // release: slot buttons (tap = select, hold = record) vs the
            // turntable pad (hold = play, X drag = speed/direction).
            if (down && !was_down) {
                vox_press_slot = -1;
                vox_in_pad = false;
                if (hit(&BTN_BACK, x, y)) {
                    if (vox_recording()) vox_rec_stop();
                    vox_play_release();      // HOLD (if latched) keeps looping
                    page = PAGE_MAIN;
                    draw_main_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: main (sample %s)\n",
                         vox_held() ? "looping HELD" : "stopped");
                } else if (hit(&BTN_V_HOLD, x, y)) {
                    vox_hold_set(!vox_held());
                    draw_vox_hold();
                    draw_vox_status();
                    DIAG("vox: hold %s\n", vox_held() ? "ON (hands-free)" : "OFF");
                } else if (hit(&BTN_V_SRC, x, y)) {
                    // Sampler input toggles MIC IN <-> PDM MICS (mic stays
                    // on). Park the pump around the switch (see BTN_SRC).
                    bool was = playing;
                    playing = false;
                    if (was) { codec_nau88c10_dac_mute(true); sleep_ms(9); }
                    use_pdm = !use_pdm;
                    if (use_pdm) pdm_input_reset();
                    if (was) { tx_play(silence_buf, 64); mic_armed = false; }
                    playing = was;
                    if (was) { sleep_ms(2); codec_nau88c10_dac_mute(false); }
                    vol_clamp_quiet();       // PDM can't run loud
                    draw_vox_src();
                    draw_vox_vol();
                    draw_vox_status();
                    DIAG("vox: source %s\n", use_pdm ? "pdm mics" : "jack");
                } else if (hit(&BTN_V_VOL, x, y)) {
                    vol_cycle();
                    draw_vox_vol();
                    DIAG("vox: %s\n", VOL_STEPS[vol_idx].label);
                } else {
                    for (int i = 0; i < VOX_UI_SLOTS; i++)
                        if (hit(&BTN_V_SLOT[i], x, y)) { vox_press_slot = i; break; }
                    if (vox_press_slot >= 0) {
                        vox_press_t = get_absolute_time();
                    } else if ((int)y >= VPAD_Y && (int)y < VPAD_Y + VPAD_H) {
                        vox_in_pad = true;
                        // Needle drops already spinning at the press rate:
                        // right of center = forward from the start, left of
                        // center = reverse from the end.
                        // Bipolar about the painted detent, full scale at
                        // Band edges ARE +/-1x now, so 1x needs no stretch:
                        // ("100% playback speed") could not be selected.
                        float r = touch_norm_bipolar((float)x, VPAD_X,
                                                     VPAD_X + VPAD_W / 2,
                                                     VPAD_X + VPAD_W,
                                                     TRACK_PAD, TRACK_PAD);
                        if (r > -0.04f && r < 0.04f) r = 0.0f;
                        vox_play_press(r);
                    }
                }
            }
            if (down && vox_press_slot >= 0) {
                if (!vox_recording() &&
                    absolute_time_diff_us(vox_press_t, get_absolute_time())
                        > 600000) {
                    vox_rec_start(vox_press_slot);   // held: record until release
                    draw_vox_slots();
                    draw_vox_status();
                    vwf_rec_begin();
                    DIAG("vox: REC %s...\n", BTN_COLOR_NAMES[vox_press_slot]);
                } else if (vox_recording()) {
                    static unsigned last_ds = 0xFFFF;
                    unsigned ds = vox_rec_ds();
                    if (ds != last_ds) { last_ds = ds; draw_vox_status(); }
                }
            } else if (down && vox_in_pad) {
                // X -> rate: right = 1x forward, left = 1x reverse, small
                // center dead zone holds the platter still. Same reachable
                // mapping as the needle drop above -- they must agree, or a
                // drag would jump the rate the instant the finger moved.
                float r = touch_norm_bipolar((float)x, VPAD_X,
                                             VPAD_X + VPAD_W / 2,
                                             VPAD_X + VPAD_W,
                                             TRACK_PAD, TRACK_PAD);
                if (r > -0.04f && r < 0.04f) r = 0.0f;
                vox_play_rate(r);
                vox_cursor((int)x, (int)y);
            } else if (!down && was_down) {
                if (vox_press_slot >= 0) {
                    if (vox_recording()) {
                        vox_rec_stop();
                        DIAG("vox: REC %s done (%u.%us)\n",
                             BTN_COLOR_NAMES[vox_press_slot],
                             vox_len_ds(vox_press_slot) / 10u,
                             vox_len_ds(vox_press_slot) % 10u);
                    } else {
                        vox_select(vox_press_slot);
                        DIAG("vox: slot %s selected\n",
                             BTN_COLOR_NAMES[vox_press_slot]);
                    }
                    draw_vox_slots();
                    draw_vox_status();
                    vwf_rescan();            // new slot / freshly committed take
                    vwf_draw_all();
                    vox_press_slot = -1;
                }
                if (vox_in_pad) {
                    vox_play_release();      // needle lifts
                    vox_in_pad = false;
                    vox_cursor(-1, -1);
                }
            }
        } else if (page == PAGE_FXMENU) {
            // TAP a button = toggle that effect (any combination can run at
            // once); HOLD ~0.6 s = open its settings page. Sliding off the
            // button cancels the press.
            if (down && !was_down) {
                if (hit(&BTN_BACK, x, y)) {
                    page = PAGE_MAIN;        // effects keep running
                    draw_main_page();
                    mic_ring_rephase();      // full-screen draw starved pump
                    DIAG("page: main\n");
                } else {
                    menu_press_btn = menu_btn_at(x, y);
                    menu_press_t = get_absolute_time();
                }
            } else if (down && menu_press_btn >= 0) {
                int64_t el = absolute_time_diff_us(menu_press_t,
                                                   get_absolute_time());
                if (menu_btn_at(x, y) != menu_press_btn) {
                    draw_menu_btn(menu_press_btn);   // erase the hold bar
                    menu_press_btn = -1;     // slid off: cancel
                } else if (el > 450000) {    // was 600 ms: felt sluggish
                    menu_enter_page(menu_press_btn);
                    ignore_until_up = true;  // don't leak into the new page
                    menu_press_btn = -1;
                } else if (el > 100000) {
                    // Keep holding: a white bar fills along the button's
                    // bottom edge so the user can SEE the hold arming.
                    const rect_t *r = MENU_RECTS_AT(menu_press_btn);
                    int w = (int)((el - 100000) * (int64_t)(r->w - 8)
                                  / 350000);
                    st7796_fill_rect(r->x + 4, r->y + r->h - 9, w, 5,
                                     rgb565_be(255, 255, 255));
                }
            } else if (!down && was_down && menu_press_btn >= 0) {
                menu_toggle(menu_press_btn);           // released early = tap
                menu_press_btn = -1;
            }
        } else if (page == PAGE_MAIN && down && !was_down) {
            // (must be page-guarded: this used to be the bare fallthrough,
            // which swallowed SETUP-page taps — tapping OUT opened VIZ
            // because the tile behind it got the hit)
            if (hit(&BTN_GFX, x, y)) {
                page = PAGE_FXMENU;
                // The FX menu is a live playing surface: make sure the mic
                // path runs. Fresh start only if it wasn't already live so
                // re-entering the menu never resets active effects.
                if (!(playing && mic)) {
                    // Park the pump, set up the path, THEN resume — the old
                    // order let core 1 process a block mid-fx_reset().
                    playing = false;
                    codec_nau88c10_dac_mute(true);   // click-free swap
                    sleep_ms(9);
                    tx_play(silence_buf, 64);
                    fx_reset();
                    fx_apply_all();
                    if (use_pdm) pdm_input_reset();   // drop stale capture
                    mic = true;
                    mic_armed = false;
                    playing = true;
                    sleep_ms(2);
                    codec_nau88c10_dac_mute(false);
                }
                draw_fxmenu_page();
                mic_ring_rephase();          // full-screen draw starved pump
                DIAG("page: fx menu (mic live)\n");
            } else if (hit(&BTN_VOX, x, y)) {
                page = PAGE_VOX;
                // Same live-arm as the FX menu: the sampler needs the input
                // path running to record.
                if (!(playing && mic)) {
                    playing = false;             // park -> set up -> resume
                    sleep_ms(7);
                    tx_play(silence_buf, 64);
                    fx_reset();
                    fx_apply_all();
                    if (use_pdm) pdm_input_reset();
                    mic = true;
                    mic_armed = false;
                    playing = true;
                    sleep_ms(2);
                    codec_nau88c10_dac_mute(false);
                }
                draw_vox_page();
                mic_ring_rephase();          // full-screen draw starved pump
                DIAG("page: sample\n");
            } else if (hit(&BTN_SYN, x, y)) {
                page = PAGE_SYNTH;
                // Synth needs the audio path live (it replaces the input).
                if (!(playing && mic)) {
                    playing = false;
                    codec_nau88c10_dac_mute(true);   // click-free swap
                    sleep_ms(9);
                    tx_play(silence_buf, 64);
                    fx_reset();
                    fx_apply_all();
                    if (use_pdm) pdm_input_reset();
                    mic = true;
                    mic_armed = false;
                    playing = true;
                    sleep_ms(2);
                    codec_nau88c10_dac_mute(false);
                }
                synth_reset();
                draw_synth_page();
                DIAG("page: synth\n");
            } else if (hit(&BTN_DRM, x, y)) {
                page = PAGE_DRUMS;
                if (!(playing && mic)) {       // drums mix into the live path
                    playing = false;
                    codec_nau88c10_dac_mute(true);
                    sleep_ms(9);
                    tx_play(silence_buf, 64);
                    fx_reset();
                    fx_apply_all();
                    if (use_pdm) pdm_input_reset();
                    mic = true;
                    mic_armed = false;
                    playing = true;
                    sleep_ms(2);
                    codec_nau88c10_dac_mute(false);
                }
                drum_ph_drawn = -1;
                draw_drums_page();
                DIAG("page: drums\n");
            } else if (hit(&BTN_THM, x, y)) {
                page = PAGE_THMN;
                // Same live-arm as the synth (the voice replaces the input).
                if (!(playing && mic)) {
                    playing = false;
                    codec_nau88c10_dac_mute(true);   // click-free swap
                    sleep_ms(9);
                    tx_play(silence_buf, 64);
                    fx_reset();
                    fx_apply_all();
                    if (use_pdm) pdm_input_reset();
                    mic = true;
                    mic_armed = false;
                    playing = true;
                    sleep_ms(2);
                    codec_nau88c10_dac_mute(false);
                }
                synth_reset();
                synth_wave(SYNTH_SINE);              // theremin voice
                synth_xy(0.5f, 0.8f);                // open the filter (~3 kHz)
                synth_glide(thm_mag_mode ? 0.0015f : 0.0006f);   // portamento
                thm_hold_on = false; thm_gate_ui = false;
                thm_lg_seeded = false;                   // re-cal to the room
                thm_b_seeded  = false;                   // and re-zero the magnet
                thm_lg_manual = false; thm_cal_flash = 0;
                draw_thm_page();
                mic_ring_rephase();          // full-screen draw starved pump
                DIAG("page: theremin\n");
            } else if (hit(&BTN_KEY, x, y)) {
                page = PAGE_KEYS;
                // The KEYS tile sits where the piano will be drawn, so the very
                // touch that opens the page used to land on a key and sound an
                // unwanted note (verified 2026-08-03: entering struck C). Make
                // the page wait for a fresh press before it will strike.
                keys_need_release = true;
                // Chords mix into the live path (same arm as drums).
                if (!(playing && mic)) {
                    playing = false;
                    codec_nau88c10_dac_mute(true);
                    sleep_ms(9);
                    tx_play(silence_buf, 64);
                    fx_reset();
                    fx_apply_all();
                    if (use_pdm) pdm_input_reset();
                    mic = true;
                    mic_armed = false;
                    playing = true;
                    sleep_ms(2);
                    codec_nau88c10_dac_mute(false);
                }
                draw_keys_page();
                mic_ring_rephase();          // full-screen draw starved pump
                DIAG("page: keys\n");
            } else if (hit(&BTN_VIZ, x, y)) {
                page = PAGE_VIZ;
                // Live-arm like the FX menu: the visualizer shows the DAC
                // mix, so make sure there is one.
                if (!(playing && mic)) {
                    playing = false;
                    codec_nau88c10_dac_mute(true);
                    sleep_ms(9);
                    tx_play(silence_buf, 64);
                    fx_reset();
                    fx_apply_all();
                    if (use_pdm) pdm_input_reset();
                    mic = true;
                    mic_armed = false;
                    playing = true;
                    sleep_ms(2);
                    codec_nau88c10_dac_mute(false);
                }
                viz_open();
                mic_ring_rephase();          // full-screen draw starved pump
                DIAG("page: viz\n");
            } else if (hit(&BTN_PLAY, x, y)) {
                if (!playing) {
                    if (use_pdm) pdm_input_reset();   // drop stale capture
                    if (mic) {
                        mic_armed = false;            // pump arms on 1st block
                        playing = true;
                    } else {
                        tx_retarget(tone_buf, TONE_FRAMES, false, true);
                    }
                    draw_button(&BTN_PLAY, C_RED, 2, "STOP");
                    DIAG("play: %s -> %s\n", mic ? "mic" : "tone",
                         jack ? "jack" : "speaker");
                } else {
                    tx_retarget(silence_buf, 64, mic, false);
                    draw_button(&BTN_PLAY, C_GREEN, 2, "PLAY");
                    DIAG("play: STOP\n");
                }
                draw_status(C_BG, playing, mic, jack);
            } else if (hit(&BTN_SETUP, x, y)) {
                page = PAGE_SETUP;
                draw_setup_page();
                mic_ring_rephase();          // full-screen draw starved pump
                DIAG("page: setup\n");
            } else if (hit(&BTN_TUNERQ, x, y)) {
                // Quick-access tuner (it's a utility, not an effect).
                // Needs the live input path for pitch detection.
                if (!(playing && mic)) {
                    playing = false;
                    codec_nau88c10_dac_mute(true);
                    sleep_ms(9);
                    tx_play(silence_buf, 64);
                    fx_reset();
                    fx_apply_all();
                    if (use_pdm) pdm_input_reset();
                    mic = true;
                    mic_armed = false;
                    playing = true;
                    sleep_ms(2);
                    codec_nau88c10_dac_mute(false);
                }
                tuner_from_main = true;
                page = PAGE_TUNER;
                fx_tune_probe(1);            // detection-only, audio stays dry
                draw_tuner_page();
                mic_ring_rephase();
                DIAG("page: tuner (quick access)\n");
            }
        } else if (page == PAGE_SETUP && down && !was_down) {
            if (hit(&BTN_BACK, x, y)) {
                page = PAGE_MAIN;
                draw_main_page();
                mic_ring_rephase();          // full-screen draw starved pump
                DIAG("page: main\n");
            } else if (hit(&BTN_SRC, x, y)) {
                // Cycle TONE 1KHZ -> MIC IN (jack) -> PDM MICS (4-mic array).
                // Park the pump first: flags and the TX ring must change
                // while core 1 is guaranteed off the audio path.
                bool was = playing;
                playing = false;
                if (was) { codec_nau88c10_dac_mute(true); sleep_ms(9); }
                if (!mic)          { mic = true;  use_pdm = false; }
                else if (!use_pdm) { use_pdm = true; pdm_input_reset(); }
                else               { mic = false; use_pdm = false; }
                if (was) {                        // hot-switch the source
                    mic_armed = false;
                    if (mic) tx_play(silence_buf, 64);
                    else     tx_play(tone_buf, TONE_FRAMES);
                }
                playing = was;
                if (was) { sleep_ms(2); codec_nau88c10_dac_mute(false); }
                vol_clamp_quiet();                // PDM can't run loud
                if (mic && !use_pdm)
                    draw_button(&BTN_IN, in_idx ? C_PURPLE : C_GRAY, 2,
                                IN_PROFILES[in_idx].label);
                else
                    st7796_fill_rect(BTN_IN.x, BTN_IN.y, BTN_IN.w, BTN_IN.h,
                                     C_BG);
                draw_button(&BTN_SRC, mic ? C_PURPLE : C_GRAY, 2,
                            mic ? (use_pdm ? "IN: PDM MICS" : "IN: 3.5MM") : "IN: 1KHZ TONE");
                draw_button(&BTN_VOL, vol_idx == 2 ? C_RED : C_GRAY, 2,
                            VOL_STEPS[vol_idx].label);
                DIAG("source: %s\n",
                     mic ? (use_pdm ? "pdm 4-mic array" : "jack passthrough")
                         : "tone");
                draw_status(C_BG, playing, mic, jack);
            } else if (mic && !use_pdm && hit(&BTN_IN, x, y)) {
                // Profile button exists only for the 3.5mm jack source.
                in_idx = (in_idx + 1) % IN_PROFILE_COUNT;
                apply_in_profile(in_idx);
                draw_button(&BTN_IN, in_idx ? C_PURPLE : C_GRAY, 2,
                            IN_PROFILES[in_idx].label);
            } else if (hit(&BTN_OUT, x, y)) {
                jack = !jack;
                spk_route = !jack;           // audio core: engage speaker HP
                codec_nau88c10_set_output(jack ? CODEC_OUT_HEADPHONE
                                               : CODEC_OUT_SPEAKER);
                // Guard: hot levels are jack-only (speaker distorts, PDM
                // feeds back) — clamp on switch-over to speaker.
                vol_clamp_quiet();
                draw_button(&BTN_VOL, vol_idx == 2 ? C_RED : C_GRAY, 2,
                            VOL_STEPS[vol_idx].label);
                draw_button(&BTN_OUT, jack ? C_BLUE : C_GRAY, 2,
                            jack ? "OUT: 3.5MM" : "OUT: SPEAKER");
                DIAG("route: output -> %s\n", jack ? "3.5mm jack" : "speaker");
                codec_nau88c10_log_output();
                draw_status(C_BG, playing, mic, jack);
            } else if (hit(&BTN_VOL, x, y)) {
                vol_cycle();                      // guard: loud is jack+non-PDM
                draw_button(&BTN_VOL, vol_idx == 2 ? C_RED : C_GRAY, 2,
                            VOL_STEPS[vol_idx].label);
            } else if (hit(&BTN_APWR, x, y)) {
                // Ask the coprocessor for rails. Same policy as the boot
                // block: live rails OR needed, radios/compute never added
                // (see the policy block above VOICE_SLOT). The device
                // applies masks as a blocking walk, so the result takes a
                // moment.
                draw_button(&BTN_APWR, C_GRAY, 2, "REQUESTING...");
                uint32_t live = 0;
                picpwr_rails(&live);
                picpwr_cfg_t req = { .awake = (live | DEMO_ZONES_NEEDED)
                                              & ~DEMO_ZONES_FORBIDDEN,
                                     .sleep = DEMO_ZONES_SLEEP,
                                     .wake = 0, .wake2 = 0 };
                bool sent = picpwr_send(&req);
                uint32_t rails = 0;
                for (int i = 0; i < 100; i++) {   // ride out the rail walk
                    fw2_app_recovery_task();
                    sleep_ms(25);
                }
                bool on = picpwr_rails(&rails) &&
                          (rails & picpwr_zone_bit(PICPWR_ZONE_AUDIO));
                draw_button(&BTN_APWR, on ? C_GREEN : C_RED, 2,
                            on ? "AUDIO PWR OK" : "AUDIO PWR ?");
                DIAG("picpwr: request sent=%d rails=0x%06x audio=%s\n",
                     (int)sent, (unsigned)rails, on ? "ON" : "off");
                if (on) {                          // bring the codec up now
                    codec_nau88c10_init();
                    nau_write(0x07, 0x0000);
                    codec_ok = codec_nau88c10_input_ok();
                    apply_in_profile(in_idx);
                    apply_vol(vol_idx);
                    codec_nau88c10_set_output(jack ? CODEC_OUT_HEADPHONE
                                                   : CODEC_OUT_SPEAKER);
                    DIAG("codec: re-init after rail up, input %s\n",
                         codec_ok ? "ok" : "FAIL");
                }
            }
        }
        was_down = down;

        // Mic pump + LED meter. Blocks complete every ~16 ms; consume promptly
        // (the pointer aliases a live ping-pong buffer, ~one block period).
        // Audio runs entirely on core 1 — the UI just mirrors its meters.
        // (WS2812 is core-0-only hardware: two cores pushing the same PIO
        // FIFO would interleave pixels.)
        {
            static uint32_t led_blk = 0;
            uint32_t pb = pump_blocks;
            if (pb != led_blk && page != PAGE_TUNER) {
                led_blk = pb;
                vu_led_meter(pump_vu_avg, gli_on && fx_glitch_active());
            }
        }
        // Audio-core load: busy-us inside audio_pump_block vs wall time,
        // 2 s cadence. THE number for "can we afford another effect".
        {
            static uint32_t t_load = 0, busy_last = 0;
            uint32_t now = time_us_32();
            if ((uint32_t)(now - t_load) >= 2000000u) {
                if (t_load != 0) {
                    uint32_t bd = pump_busy_us - busy_last;
                    uint32_t wd = now - t_load;
                    DIAG("load: audio core %d%% (%d us busy / 2 s)\n",
                         (int)((uint64_t)bd * 100u / wd), (int)bd);
                }
                t_load = now;
                busy_last = pump_busy_us;
            }
        }
        sleep_ms(2);   // UI poll pace; audio no longer depends on this loop
    }
}
