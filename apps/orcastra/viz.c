// viz.c — audio visualizer page. See viz.h.
//
// Rendering: the region below a 24 px chrome bar (rows 24..319, 296 lines)
// is drawn as 8 strips of 37 lines, double-buffered: strip N+1 is computed
// on the CPU while strip N rides the SPI DMA (st7796_flush_async). At the
// panel's proven SPI rate a full frame lands in ~35-40 ms (~25 fps), and
// the per-pixel work below fits inside the DMA time, so the page is
// flush-bound, not compute-bound.
//
// Analysis (bench-tuned against a real guitar, 2026-07-28): one 512-point
// radix-2 FFT per frame over the newest 512 samples (Hann, float; 95 Hz
// bins so a low-E fundamental resolves). SPECTRUM spans 95 Hz..8 kHz —
// the previous 24 kHz span left the right half of the screen permanently
// dark on instruments. SCOPE auto-gains like a real oscilloscope (peak
// follower, floor 300 counts) so soft playing still draws. PLASMA maps
// LOUDNESS -> COLOR (quiet = slow dark ink, loud = fire) on a heat
// palette, and bass pumps the animation speed — a rule you can see.
#include "viz.h"
#include "display/st7796.h"
#include "pico/stdlib.h"        // __uninitialized_psram linker placement
#include <math.h>
#include <string.h>

#define FFT_N    512
#define RING_MSK (FFT_N - 1)
#define DECIM    8                   // analyzer rate = FX_FS/8 ≈ 6.1 kHz
#define BARS     32
// Display span, set from a recorded range take on the user's guitar
// (2026-07-28: fundamentals 82.9..1116 Hz, strums 90% < 1.5 kHz, 99% of
// all energy < 4.4 kHz). At 11.9 Hz/bin: bin 6 ≈ 72 Hz .. bin 218 ≈
// 2.6 kHz — low E and open A resolve into separate bars, the highest
// fret lands ~80% across, harmonics own the right edge.
#define BIN_LO   6
#define BIN_HI   218
#define CHROME_H 32
#define REGION_H (ST7796_H - CHROME_H)     // 288
#define STRIP_H  24                        // 12 strips x 24 = 288 (24-line
#define STRIPS   (REGION_H / STRIP_H)      // strips halve the SRAM cost vs
                                           // 37-line ones; SPI is the limit)
#define BIN_TOP  84                        // 8 kHz at 95.4 Hz/bin

// ---- shared with the audio core ----
static volatile int16_t viz_ring[FFT_N];   // rolling DAC-mix history
static volatile int     viz_pos;           // next write (torn reads are fine)

void viz_feed(const int16_t *b, int n) {
    // Decimate x8 with a box average (cheap anti-alias; guitar has ~no
    // energy near the 3 kHz fold). 512 decimated samples = an 84 ms
    // analysis window at 11.9 Hz/bin.
    int p = viz_pos;
    for (int i = 0; i + DECIM <= n; i += DECIM) {
        int acc = 0;
        for (int k = 0; k < DECIM; k++) acc += b[i + k];
        viz_ring[p] = (int16_t)(acc >> 3);
        p = (p + 1) & RING_MSK;
    }
    viz_pos = p;
}

// ---- local pixel helpers ----
static inline uint16_t px_be(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t c = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8));
}

// ---- state ----
// SRAM strip double-buffer. NOTE (2026-07-29): a full PSRAM double
// framebuffer + single-DMA frame was tried (kills the strip-cadence
// shear bands) but the board suffered two display/LED power-loss
// incidents on that build in one afternoon vs zero all day on strips —
// intermittent, not reproduced on demand (3-min soak survived), so the
// conservative pipeline is back until the PIC power-controller questions
// are answered. The PSRAM approach lives in git history (34c1c6b).
static uint16_t strip_buf[2][ST7796_W * STRIP_H];
static int   viz_mode = 0;                          // 0 spec, 1 scope, 2 plasma

// Pick the mode before viz_open(). The PAGE button jumps straight here
// from anywhere and wants plasma: spectrum and scope are informative,
// plasma is the one that makes someone stop and look.
void viz_mode_set(int mode) {
    if (mode < 0) mode = 0;
    if (mode > 2) mode = 2;
    viz_mode = mode;
}
static const char *MODE_NAMES[3] = { "SPECTRUM", "SCOPE", "PLASMA" };

// analysis products (computed once per frame)
static int16_t snap[FFT_N];          // time-ordered UI-side copy
static float  bh[BARS], bcap[BARS];  // bar heights + caps, 0..1
static int    bar_edge[BARS + 1];    // FFT bin edges (log-spaced)
static float  bar_tilt[BARS];        // display tilt (dB) per bar
static int    scope_ymin[ST7796_W], scope_ymax[ST7796_W];
static float  sc_peak = 600.0f;      // scope auto-gain follower (counts)
static uint16_t sc_line;             // trace color (tracks dominant pitch)
static int    sc_halfw = 1;          // trace half-thickness (tracks volume)
static float  e_bass = 0.0f, e_amp = 0.0f;   // plasma: speed + color drives
static float  lvl_disp = 0.0f;               // displayed loudness 0..1
static uint32_t pl_t = 0;
static float  pl_ang = 0.0f, pl_zoom = 1.0f; // swirl rotation + zoom

// tables (built once)
static int    tabs_ready = 0;
static float  hann[FFT_N];
static float  tw_c[FFT_N / 2], tw_s[FFT_N / 2];
static int16_t slut[256];            // sin, Q7
#define PALS 4
static uint16_t pal4[PALS][256];     // plasma palettes, all dark -> bright
static int      cur_pal = 0;         // a hard transient rerolls this
static uint16_t grad[REGION_H];      // spectrum gradient by row

// 6-stop ramp interpolator (each palette = 6 RGB stops, dark to bright).
static void ramp6(const uint8_t stop[6][3], int i,
                  uint8_t *r, uint8_t *g, uint8_t *b) {
    float t = (float)i * (5.0f / 255.0f);
    int   s = (int)t;
    if (s > 4) s = 4;
    float f = t - (float)s;
    *r = (uint8_t)((float)stop[s][0] + f * ((float)stop[s + 1][0] - stop[s][0]));
    *g = (uint8_t)((float)stop[s][1] + f * ((float)stop[s + 1][1] - stop[s][1]));
    *b = (uint8_t)((float)stop[s][2] + f * ((float)stop[s + 1][2] - stop[s][2]));
}

// HSV -> RGB for the rainbow palette (s = 1).
static void hue_rgb(float h, float v, uint8_t *r, uint8_t *g, uint8_t *b) {
    h = h - floorf(h);
    float hf = h * 6.0f;
    int   s6 = (int)hf;
    float f = hf - (float)s6;
    float p = 0, q = v * (1.0f - f), t = v * f;
    float rr = 0, gg = 0, bb = 0;
    switch (s6 % 6) {
    case 0: rr = v; gg = t; bb = p; break;
    case 1: rr = q; gg = v; bb = p; break;
    case 2: rr = p; gg = v; bb = t; break;
    case 3: rr = p; gg = q; bb = v; break;
    case 4: rr = t; gg = p; bb = v; break;
    default: rr = v; gg = p; bb = q; break;
    }
    *r = (uint8_t)(rr * 255); *g = (uint8_t)(gg * 255); *b = (uint8_t)(bb * 255);
}

static void build_tables(void) {
    if (tabs_ready) return;
    tabs_ready = 1;
    for (int i = 0; i < FFT_N; i++)
        hann[i] = 0.5f - 0.5f * cosf(6.2831853f * (float)i / (FFT_N - 1));
    for (int i = 0; i < FFT_N / 2; i++) {
        tw_c[i] = cosf(6.2831853f * (float)i / FFT_N);
        tw_s[i] = sinf(6.2831853f * (float)i / FFT_N);
    }
    for (int i = 0; i < 256; i++)
        slut[i] = (int16_t)(127.0f * sinf(6.2831853f * (float)i / 256.0f));
    // Four palettes, all dark->bright along the index so the loudness
    // window dims quiet passages without draining the color out of them.
    static const uint8_t HEAT[6][3] = {
        { 2, 4, 20 }, { 16, 36, 150 }, { 96, 20, 160 },
        { 210, 32, 28 }, { 255, 150, 0 }, { 255, 255, 190 },
    };
    static const uint8_t OCEAN[6][3] = {
        { 1, 4, 16 }, { 4, 24, 90 }, { 0, 80, 170 },
        { 0, 170, 190 }, { 90, 240, 200 }, { 220, 255, 235 },
    };
    static const uint8_t NEON[6][3] = {
        { 10, 2, 18 }, { 70, 8, 110 }, { 190, 20, 160 },
        { 255, 60, 120 }, { 255, 150, 60 }, { 255, 240, 150 },
    };
    for (int i = 0; i < 256; i++) {
        uint8_t r, g, b;
        // [0] rainbow: TWO hue cycles with a brightness ramp — even a
        // narrow quiet-window slice still spans several hues.
        hue_rgb((float)i * (2.0f / 256.0f), 0.25f + 0.75f * (float)i / 255.0f,
                &r, &g, &b);
        pal4[0][i] = px_be(r, g, b);
        ramp6(HEAT, i, &r, &g, &b);  pal4[1][i] = px_be(r, g, b);
        ramp6(OCEAN, i, &r, &g, &b); pal4[2][i] = px_be(r, g, b);
        ramp6(NEON, i, &r, &g, &b);  pal4[3][i] = px_be(r, g, b);
    }
    // spectrum bar gradient: green floor -> yellow -> orange -> red tip
    for (int y = 0; y < REGION_H; y++) {
        float fy = (float)y / (float)(REGION_H - 1);   // 0 bottom .. 1 top
        uint8_t r, g;
        if (fy < 0.55f) { r = (uint8_t)(255.0f * (fy / 0.55f)); g = 230; }
        else            { r = 255; g = (uint8_t)(230.0f * (1.0f - (fy - 0.55f) / 0.45f)); }
        grad[y] = px_be(r, g, 24);
    }
    // 32 log-spaced bin edges across BIN_LO..BIN_HI (72 Hz .. 2.6 kHz)
    for (int b = 0; b <= BARS; b++) {
        float e = (float)BIN_LO
                * powf((float)BIN_HI / (float)BIN_LO, (float)b / (float)BARS);
        int bin = (int)(e + 0.5f);
        if (bin < BIN_LO) bin = BIN_LO;
        if (bin > BIN_HI) bin = BIN_HI;
        bar_edge[b] = bin;
    }
    for (int b = 1; b <= BARS; b++)          // strictly increasing edges
        if (bar_edge[b] <= bar_edge[b - 1]) bar_edge[b] = bar_edge[b - 1] + 1;
    // Display tilt: +3 dB/octave above 200 Hz, capped +12 dB. Range take 2
    // (2026-07-28): high fretted notes measure ~15 dB under an open low E
    // through the pickup — flat dB mapping made correctly-placed high
    // notes look stubby. The tilt evens musical effort across the screen.
    for (int b = 0; b < BARS; b++) {
        float fc = 11.92f * 0.5f * (float)(bar_edge[b] + bar_edge[b + 1]);
        float t = fc > 200.0f ? 3.0f * log2f(fc / 200.0f) : 0.0f;
        if (t > 12.0f) t = 12.0f;
        bar_tilt[b] = t;
    }
}

// ---- 512-point iterative radix-2 FFT (float, FPU) ----
static void fft_run(float *re, float *im) {
    for (int i = 1, j = 0; i < FFT_N; i++) {      // bit-reversal permutation
        int bit = FFT_N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j |= bit;
        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (int len = 2; len <= FFT_N; len <<= 1) {
        int half = len >> 1, step = FFT_N / len;
        for (int i = 0; i < FFT_N; i += len)
            for (int k = 0; k < half; k++) {
                float wr = tw_c[k * step], wi = -tw_s[k * step];
                float xr = re[i + k + half] * wr - im[i + k + half] * wi;
                float xi = re[i + k + half] * wi + im[i + k + half] * wr;
                re[i + k + half] = re[i + k] - xr;
                im[i + k + half] = im[i + k] - xi;
                re[i + k] += xr;
                im[i + k] += xi;
            }
    }
}

static void analyze(void) {
    // time-ordered copy: oldest sample first (viz_pos = next write = oldest)
    int p = viz_pos;
    for (int i = 0; i < FFT_N; i++) snap[i] = viz_ring[(p + i) & RING_MSK];
    static float re[FFT_N], im[FFT_N];
    for (int i = 0; i < FFT_N; i++) {
        re[i] = (float)snap[i] * hann[i] * (1.0f / 32768.0f);
        im[i] = 0.0f;
    }
    fft_run(re, im);
    // drives: bass bins 7..21 (~80..250 Hz), block peak for loudness
    float lo = 0;
    for (int k = 7; k <= 21; k++) lo += re[k] * re[k] + im[k] * im[k];
    lo = sqrtf(lo);
    static float lo_prev = 0.0f;
    float tr = lo - lo_prev;                     // bass transient (kick)
    lo_prev = lo;
    if (tr < 0) tr = 0;
    e_bass += 0.4f * (lo - e_bass);
    int pk = 0;
    for (int i = FFT_N - 256; i < FFT_N; i++) {  // newest block's peak
        int v = snap[i]; if (v < 0) v = -v;
        if (v > pk) pk = v;
    }
    e_amp += 0.25f * ((float)pk * (1.0f / 32768.0f) - e_amp);
    float ld = e_amp * 3.0f + tr * 3.0f;         // loudness -> color drive
    if (ld > 1.0f) ld = 1.0f;
    lvl_disp += (ld > lvl_disp ? 0.5f : 0.06f) * (ld - lvl_disp);
    // Hard downstroke = reroll the plasma palette (refractory ~0.4 s so a
    // strummed chord is one switch, not four).
    static uint32_t pal_cool = 0, prng = 0xC0FFEE21u;
    if (pal_cool) pal_cool--;
    if (tr > 0.10f && pal_cool == 0) {
        pal_cool = 10;
        prng ^= prng << 13; prng ^= prng >> 17; prng ^= prng << 5;
        cur_pal = (cur_pal + 1 + (int)(prng % (PALS - 1))) % PALS;
    }
    // 32 log bars, dB mapped -60..0
    for (int b = 0; b < BARS; b++) {
        float acc = 0;
        for (int k = bar_edge[b]; k < bar_edge[b + 1]; k++)
            acc += re[k] * re[k] + im[k] * im[k];
        acc = sqrtf(acc / (float)(bar_edge[b + 1] - bar_edge[b]));
        // NORMALIZE: raw radix-2 magnitudes scale with N — a full-scale
        // sine peaks at A*N/4 (Hann halves, single-sided doubles). Without
        // this the whole display sat ~42 dB hot: every bar red-lined on a
        // strum and window leakage cleared the floor (the "high note
        // lights the low end" complaint). Post-normalize, 0 dB = full
        // scale, leakage lives ~-31 dB below the note, and bars finally
        // track WHAT is played, not just THAT something is played.
        acc *= 4.0f / (float)FFT_N;
        float db = 20.0f * log10f(acc + 1e-6f) + bar_tilt[b];
        float t = (db + 48.0f) * (1.0f / 45.0f);
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        if (t >= bh[b]) bh[b] += 0.6f * (t - bh[b]); // fast attack
        else            bh[b] -= 0.045f;             // slow fall
        if (bh[b] < 0) bh[b] = 0;
        if (bh[b] > bcap[b]) bcap[b] = bh[b];        // caps drip down
        else { bcap[b] -= 0.008f; if (bcap[b] < 0) bcap[b] = 0; }
    }
    // scope styling: color glides with the dominant pitch (strongest bar,
    // red = low .. violet = high) and the trace fattens with volume.
    {
        int bmax = 0;
        float bbest = 0.0f;
        for (int b = 0; b < BARS; b++)
            if (bh[b] > bbest) { bbest = bh[b]; bmax = b; }
        if (bbest > 0.12f) {
            uint8_t r, g, b3;
            hue_rgb((float)bmax * (0.8f / (float)(BARS - 1)), 1.0f,
                    &r, &g, &b3);
            sc_line = px_be(r, g, b3);
        }
        sc_halfw = 1 + (int)(e_amp * 12.0f);
        if (sc_halfw > 5) sc_halfw = 5;
    }
    // scope: auto-gain like a real oscilloscope — normalize the trace to a
    // decaying peak follower so soft playing still fills the screen.
    sc_peak = (float)pk > sc_peak ? (float)pk : sc_peak * 0.985f;
    if (sc_peak < 300.0f) sc_peak = 300.0f;      // noise-floor guard
    int amp = (int)sc_peak;
    // trigger on a rising zero cross inside the newest block, then spread
    // 192 samples across 480 columns; per-column y-span for the strip pass
    int base = FFT_N - 256;
    int trig = base;
    for (int i = base + 1; i < base + 60; i++)
        if (snap[i - 1] < 0 && snap[i] >= 0) { trig = i; break; }
    int prev_y = -1;
    for (int x = 0; x < ST7796_W; x++) {
        int idx = trig + (x * 192) / ST7796_W;
        int y = CHROME_H + (REGION_H / 2)
              - ((int)snap[idx] * (REGION_H / 2 - 8)) / amp;
        if (y < CHROME_H) y = CHROME_H;
        if (y > ST7796_H - 1) y = ST7796_H - 1;
        if (prev_y < 0) prev_y = y;
        scope_ymin[x] = y < prev_y ? y : prev_y;
        scope_ymax[x] = y > prev_y ? y : prev_y;
        prev_y = y;
    }
}

// ---- per-mode strip renderers (y0 = absolute panel row of the strip) ----
static void render_spectrum(int y0, uint16_t *buf) {
    const uint16_t bg = px_be(6, 6, 22);
    for (int r = 0; r < STRIP_H; r++) {
        int y = y0 + r;                              // panel row
        int fy = (ST7796_H - 1) - y;                 // 0 at bottom
        uint16_t *dst = buf + r * ST7796_W;
        uint16_t on = grad[fy >= REGION_H ? REGION_H - 1 : fy];
        for (int x = 0; x < ST7796_W; x++) dst[x] = bg;
        for (int b = 0; b < BARS; b++) {
            int hpx = (int)(bh[b] * (float)(REGION_H - 10));
            int cpx = (int)(bcap[b] * (float)(REGION_H - 10));
            int lit = fy < hpx;
            int cap = fy >= cpx && fy < cpx + 4 && cpx > 2;
            if (!lit && !cap) continue;
            uint16_t c = cap ? 0xFFFF : on;
            uint16_t *d = dst + b * 15 + 2;
            for (int i = 0; i < 12; i++) d[i] = c;
        }
    }
}

static void render_scope(int y0, uint16_t *buf) {
    const uint16_t bg  = px_be(4, 8, 18);
    const uint16_t mid = px_be(24, 40, 60);
    uint16_t line = sc_line;
    int hw = sc_halfw;
    int ymid = CHROME_H + REGION_H / 2;
    for (int r = 0; r < STRIP_H; r++) {
        int y = y0 + r;
        uint16_t *dst = buf + r * ST7796_W;
        uint16_t row_bg = (y == ymid) ? mid : bg;
        for (int x = 0; x < ST7796_W; x++)
            dst[x] = (y >= scope_ymin[x] - hw && y <= scope_ymax[x] + hw)
                   ? line : row_bg;
    }
}

static void render_plasma(int y0, uint16_t *buf) {
    // SWIRL: the whole sin-field rotates about the screen center (rotated
    // coordinates u,w in Q6 fixed point — two mults per pixel). The rules
    // a viewer can learn in ten seconds:
    //   bass       -> spin speed        loudness -> zoom + brightness
    //   downstroke -> new color palette
    int t = (int)pl_t;
    int cs = (int)(cosf(pl_ang) * 64.0f * pl_zoom);
    int sn = (int)(sinf(pl_ang) * 64.0f * pl_zoom);
    int gain = 72 + (int)(lvl_disp * 184.0f);
    const uint16_t *p = pal4[cur_pal];
    for (int r = 0; r < STRIP_H; r++) {
        int yc = (y0 + r) - (CHROME_H + REGION_H / 2);
        int ycs = yc * cs, ysn = yc * sn;
        uint16_t *dst = buf + r * ST7796_W;
        for (int x = 0; x < ST7796_W; x++) {
            int xc = x - (ST7796_W / 2);
            // rotation: u = x cosθ - y sinθ, w = x sinθ + y cosθ
            int u = (xc * cs - ysn) >> 6;
            int w = (xc * sn + ycs) >> 6;
            // Radial + rotating-arm composition. The old all-plane-wave
            // stack read as stacked scrolling stripes ("fat horizontal
            // rows") — anchoring the field on center-origin RINGS plus
            // rotating arms makes it one cohesive breathing swirl.
            int rr = (u * u + w * w) >> 9;
            int v = slut[(rr - (t << 1)) & 255]         // breathing rings
                  + slut[(u + t) & 255]                 // rotating arm
                  + slut[(w + (rr >> 1) - t) & 255]     // ring-warped arm
                  + slut[(((u + w) >> 1) + t) & 255];
            int idx = (((v >> 2) + 128) * gain) >> 8;
            dst[x] = p[idx > 255 ? 255 : idx];
        }
    }
}

// ---- chrome (static top bar; never re-rendered by the strip loop) ----
static void draw_chrome(void) {
    st7796_fill_rect(0, 0, ST7796_W, CHROME_H, px_be(20, 20, 36));
    st7796_draw_text(8, 8, 2, px_be(220, 220, 230), px_be(20, 20, 36),
                     "< BACK");
    st7796_draw_text(200, 8, 2, px_be(0, 230, 190), px_be(20, 20, 36),
                     MODE_NAMES[viz_mode]);
    st7796_draw_text(366, 12, 1, px_be(150, 150, 160), px_be(20, 20, 36),
                     "TAP = NEXT MODE");
}

void viz_open(void) {
    build_tables();
    for (int b = 0; b < BARS; b++) { bh[b] = 0; bcap[b] = 0; }
    sc_peak = 600.0f;
    sc_line = px_be(0, 255, 200);    // until a note picks the pitch color
    st7796_fill_screen(px_be(6, 6, 22));
    draw_chrome();
}

void viz_frame(void) {
    analyze();
    pl_t += 1 + (uint32_t)(e_bass * 3.0f);   // bass pumps the texture drift
    pl_ang += 0.004f + e_bass * 0.045f;      // ...and SPINS the swirl
    if (pl_ang > 6.2831853f) pl_ang -= 6.2831853f;
    float zt = 0.7f + lvl_disp * 0.9f;       // loudness zooms the field
    pl_zoom += 0.08f * (zt - pl_zoom);
    static int cur = 0;
    for (int s = 0; s < STRIPS; s++) {
        int y0 = CHROME_H + s * STRIP_H;
        uint16_t *buf = strip_buf[cur];
        switch (viz_mode) {
        case 0:  render_spectrum(y0, buf); break;
        case 1:  render_scope(y0, buf);    break;
        default: render_plasma(y0, buf);   break;
        }
        while (st7796_flush_busy()) ;        // previous strip off the wire
        st7796_flush_async(0, (uint16_t)y0, ST7796_W - 1,
                           (uint16_t)(y0 + STRIP_H - 1), buf, 0);
        cur ^= 1;
    }
    while (st7796_flush_busy()) ;
}

int viz_touch(int x, int y) {
    if (y < CHROME_H + 16 && x < 130) return 1;      // BACK zone (generous)
    (void)x;
    viz_mode = (viz_mode + 1) % 3;
    draw_chrome();
    return 0;
}
