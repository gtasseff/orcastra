// viz.h — full-screen audio visualizer page (90s media-player energy, strip-
// rendered). Modes: SPECTRUM (32-bar log FFT, gradient + peak caps), SCOPE
// (triggered waveform), PLASMA (audio-modulated demoscene plasma). The audio
// core feeds the latest DAC-mix block via viz_feed(); the UI core renders
// whole frames through the driver's async DMA flush, double-buffered in
// 37-line strips (a full 480x320 framebuffer doesn't fit free SRAM).
#ifndef VIZ_H
#define VIZ_H
#include <stdint.h>

void viz_feed(const int16_t *block, int n);  // audio core: final DAC mix
void viz_mode_set(int mode);                 // 0 spectrum, 1 scope, 2 plasma
void viz_open(void);                         // entering the page (chrome+init)
void viz_frame(void);                        // render one frame (UI loop)
// Touch, down edge only. Returns 1 if the tap asked to leave the page
// (BACK zone); otherwise cycles the mode and returns 0.
int  viz_touch(int x, int y);

#endif // VIZ_H
