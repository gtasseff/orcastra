# AGENTS.md — Orcastra engineering notes

**Read `wilibsp/AGENTS.md` completely, through EOF, before inspecting
or changing this project.** That file is the authoritative FREE-WILi 2 BSP and
app contract — build wrappers, the FW2App requirements every app must meet, the
power-zone rules and the hardware-verification procedure. It is intentionally
long: if your reader truncates output, continue from the last line in additional
chunks until you reach EOF. Do not act after reading only its first chunk. The
contract is not restated here, because duplicated rules drift.

Then read this file. It covers what is specific to Orcastra: the audio path, the
PSRAM execution scheme, the UI, and the hardware constraints this app runs into.
It is dense on purpose — treat every invariant below as load-bearing.

If you are here to add an effect, read
[`docs/ADDING-AN-EFFECT.md`](docs/ADDING-AN-EFFECT.md) for the mechanics and
this file for the constraints.

## What this is

A real-time audio multi-tool for the **FREE-WILi 2** ("FW2") — effects,
sampler, drum machine, synth, theremin, chord keys, tuner and visualiser. It
builds against the upstream `freewili2_bsp` library (the `wilibsp`
submodule) and nothing else; see invariant 22 for why that constraint is
load-bearing rather than incidental.

The FW2 display processor is a Raspberry Pi **RP2350B** (48 GPIO, 16 MB flash,
8 MB PSRAM) with a 480×320 touch LCD, NAU88C10 audio codec (speaker + 3.5 mm
TRRS jack), 4-mic PDM array, 16 WS2812 LEDs, CC1101 sub-GHz radio, IR, sensors
and USB host. Orcastra uses the codec, the LCD and touch, the PDM mics, the
LEDs, the IMU, the light sensor and the magnetometer.

The app runs on the display processor and **executes from PSRAM** so the audio
path keeps its SRAM. That is the single most consequential design decision in
the repo, and invariant 21 is the one to read before touching it.

## Repo map

```
orcastra/
  CMakeLists.txt          top-level: PICO_BOARD=freewili2, pico_sdk_init,
                          add_subdirectory(wilibsp/bsp) + apps.
                          ALSO sets PICO_DEFAULT_PSRAM_MIN_DESELECT (inv. 23)
                          and the FW2_AGENTIO option (harness, PSRAM target only)
  CMakePresets.json       "target" configure/build preset (Ninja, build/)
  pico_sdk_import.cmake   standard Pico SDK locator (from wilibsp)
  apps/orcastra/
    main.c                UI, pages, touch/buttons, the audio pump      (the bulk)
    fx.c/.h               the effects, the chain, EQ, pitch correction
    vox.c/.h              five-slot sampler, buffers in PSRAM
    synth.c/.h            XY synth voice (PolyBLEP)
    drums.c/.h            drum voices + step sequencer
    keys.c/.h             chord keyboard (FM e-piano / saw strings)
    viz.c/.h              spectrum + scope visualiser
    voice_data.c/.h       synthesised voice, generated; see LICENSE for provenance
    sd_entry.c            launched-from-SD entry: IRQ handover, PSRAM re-time,
                          clocks. THE most delicate file in the repo (inv. 21)
    psram_xip_link/       linker overrides: execute-in-place from PSRAM, with
                          the timing-critical + audio set forced into SRAM
    psram_link/           the copy_to_ram variant (staged in PSRAM, runs in SRAM)
  tools/
    fw.py / fw.cmd        build + RTT driver (no flash path — see below)
    agentio.py            screenshot + input injection over SWD
    check_psram_xip.py    structural verification of the PSRAM-execution build
    read_boot_stage.py    read the boot breadcrumb from a hung app, no halt
    make_voice_sidecar.py generate the voice blob
    openocd/*.cfg         CMSIS-DAP + RP2350 OpenOCD configs
  docs/
    ADDING-AN-EFFECT.md   the extension guide
  prebuilt/orcastra.uf2   the built app, so it can be tried without a toolchain
  wilibsp        the ONE submodule (invariant 22)
```

## Command vocabulary

Run from the repo root. `fw` = `python tools/fw.py` (or `tools\fw.cmd`).
The CLI self-locates the Pico VS Code extension toolchain under `~/.pico-sdk`
(Pico SDK, ARM GCC, CMake, Ninja, prebuilt picotool + pioasm, OpenOCD) — no
global environment setup needed. Built against SDK 2.3.0; the driver resolves
whatever version the extension installed rather than pinning one, so do not
hardcode a version here.

| Command | What it does |
|---|---|
| `fw build [target]` | Configure (first run) + build one target (default `orcastra_psram`) → `build/apps/orcastra/<target>.elf`. The app DIRECTORY is `apps/orcastra`; the TARGETS are `orcastra_psram` and `orcastra_sd`. Target name ≠ directory name, so never compose a path from the target name. |
| `fw rtt [-s N]` | Stream SEGGER RTT diagnostics (OpenOCD RTT server, port 9090); `-s N` captures N seconds then exits |

**This driver has no `flash` command, by design.** Both targets load at
`0x11000000`; the only thing the probe could program is QSPI flash at
`0x10000000`, which holds the stock DISPLAY firmware — the bootloader that
launches apps from the card. Install an app by copying its `.uf2` to `/apps`, or
with wilibsp's `fw install-app`, which writes the card over MAIN's serial link.
The probe is how you read RTT and drive the agentio harness.

Before flashing: `Get-Process openocd -ErrorAction SilentlyContinue | Stop-Process -Force`
(a stale OpenOCD instance holds the probe). If flashing fails with "unable to
find a matching CMSIS-DAP device", re-seat the board's USB cable.
GDB attaches on port 3333 while OpenOCD runs.

## The one submodule

`wilibsp` — the FREE-WILi 2 board support package
(<https://github.com/freewili/wilibsp>), MIT. We link its `freewili2_bsp`
static library directly and use nothing else. Worth reading in it:
`bsp/` (the peripheral drivers), `docs/hardware/{pinmap,facts,catalog}.md`,
`docs/drivers/*.md`, and its own `AGENTS.md`.

Always clone with `--recurse-submodules`, or run
`git submodule update --init --recursive` after the fact. The pin is
deliberate — see invariant 22 before moving it.

## Invariants — do NOT relearn these the hard way

1. **Board selection is CMake-only.** `set(PICO_BOARD freewili2)` lives in the
   top-level CMakeLists.txt (RP2350B, `PICO_RP2350A=0`, 48 GPIO). **Never pass
   `-DPICO_BOARD` on a cmake command line** — it overrides to the wrong chip.
2. **250 MHz default clock, vreg 1.25 V, and `clk_peri` re-sourced from
   `clk_sys`** — all done by `board_init()`; call it first in every app.
   Without the re-source, SPI has no clock and the LCD is dead. 250 MHz is
   **audio-optimal**: MCLK = clk_sys/61 = 4.0984 MHz ≈ 256 × 16 kHz.
3. **Diagnostics are SEGGER RTT only** — `DIAG(...)` from `platform/diag.h`.
   No UART/USB stdio (USB is host-mode). RTT printf has **no `%f`** — format
   floats as scaled ints.
4. **`DMA_IRQ_0` is SHARED** (display flush + audio capture). Any new user
   must `irq_add_shared_handler(DMA_IRQ_0, ...)` and check only its own
   channel. Never `irq_set_exclusive_handler`.
5. **GPIO8 is dual-function**: `PIN_LCD_DC` and `PIN_CC1101_MISO` on shared
   SPI1. `PIN_CC1101_CS` = GPIO40 is parked HIGH in `board_init()`.
6. **16 WS2812 LEDs** on GPIO21 (pio1, inverted driver). Note that
   `FwDisplayVibe.md` (in wilibsp) says 7 LEDs, and gives CC1101 CS as 23 rather
   than 40; both are out of date. Treat `bsp/platform/board.h` as the pin-map
   source of truth whenever a doc disagrees with it. First LED
   frame after PIO start may not latch — refresh periodically.
7. **Binary type is a real decision, not a default.** The BSP's apps are
   normally `copy_to_ram` (code+data+bss all inside 512 KB of SRAM — watch the
   budget), and big buffers go in **PSRAM** (`PSRAM_BASE 0x11000000`, 8 MB
   APS6404L, `bsp/platform/psram.h`). **Orcastra's shipping target does NOT do
   this**: `orcastra_psram` executes in place from PSRAM and only copies `.data`
   to SRAM, which is what buys the audio path its headroom — see invariant 21
   for the scheme and the trap in it. A UI-heavy app whose `.text` outgrows SRAM
   has the same two choices. Never set `PICO_EMBED_XIP_SETUP=1` (boot-loops).
8. **Audio MCLK/LRCK lock — UPDATED 2026-07-24 (local wilibsp fix)**: the
   original `div = 8*ticks/3` claim of "LRCK == MCLK/256 exactly" was only
   true when ticks divided by 3 — otherwise the 8.8-bit fractional divider
   rounds and LRCK drifts against MCLK by ONE FULL SAMPLE every ~0.84 s at
   48 kHz (~7.8 s at 16 kHz): the MCLK-clocked codec slips a sample and
   emits a metronomic click (present since bring-up; root-caused via
   line-out recording — measured periods matched the rounding prediction
   to <0.1%). The fix: the i2s_duplex PIO program runs 4 cycles/bit (128*fs) so
   the divider is the INTEGER `2*ticks` at any rate. Verified click-free at 16 k
   and 48 k. **This is upstream in wilibsp** and present at the pinned commit, so
   nothing local is required — see invariant 22. The older lore stands: never
   "simplify" to `clk_sys/(96*fs)` (~9 Hz click, −17 dB sidebands).
9. **PIO budget**: pio0 = I2S audio; pio1 = WS2812 + PDM mics (init PDM before
   LEDs); pio2 = radio GDO capture + IR (31/32 instructions used; radio-first
   init order — `gdo_capture_init()` before any IR init).
10. **I/O expander power gates** (PCAL6524 @ I2C1 0x23): IR rail, mic power,
    USB hub ports (HP1/HP2), CC1101 antenna select — all **off at power-on**.
    Drivers gate what they need (`ioexp_mic_pwr`, `ioexp_usb_pwr`,
    `ioexp_ir_pwr`); PDM needs ~50 ms settle after mic power.
11. **I2C1** (SDA=26, SCL=27, 400 kHz) is shared by touch FT6336 (0x38),
    codec NAU88C10 (0x1A), ioexp PCAL6524 (0x23), and sensors OPT4001 (0x45),
    SHT40 (0x44), BMI323 (0x68), BMM350 (0x14). BMI323/BMM350 reads return
    **2 leading dummy bytes**; BMM350 init needs its ~130 ms of settles.
12. **USB host**: everything attaches behind the onboard hub; VBUS is
    hardwired — power-cycle via `ioexp_usb_pwr()` at boot so devices re-enum
    after SWD resets. Isochronous IN needs single-buffer SOF-paced EPX.
13. **Full-screen LCD draws starve the mic pump** (480×320 fill over SPI
    ≈ 50 ms ≈ 3 capture blocks; capture is a LOSSY ping-pong while the TX
    DMA ring keeps looping). Missed blocks slip the ring's write/read phase;
    landing on write==read tears every block — permanent "fuzz" that no
    effect toggle fixes (bench-diagnosed 2026-07-16). After ANY full-page
    draw while the mic ring is armed, force a pump re-arm (orcastra:
    `mic_ring_rephase()` → `mic_armed = false`) to restore the one-slot
    lead. Button/slider-sized redraws (~2 ms) are safe.

    **Corollary — a modal overlay must HOLD the background painters, not just
    re-arm the pump** (2026-08-23). The volume overlay is an opaque 320×104
    panel drawn over whatever page is showing. Re-arming the pump around it
    fixed the audio dropout, but the drums page kept chasing its playhead
    underneath — `draw_drum_playhead()` and `draw_drum_banks()` repainting
    straight THROUGH the panel ~16 times a second, which reads as "the screen
    looks glitched out and weird" and only clears when the overlay expires and
    `page_repaint()` runs. Any per-frame painter must be gated on
    `!vov_showing`. Skipping is safe here because `draw_drum_playhead()` erases
    its previous column via `drum_cell()`, so a stale `drum_ph_drawn` across the
    gap repairs itself; check that property before gating a different painter.
14. **OpenOCD's reset is core-only — quiesce DMA before flashing** (ROOT
    CAUSE, bench-proven 2026-07-17 after several wrong theories). The app's
    free-running DMA channels keep running straight through `reset halt`:
    with both cores halted, the PDM capture channel (ch1, endless transfer
    count from pio1 RXF) was watched advancing its WRITE_ADDR. Its 32 KiB
    SRAM ring overlaps OpenOCD's flash work area (0x20010000), so the
    uploaded flash algorithm got sprayed with PDM bits mid-run: "Failed to
    invoke flash programming code", "Failed to call ROM function batch",
    "timed out while waiting for target halted", garbage QSPI ids, and
    partial erases that corrupted the resident image (also the cause of
    the "app sometimes doesn't boot after power-on" mystery). Short probe
    trampolines usually squeaked through; RTT (pure SWD reads) never
    suffered. Day-1 builds flashed fine because their only free-running
    DMA wrote small fixed buffers near 0x20081974, missing the work area;
    the PDM build's big ring landed on it.
    FIX: halt, CHAN_ABORT all 16 channels, clear every CHx_CTRL_TRIG, then
    program — cores stay halted through program's internal reset so nothing
    re-arms. Verified: flash over a RUNNING app, and RTT-then-immediate-
    reflash, both first-try. The sequence lives in wilibsp's `tools/fw.py`,
    and applies to anything that programs this board while an orcastra build
    is running — including restoring the stock DISPLAY image. If symptoms ever return, check for
    post-reset DMA activity first: `mdw 0x50000000 64` twice, look for
    advancing WRITE_ADDRs.

## Audio cheat-sheet (the heart of this project)

Pins: DIN=GPIO4 (ADC in), DATA=GPIO5 (DAC out), LRCK=GPIO6, BCLK=GPIO7
(pio0 SM0 sideset), MCLK=GPIO22 (PWM, 256×fs). Codec NAU88C10 @ I2C1 0x1A,
48 kHz nominal — ACTUAL 48,828 Hz (orcastra-era; integer-MCLK, see FX_FS in
apps/orcastra/fx.h; the driver examples below still say 16 kHz — same
mechanism, pass 48000 and override codec reg 0x07 to 0x0000 for 48 kHz
filters). Mono speaker / stereo-capable jack. Docs:
`wilibsp/docs/drivers/audio.md` and `.../pdm.md`.

```c
board_init();                                  // clocks first, always
codec_nau88c10_init();                         // 16 kHz, starts soft-muted, speaker out
bool ok = codec_nau88c10_input_ok();           // expect rev=0x01A, pm2=0x015
codec_nau88c10_dac_mute(false);
audio_i2s_duplex_init(16000);                  // RX runs; TX silent until armed
audio_capture_start();                         // REQUIRED — see below
static uint32_t silence[64] __attribute__((aligned(256)));
audio_i2s_duplex_play_loop(silence, 64);       // REQUIRED — see below
// THE DUPLEX SM STALLS IN BOTH DIRECTIONS (bench-diagnosed 2026-07-13, fdebug
// TX/RXSTALL): one PIO SM does TX out + RX in with autopull/autopush. If the
// RX FIFO isn't drained (no capture DMA) OR the TX FIFO runs empty (play_stop
// with nothing re-armed), the SM stalls -> I2S clocks freeze -> BOTH paths
// die. Rules: always run audio_capture_start(), and never leave TX unarmed —
// "stop" means switching the TX DMA ring to a silence buffer, not stopping.
// Swap rings via play_stop() THEN play_loop(other_buf, n).

// Playback: zero-CPU DMA read-ring. Buffer = whole periods, power-of-two
// BYTES, aligned to its byte size. Frame = [L16|R16] packed uint32.
static uint32_t tone[64] __attribute__((aligned(256)));
audio_i2s_duplex_play_loop(tone, 64);
audio_i2s_duplex_play_stop();                  // park DAC at silence

// Output routing — safe during playback:
codec_nau88c10_set_output(CODEC_OUT_SPEAKER);  // or CODEC_OUT_HEADPHONE (3.5mm)
codec_nau88c10_log_output();                   // RTT: R36/R38 confirm the switch

// Capture (codec ADC = 3.5 mm jack mic, mono on RIGHT slot):
if (audio_capture_block_ready()) {
    const uint32_t *b = audio_capture_block(); // 256 frames or NULL
    uint16_t pk = vu_peak(b, AUDIO_CAPTURE_BLOCK_FRAMES, AUDIO_MIC_I2S_SLOT);
}
```

- Pure DSP helpers (host-testable, no SDK): `tone_gen_fill()`, `vu_meter`,
  `dsp/cic`, `dsp/dcblock`. Our own FX DSP lives in `apps/orcastra/fx.c`
  (dual-tap pitch shifter + feedback echo, mono blocks, ~25 flops/sample).
- **Passthrough/FX latency (verified by ear 2026-07-13): ~24–32 ms** — 4-slot
  TX ring written 1 slot ahead of the DMA reader (rate-locked, same SM).
  Musician verdict: acceptable and fun. Lower would need smaller capture
  blocks (BSP change) — don't bother unless asked.
- The **PDM 4-mic array** (GPIO 28/29/30, 1.024 MHz clock, CIC ×64 → 16 kHz)
  is separate hardware from the codec mic. Physical order D,B,A,C. Consumer
  must drain its DMA ring within ~64 ms or samples are silently lost.
- Routing registers (empirically derived): speaker-only = R3 0xED + R36 0x3F +
  R38 0x04 + R45 0x05 (5 V boost); jack-only = R3 0xED + R36 0x40 (spk mute) +
  R38 0x01 (HP on) + R45 0x00.
- Codec init default output = speaker; DAC starts soft-muted.
- **Input gain staging (bench-verified 2026-07-13):** the BSP init enables the
  +20 dB PGA boost (reg 0x2F=0x100) — correct for a headset electret, but line
  level CLIPS the ADC hard ("overdriven garbage"). Stage analog gain per source:
  dynamic mic → boost + PGA +12 dB (0x2D=0x020, 0x2F=0x100); line level (synth
  RCA out) → boost OFF, PGA 0 dB (0x2D=0x010, 0x2F=0x000). PGA minimum
  (0x2D=0x000, −12 dB) audibly cuts low end (bass drum) — avoid; prefer DAC
  volume (reg 0x0B, 0.5 dB/step, 0xFF=0 dB) for level trim. Onboard speaker is
  capped at DAC vol ≤ −9 dB (0x0ED) — see invariant 16 for the power arithmetic
  and why not −6 dB.
- For 48 kHz output, the pattern that works elsewhere on this silicon is a ×3
  polyphase FIR upsampler with the resampling offloaded to core 1 — core 0
  overloads if it tries to carry both.

## UI / input quick facts

- Display: ST7796 480×320, RGB565 **big-endian** (byte-swapped), SPI1 @
  100 MHz + DMA. `st7796_fill_rect`, `st7796_draw_text` (5×7 font, scale 1–4,
  uppercase-only), `st7796_blit_rect`, async `st7796_flush_async`. No LCD
  reset GPIO — SWRESET only, ioexp releases RESX.
- Touch: FT6336U, polled (`ft6336_poll(&x,&y)` → true on exactly one finger,
  coordinates already in 480×320 landscape). Edge-detect for tap semantics.
- Backlight: `board_backlight_set(1)` (GPIO25 on/off).
- For rich UIs, LVGL 9 works on this hardware but its `.text` outgrows SRAM, so
  an LVGL app has to run flash/XIP or execute from PSRAM (invariant 21).
  Orcastra draws directly with the `st7796_*` primitives instead — no toolkit,
  which is why the whole UI fits alongside the audio path.

## Still-TODO peripherals (no driver anywhere yet)

NFC (ST25R3916B, I2C1) · haptic motor (GPIO46) · Pico-PIO-USB host (GPIO42/43,
mutually exclusive with native USB host). The ESP32-C5, FPGA, CM0, LoRa and
CAN FD blocks belong to other processors on the board and are outside what the
display-side BSP covers.

**14-button coprocessor: SOLVED (2026-07-23 BSP update).** `bsp/input/uartkbd`
— UART1 @ 62500 8N1 on GPIO38(TX, claimed unused)/39(RX), RX-only unsolicited
frames, POLLED (`uartkbd_init()` + `uartkbd_task()` every loop, core 0).
Buttons: GREY/YELLOW/GREEN/BLUE/RED + nav cluster + HOME/OK/CANCEL/PAGE
(`uartkbd_btn_t`, press/release events via `uartkbd_next_event`, level bitmap
via `uartkbd_buttons()`; active-low quirk handled inside the driver). Frames
also carry CHARGER telemetry (`uartkbd_charger`). `bsp/keyboard/fw2kb` layers a
two-press chord text engine on top, if you need text entry.

## Workflow expectations

- **Reuse before writing**: look for the proven driver in `wilibsp`
  before writing hardware glue, and keep its naming (`st7796_*`, `ft6336_*`,
  `codec_nau88c10_*`, ...). Almost every peripheral on this board already has a
  driver there that someone has debugged on real hardware.
- Keep new pure logic (DSP, parsers) SDK-free so it can be host-tested;
  hardware glue is verified on the device (RTT + ears/eyes).
- Verify on hardware before claiming success, on the path users actually run:
  `fw build` → copy `build/apps/orcastra/orcastra.uf2` to `/apps` on the card
  (or `fw install-app` from wilibsp) → launch it from the on-device picker →
  `fw rtt -s 8` (expect boot banner + `codec: input path ready`). With
  `-DFW2_AGENTIO=ON`, add `python tools/agentio.py screenshot -o shot.png` and
  actually LOOK at the PNG — a capture that succeeds and shows the wrong thing
  is the failure worth catching. If the app dies before its first DIAG, read
  the breadcrumb with `tools/read_boot_stage.py` (no halt, so the three-finger
  YELLOW+GREEN+BLUE reset still works — invariant 24).
- Conventional Commits (`feat:`, `fix:`, `docs:`, ...), imperative subject.
- **This repo is PUBLIC.** Everything committed here is world-readable, so:
  no unit serial numbers, no local absolute paths, no named colleagues, and
  nothing under NDA. Be careful with **on-screen strings and `DIAG()` output**
  specifically — both ship inside the binary, and on-screen text is the
  highest-risk surface of all. Third-party trademarks do not belong in effect
  names, comments or UI text; describe the sound generically instead ("a
  mid-focused overdrive pushing a scooped high-gain amp", not two brand names).
  The same goes for artists, songs and the hardware an effect was modelled on.

15. **Audio runs on core 1 — keep it that way** (2026-07-17). The whole
    signal path (capture -> PDM -> sampler -> FX chain -> TX ring) lives in
    `audio_pump_block()` on core 1; core 0 owns display/touch/LED/RTT. This
    is the fix for the day-one intermittent static/crackle ("STOP/PLAY
    heals it"): capture is a lossy ping-pong with a one-block deadline
    (5.2 ms at 48.8 kHz) that UI drawing kept blowing. Rules: RTT/DIAG is
    NOT multicore-safe — core 1 never prints, it exports `pump_*` mirrors;
    WS2812/display/I2C are core-0-only; while `playing && mic` core 1 owns
    the TX DMA, so any UI-side `tx_play` must park the pump first
    (`playing = false`, wait ~7 ms, swap, restore) — see the SRC/PLAY
    handlers; cross-core flags are volatile; core 1 uses a dedicated 4 KiB
    stack (`multicore_launch_core1_with_stack`).

16. **Onboard speaker: 300 mW rated / 500 mW ABSOLUTE MAX, 8 ohm, passband
    ~1.7-20 kHz** (confirmed from the actual fitted part 2026-07-23 — early
    documentation errantly said 2 W, and the first bench speaker died from
    full-scale sine testing at 0 dB, which delivers ~0.7-1.1 W). Two guards
    in orcastra, keep BOTH in any future app: (a) volume policy — speaker
    output is capped at **-9 dB** (worst-case continuous 88-139 mW, i.e.
    29-46% of the rating); (b) a 2nd-order 900 Hz high-pass on the speaker
    route (audio pump) — content below the driver's ~1.7 kHz self-resonance
    produces raw cone excursion that damages the driver and sounds broken
    regardless of level. The 3.5 mm jack path is full-range and unlimited --
    correct for line-level gear, and genuinely loud on headphones: in-ear
    monitors were verified working on 2026-08-24, with -12 dB comfortable and
    the top of the range far past what anyone would want in their ears. The pad
    is the only thing between a listener and full scale, so default low.

    The cap was **-12 dB until 2026-08-23**, raised on bench feedback that
    -12/-24 dB are "SO quiet" on the speaker. Derate against the 300 mW
    CONTINUOUS number, not the 500 mW peak: this app really does sustain
    full-scale sines indefinitely (1 kHz tone, theremin, synth with HOLD on).

    | DAC pad | speaker power | % of 300 mW rated |
    |---------|---------------|-------------------|
    |   0 dB  | 0.7-1.1 W     | 230-370%  killed a speaker |
    |  -6 dB  | 175-275 mW    |  58-92%   in spec, no margin |
    |  -9 dB  |  88-139 mW    |  29-46%   **the cap** |
    | -12 dB  |  44-69 mW     |  15-23%   previous cap |

    -6 dB was considered and declined: the datasheet permits it, but 58-92%
    leaves nothing for the ~0.7-1.1 W full-scale figure being an ESTIMATE, and
    that estimate has already been wrong once on this bench. -9 dB is 2x the
    acoustic power of -12 dB (~1.4x perceived loudness) for half the risk.
    If -6 dB is ever genuinely needed, the right way to buy it is an RMS-
    tracking limiter on the speaker route, not a bigger static number.

    **The two guards must agree.** They did not: `vol_cycle()` and
    `vol_clamp_quiet()` held the speaker at -12 dB while the D-pad handler kept
    its OWN copy of the loudness order and capped with
    `if (ORDER[oi] == 2 && !vol_loud_ok()) oi = 2;` — which resolves to
    `ORDER[2] == 1 ==` -6 dB. So D-pad up reached a level the settings menu
    refused, silently, for as long as both existed. There is now one
    `VOL_ORDER` table plus `VOL_ORDER_SPEAKER_MAX`, and `vol_rank()` /
    `vol_cycle()` / `vol_clamp_quiet()` / the D-pad all read them. Do not
    reintroduce a second copy of the ramp.

17. **Any display-side SWD reset desyncs the power-coprocessor link**
    (bench-proven 2026-08-05, cost an afternoon). `program ... verify reset`
    (whatever emits it — wilibsp's `fw flash`, or a manual OpenOCD invocation
    restoring the stock image) or any `reset` on
    OpenOCD interface 0 reboots DISPLAY alone; the always-on battery-rail
    power coprocessor keeps its stale link state and does NOT re-sync on its
    own. Unplugging USB does not reset it; cycling the display rail does not
    reset it. Recovery: hold RED ~3 s for a TRUE power-down, then power up.
    Rule: after any probe-side reset or reflash, RED-power-cycle before
    trusting buttons, zone telemetry, or SD-loader behavior.

18. **Power zones: never use per-zone "Set Zone" on the MAIN console** — it
    resolves to an ABSOLUTE awake mask seeded from cached state, so with
    telemetry down `Set Zone 16 0` turned off every rail (bench-proven
    2026-08-05). Use "Set Zone Mask" only, only after "Get Zones" returns a
    non-zero mask, always including zone 7 (SD) and zone 8 (USB hub — you
    are typing through it; zone frames can drop the CDC console mid-write,
    which truncates the value: verify with Get Zones after reconnect, never
    assume the command landed). picpwr's own mask-based API already follows
    this rule (see bsp/input/picpwr.h).

19. **SD-card app delivery (`/apps` on the card): the loader routes a UF2 by
    the address its blocks target, never by filename.** 0x11000000 = PSRAM
    app (flash untouched — OUR path, targets `orcastra_sd` and
    `orcastra_psram`), 0x20000000 = RAM app (192 KiB loaded-content cap — an
    earlier 448 KB figure came from an abandoned loader branch and is wrong),
    0x10000000 = FIRMWARE UPDATE that overwrites the display firmware.
    NEVER put a raw .bin in /apps — it carries no address and goes down the
    flash path. `pico_add_extra_outputs`' ELF->UF2 drops PSRAM segments
    ("entry point is not in mapped part of file"); convert the flat .bin with
    `picotool uf2 convert -o 0x11000000 --family rp2350-arm-s` (wired into
    the orcastra_sd target). The app still executes from SRAM:
    copy_to_ram crt0 copies out of PSRAM at the inherited clock BEFORE
    board_init re-clocks — that ordering is mandatory (QMI re-time cannot run
    from the memory being re-timed). Runtime PSRAM data was moved to
    0x11100000 so the loaded image (0x11000000, 1 MB reserved) cannot overlap
    it. Launch-path status as of 2026-08-06 (TRUNK — the loader branch was
    abandoned; forget h\v\a and the 448 KB RAM ceiling): EVERY launch path
    (APPS picker, h\v\p Run PSRAM App, h\v\s Load PSRAM Data, RAM apps via
    the picker) enters through the OTP-fused display bootloader, so a unit
    that fails "Display Bootloader Version" has NO launch path on any
    firmware (provisioning fingerprint: lock rows 0xF80+ over SWD, NOT
    image rows — image pages read as zeros on provisioned units). RAM
    apps: 192 KiB loaded-content cap, picker-only. PSRAM apps: no
    practical cap; assets can ship as raw sidecar blobs via
    `h\v\s <file> <hex offset>` — stage FIRST and never reset the display
    between stage and launch (a reset drops stub + data); the offset is an
    unchecked contract, so put a magic+length header in the blob.
    Launched-app hygiene (both learned the hard way): compile with
    PICO_RUNTIME_SKIP_INIT_PSRAM=1 and clear inherited NVIC enables at
    RUNTIME_INIT_EARLIEST (apps/orcastra/sd_entry.c); read clk_sys at
    runtime, never assume it. When bringing up a new launch path, walk it in
    stages rather than debugging the whole app at once: a bare fill-the-screen app first, then
    the sidecar-blob contract, then orcastra_sd — whose boot-trace markers
    name any stage that freezes. `tools/read_boot_stage.py` reads the
    breadcrumb over SWD without halting the core.

20. **Continuous touch controls: use the shared mappers, never hand-roll the
    arithmetic** (2026-08-06). Two corrections apply to EVERY slider, strip and
    XY pad on this hardware, and hand-copying them is how five pages ended up
    shipping without them:
    (a) **full scale at `REACH_R` (450), not the control's drawn right edge** —
    the pads are 460–472 px wide, so they run to x=470..472, but the 8×5 reach
    grid says a thumb is comfortable only to ~450, and the top of the range is
    then not selectable at all; (b) **end saturation** — the touch controller
    reports a CENTROID that lands short of the fingertip, so an outer pad at
    each end must SATURATE to the extreme rather than approach it (`TRACK_PAD`,
    12 px; the older 26-80 px `TOUCH_PAD_*` macros are deleted — they existed to
    dodge the bezel, which the interaction band below made unnecessary). Call `touch_norm()` (0..1), `touch_norm_bipolar()` (−1..+1,
    for controls with a **painted centre detent** — it stretches each half
    independently so zero stays exactly where it is drawn) or `slider_val()`
    (integer lo..hi on the `SLIDER_X`/`SLIDER_W` track). Leave DRAWN geometry
    spanning the full track: the reading stays honest, only the mapping
    saturates. Vertical axes usually need NO padding, because the pads' top and
    bottom are interior to the screen and a finger can overshoot and clamp —
    the exception is an axis whose end is fenced by a gated branch or an
    adjacent button row (synth cutoff's top end), which needs saturation at
    that end only. Symptom to recognise: "I can't get to 100% / to the highest
    value" — it is this, not a sensor or scaling bug.

    **Updated 2026-08-23 — banding vs. widening.** The fix above (draw full
    width, saturate the mapping) was superseded for a while by an INTERACTION
    BAND: draw every continuous control inside `TRACK_L`..`TRACK_R` (90..390) so
    drawn == usable. That fixed reachability and the bench confirmed it ("now I
    can get to full 1x or -1x play speed"), but it also made several pages look
    hollow — 90 px of dead margin each side — and the EQ page lost real plot
    resolution. Neither policy is right everywhere. **Decide per control, on
    whether its extreme is a NAMED VALUE or just the end of a range:**

    - **Named extreme → BAND it.** The sampler strip (1x forward / 1x reverse),
      the theremin volume bar (full volume), parameter sliders (100%), the drums
      BPM strip (60 / 184). There is no way to make an unreachable "1x" mean
      something else, so the control has to come to the thumb.
    - **Range extreme → draw FULL WIDTH and widen the range instead.** The synth
      pad (2.0 → 2.5 octaves) and the FX X/Y pitch pad (±1.0 → ±1.5 octaves).
      The comfortable middle (x 90..390, i.e. |0.68| of the axis) then still
      carries the musically important span, and the extra sits out in the
      corners you have to stretch for. Costs nothing a user can name.
    - **Precision/legibility dominates → FULL WIDTH.** The EQ page: it is a
      dense response plot navigated with the coloured buttons and the D-pad as
      well as by dragging, so the plot IS the information and shrinking it was a
      straight loss.

    When reverting a banded control to full width, change EVERY reference —
    track draw, HANDLE position, hit-test and label. The EQ Q strip was reverted
    with the handle still positioned off `SLIDER_X`/`SLIDER_W`, so it rendered
    between x=96 and x=384 on a track drawn 10..472: it reached neither end and
    never sat under the finger. Give a reverted page its own geometry defines
    (`EQ_SLX`/`EQ_SLW`) rather than editing the shared `SLIDER_*`, or the revert
    drags every effect settings page along with it.

21. **Two PSRAM app schemes exist and the difference is SRAM, not speed**
    (2026-08-07). Both stage the image at `0x11000000` — the loader routes on
    that address (invariant 19) — but they execute differently:
    `apps/orcastra/psram_link` + `pico_set_binary_type(copy_to_ram)` copies
    `.text`/`.rodata`/`.data` into SRAM and runs there (~125 KB of SRAM);
    `apps/orcastra/psram_xip_link` with **no** `pico_set_binary_type()` call
    leaves `.text`/`.rodata` at VMA==LMA in PSRAM to execute in place and copies
    only ~28 KB (`orcastra_psram`). The second frees ~96 KB — 22.6 KB of SRAM
    free becomes 118.6 KB — which matters because `.bss` alone is 368 KB
    (`gbuf` 131,072, `strip_buf` 46,080, `sbuf`/`s_raw` 32,768 each). Latency is
    NOT a reason to avoid XIP-from-PSRAM: a production build using it was
    demoed and latency-measured with no audible penalty. **THE TRAP:** `board_init()` raises
    `clk_sys` to 250 MHz, which invalidates the QMI M1 timing for PSRAM until
    `psram_reinitialize()` completes, and under the XIP scheme the CPU fetches
    its own instructions through that mis-timed window — so everything executing
    in the gap (`psram.c`, `flash.c`, `clocks.c`, `pll.c`, `vreg.c`, `timer.c`,
    `xip_cache.c` and **`board.c` itself**, the orchestrator that is easiest to
    forget) must be forced RAM-resident. Do that by naming objects in an
    overridden `default_text_excludes.incl`: whatever is excluded from the flash
    `.text` section is picked up by `*(.text*)` in `section_default_data.incl`'s
    RAM `.data` section. The audio objects (`fx`/`vox`/`synth`/`drums`/`keys`,
    21,744 bytes) are in that list for the 5.2 ms deadline, not for correctness.
    Verify with `python tools/check_psram_xip.py` — it asserts VMA==LMA in PSRAM,
    SRAM residency of the timing-critical and audio symbols, that `main` stays
    in PSRAM, and that the image fits the 1 MB reservation. Cross-checked against a
    known-good PSRAM-resident build: our image reproduces its address-literal
    ratio (1.63) and copy-region size (28,024 vs 29,108) almost exactly, which
    is the cheapest available confirmation that the scheme is really in force.

22. **ONE submodule: `wilibsp`. Keep it that way** (2026-08-23). The
    project declares exactly one dependency, and that is the point — the app is
    buildable from the FREE-WILi 2 board support package alone. 17 other
    submodules used to be declared; the build referenced **none** of them, and
    the only non-BSP one that was wired in (`external/onewili`) had **zero
    `ow_*` call sites** — power zones actually go through the BSP's
    `input/picpwr.h`. All removed. If the OneWili display-link API is ever
    genuinely needed, do NOT add a top-level submodule: wilibsp already vendors
    it at `wilibsp/libs/onewili`, so point at that and the
    one-dependency story survives.
    **THE PIN IS `8cdd5cb`** ("merge:
    resolve FW2App contract with target enforcement"), which is an ancestor of
    `freewili/wilibsp` master — so `git clone --recursive` reaches it. Verify
    that property with `git merge-base --is-ancestor <pin> origin/master` before
    ever moving the pin: an earlier pin pointed at a commit that existed on no
    remote at all, which builds fine locally and is unclonable by anyone else —
    the single easiest way to publish a repo nobody can build. All eight former
    BSP fixes — `ft6336-two-point-poll`, `picpwr-reassert-superset`,
    `i2s-integer-clkdiv-v2`, `bound-i2c-transfers`, `capture-dma-ring-clamp`,
    `speaker-output-ceiling`, `power-zone-requests`, `power-up-example-apps` —
    are upstream at or before that commit, verified with
    `git merge-base --is-ancestor`.

    **Requirements for building against a newer pin.** Two things must be in
    place before the pin can advance past `a5baa71` ("feat(usb): add PIO HID
    host driver"), which makes `bsp/CMakeLists.txt` compile the PIO USB host
    sources:

    1. Point the SDK at the Pico-PIO-USB copy that wilibsp vendors, BEFORE
       `include(pico_sdk_import.cmake)` in the top-level CMakeLists.txt:

           set(PICO_PIO_USB_PATH
               "${CMAKE_CURRENT_LIST_DIR}/wilibsp/bsp/third_party/Pico-PIO-USB")

    2. Initialize the Pico SDK's own TinyUSB submodule, which the VS Code
       extension leaves unpopulated:

           git -C ~/.pico-sdk/sdk/2.3.0 submodule update --init lib/tinyusb

       Without it the build fails at the CMake GENERATE step — not at
       configure, which makes it easy to misread — with
       `Cannot find source file: .../lib/tinyusb/src/tusb.c`.

    With both in place the app builds against master and passes
    `tools/check_psram_xip.py` and wilibsp's `tools/check_app_uf2.py`. Advancing
    the pin is a prerequisite for adopting `fw2_psram_app()` (see the roadmap in
    README.md): the commits above `8cdd5cb` are largely PSRAM-app bootstrap
    fixes to that helper.

    **SDK 2.3.0 is a hard floor.** 2.2.0 has no
    `pico_add_linker_script_override_path`, so it cannot configure this project.

23. **PSRAM tCPH lives in the TOP-LEVEL CMakeLists, not per-app** (2026-08-23).
    `PICO_DEFAULT_PSRAM_MIN_DESELECT=22` is set with `add_compile_definitions()`
    before `add_subdirectory(wilibsp/bsp bsp)`. It MUST be, because the
    consumer is `board_init()` in the BSP, which compiles **once** into the
    shared `freewili2_bsp` static library — a
    `target_compile_definitions(<app> PRIVATE ...)` silently does nothing, and
    that was confirmed the hard way: `board.c.obj` still read `movs r2, #18`
    with a per-target define apparently in place. Verify a change actually
    landed by disassembling `board.c.obj` and looking for `movs r2, #NN`
    immediately before `bl psram_configure_params` — do not trust the CMake.
    Why 22: the SDK computes `FIELD = ceil(ns/period) - ceil(divisor/2)` and the
    hardware deselect is `1 + FIELD` cycles, so at 250 MHz 18 ns gives 20.00 ns
    (only 10% over the APS6404L minimum, itself ambiguous 15-20 ns) and 22 ns
    gives 24.00 ns. The whole 21..24 ns window maps to the same field, so a
    small clock change cannot silently drop a cycle. It matters most for
    `orcastra_psram`, which fetches instructions from PSRAM: there a marginal
    read is a corrupted instruction, not a dropped pixel.

24. **Three-finger reset: YELLOW + GREEN + BLUE held briefly returns the display
    to the main navigation menu** (2026-08-23, found on the bench). This is the
    way out of a hung or black-screened launched app — no power cycle, no
    reflash, no SWD. Use it between launch attempts; it turns "one test per
    power cycle" into a fast iteration loop, which matters because a failed
    PSRAM app leaves the display sitting in whatever state it died in.
    **Do NOT reach for RED as the escape hatch**: holding RED is the power
    coprocessor's own power-off gesture, it fires beneath anything this firmware
    can intercept (invariant 18 and the reason the fifth sampler slot was
    deleted), and it takes the whole unit down rather than just the app.
