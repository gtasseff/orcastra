# Adding your own effect

The audio engine is deliberately dull to extend. Every effect is one function
with the same shape, the chain is assembled in one readable place, and
parameter pages are a table rather than code. This walks through the whole
path, using effects that already exist as the reference — so when this document
drifts, the code wins.

Read [`../AGENTS.md`](../AGENTS.md) before you touch the audio path. It has the
constraints that will bite you (the capture deadline, the SRAM residency rules,
the redraw hazard). This document only covers the mechanics.

---

## 0. The one hard rule

```
16-bit mono, 48,828 Hz, processed in blocks, with a 5.2 ms deadline per block.
```

Your `process()` runs on the audio core inside that budget, alongside every
other enabled effect. Two consequences:

- **No allocation, no blocking, no printf, no file I/O** in the audio path.
  Pre-allocate your state as file-scope statics.
- **Your code must live in SRAM**, not PSRAM. The app executes from PSRAM,
  which is slower and shares a bus with the display. Add your object file to
  the RAM-resident set in
  [`../apps/orcastra/psram_xip_link/default_text_excludes.incl`](../apps/orcastra/psram_xip_link/default_text_excludes.incl)
  — the existing `*fx.c.obj *vox.c.obj *synth.c.obj ...` list is exactly this.
  `python tools/check_psram_xip.py` will tell you if you got it wrong.

If your effect needs a big buffer (a delay line, a reverb tank), look at how
`fx.c` sizes its int16 lines and says why. SRAM is the scarce resource here,
not CPU.

---

## 1. Write the DSP

Everything in `fx.c` follows one signature:

```c
void fx_myfx_process(const int16_t *in, int16_t *out, int n);
```

In-place is normal — the chain calls every effect as
`fx_x_process(m_buf, m_buf, N)`. Bypass is your job and it should be the first
line, because the function runs on *every* block whether the effect is engaged
or not:

```c
static bool  my_on   = false;     // engaged?
static float my_amt  = 0.5f;      // 0..1, set from the UI
static float my_st   = 0.0f;      // whatever state you need

void fx_myfx_process(const int16_t *in, int16_t *out, int n) {
    if (!my_on) { if (in != out) memcpy(out, in, n * sizeof *out); return; }
    for (int i = 0; i < n; i++) {
        float x = (float)in[i];
        /* ... your DSP ... */
        if (x >  32700.0f) x =  32700.0f;   // always clamp before the cast
        if (x < -32700.0f) x = -32700.0f;
        out[i] = (int16_t)x;
    }
}
```

Two habits worth copying from the existing effects:

- **Slew your parameters per block, not per sample.** Discrete touch positions
  become zipper noise otherwise. `fx_set_params()` shows the pattern:
  `cur += 0.25f * (tgt - cur)` once per block, ~4 blocks to settle.
- **Clamp before every `int16_t` cast.** Wrapping sounds like a gunshot.

Declare it in `fx.h` next to its neighbours, with an `FX_ENABLE_MYFX` define if
you want it switchable.

## 2. Put it in the chain

One line in the audio pump in `main.c` — search for `fx_tune_process`. The
order is the whole design, and the comments there explain each position:
pitch correction first (it wants the cleanest signal), then the envelope filter
*before* the gain stages (behind a distortion its follower barely sweeps), then
drive lowest-to-highest, then EQ, then modulation, then time and space.

Put yours where a pedal of that type would go, and if it's somewhere
surprising, say why in a comment.

## 3. Give it a menu tile

The FX menu is a 4×3 grid, and **twelve is the number that fits** — 4×3 with no
empty cell to special-case. There are already four effects benched behind
`FX_ENABLE_*` flags for exactly this reason, so your realistic options are:

- **Take a dormant slot.** Flip `FX_ENABLE_TREMOLO` (or PHASER / RINGMOD /
  VOWEL) to `0` if it isn't already, and put yours in its index.
- **Displace one you don't want.** The notes at the top of `fx.h` record why
  each benched effect lost its slot, which is a decent guide to what is
  expendable.

Three things to edit in `main.c`, all near each other:

| what | where | note |
|---|---|---|
| `MENU_LABELS[]` | indexed by effect id | **8 characters max** — at 12 px/char a 9-char label is 108 px in a 110 px button |
| `MENU_GRID[]` | row-major cell → effect id | this is what sets the on-screen position |
| `menu_active()` | `case <id>: return my_on;` | drives the tile's lit/unlit state |

## 4. Give it a parameter page

If your effect has one to three knobs, **you write no UI code at all.** Add one
entry to `KNOB_PAGES[]` in `main.c`:

```c
{ "MYFX", "A SHORT ALL-CAPS BLURB THAT TELLS SOMEONE WHAT TO TRY.",
  &my_on, fx_myfx_set, 2,
  { { "AMOUNT", "SUBTLE", "EXTREME", 0, 10, &my_amt_i, fx_myfx_amount },
    { "MODE",   "SOFT",   "HARD",    0, 1,  &my_mode,  fx_myfx_mode,
      MY_MODES, NULL } } },
```

Per knob: display name, low and high end labels, the integer range, a pointer
to the backing int, and the setter to call as it changes. The two optional
trailing fields are `names` (a value→label table, for a knob that picks a
*mode* rather than a magnitude — the readout shows `names[v]` instead of the
number) and `unit` (a suffix like `"%"`).

Three knobs get a tab row, selectable with UP/DOWN or a tap, sharing one
slider. One generic draw function and one generic input handler serve every
page. That table exists because copy-pasting near-identical pages is how
`main.c` got long in the first place — please keep it that way.

If your effect needs something a slider can't express (the EQ's draggable
response plot, the XY pad), you'll be writing a real page. Copy the EQ page as
your model, and read the next section first.

## 5. Touch: use the shared mappers

**Do not hand-roll touch arithmetic.** Call `touch_norm()` (0..1),
`touch_norm_bipolar()` (−1..+1, for a control with a painted centre detent) or
`slider_val()` (integer lo..hi on the standard track).

There is a real reason this is a rule. Two corrections apply to every
continuous control on this hardware — reachable-end saturation, and the fact
that a thumb in the production case cannot comfortably get to the outer ~30 px
of the panel. They were originally fixed one page at a time, and every page
that hadn't been played yet still shipped the bug. The symptom is *"I can't get
to 100%"*, and it is never a sensor problem.

`AGENTS.md` invariant 20 also explains when a control should be drawn narrow
(when its extreme is a **named value** like "1× forward" that must be
selectable) versus drawn full width with a wider range (when the extreme is
just the end of a range, like an octave span). Pick deliberately.

## 6. Redrawing while audio runs

Anything bigger than a button is a hazard. A full-screen fill takes ~50 ms over
SPI, which is three capture blocks; missing them slips the microphone ring's
read/write phase and every block tears afterwards — permanent fuzz that no
effect toggle clears.

So after any full-page draw, call `mic_ring_rephase()`. And if you add a
per-frame painter, gate it while a modal overlay is up. Both traps, and their
bench symptoms, are written up in `AGENTS.md` invariant 13.

---

## 7. Test it

```bash
python tools/fw.py build
```

```bash
python tools/check_psram_xip.py
```

The second one is not optional if you touched linker scripts or added a source
file — it asserts that the timing-critical set is still in SRAM, that nothing
reachable from the boot re-clock lives in PSRAM, and that you haven't run out
of SRAM headroom.

Then, with `-DFW2_AGENTIO=ON`, you can drive the device from the host and check
your page actually looks right:

```bash
python tools/agentio.py touch 240 160 && python tools/agentio.py screenshot -o shot.png
```

That harness is how every screenshot in the README was produced, and walking
every page with it is how several overlapping-label bugs were found that nobody
had noticed by hand.

---

## Ideas, if you want somewhere to start

- **Turn a dormant effect back on** and give it a parameter page — the smallest
  possible real change, and it teaches you the whole path.
- **A new drum voice.** `drums.c` synthesises all five; a sixth is
  self-contained.
- **A new synth waveform.** `synth.c` uses PolyBLEP for band-limited edges;
  adding a pulse with variable width is a contained exercise.
- **Use a sensor nobody used yet.** The theremin drives pitch from the ambient
  light sensor and can run off the magnetometer instead; the synth uses the IMU
  for modulation while a note is held. Nothing uses the temperature sensor, or
  the IMU as a step trigger.
- **A different visualiser mode.** `viz.c` already owns the framebuffer and the
  FFT.
