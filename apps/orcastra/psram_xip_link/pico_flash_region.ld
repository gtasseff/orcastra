/* PSRAM-EXECUTION app: the image both LOADS and RUNS here.
 *
 * This is the sibling of ../psram_link/pico_flash_region.ld and the difference
 * between them is the whole point, so read both.
 *
 *   ../psram_link  + copy_to_ram : image STAGED at 0x11000000, crt0 copies
 *                                  .text/.rodata/.data into SRAM, runs there.
 *                                  Costs ~125 KB of SRAM.
 *   this file      + default     : image STAGED at 0x11000000 and .text/.rodata
 *                                  EXECUTE IN PLACE from PSRAM via XIP. Only
 *                                  .data (+ the RAM-resident set below) is
 *                                  copied to SRAM. Costs ~29 KB of SRAM.
 *
 * Provenance: this is the scheme a known-good production build of this app
 * uses. Recovered by reading that binary rather than guessing: the crt0 copy
 * descriptor at 0x11000128 points to 0x1102D64C -- 186 KB into a 215 KB image
 * -- so only the trailing ~28.6 KB lands in SRAM. Ours pointed at 0x1100020C,
 * its .text LMA, i.e. the whole image. Corroborated by an address-literal
 * census over both full images: SRAM/PSRAM literal ratio 1.63 his vs 30.34
 * ours. He did this because SRAM was, in his words, "so close to the brink":
 * .bss alone is 368 KB and our copy_to_ram build had 22.6 KB spare.
 *
 * THE HAZARD THIS SCHEME CREATES, AND WHY default_text_excludes.incl EXISTS
 * NEXT TO THIS FILE:
 *
 * board_init() raises clk_sys to 250 MHz (MCLK = clk_sys/61 = 256 x fs, the
 * audio-optimal ratio). That INVALIDATES the QMI M1 timing for PSRAM, so
 * psram_reinitialize() must follow. Between set_sys_clock_khz() and that call,
 * the PSRAM window is live but mis-timed -- and under this scheme the CPU is
 * fetching its instructions THROUGH that window. Every function executing in
 * that gap must therefore be RAM-resident, which is what the sibling
 * default_text_excludes.incl override forces. Under copy_to_ram the problem
 * cannot arise, which is exactly what bsp/platform/board.c's comment relies on
 * ("safe here because every app is copy_to_ram").
 *
 * 1 MB reserved, matching the loader's reservation at 0x11000000. Current
 * image with the voice sample baked in is ~210 KB. PSRAM *data* lives above
 * this window -- see pico_psram_region.ld.
 */
FLASH(rx) : ORIGIN = 0x11000000, LENGTH = (1 * 1024 * 1024)
