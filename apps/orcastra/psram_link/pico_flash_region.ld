/* SD-card-loadable PSRAM app: retarget the copy_to_ram LOAD region into PSRAM.
 *
 * The stock DISPLAY loader routes an /apps image by the address its UF2 blocks
 * target -- NOT by filename (the `_RAM_ONLY` suffix on the stock apps is purely
 * a human convention):
 *
 *   0x10000000 -> FIRMWARE UPDATE. Written into DISPLAY flash, then reset.
 *                 This REPLACES the stock display firmware. Never ship an app
 *                 built this way; a raw .bin is also classified this way
 *                 because it carries no address at all.
 *   0x20000000 -> RAM app. Hard 192 KiB ceiling (the fused bootloader
 *                 accumulates into a single 192 KiB staging buffer and
 *                 silently drops the rest). Our image is ~300 KB, so this
 *                 path cannot hold it.
 *   0x11000000 -> PSRAM app. 8 MB window, staged and jumped to, flash untouched.
 *
 * So the load image goes here at 0x11000000. Execution does NOT stay in PSRAM:
 * the SDK's copy_to_ram crt0 copies .text/.data into SRAM and runs from there,
 * which is mandatory rather than cosmetic. board_init() raises clk_sys to
 * 250 MHz (MCLK = clk_sys/61 = 256 x fs, the audio-optimal ratio), and that
 * invalidates the QMI M1 timing for PSRAM, so psram_reinitialize() must follow
 * -- a call that is documented unsafe while executing from the very memory
 * being re-timed (see bsp/platform/board.c). Copy first at the inherited
 * 150 MHz where the window is already valid, jump to SRAM, and only then
 * re-clock. Order matters: copy, jump, re-clock.
 *
 * 1 MB reserved for the image (currently ~300 KB including the 83 KB
 * __in_flash("voice") array, which lands here in PSRAM and so costs no SRAM).
 * PSRAM *data* is pushed above this window -- see pico_psram_region.ld.
 */
FLASH(rx) : ORIGIN = 0x11000000, LENGTH = (1 * 1024 * 1024)
