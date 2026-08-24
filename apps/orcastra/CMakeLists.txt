set(ORCASTRA_SRCS main.c fx.c vox.c synth.c drums.c viz.c keys.c
                  voice_data.c)   # GENERATED: flash-resident robot-voice sample

# EVERY TARGET LOADS AT 0x11000000. Do not add one that loads into flash.
#
# QSPI flash at 0x10000000 holds the stock DISPLAY firmware, which is the
# bootloader that launches apps from the SD card. An image linked there and
# programmed over the debug probe replaces it, and this repository ships no
# image to restore it.
#
# Install by copying the .uf2 to /apps on the card, or with wilibsp's
# `fw install-app`, which writes the card over MAIN's serial link. The debug
# probe remains the way to read RTT and to drive the agentio harness; neither
# writes to the device.

# ---------------------------------------------------------------------------
# orcastra: executes in place from PSRAM, staged at 0x11000000 by the
# display loader.
#
# fw2_psram_app() (wilibsp/bsp/CMakeLists.txt) supplies the whole scheme: the
# psram_link scripts, the SRAM-resident bootstrap that copies .data, clears
# .bss, installs the vector table and performs the clk_sys raise and QMI
# re-time before jumping into PSRAM-resident main(), the FW2App metadata
# record, and UF2 generation plus validation.
#
# main() still calls board_init(). The bootstrap has already configured clocks
# and QMI and records that, so board_init() initializes only the inherited
# peripherals. Do not reintroduce a hand-rolled clock or QMI re-time here; that
# belongs in the BSP.
#
# The DSP runs from PSRAM. Forcing it into SRAM was measured against the 5.2 ms
# capture deadline and bought about 1.5 points of audio-core load under a heavy
# effect chain, which does not justify carrying linker overrides for it.
#
# No POWER_ZONES are declared. Declaring them makes fw2_app_recovery_init()
# block until every listed rail reports powered, with no timeout; this app
# requests its rails itself and tolerates their absence.
add_executable(orcastra ${ORCASTRA_SRCS})
target_link_libraries(orcastra freewili2_bsp pico_multicore)

# Adds libgcc to the BSP's .sram_bootstrap section. board_init_psram() raises
# clk_sys and then calls clock_configure() and psram_configure_params() before
# psram_reinitialize() re-times the QMI; both divide 64-bit values, which
# branches through a veneer into __aeabi_uldivmod. Without this, that helper is
# PSRAM-resident and is fetched through the window being reconfigured.
#
# MUST precede fw2_psram_app(): the SDK documents that an override path will
# not displace one registered before it. Delete this directory and this call
# once the BSP's own bootstrap section carries libgcc.
pico_add_linker_script_override_path(orcastra
    ${CMAKE_CURRENT_LIST_DIR}/psram_link_override
    FILES sections_no_flash.incl)

fw2_psram_app(orcastra
    NAME "Orcastra"
    VERSION 001
    DESCRIPTION "FreeWili 2 audio multi-tool: synth, sampler, drums, theremin, FX chain"
    REPOSITORY "https://github.com/gtasseff/orcastra")
