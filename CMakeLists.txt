cmake_minimum_required(VERSION 3.13)

# FREE-WILi 2 board header (PICO_RP2350A=0 -> RP2350B / 48 GPIO, 16 MB flash).
# Never pass -DPICO_BOARD on the cmake command line — it would override this.
set(PICO_BOARD freewili2 CACHE STRING "Board type")
list(APPEND PICO_BOARD_HEADER_DIRS "${CMAKE_CURRENT_LIST_DIR}/external/wilibsp/bsp/boards")

# wilibsp vendors Pico-PIO-USB rather than fetching it, and the SDK's
# tinyusb_pico_pio_usb wiring needs this pointed at the vendored copy BEFORE
# pico_sdk_import.cmake runs. The BSP compiles its PIO USB host sources
# unconditionally, so without this the build fails on a missing pio_usb.h.
set(PICO_PIO_USB_PATH
    "${CMAKE_CURRENT_LIST_DIR}/external/wilibsp/bsp/third_party/Pico-PIO-USB")

include(pico_sdk_import.cmake)
project(orcastra C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

# Needed by the FW2App metadata generator (wilibsp tools/gen_uf2_info.py).
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Agent E2E harness (input injection + screen capture) — used to script the
# demo tour and pull pixel-perfect frames for marketing GIFs.
# E2E harness (input injection + screen capture) behind `fw screenshot`,
# `fw touch`, `fw press`. A CACHE OPTION, not a hardcoded value, because the two
# settings build DIFFERENT SETS OF TARGETS and both are legitimate:
#
#   OFF (default)  every target builds. The flash/SWD dev loop (orcastra) works.
#   ON             orcastra and orcastra_sd DO NOT FIT and are skipped below;
#                  only orcastra_psram builds, with ~112 KB SRAM free.
#
# The cost is ~45 KB of SRAM and FW2_AGENTIO is GLOBAL -- freewili2_bsp is one
# shared static library that exports it to every target -- so the copy_to_ram
# builds cannot opt out. With it ON they fail as:
#   ld: section `.bss' will not fit in region `RAM'
#   ld: region `RAM' overflowed by 16652 bytes
# Skipping them (see apps/orcastra/CMakeLists.txt) turns that link error into an
# explicit status message, because a build that fails by design should say so.
#
# That the PSRAM-resident target is the ONLY variant with room for the harness is
# the clearest justification for it existing.
#
#   cmake -B build -DFW2_AGENTIO=ON     # screenshot/touch sessions
#   cmake -B build -DFW2_AGENTIO=OFF    # normal development
option(FW2_AGENTIO "Build the agentio E2E harness (PSRAM target only)" OFF)

# PSRAM tCPH (deselect time), widened from the SDK's 18 ns default.
#
# MUST be set here, before the BSP is added, and NOT as a per-target define:
# board_init() is the consumer and it compiles ONCE into the shared
# freewili2_bsp static library, so `target_compile_definitions(<app> PRIVATE ...)`
# silently does nothing. That was verified the hard way -- the call still read
# `movs r2, #18` in board.c.obj after a per-target define appeared to be set.
#
# WHY 22: the SDK computes  FIELD = ceil(ns / clock_period) - ceil(divisor / 2),
# and the hardware's actual deselect is 1 + FIELD cycles. At our 250 MHz
# (4.00 ns period, divisor 2):
#     18 ns -> FIELD 4 -> 5 cycles -> 20.00 ns   only 10% over the APS6404L's
#                                                18 ns minimum, and the spec is
#                                                itself ambiguous 15-20 ns
#     22 ns -> FIELD 5 -> 6 cycles -> 24.00 ns   33% over
# 22 rather than 21 or 24 because the whole 21..24 ns window maps to FIELD 5, so
# a small clock change cannot silently drop a cycle. Cost is one idle cycle
# between chip-select assertions: irrelevant against the 5.2 ms capture deadline,
# and nothing here is bandwidth-bound.
#
# It matters most for orcastra_psram, which FETCHES ITS INSTRUCTIONS from PSRAM
# -- there a marginal read is a corrupted instruction, not a dropped pixel. It is
# applied globally because the BSP is shared, and because being more conservative
# costs the copy_to_ram apps nothing.
#
# Note we do NOT inherit the stock loader stub's own widened value: board_init()
# re-runs psram_configure_params() + psram_reinitialize() at 250 MHz and
# overwrites whatever the stub left. This is what that re-time picks up.
add_compile_definitions(PICO_DEFAULT_PSRAM_MIN_DESELECT=22)

# Shared FREE-WILi 2 BSP (freewili2_bsp STATIC lib) from the wilibsp submodule.
add_subdirectory(external/wilibsp/bsp bsp)

# NO SECOND SUBMODULE ON PURPOSE. wilibsp above is the ONLY dependency this
# project declares, which is the point: the app is buildable from the FREE-WILi 2
# board support package alone.
#
# There used to be `add_subdirectory(external/onewili onewili)` here, for
# commanding the MAIN CPU's power zones over the FwGUI UART0 link. It is gone
# because it was never used: the app has zero `ow_*` call sites, and power zones
# are driven through the BSP's own `input/picpwr.h` (`picpwr_rails()`,
# `picpwr_zone_bit()`) instead. Only two #include lines and this line ever
# referenced it.
#
# If that link is ever genuinely needed, do NOT add a top-level submodule for
# it: wilibsp already vendors the same library at `external/wilibsp/libs/onewili`
# (its own submodule), so point at that and the one-dependency story survives.

# Our apps. Each links freewili2_bsp; add new apps here.
add_subdirectory(apps/orcastra)

