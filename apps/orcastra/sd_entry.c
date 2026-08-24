// sd_entry.c - entry hygiene for SD-launched builds. Compiled into BOTH
// orcastra_sd and orcastra_psram -- the latter is the target that ships, so this
// file is on the shipping path, not a side experiment. It is also the most
// delicate file in the repo: three separate attempts to launch from PSRAM hung
// before the cause was found here (see runtime_init_early_resets below).
//
// Two facts about the launched environment, both of which follow from the
// loader contract and the BSP's own PSRAM-app support in bsp/CMakeLists.txt:
//
//   1. The loader/stub enters with cpsid i but with PERIPHERAL IRQ ENABLES
//      INHERITED from whatever firmware ran before us. Clear every enable and
//      pending flag before our init starts wiring handlers, or stale enables
//      fire into our half-initialized handlers the moment IRQs unmask.
//   2. (Paired with PICO_RUNTIME_SKIP_INIT_PSRAM=1 in CMakeLists) the PSRAM/QMI
//      window we were staged through is already configured; the SDK's runtime
//      re-probe wiggles its CS and can wedge the window - the documented
//      "do NOT re-run PSRAM setup" contract.
//
// Runs at PICO_RUNTIME_INIT_EARLIEST, i.e. after crt0 has copied us into SRAM
// (safe) and before any SDK subsystem touches an IRQ.
#include "pico/runtime_init.h"
#include "hardware/irq.h"
#include "hardware/regs/intctrl.h"
#include "hardware/flash.h"
#include "platform/diag.h"
#include "hardware/structs/qmi.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/timer.h"
#include "hardware/resets.h"
#include "fw2.h"            // board_init_inherited(), BOARD_SYS_CLOCK_KHZ


// ---- Boot-stage marker: a debugger-readable breadcrumb ---------------------
//
// WHY THIS AND NOT BREAKPOINTS. A launched app that dies before its first DIAG()
// leaves nothing to read: the screen is black, RTT is all zeros, and by the time
// the CPU is in LOCKUP the PC is 0xEFFFFFFE and tells you nothing. Hardware
// breakpoints are the obvious instrument and they do not work well here: this
// target has TWO Cortex-M cores, so a scripted halt/resume pair resumes only one
// and errors out, leaving the display FROZEN -- which also means nobody can tap
// the menu to launch the app you are trying to catch. Two attempts produced no
// data that way.
//
// So: write a monotonically increasing stage number to a fixed, never-zeroed
// SRAM word. It costs one store per milestone, needs no halt, survives LOCKUP,
// and is read with a single `mdw` while the target runs. Read it with
// tools/read_boot_stage.py after a failed launch.
//
// __uninitialized_ram keeps it out of .bss so crt0's zeroing cannot erase the
// evidence, and so a stale value from a previous attempt is distinguishable from
// a fresh zero.
uint32_t __uninitialized_ram(sd_boot_stage);
// ORDER-INDEPENDENT companion. sd_boot_stage alone was ambiguous and misled me
// once: both hooks below register at PICO_RUNTIME_INIT_EARLIEST, so their
// relative order is LINK-determined, not the order they are numbered. A final
// value of 2 therefore could mean "died before stage 3" OR "devinfo ran first
// (3..7), then irq_handover (1,2), and we died after that". The bitmask removes
// the guesswork: one bit per stage, OR-ed, so order cannot hide a visit.
uint32_t __uninitialized_ram(sd_boot_visited);

#define SD_STAGE(n)  do {                                                      \
        sd_boot_stage = (n);                                                    \
        if ((n) < 32) sd_boot_visited |= (1u << (n));                           \
        __compiler_memory_barrier();                                            \
    } while (0)

static void sd_entry_irq_handover(void) {
    SD_STAGE(1);                  // crt0 copied us to SRAM and runtime init began
    for (uint i = 0; i < NUM_IRQS; i++) {
        irq_set_enabled(i, false);
        irq_clear(i);
    }
#if ORCASTRA_PSRAM_INHERIT_TIMING
    // Re-enable interrupts. NOT cosmetic, and I dismissed it as such once: the
    // PSRAM loader stub branches to vector word 1 with `cpsid i` still in force,
    // and crt0 never issues `cpsie i` because a bootrom handover normally
    // arrives with interrupts already enabled. Left masked, the first sleep_ms()
    // waits forever on a timer IRQ that can never fire. Safe here specifically
    // because the sweep above dropped everything the stub left enabled or
    // pending, so unmasking cannot immediately vector into
    // __unhandled_user_irq.
    __asm volatile ("cpsie i" ::: "memory");
#endif
    SD_STAGE(2);                  // survived the IRQ handover loop
}
PICO_RUNTIME_INIT_FUNC_RUNTIME(sd_entry_irq_handover, PICO_RUNTIME_INIT_EARLIEST);

#if ORCASTRA_PSRAM_INHERIT_TIMING
// ---- THE ONE THAT KILLED THREE ATTEMPTS -----------------------------------
//
// Strong override of the SDK's __weak runtime_init_early_resets(), which runs at
// PICO_RUNTIME_INIT_EARLY_RESETS ("00100") -- immediately after the EARLIEST
// hooks above, and long before main().
//
// The stock version resets IO_BANK0 and PADS_BANK0. It already spares the QSPI
// bank, with the SDK's own comment saying that is "fatal if running from flash".
// The same argument applies with equal force to PSRAM and the SDK does not make
// it: **the PSRAM chip select is XIP_CS1n on GPIO 47, which lives in BANK 0.**
// Resetting bank 0 strips the CS function off that pad, so the very next
// instruction fetch cannot select the chip and takes an IBUSERR.
//
// That is exactly what we measured for three attempts: CFSR 0x00000100
// (IBUSERR on an instruction fetch), escalating to HardFault and LOCKUP, with
// PSRAM reading the same bytes at every address because the part was genuinely
// unselectable -- not, as I first argued, because a wedged QMI was returning
// bus noise. The boot breadcrumb put death right after the EARLIEST hooks, which
// is where this runs.
//
// Body is otherwise verbatim from the SDK; only the two bank-0 bits are added to
// the spare mask. The SDK hands you the argument for free -- its own source
// comment says sparing the QSPI bank is necessary because resetting it is
// "fatal if running from flash". Executing from PSRAM is the same statement
// with a different bank, and bank 0 is where XIP_CS1n lives.
//
// MUST be RAM-resident, which sd_entry.c is (psram_xip_link's exclude list) --
// a PSRAM-resident copy of this function would fault on its own reset write.
void runtime_init_early_resets(void) {
    SD_STAGE(8);
    reset_block_mask(~(
            (1u << RESET_IO_QSPI)   |
            (1u << RESET_PADS_QSPI) |
            (1u << RESET_IO_BANK0)  |     // <-- PSRAM CS (XIP_CS1n, GPIO 47)
            (1u << RESET_PADS_BANK0) |    // <-- and its pad
            (1u << RESET_PLL_USB)   |
            (1u << RESET_USBCTRL)   |
            (1u << RESET_SYSCFG)    |
            (1u << RESET_PLL_SYS)
    ));

    unreset_block_mask_wait_blocking(RESETS_RESET_BITS & ~(
            (1u << RESET_HSTX)   |
            (1u << RESET_ADC)    |
            (1u << RESET_SPI0)   |
            (1u << RESET_SPI1)   |
            (1u << RESET_UART0)  |
            (1u << RESET_UART1)  |
            (1u << RESET_USBCTRL)
    ));
    SD_STAGE(9);                  // survived the early resets with bank 0 intact
}
#endif

// PICO_RUNTIME_SKIP_INIT_PSRAM=1 keeps us off the SDK's PSRAM re-probe (the
// loader contract: the window we were staged through is already configured).
// But skipping it also leaves flash_devinfo's CS1 entry UNSET, and
// psram_reinitialize() hard-requires it:
//
//     invalid_params_if_and_return(HARDWARE_PSRAM,
//         flash_devinfo_get_cs_size(1) == FLASH_DEVINFO_SIZE_NONE,
//         PICO_ERROR_PRECONDITION_NOT_MET);      // sdk hardware_psram/psram.c
//
// board_init_clk() calls psram_reinitialize() after raising clk_sys to 250 MHz
// and IGNORES its return value, so on a launched app the re-time silently did
// not happen: PSRAM kept the QMI M1 timing the loader stub set for ITS clock
// while we ran the bus at 250 MHz. An app that never touches PSRAM after that
// survives (smoke_sd); one that does (orcastra_sd: vox slots, sample seeding,
// FX buffers) dies. That is the real story behind the apparent "large images
// fail" correlation - the variable was PSRAM use, not image size.
//
// Fix: describe CS1 from the board header before board_init() runs, so the
// precondition holds and the re-time actually lands. Same values the SDK's
// runtime_init would have written; we are restoring the one piece of state we
// suppressed, not re-probing the window (no CS wiggling, no XIP exit sequence).
#if ORCASTRA_PSRAM_INHERIT_TIMING
// ---- PSRAM-RESIDENT clock raise ------------------------------------------
//
// Three things we had wrong before reading the BSP and the SDK's PSRAM source
// carefully enough:
//
//   1. NEVER call psram_reinitialize(). Write qmi_hw->m[1].timing DIRECTLY --
//      one register store, no chip teardown, no XIP exit. psram_reinitialize()
//      tears the chip down and re-initialises it, which is fatal when that chip
//      holds the instruction stream. NOTE: wilibsp's own board_init_psram() is
//      misleadingly named -- it calls psram_reinitialize() too, so it is NOT
//      usable by a PSRAM-resident app either.
//   2. PRE-LOAD a timing legal at BOTH the old and the new clock BEFORE changing
//      the clock, so there is never an invalid window to execute through. Raising
//      clk_sys shortens SCK and CS-high the instant it takes effect; if the
//      timing is only correct afterwards, the fetches in between come through an
//      out-of-spec window. That is precisely how we locked up.
//   3. Do NOT call board_init(). Its prologue is the clock + psram_reinitialize()
//      sequence; only its peripheral tail is wanted, which wilibsp exposes as
//      board_init_inherited().
//
// FIELD ARITHMETIC for our 250 MHz (redo it if you change BOARD_SYS_CLOCK_KHZ:
// every number below is a function of the clock):
//   MIN_DESELECT counts clk_sys periods with ONE IMPLIED, so field N holds CS
//   high for (1+N) periods. At 250 MHz (4.00 ns) field 4 gives 20.0 ns, clearing
//   the APS6404L 18 ns tCPH minimum; at the 150 MHz entry clock the same field
//   gives 33.3 ns, which is longer and therefore also safe.
//   MAX_SELECT counts 64 clk_sys periods against an 8 us tCEM, allowing 18 at
//   150 MHz and 31 at 250. So 18 is LEGAL AT BOTH and is what we pre-load; it
//   stays legal at 250 (4.6 us), so the post-raise tighten to 31 is a
//   performance choice, not a correctness one.
//   CLKDIV 2 puts PSRAM at 75 MHz entering and 125 MHz after, both inside the
//   133 MHz part limit. RXDELAY 2 is what the loader stub itself leaves in the
//   register; if reads prove marginal at 125 MHz, 3 is the next thing to try -- the
//   SDK's own RXDELAY calculation crosses into 3 above a ~100 MHz PSRAM clock,
//   and 125 MHz is past that, so 2 is the aggressive end of the legal pair.
#define TP_QMI_M1_BASE (                                                       \
      ((uint32_t)1u << QMI_M1_TIMING_COOLDOWN_LSB)                             \
    | ((uint32_t)QMI_M1_TIMING_PAGEBREAK_VALUE_1024 << QMI_M1_TIMING_PAGEBREAK_LSB) \
    | ((uint32_t)4u << QMI_M1_TIMING_MIN_DESELECT_LSB)                         \
    | ((uint32_t)2u << QMI_M1_TIMING_RXDELAY_LSB)                              \
    | ((uint32_t)2u << QMI_M1_TIMING_CLKDIV_LSB))

// RAM-resident by virtue of sd_entry.c being in psram_xip_link's exclude list --
// mandatory, since this executes across the clock change.
void sd_entry_board_init_psram_resident(void) {
    SD_STAGE(20);
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    busy_wait_us_32(10000);

    SD_STAGE(21);
    qmi_hw->m[1].timing = TP_QMI_M1_BASE | (18u << QMI_M1_TIMING_MAX_SELECT_LSB);
    SD_STAGE(22);
    set_sys_clock_khz(250000, true);
    SD_STAGE(23);                 // survived the clock change - the old killer
    qmi_hw->m[1].timing = TP_QMI_M1_BASE | (31u << QMI_M1_TIMING_MAX_SELECT_LSB);

    SD_STAGE(24);
    uint32_t f = clock_get_hz(clk_sys);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, f, f);

    SD_STAGE(25);
    board_init_inherited();       // wilibsp's peripheral tail, no clock/PSRAM
    SD_STAGE(26);
}

// Suppress board_init()'s PSRAM re-time for the EXECUTE-FROM-PSRAM build.
//
// WHY. board_init_clk() raises clk_sys to 250 MHz and then calls
// psram_configure_params() + psram_reinitialize() to re-time the QMI for the new
// clock. That is correct and necessary for a copy_to_ram app. For THIS build it
// is fatal: the chip being re-timed is the one holding the code we are executing,
// and re-running the SDK's init also discards the value the stock loader stub
// deliberately programmed before staging us.
//
// That inheritance is observable rather than assumed: read QMI M1_TIMING over
// SWD in a launched app before touching it and MIN_DESELECT already reads the
// stub's widened value, not the SDK default. Re-running the SDK's PSRAM init
// puts the default back -- which is exactly the failure below.
//
// OBSERVED FAILURE without this (production unit, first hardware test): loader progress
// bar completes, then a black screen with no backlight and no boot-trace
// markers. Over SWD: PC 0xEFFFFFFE (LOCKUP), CFSR 0x00000100 (IBUSERR -- bus
// fault on an INSTRUCTION FETCH), HFSR FORCED, stacked LR 0x1100def1 (PSRAM) and
// a stacked PC of 0x88888888. Every PSRAM address read back the same pattern
// (00000000 88880000 88888888 88888888) regardless of address, and sweeping
// MIN_DESELECT/RXDELAY over SWD changed nothing -- so the chip was left
// non-responsive rather than merely mis-sampled. The RTT control block was still
// all zeros, i.e. not one DIAG() had completed, which places death at the first
// DIAG in board_init_clk -- immediately before this very call.
//
// Making the callers RAM-resident (invariant 21) was necessary but NOT
// sufficient: it stops us fetching THROUGH the stale window, and does nothing
// about the chip being left unusable afterwards.
//
// Linker --wrap rather than a BSP edit, so wilibsp stays untouched and this
// stays a property of one target. psram_configure_params() is left alone -- it
// only stores statics and writes no register.
int __wrap_psram_reinitialize(void) {
    return 0;   /* inherit the loader stub's QMI M1 timing verbatim */
}
#endif

static void sd_entry_psram_devinfo(void) {
    SD_STAGE(3);                  // reached the devinfo hook
    if (flash_devinfo_get_cs_size(1) == FLASH_DEVINFO_SIZE_NONE) {
        SD_STAGE(4);              // CS1 was unset, about to restore it
        flash_devinfo_set_cs_gpio(1, PICO_PSRAM_CS_PIN);
        flash_devinfo_set_cs_size(1,
            flash_devinfo_bytes_to_size(PICO_PSRAM_SIZE_BYTES));
        SD_STAGE(5);              // OTP-backed devinfo writes survived
        DIAG("sd_entry: flash_devinfo CS1 restored (%u MB @ gpio %u)\n",
             (unsigned)(PICO_PSRAM_SIZE_BYTES / (1024u * 1024u)),
             (unsigned)PICO_PSRAM_CS_PIN);
        SD_STAGE(6);              // FIRST DIAG COMPLETED - RTT is alive
    } else {
        SD_STAGE(7);              // CS1 already set by the loader, nothing to do
    }
}
PICO_RUNTIME_INIT_FUNC_RUNTIME(sd_entry_psram_devinfo, PICO_RUNTIME_INIT_EARLIEST);
