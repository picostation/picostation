#include "power_monitor.h"

#include <stdio.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "logging.h"
#include "values.h"

#if DEBUG_MAIN
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) while (0)
#endif

namespace {

// ADC3 (GPIO29) is wired on a genuine Raspberry Pi Pico module to a factory voltage
// divider off VSYS, and is not used anywhere else in this firmware (confirmed against
// boards/picostation_pico1.cmake and boards/picostation_plus_pico1.cmake).
constexpr unsigned int c_vsysAdcGpio = 29;
constexpr unsigned int c_vsysAdcInput = 3;

// PLACEHOLDER. adc_read() on this input returns VSYS/3 as a 12-bit raw value
// (0-4095 for a 0-3.3V ADC range). This raw threshold has NOT been derived from a
// real PicoStation board and must be measured/calibrated on actual hardware -
// see the PR description for the open question this raises.
constexpr uint16_t c_vsysLowThresholdRaw = 1200;

// How often the brownout watch samples VSYS. Short enough to catch a fast collapse,
// long enough to keep the ADC off the time-critical core0/core1 loops (this timer runs
// independently of both, see PR description).
constexpr int32_t c_pollIntervalMs = 5;

repeating_timer_t s_powerMonitorTimer;
volatile bool s_powerLossLatched = false;

bool powerMonitorCallback(repeating_timer_t *rt) {
    if (s_powerLossLatched) {
        return true;  // already handled, keep the timer alive but do nothing further
    }

    adc_select_input(c_vsysAdcInput);
    const uint16_t vsysRaw = adc_read();

    if (vsysRaw < c_vsysLowThresholdRaw) {
        s_powerLossLatched = true;

        // Deselect the SD card immediately via raw GPIO, without going through FatFs.
        // FatFs is built with FF_FS_REENTRANT=0 (see include/ffconf.h), so it is not
        // safe to call into it from this timer callback while core1 may have a
        // f_read() in flight. Forcing CS high here can abort a transaction mid-block,
        // but that is the intended trade-off: stop the bus immediately rather than
        // wait for a handshake we may no longer have time to complete.
        gpio_put(Pin::SD_CS, 1);

        DEBUG_PRINT("power_monitor: VSYS low (raw=%u), SD card deselected\n", vsysRaw);
    }

    return true;  // keep the repeating timer running
}

}  // namespace

void picostation::initPowerMonitor() {
    adc_init();
    adc_gpio_init(c_vsysAdcGpio);

    add_repeating_timer_ms(-c_pollIntervalMs, powerMonitorCallback, nullptr, &s_powerMonitorTimer);
}
