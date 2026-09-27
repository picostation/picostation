#pragma once

namespace picostation {

// Starts a background VSYS brownout watch on ADC3 (GPIO29). On a genuine Raspberry Pi
// Pico module this pin is already wired at the factory to a VSYS voltage divider, so no
// extra wiring is required to call this function.
//
// NOTE: VSYS_LOW_THRESHOLD in power_monitor.cpp is a placeholder and MUST be calibrated
// against the real supply rail on actual PicoStation hardware before this offers any
// real protection. See the PR description for details and open questions.
void initPowerMonitor();

}  // namespace picostation
