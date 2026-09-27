#include "picostation.h"

#include <stdio.h>
#include <time.h>

#include "cmd.h"
#include "disc_image.h"
#include "drive_mechanics.h"
#include "hardware/pwm.h"
#include "i2s.h"
#include "logging.h"
#include "main.pio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "power_monitor.h"
#include "pseudo_atomics.h"
#include "subq.h"
#include "values.h"

#if DEBUG_MAIN
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) while (0)
#endif

// To-do: Implement lid switch behavior
// To-do: Implement a console side menu to select the cue file
// To-do: Implement level meter mode to command $AX - AudioCTRL
// To-do: Fix seeks that go into the lead-in + track 1 pregap areas, possibly sending bad data over I2S

// To-do: Make an ODE class and move these to members
static picostation::I2S m_i2s;
static picostation::MechCommand m_mechCommand;

bool picostation::g_subqDelay = false;  // core0: r/w

static int s_currentPlaybackSpeed = 1;
int picostation::g_targetPlaybackSpeed = 1;  // core0: r/w

mutex_t picostation::g_mechaconMutex;
pseudoatomic<bool> picostation::g_coreReady[2];

unsigned int picostation::g_audioCtrlMode = audioControlModes::NORMAL;
// pseudoatomic<int32_t> picostation::g_audioPeak;
// pseudoatomic<int32_t> picostation::g_audioLevel = 0;

pseudoatomic<picostation::FileListingStates> picostation::g_fileListingState;
pseudoatomic<uint32_t> picostation::g_fileArg;

static unsigned int s_mechachonOffset;
unsigned int picostation::g_soctOffset;
unsigned int picostation::g_subqOffset;

static bool s_resetPending = false;

static picostation::PWMSettings pwmDataClock = {
    .gpio = Pin::DA15, .wrap = (1 * 32) - 1, .clkdiv = 4, .invert = true, .level = (32 / 2)};

static picostation::PWMSettings pwmLRClock = {
    .gpio = Pin::LRCK, .wrap = (48 * 32) - 1, .clkdiv = 4, .invert = false, .level = (48 * (32 / 2))};

static picostation::PWMSettings pwmMainClock = {.gpio = Pin::CLK, .wrap = 1, .clkdiv = 2, .invert = false, .level = 1};

static void initPWM(picostation::PWMSettings *settings);
static void interruptHandler(unsigned int gpio, uint32_t events);

static void interruptHandler(unsigned int gpio, uint32_t events) {
    static uint32_t lastLowEvent = 0;

    switch (gpio) {
        case Pin::RESET: {
            if (events & GPIO_IRQ_LEVEL_LOW) {
                lastLowEvent = time_us_32();
                // Disable low signal edge detection
                gpio_set_irq_enabled(Pin::RESET, GPIO_IRQ_LEVEL_LOW, false);
                // Enable high signal edge detection
                gpio_set_irq_enabled(Pin::RESET, GPIO_IRQ_LEVEL_HIGH, true);
            } else if (events & GPIO_IRQ_LEVEL_HIGH) {
                // Disable the rising edge detection
                gpio_set_irq_enabled(Pin::RESET, GPIO_IRQ_LEVEL_HIGH, false);

                const uint32_t c_now = time_us_32();
                const uint32_t c_timeElapsed = c_now - lastLowEvent;
                if (c_timeElapsed >= 500U)  // Debounce, only reset if the pin was low for more than 500us(.5 ms)
                {
                    s_resetPending = true;
                } else {
                    // Enable the low signal edge detection again
                    gpio_set_irq_enabled(Pin::RESET, GPIO_IRQ_LEVEL_LOW, true);
                }
            }
        } break;

        case Pin::XLAT: {
            m_mechCommand.processLatchedCommand();
        } break;
    }
}

[[noreturn]] void __time_critical_func(picostation::core0Entry)() {
    SubQ subq(&g_discImage);
    uint64_t subqDelayTime = 0;

    g_coreReady[0] = true;
    while (!g_coreReady[1].Load()) {
        tight_loop_contents();
    }

    while (true) {
        if (s_resetPending) {
            while (gpio_get(Pin::RESET) == 0) {
                tight_loop_contents();
            }
            reset();
        }

        // Update latching, output SENS
        m_mechCommand.updateMechSens();

        const int currentSector = g_driveMechanics.getSector();

        // Limit Switch
        gpio_put(Pin::LMTSW, currentSector > 3000);

        updatePlaybackSpeed();

        // Soct/Sled/seek
        if (m_mechCommand.getSoct()) {
            if (pio_sm_get_rx_fifo_level(PIOInstance::SOCT, SM::SOCT)) {
                pio_sm_drain_tx_fifo(PIOInstance::SOCT, SM::SOCT);
                m_mechCommand.setSoct(false);
                pio_sm_set_enabled(PIOInstance::SOCT, SM::SOCT, false);
            }
        } else if (!g_driveMechanics.isSledStopped()) {
            g_driveMechanics.moveSled(m_mechCommand);
        } else if (m_mechCommand.getSens(SENS::GFS)) {
            if (g_subqDelay) {
                if ((time_us_64() - subqDelayTime) > c_MaxSubqDelayTime) {
                    g_subqDelay = false;
                    subq.start_subq(currentSector);

                    gpio_put(Pin::SCOR, 1);
                    add_alarm_in_us(
                        135,
                        [](alarm_id_t id, void *user_data) -> int64_t {
                            gpio_put(Pin::SCOR, 0);
                            return 0;
                        },
                        NULL, true);
                }
            } else if (m_i2s.getSectorSending() == currentSector) {
                g_driveMechanics.moveToNextSector();
                g_subqDelay = true;
                subqDelayTime = m_i2s.getLastSectorTime();
            }
        }
    }
}

[[noreturn]] void picostation::core1Entry() {
    m_i2s.start(m_mechCommand);
    while (1) asm("");
    __builtin_unreachable();
}

void picostation::initHW() {
#if DEBUG_LOGGING_ENABLED
    stdio_init_all();
    stdio_set_chars_available_callback(NULL, NULL);
    sleep_ms(1250);
#endif
    DEBUG_PRINT("Initializing...\n");

    mutex_init(&g_mechaconMutex);

    for (const unsigned int pin : Pin::allPins) {
        gpio_init(pin);
    }

    gpio_set_dir(Pin::XLAT, GPIO_IN);
    gpio_set_dir(Pin::SQCK, GPIO_IN);
    gpio_set_dir(Pin::LMTSW, GPIO_OUT);
    gpio_set_dir(Pin::DOOR, GPIO_IN);
    gpio_set_dir(Pin::RESET, GPIO_IN);
    gpio_set_dir(Pin::SENS, GPIO_OUT);
    gpio_set_dir(Pin::DA15, GPIO_OUT);
    gpio_set_dir(Pin::DA16, GPIO_OUT);
    gpio_set_dir(Pin::LRCK, GPIO_OUT);
    gpio_set_dir(Pin::SCOR, GPIO_OUT);
    gpio_put(Pin::SCOR, 0);
    gpio_set_dir(Pin::SQSO, GPIO_OUT);
    gpio_put(Pin::SQSO, 0);
    gpio_set_dir(Pin::CLK, GPIO_OUT);
    gpio_set_dir(Pin::CMD_CK, GPIO_IN);
    gpio_set_dir(Pin::CMD_DATA, GPIO_IN);

    gpio_set_input_hysteresis_enabled(Pin::XLAT, true);
    gpio_set_input_hysteresis_enabled(Pin::SQCK, true);
    gpio_set_input_hysteresis_enabled(Pin::RESET, true);
    gpio_set_input_hysteresis_enabled(Pin::CMD_CK, true);

    initPWM(&pwmMainClock);
    initPWM(&pwmDataClock);
    initPWM(&pwmLRClock);

    unsigned int i2s_pio_offset = pio_add_program(PIOInstance::I2S_DATA, &i2s_data_program);
    i2s_data_program_init(PIOInstance::I2S_DATA, SM::I2S_DATA, i2s_pio_offset, Pin::DA15, Pin::DA16);

    s_mechachonOffset = pio_add_program(PIOInstance::MECHACON, &mechacon_program);
    mechacon_program_init(PIOInstance::MECHACON, SM::MECHACON, s_mechachonOffset, Pin::CMD_DATA);

    g_soctOffset = pio_add_program(PIOInstance::SOCT, &soct_program);
    g_subqOffset = pio_add_program(PIOInstance::SUBQ, &subq_program);

    pio_sm_set_enabled(PIOInstance::I2S_DATA, SM::I2S_DATA, true);
    pwm_set_mask_enabled((1 << pwmLRClock.sliceNum) | (1 << pwmDataClock.sliceNum) | (1 << pwmMainClock.sliceNum));

    uint64_t startTime = time_us_64();
    gpio_set_dir(Pin::RESET, GPIO_OUT);
    gpio_put(Pin::RESET, 0);
    sleep_ms(300);
    gpio_set_dir(Pin::RESET, GPIO_IN);

    while ((time_us_64() - startTime) < 30000) {
        if (gpio_get(Pin::RESET) == 0) {
            startTime = time_us_64();
        }
    }

    while ((time_us_64() - startTime) < 30000) {
        if (gpio_get(Pin::CMD_CK) == 0) {
            startTime = time_us_64();
        }
    }

    gpio_set_irq_enabled_with_callback(Pin::RESET, GPIO_IRQ_LEVEL_LOW, true, &interruptHandler);
    gpio_set_irq_enabled_with_callback(Pin::XLAT, GPIO_IRQ_EDGE_FALL, true, &interruptHandler);

    pio_sm_set_enabled(PIOInstance::MECHACON, SM::MECHACON, true);

    g_coreReady[0] = false;
    g_coreReady[1] = false;

    initPowerMonitor();

    DEBUG_PRINT("ON!\n");
}

static void initPWM(picostation::PWMSettings *settings) {
    gpio_set_function(settings->gpio, GPIO_FUNC_PWM);
    settings->sliceNum = pwm_gpio_to_slice_num(settings->gpio);
    settings->config = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&settings->config, PWM_DIV_FREE_RUNNING);
    pwm_config_set_wrap(&settings->config, settings->wrap);
    pwm_config_set_clkdiv(&settings->config, settings->clkdiv);
    pwm_config_set_output_polarity(&settings->config, settings->invert, settings->invert);
    pwm_init(settings->sliceNum, &settings->config, false);
    pwm_set_both_levels(settings->sliceNum, settings->level, settings->level);
}

void picostation::updatePlaybackSpeed() {
    static constexpr unsigned int c_clockDivNormal = 4;
    static constexpr unsigned int c_clockDivDouble = 2;

    if (s_currentPlaybackSpeed != g_targetPlaybackSpeed) {
        s_currentPlaybackSpeed = g_targetPlaybackSpeed;
        const unsigned int clock_div = (s_currentPlaybackSpeed == 1) ? c_clockDivNormal : c_clockDivDouble;
        pwm_set_mask_enabled(0);
        pwm_config_set_clkdiv_int(&pwmDataClock.config, clock_div);
        pwm_config_set_clkdiv_int(&pwmLRClock.config, clock_div);
        pwm_hw->slice[pwmDataClock.sliceNum].div = pwmDataClock.config.div;
        pwm_hw->slice[pwmLRClock.sliceNum].div = pwmLRClock.config.div;
        pwm_set_mask_enabled((1 << pwmLRClock.sliceNum) | (1 << pwmDataClock.sliceNum) | (1 << pwmMainClock.sliceNum));
        DEBUG_PRINT("x%i\n", s_currentPlaybackSpeed);
    }
}

void picostation::reset() {
    DEBUG_PRINT("RESET!\n");
    pio_sm_set_enabled(PIOInstance::SUBQ, SM::SUBQ, false);
    pio_sm_set_enabled(PIOInstance::SOCT, SM::SOCT, false);
    pio_sm_restart(PIOInstance::MECHACON, SM::MECHACON);

    pio_sm_clear_fifos(PIOInstance::MECHACON, SM::MECHACON);
    pio_sm_clear_fifos(PIOInstance::SOCT, SM::SOCT);
    pio_sm_clear_fifos(PIOInstance::SUBQ, SM::SUBQ);

    g_targetPlaybackSpeed = 1;
    updatePlaybackSpeed();

    mechacon_program_init(PIOInstance::MECHACON, SM::MECHACON, s_mechachonOffset, Pin::CMD_DATA);
    g_subqDelay = false;
    m_mechCommand.setSoct(false);

    gpio_put(Pin::SCOR, 0);
    gpio_put(Pin::SQSO, 0);

    uint64_t startTime = time_us_64();

    while ((time_us_64() - startTime) < 30000) {
        if (gpio_get(Pin::RESET) == 0) {
            startTime = time_us_64();
        }
    }

    while ((time_us_64() - startTime) < 30000) {
        if (gpio_get(Pin::CMD_CK) == 0) {
            startTime = time_us_64();
        }
    }

    pio_sm_set_enabled(PIOInstance::MECHACON, SM::MECHACON, true);
    s_resetPending = false;
    gpio_set_irq_enabled(Pin::RESET, GPIO_IRQ_LEVEL_LOW, true);
}
