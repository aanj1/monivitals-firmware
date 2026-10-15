#pragma once

/* ESP32-C3 Super Mini ↔ MAX30102 default wiring.
 * Prefer GPIO 6/7 over labeled SDA/SCL (8/9) — those are strapping pins.
 * Override via platformio.ini build_flags if needed. */

#ifndef SB_PIN_I2C_SDA
#define SB_PIN_I2C_SDA  6
#endif
#ifndef SB_PIN_I2C_SCL
#define SB_PIN_I2C_SCL  7
#endif
#ifndef SB_PIN_MAX30102_INT
#define SB_PIN_MAX30102_INT  5
#endif

#ifndef SB_DEVICE_NAME_PREFIX
#define SB_DEVICE_NAME_PREFIX "MoniVitals"
#endif

#define SB_I2C_FREQ_HZ  400000
