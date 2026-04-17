// firmware/src/pin_registry.cpp
// Single source of truth for GPIO pins owned by firmware subsystems.
// Does NOT include the main servo pins (SERVO_PINS[0..7]) because probe_pin
// allows those when disengaged — that conditional check lives in the handler.
#include "pin_registry.h"
#include "config.h"

static inline void set_reason(const char** reason_out, const char* reason) {
    if (reason_out) *reason_out = reason;
}

bool pin_is_reserved(uint8_t pin, const char** reason_out) {
    // Aux servo ports — always reserved when the feature is compiled in.
#if AUX_SERVOS_ENABLED
    for (int i = 0; i < AUX_SERVO_COUNT; i++) {
        if (AUX_SERVO_PINS[i] == pin) {
            set_reason(reason_out, "aux_servo");
            return true;
        }
    }
#endif

    // Buzzer
    if (pin == BUZZER_PIN) {
        set_reason(reason_out, "buzzer");
        return true;
    }

    // Onboard LED
    if (pin == ONBOARD_LED_PIN) {
        set_reason(reason_out, "led");
        return true;
    }

    // User button
    if (pin == BUTTON_PIN) {
        set_reason(reason_out, "button");
        return true;
    }

    // I2C bus 1 (IMU + sonar)
    if (pin == I2C_SDA_PIN || pin == I2C_SCL_PIN) {
        set_reason(reason_out, "i2c1");
        return true;
    }

    // I2C bus 2 (J4 expansion connector)
    if (pin == I2C2_SDA_PIN || pin == I2C2_SCL_PIN) {
        set_reason(reason_out, "i2c2");
        return true;
    }

    // Battery ADC
    if (pin == BATTERY_ADC_PIN) {
        set_reason(reason_out, "battery");
        return true;
    }

    // IMU interrupt (input-only GPIO)
    if (pin == IMU_INT_PIN) {
        set_reason(reason_out, "imu_int");
        return true;
    }

    // Input-only ADC pins — cannot be driven as output
    if (pin == 36 || pin == 37 || pin == 38 || pin == 39) {
        set_reason(reason_out, "adc_only");
        return true;
    }

    return false;
}
