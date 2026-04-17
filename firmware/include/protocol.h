#pragma once

// Host -> Firmware commands
constexpr const char* MSG_CMD_MOVE      = "cmd_move";
constexpr const char* MSG_CMD_STAND     = "cmd_stand";
constexpr const char* MSG_CMD_BALANCE   = "cmd_balance";
constexpr const char* MSG_CMD_LED       = "cmd_led";
constexpr const char* MSG_CMD_BUZZER    = "cmd_buzzer";
constexpr const char* MSG_CMD_SERVO     = "cmd_servo";
constexpr const char* MSG_CMD_ENGAGE    = "cmd_engage";
constexpr const char* MSG_CMD_I2C_WRITE = "cmd_i2c_write";   // deprecated: use cmd_i2c with op="write"
constexpr const char* MSG_CMD_I2C       = "cmd_i2c";          // unified I2C scan/read/write
constexpr const char* MSG_TELEM_I2C     = "telem_i2c";
constexpr const char* MSG_CMD_OFFSET    = "cmd_offset";
constexpr const char* MSG_CMD_TRANSFORM   = "cmd_transform";
constexpr const char* MSG_CMD_GAIT_PARAMS    = "cmd_gait_params";
constexpr const char* MSG_CMD_BALANCE_CONFIG = "cmd_balance_config";
constexpr const char* MSG_PING          = "ping";
constexpr const char* MSG_CMD_OTA_UPDATE    = "cmd_ota_update";
constexpr const char* MSG_CMD_PROBE_PIN  = "cmd_probe_pin";
constexpr const char* MSG_CMD_GPIO       = "cmd_gpio";
constexpr const char* MSG_TELEM_GPIO     = "telem_gpio";
constexpr const char* MSG_OTA_STATUS     = "ota_status";

// Firmware -> Host telemetry
constexpr const char* MSG_TELEM_IMU     = "telem_imu";
constexpr const char* MSG_TELEM_SONAR   = "telem_sonar";
constexpr const char* MSG_TELEM_BATTERY = "telem_battery";
constexpr const char* MSG_TELEM_STATUS  = "telem_status";
constexpr const char* MSG_TELEM_EVENT   = "telem_event";  // edge-triggered: engage, disengage, heartbeat, battery
constexpr const char* MSG_TELEM_BUTTON  = "telem_button";
constexpr const char* MSG_ACK           = "ack";
constexpr const char* MSG_PONG          = "pong";

// Protocol constants
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr size_t MAX_MESSAGE_SIZE   = 512;

// Movement directions
enum class Direction {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
};

Direction direction_from_string(const char* str);    // defined in command_handlers.cpp
