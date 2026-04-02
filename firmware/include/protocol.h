#pragma once

// Message types — must match docs/protocol.md and host/comms.py

// Host → Firmware commands
constexpr const char* MSG_CMD_MOVE     = "cmd_move";
constexpr const char* MSG_CMD_STAND    = "cmd_stand";
constexpr const char* MSG_CMD_SET_GAIT = "cmd_set_gait";
constexpr const char* MSG_CMD_PATROL   = "cmd_patrol";
constexpr const char* MSG_CMD_BALANCE  = "cmd_balance";
constexpr const char* MSG_PING         = "ping";

// Firmware → Host telemetry & events
constexpr const char* MSG_TELEM_IMU    = "telem_imu";
constexpr const char* MSG_TELEM_STATUS = "telem_status";
constexpr const char* MSG_ACK          = "ack";
constexpr const char* MSG_EVENT_FALL   = "event_fall";
constexpr const char* MSG_EVENT_RECOVERED = "event_recovered";
constexpr const char* MSG_PONG         = "pong";

// Protocol constants
constexpr unsigned long SERIAL_BAUD        = 115200;
constexpr unsigned long HEARTBEAT_TIMEOUT_MS = 5000;
constexpr unsigned long TELEM_IMU_INTERVAL_MS = 50;   // 20 Hz
constexpr unsigned long TELEM_STATUS_INTERVAL_MS = 1000; // 1 Hz
constexpr size_t        MAX_MESSAGE_SIZE   = 512;

// Movement directions
enum class Direction {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
};

Direction direction_from_string(const char* str);
const char* direction_to_string(Direction dir);
