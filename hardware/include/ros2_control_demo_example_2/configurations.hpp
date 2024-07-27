#ifndef CONFIGURATIONS_HPP
#define CONFIGURATIONS_HPP

#include <array>
#include <string>

// Serial communication configs
#define SERIAL_PORT "/dev/ttyUSB_esp32" //added to launch file now
#define BAUD_RATE 115200
#define HEADER 0xC8
#define TAIL 0xC7
#define SIZE_OF_RX_DATA 52
#define SIZE_OF_TX_DATA 52
#define QUEUE_CAPACITY 100

// ROS configs
#define NODE_NAME "turtlebot_motorBridge"
#define PARENT_FRAME_ID "base_link"

typedef enum {
    VEL_CMD = 1,
    PID_CMD = 3,
    RESET_ENCODERS = 4,
} MessageType_e;

typedef struct {
    MessageType_e type;
    std::array<uint8_t, SIZE_OF_TX_DATA> data;
} SerialMessage_t;

typedef struct {
    std::string wheelName = "";
    int64_t enc = 0;
    double cmd = 0;
    double vel = 0;
    double pos = 0;
} Motor;

typedef struct {
    Motor leftWheel;
    Motor rightWheel;
    int baud_rate = BAUD_RATE;
    int enc_counts_per_rev = 660;
    int timeout = 1000;
    double rads_per_count = 0;
    float loop_rate = 30;
    std::string device = SERIAL_PORT;
} Vehicle;

#endif // CONFIGURATIONS_HPP