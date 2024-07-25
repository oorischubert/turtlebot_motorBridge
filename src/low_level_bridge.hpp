#ifndef LOW_LEVEL_BRIDGE_HPP
#define LOW_LEVEL_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <array>
#include <string>

// ROS2 Hardware Interface (for ros2_control)
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "visibility_control.h"

namespace turtlebot_namespace 
{
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

using hardware_interface::return_type;

class MotorBridge : public hardware_interface::SystemInterface {

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MotorBridge)

    TURTLEBOT_MOTORBRIDGE_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    TURTLEBOT_MOTORBRIDGE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    TURTLEBOT_MOTORBRIDGE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    TURTLEBOT_MOTORBRIDGE_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    TURTLEBOT_MOTORBRIDGE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    TURTLEBOT_MOTORBRIDGE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    TURTLEBOT_MOTORBRIDGE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    MotorBridge();
    ~MotorBridge();
    //void spin(); //deprecated??
    void motorCallback();
    void serialThread();
    void sendSerialData(const SerialMessage_t& msg);
    void processReceivedData(const std::vector<uint8_t>& buffer);
    bool enqueueMessage(const SerialMessage_t& msg);

private:

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    boost::thread serial_thread_;
    std::string serial_port_;

    Vehicle turtlebot;

    boost::lockfree::queue<SerialMessage_t> message_queue_{QUEUE_CAPACITY};
    
};
} // namespace turtlebot_namespace

#endif // LOW_LEVEL_BRIDGE_HPP
