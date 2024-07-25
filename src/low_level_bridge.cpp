#include "low_level_bridge.hpp"
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace turtlebot_namespace
{
void packFloatIntoArray(std::array<uint8_t, SIZE_OF_TX_DATA>& data, float value, size_t offset) {
    if (offset + sizeof(float) <= data.size()) {
        std::memcpy(&data[offset], &value, sizeof(float));
    }
}

hardware_interface::CallbackReturn MotorBridge::on_init(const hardware_interface::HardwareInfo & info) {
    //Node(NODE_NAME) { //, serial_(io_), serial_port_(declare_parameter<std::string>("device", SERIAL_PORT)), baud_rate_(BAUD_RATE) {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    

    RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Configuring...");
    turtlebot.leftWheel.wheelName = info_.hardware_parameters["left_wheel_name"];
    turtlebot.rightWheel.wheelName = info_.hardware_parameters["right_wheel_name"];
    turtlebot.device = info_.hardware_parameters["device"];
    turtlebot.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    turtlebot.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    turtlebot.timeout = std::stoi(info_.hardware_parameters["timeout"]);
    turtlebot.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    // Setup serial port
    try {
    serial_.open(turtlebot.device);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(turtlebot.baud_rate));
    RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Successfully connected to USB port: %s", turtlebot.device.c_str());
    } catch (const boost::system::system_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Failed to open serial port: %s", e.what());
    throw;
    }
    // Start the serial communication thread
    serial_thread_ = boost::thread(boost::bind(&MotorBridge::serialThread, this));
    // Send startup message

    //Error fishing:
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // DiffBotSystem has exactly two states and one command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger(NODE_NAME),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger(NODE_NAME),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger(NODE_NAME),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger(NODE_NAME),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger(NODE_NAME),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
        }
  }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotorBridge::export_state_interfaces()
{
  // position and a velocity interfaces:
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(turtlebot.leftWheel.wheelName, hardware_interface::HW_IF_VELOCITY, &turtlebot.leftWheel.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(turtlebot.leftWheel.wheelName, hardware_interface::HW_IF_POSITION, &turtlebot.leftWheel.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(turtlebot.rightWheel.wheelName, hardware_interface::HW_IF_VELOCITY, &turtlebot.rightWheel.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(turtlebot.rightWheel.wheelName, hardware_interface::HW_IF_POSITION, &turtlebot.rightWheel.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorBridge::export_command_interfaces()
{
  //velocity command interfaces:
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(turtlebot.leftWheel.wheelName, hardware_interface::HW_IF_VELOCITY, &turtlebot.leftWheel.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(turtlebot.rightWheel.wheelName, hardware_interface::HW_IF_VELOCITY, &turtlebot.rightWheel.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn MotorBridge::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Starting Controller...");

  //send pid values! not necessary atm but do it!

  RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorBridge::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Stopping Controller...");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotorBridge::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
    // Read the current state of the robot
    //Not necesary, the serial thread does this!
    motorCallback();
    return return_type::OK;
}

hardware_interface::return_type MotorBridge::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

    return return_type::OK;
}

MotorBridge::~MotorBridge() {
    io_.stop();
    serial_thread_.join();
    serial_.close();
}

bool MotorBridge::enqueueMessage(const SerialMessage_t& msg) {
    if (!message_queue_.push(msg)) {
        // Queue is full, pop one item to make space
       
        SerialMessage_t discarded_msg;

        if (!message_queue_.pop(discarded_msg)) {
            RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Failed to pop from the queue even though it's reported as full.");
            return false;
        }

        // Try again to push the new message
        if (!message_queue_.push(msg)) {
            RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Failed to enqueue message after making space.");
            return false;
        }
    }
    return true;
}

void MotorBridge::motorCallback() {
    SerialMessage_t serial_msg;
    serial_msg.type = VEL_CMD;

    // Pack position
    packFloatIntoArray(serial_msg.data, turtlebot.leftWheel.cmd / ((2*M_PI)/turtlebot.enc_counts_per_rev) / turtlebot.loop_rate, 0);
    packFloatIntoArray(serial_msg.data, turtlebot.rightWheel.cmd / ((2*M_PI)/turtlebot.enc_counts_per_rev) / turtlebot.loop_rate, 4);
    
    if (!enqueueMessage(serial_msg)) {
        RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Unable to enqueue message.");
    }
}

void MotorBridge::serialThread() {
    std::vector<uint8_t> read_buffer(SIZE_OF_RX_DATA, 0);

    while (rclcpp::ok()) {
        // Try to send one outgoing data message if available
        SerialMessage_t msg;
        if (message_queue_.pop(msg)) {
            sendSerialData(msg);
        }

        // Handle incoming data
        boost::system::error_code ec;
        size_t len = boost::asio::read(serial_, boost::asio::buffer(read_buffer), boost::asio::transfer_at_least(1), ec);
        
        if (!ec && len > 0) {
            processReceivedData(read_buffer);
        } else if (ec) {
            RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Error reading from serial: %s", ec.message().c_str());
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(10)); // Small sleep to prevent hogging CPU
    }
}


void MotorBridge::sendSerialData(const SerialMessage_t& msg) {
    std::array<uint8_t, SIZE_OF_TX_DATA> buffer;

    // Set headers
    buffer[0] = HEADER;
    buffer[1] = HEADER;

    // Set message type indicator
    buffer[2] = static_cast<uint8_t>(msg.type);

    // Copy data from SerialMessage into the buffer
    std::memcpy(&buffer[3], msg.data.data(), SIZE_OF_TX_DATA - 5); // SIZE_OF_TX_DATA - 5 to account for header, type, checksum, and tail

    // Compute checksum
    uint8_t checksum = 0;
    for (size_t i = 3; i < SIZE_OF_TX_DATA - 2; ++i) {
        checksum += buffer[i];
    }

    // Append checksum and tail
    buffer[SIZE_OF_TX_DATA - 2] = checksum;
    buffer[SIZE_OF_TX_DATA - 1] = TAIL;

    // Send data over serial port using boost::asio
    boost::asio::write(serial_, boost::asio::buffer(buffer, SIZE_OF_TX_DATA));
}

void MotorBridge::processReceivedData(const std::vector<uint8_t>& buffer) {
    if (buffer[0] == HEADER && buffer[1] == HEADER && buffer[SIZE_OF_RX_DATA - 1] == TAIL) {
        uint8_t checksum = 0;
        for (size_t i = 2; i < SIZE_OF_RX_DATA - 2; i++) {
            checksum += buffer[i];
        }
        uint8_t message_checksum = buffer[SIZE_OF_RX_DATA - 2];
        if (checksum == message_checksum) {

            memcpy(&turtlebot.leftWheel.vel, &buffer[2], 4);
            memcpy(&turtlebot.rightWheel.vel, &buffer[6], 4);
            memcpy(&turtlebot.leftWheel.enc, &buffer[10], 8);
            memcpy(&turtlebot.rightWheel.enc, &buffer[8], 8);

        } else {
            RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Checksum mismatch in received serial data.");
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Header or Tail mismatch in received serial data.");
    }
    turtlebot.leftWheel.pos = turtlebot.leftWheel.enc * ((2*M_PI)/turtlebot.enc_counts_per_rev);
    turtlebot.rightWheel.pos = turtlebot.rightWheel.enc * ((2*M_PI)/turtlebot.enc_counts_per_rev);
}
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  turtlebot_namespace::MotorBridge, hardware_interface::SystemInterface)

// void MotorBridge::spin() {
//     rclcpp::Rate rate(10); // 10 Hz
//     while (rclcpp::ok()) {
//         rclcpp::spin_some(this->get_node_base_interface());
//         rate.sleep();
//     }
// }

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto processor = std::make_shared<MotorBridge>();
//     processor->spin();
//     rclcpp::shutdown();
//     return 0;
// }
