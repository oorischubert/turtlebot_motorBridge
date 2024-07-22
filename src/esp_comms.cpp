#include "turtlebot_motorBridge/esp_comms.h"
// #include <ros/console.h> // For ROS_INFO
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>



void EspComms::setup(const std::string &serial_device, int32_t baud_rate)
{  

    try {
    serial_.open(serial_device);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    } catch (const boost::system::system_error& e) {
    //error handling??
    throw;
    }
  
    
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}

void EspComms::setMotorValues(int val_1, int val_2)
{
    SerialMessage_t msg;
    msg.type = CMD_VEL;
    packFloatIntoArray(msg.data, val_1, 0);
    packFloatIntoArray(msg.data, val_2, 4);
    sendSerialData(msg);
}

void EspComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    SerialMessage_t msg;
    msg.type = PID;
    packFloatIntoArray(msg.data, k_p, 0);
    packFloatIntoArray(msg.data, k_d, 4);
    packFloatIntoArray(msg.data, k_i, 8);
    packFloatIntoArray(msg.data, k_o, 12);
    sendSerialData(msg);
}

void EspComms::clearEncoderValues()
{
    SerialMessage_t msg;
    msg.type = RESET_ENCODERS;
    sendSerialData(msg);
}

void EspComms::sendSerialData(const SerialMessage_t& msg) {
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

void EspComms::processReceivedData(const std::vector<uint8_t>& buffer, &Wheel leftWheel, &Wheel rightWheel) {
    if (buffer[0] == HEADER && buffer[1] == HEADER && buffer[SIZE_OF_RX_DATA - 1] == TAIL) {
        uint8_t checksum = 0;
        for (size_t i = 2; i < SIZE_OF_RX_DATA - 2; i++) {
            checksum += buffer[i];
        }
        uint8_t message_checksum = buffer[SIZE_OF_RX_DATA - 2];
        if (checksum == message_checksum) {

            memcpy(&leftWheel.vel, &buffer[2], 4); // lw_vel
            memcpy(&rightWheel.vel, &buffer[6], 4); // rw_vel
            memcpy(&leftWheel.enc, &buffer[10], 8); // lw_encoder_count <><><>(CHECK INT64 TO DOUBLE CONVERSION)<><><>
            memcpy(&rightWheel.enc, &buffer[18], 8); // rw_encoder_count <><><>(CHECK INT64 TO DOUBLE CONVERSION)<><><>

        } else {
            //RCLCPP_WARN(this->get_logger(), "Checksum mismatch in received serial data.");
        }
    } else {
        //RCLCPP_WARN(this->get_logger(), "Header or Tail mismatch in received serial data.");
    }
}
