
#ifndef ESP_COMMS_HPP
#define ESP_COMMS_HPP

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <array>
#include <string>

#include "configurations.hpp"

class EspComms 
{
    public:
        EspComms() : serial_(io_) {}

        void init(Vehicle& turtlebot) {
        serial_.open(turtlebot.device);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(turtlebot.baud_rate));
        // Optionally start the serial communication thread
    }

        void stop() {
        io_.stop();
        serial_.close();
    }


        void packFloatIntoArray(std::array<uint8_t, SIZE_OF_TX_DATA>& data, float value, size_t offset) {
        if (offset + sizeof(float) <= data.size()) {
            std::memcpy(&data[offset], &value, sizeof(float));
        }
        }

        bool enqueueMessage(const SerialMessage_t& msg) {
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

        void motorWriteCallBack(Vehicle& turtlebot) {
            SerialMessage_t serial_msg;
            serial_msg.type = VEL_CMD;
            // Pack position

            packFloatIntoArray(serial_msg.data, turtlebot.leftWheel.cmd, 0);
            packFloatIntoArray(serial_msg.data, turtlebot.rightWheel.cmd, 4);
            
            if (!enqueueMessage(serial_msg)) {
                RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Unable to enqueue message.");
            }

             SerialMessage_t msg;
            if (message_queue_.pop(msg)) {
                sendSerialData(msg);
            }
        }

        void motorReadCallBack(Vehicle& turtlebot) {
            std::vector<uint8_t> read_buffer(SIZE_OF_RX_DATA, 0);
            // Handle incoming data
            boost::system::error_code ec;
            size_t len = boost::asio::read(serial_, boost::asio::buffer(read_buffer), boost::asio::transfer_at_least(1), ec);
            
            if (!ec && len > 0) {
                processReceivedData(read_buffer,turtlebot);
            } else if (ec) {
                RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Error reading from serial: %s", ec.message().c_str());
            }
        }

        void clearEncoders() {
            SerialMessage_t serial_msg;
            serial_msg.type = RESET_ENCODERS;

            if (!enqueueMessage(serial_msg)) {
                RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Unable to enqueue message.");
            }

             SerialMessage_t msg;
            if (message_queue_.pop(msg)) {
                sendSerialData(msg);
            }
        }

        void sendSerialData(const SerialMessage_t& msg) {
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

        void processReceivedData(const std::vector<uint8_t>& buffer, Vehicle& turtlebot) {
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
                    memcpy(&turtlebot.rightWheel.enc, &buffer[18], 8);

                } else {
                    RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Checksum mismatch in received serial data.");
                }
            } else {
                RCLCPP_WARN(rclcpp::get_logger(NODE_NAME), "Header or Tail mismatch in received serial data.");
            }
        }

    private:
        boost::asio::io_service io_;
        boost::asio::serial_port serial_;
        boost::lockfree::queue<SerialMessage_t> message_queue_{QUEUE_CAPACITY};
};

#endif // ESP_COMMS_HPP