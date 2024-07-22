#ifndef DIFFDRIVE_TURTLEBOT_ESP_COMMS_H
#define DIFFDRIVE_TURTLEBOT_ESP_COMMS_H

#include <cstring>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#define SIZE_OF_TX_DATA 52;
#define SIZE_OF_RX_DATA 52;
typedef enum {
    CMD_POS = 0,
    CMD_VEL = 1,
    ODOMETRY = 2,
    PID = 3,
    RESET_ENCODERS = 4,
} MessageType_e;

typedef struct {
    MessageType_e type;
    std::array<uint8_t, SIZE_OF_TX_DATA> data;
} SerialMessage_t;
class EspComms
{
public:

  EspComms()
  {  }

  EspComms(const std::string &serial_device, int32_t baud_rate)
    : serial_(io_), serial_port_(serial_device, baud_rate)
    {
      setup(serial_device, baud_rate);
    }

  void setup(const std::string &serial_device, int32_t baud_rate);
  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);
  void clearEncoderValues();
  void sendSerialData(const SerialMessage_t& msg)
  void processReceivedData(const std::vector<uint8_t>& buffer, &Wheel leftWheel, &Wheel rightWheel);

private:
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
};

#endif // DIFFDRIVE_TURTLEBOT_ESP_COMMS_H