// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_demo_example_2/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_2
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
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
      espComms.init(turtlebot);
      RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Successfully connected to USB port: %s", turtlebot.device.c_str());
  } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Failed to open serial port: %s", e.what());
      throw;
  } 

    espComms.initPid(turtlebot,VEL_KP,VEL_KI,VEL_KD,VEL_I_WINDUP); //set PID values

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

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
   // position and a velocity interfaces:
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(turtlebot.leftWheel.wheelName, hardware_interface::HW_IF_VELOCITY, &turtlebot.leftWheel.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(turtlebot.leftWheel.wheelName, hardware_interface::HW_IF_POSITION, &turtlebot.leftWheel.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(turtlebot.rightWheel.wheelName, hardware_interface::HW_IF_VELOCITY, &turtlebot.rightWheel.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(turtlebot.rightWheel.wheelName, hardware_interface::HW_IF_POSITION, &turtlebot.rightWheel.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  //velocity command interfaces:
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(turtlebot.leftWheel.wheelName, hardware_interface::HW_IF_VELOCITY, &turtlebot.leftWheel.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(turtlebot.rightWheel.wheelName, hardware_interface::HW_IF_VELOCITY, &turtlebot.rightWheel.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Starting Controller...");
  
  espComms.clearEncoders(); //clear encoder count
  //espComms.setPid(turtlebot); //set PID values

  RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
   RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Stopping Controller...");
    espComms.stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    espComms.motorReadCallBack(turtlebot);

    turtlebot.leftWheel.pos = turtlebot.leftWheel.enc * ((2*M_PI)/turtlebot.enc_counts_per_rev);
    turtlebot.rightWheel.pos = turtlebot.rightWheel.enc * ((2*M_PI)/turtlebot.enc_counts_per_rev);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_2 ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  espComms.motorWriteCallBack(turtlebot);
   
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)
