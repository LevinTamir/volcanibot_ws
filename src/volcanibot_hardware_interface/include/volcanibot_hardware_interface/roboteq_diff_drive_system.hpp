// Copyright 2021 ros2_control Development Team
// Adapted from github.com/konu-droid/diffdrive_ros2_control (Apache-2.0).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef VOLCANIBOT_HARDWARE_INTERFACE__ROBOTEQ_DIFF_DRIVE_SYSTEM_HPP_
#define VOLCANIBOT_HARDWARE_INTERFACE__ROBOTEQ_DIFF_DRIVE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "volcanibot_hardware_interface/roboteq_comms.hpp"
#include "volcanibot_hardware_interface/visibility_control.h"

namespace volcanibot_hardware_interface
{

class RoboteqDiffDriveSystem : public hardware_interface::SystemInterface
{
public:
  struct Config
  {
    std::string port = "/dev/roboteq";
    int baud_rate = 115200;
    int timeout_ms = 1000;
    double gear_ratio = 1.0;       // motor : wheel
    std::string left_wheel_name = "left_wheel_joint";
    std::string right_wheel_name = "right_wheel_joint";
  };

  RCLCPP_SHARED_PTR_DEFINITIONS(RoboteqDiffDriveSystem)

  VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  RoboteqComm comm_;
  Config cfg_;

  // Velocity commands from the controller (wheel rad/s).
  double left_cmd_vel_ = 0.0;
  double right_cmd_vel_ = 0.0;

  // State (wheel rad and rad/s).
  double left_position_ = 0.0;
  double right_position_ = 0.0;
  double left_velocity_ = 0.0;
  double right_velocity_ = 0.0;
};

}  // namespace volcanibot_hardware_interface

#endif  // VOLCANIBOT_HARDWARE_INTERFACE__ROBOTEQ_DIFF_DRIVE_SYSTEM_HPP_
