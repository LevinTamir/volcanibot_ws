// Copyright 2021 ros2_control Development Team
// Adapted from github.com/konu-droid/diffdrive_ros2_control (Apache-2.0).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "volcanibot_hardware_interface/roboteq_diff_drive_system.hpp"

#include <cmath>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

double rad_per_sec_to_rpm(double rad_per_sec, double gear_ratio)
{
  return rad_per_sec * 60.0 / kTwoPi * gear_ratio;
}

std::string param_or(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback)
{
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}
}  // namespace

namespace volcanibot_hardware_interface
{

hardware_interface::CallbackReturn RoboteqDiffDriveSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.port = param_or(info_, "port", cfg_.port);
  cfg_.baud_rate = std::stoi(param_or(info_, "baud_rate", std::to_string(cfg_.baud_rate)));
  cfg_.timeout_ms = std::stoi(param_or(info_, "timeout_ms", std::to_string(cfg_.timeout_ms)));
  cfg_.gear_ratio = std::stod(param_or(info_, "gear_ratio", std::to_string(cfg_.gear_ratio)));
  cfg_.counts_per_rev =
    std::stod(param_or(info_, "counts_per_rev", std::to_string(cfg_.counts_per_rev)));
  cfg_.left_encoder_sign =
    std::stod(param_or(info_, "left_encoder_sign", std::to_string(cfg_.left_encoder_sign)));
  cfg_.right_encoder_sign =
    std::stod(param_or(info_, "right_encoder_sign", std::to_string(cfg_.right_encoder_sign)));
  cfg_.left_wheel_name = param_or(info_, "left_wheel_name", cfg_.left_wheel_name);
  cfg_.right_wheel_name = param_or(info_, "right_wheel_name", cfg_.right_wheel_name);

  if (cfg_.counts_per_rev <= 0.0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RoboteqDiffDriveSystem"),
      "counts_per_rev must be > 0, got %f", cfg_.counts_per_rev);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RoboteqDiffDriveSystem"),
      "Expected exactly 2 joints (left, right), got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate joint names against the configured left/right.
  bool found_left = false, found_right = false;
  for (const auto & joint : info_.joints)
  {
    if (joint.name == cfg_.left_wheel_name) found_left = true;
    if (joint.name == cfg_.right_wheel_name) found_right = true;

    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RoboteqDiffDriveSystem"),
        "Joint '%s' must expose exactly one velocity command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RoboteqDiffDriveSystem"),
        "Joint '%s' must expose [position, velocity] state interfaces in that order",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (!found_left || !found_right)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RoboteqDiffDriveSystem"),
      "Could not match URDF joints to left_wheel_name=%s right_wheel_name=%s",
      cfg_.left_wheel_name.c_str(), cfg_.right_wheel_name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RoboteqDiffDriveSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  ifaces.emplace_back(cfg_.left_wheel_name, hardware_interface::HW_IF_POSITION, &left_position_);
  ifaces.emplace_back(cfg_.left_wheel_name, hardware_interface::HW_IF_VELOCITY, &left_velocity_);
  ifaces.emplace_back(cfg_.right_wheel_name, hardware_interface::HW_IF_POSITION, &right_position_);
  ifaces.emplace_back(cfg_.right_wheel_name, hardware_interface::HW_IF_VELOCITY, &right_velocity_);
  return ifaces;
}

std::vector<hardware_interface::CommandInterface>
RoboteqDiffDriveSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  ifaces.emplace_back(cfg_.left_wheel_name, hardware_interface::HW_IF_VELOCITY, &left_cmd_vel_);
  ifaces.emplace_back(cfg_.right_wheel_name, hardware_interface::HW_IF_VELOCITY, &right_cmd_vel_);
  return ifaces;
}

hardware_interface::CallbackReturn RoboteqDiffDriveSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("RoboteqDiffDriveSystem"),
    "Opening Roboteq on %s @ %d baud (gear_ratio=%.4f, counts_per_rev=%.1f)",
    cfg_.port.c_str(), cfg_.baud_rate, cfg_.gear_ratio, cfg_.counts_per_rev);

  if (!comm_.connect(cfg_.port, cfg_.baud_rate, cfg_.timeout_ms))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboteqDiffDriveSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  comm_.disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboteqDiffDriveSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  left_cmd_vel_ = right_cmd_vel_ = 0.0;
  left_velocity_ = right_velocity_ = 0.0;
  // Don't reset position - keep odometry continuous across activate cycles.
  // Re-baseline the encoder counters on the next read so re-activation doesn't
  // produce a spurious position jump from the absolute counter.
  encoder_initialized_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboteqDiffDriveSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the motors before handing the bus to whatever activates next.
  if (comm_.connected()) comm_.drive(0.0, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoboteqDiffDriveSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comm_.connected()) return hardware_interface::return_type::ERROR;

  long left_count = 0, right_count = 0;
  if (!comm_.read_counts(left_count, right_count))
  {
    // Don't error out on a single bad read - keep last-known state so
    // controller_manager keeps cycling.
    return hardware_interface::return_type::OK;
  }

  // First read after activation: baseline the raw counters, report zero motion.
  if (!encoder_initialized_)
  {
    left_count_prev_ = left_count;
    right_count_prev_ = right_count;
    left_velocity_ = right_velocity_ = 0.0;
    encoder_initialized_ = true;
    return hardware_interface::return_type::OK;
  }

  const double rad_per_count = kTwoPi / cfg_.counts_per_rev;
  const double left_delta =
    cfg_.left_encoder_sign * static_cast<double>(left_count - left_count_prev_) * rad_per_count;
  const double right_delta =
    cfg_.right_encoder_sign * static_cast<double>(right_count - right_count_prev_) * rad_per_count;

  left_position_ += left_delta;
  right_position_ += right_delta;

  const double dt = period.seconds();
  if (dt > 0.0)
  {
    left_velocity_ = left_delta / dt;
    right_velocity_ = right_delta / dt;
  }

  left_count_prev_ = left_count;
  right_count_prev_ = right_count;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboteqDiffDriveSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comm_.connected()) return hardware_interface::return_type::ERROR;
  comm_.drive(
    rad_per_sec_to_rpm(left_cmd_vel_, cfg_.gear_ratio),
    rad_per_sec_to_rpm(right_cmd_vel_, cfg_.gear_ratio));
  return hardware_interface::return_type::OK;
}

}  // namespace volcanibot_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  volcanibot_hardware_interface::RoboteqDiffDriveSystem, hardware_interface::SystemInterface)
