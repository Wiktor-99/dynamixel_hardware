// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)

namespace
{
constexpr const char * NAME_OF_HARDWARE_INTERFACE = "Dynamixel Hardware";
constexpr const char * GOAL_POSITION = "Goal_Position";
constexpr const char * GOAL_VELOCITY = "Goal_Velocity";
constexpr const char * MOVING_SPEED = "Moving_Speed";
constexpr const char * PRESENT_POSITION = "Present_Position";
constexpr const char * PRESENT_VELOCITY = "Present_Velocity";
constexpr const char * PRESENT_CURRENT = "Present_Current";
constexpr const char * PRESENT_SPEED = "Present_Speed";
constexpr const char * PRESENT_LOAD = "Present_Load";
}  // namespace

namespace dynamixel_hardware
{

hardware_interface::CallbackReturn DynamixelHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  set_joints_info();

  if (is_stub_used() or set_up_all_dynamixels_components()) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  return hardware_interface::CallbackReturn::ERROR;
}

bool DynamixelHardware::set_up_all_dynamixels_components()
{
  if (
    init_dynamixel_workbench() and load_dynamixels() and disable_torque() and init_dynamixels() and
    set_position_control_mode() and enable_torque() and init_control_items() and
    add_sdk_handler()) {
    return true;
  }
  return false;
}

void DynamixelHardware::set_joints_info()
{
  for (const auto & joint : info_.joints) {

    if (joint.parameters.find("id_of_joint_to_mimic") != joint.parameters.cend()) {
      joints_info_.back().id_of_joint_to_mimic = std::stoi(joint.parameters.at("id_of_joint_to_mimic"));
      RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Joint %s is mimic", joint.name.c_str());
    }
    else {
      joints_info_.push_back(Joint(std::stoi(joint.parameters.at("id"))));
      RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "joint_id %d ", joints_info_.back().id);
    }

    if (joint.parameters.find("position_multiplier") != joint.parameters.cend()) {
      joints_info_.back().position_multiplier = std::stod(joint.parameters.at("position_multiplier"));
    }
  }
}

bool DynamixelHardware::is_stub_used()
{
  if (
    info_.hardware_parameters.find("use_stub") != info_.hardware_parameters.cend() and
    info_.hardware_parameters.at("use_stub") == "true") {
    use_stub_ = true;
    RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "stub is used");
  }
  return use_stub_;
}

bool DynamixelHardware::init_dynamixel_workbench()
{
  const std::string usb_port = info_.hardware_parameters.at("usb_port");
  const int baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log{};

  RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "baud_rate: %d", baud_rate);
  if (not dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
    return false;
  }
  return true;
}

bool DynamixelHardware::load_dynamixels()
{
  const char * log{};
  for (const auto & joint : joints_info_) {
    uint16_t model_number = 0;

    if (not joint.is_mimic() and not dynamixel_workbench_.ping(joint.id, &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
      return false;
    }
  }
  return true;
}

bool DynamixelHardware::init_control_items()
{
  if (
    set_control_item(GOAL_POSITION) and set_control_item(GOAL_VELOCITY, MOVING_SPEED) and
    set_control_item(PRESENT_POSITION) and set_control_item(PRESENT_VELOCITY, PRESENT_SPEED) and
    set_control_item(PRESENT_CURRENT, PRESENT_LOAD)) {
    return true;
  }
  return false;
}

bool DynamixelHardware::set_control_item(const char * control_item_name)
{
  const uint8_t first_joint_id = joints_info_[0].id;
  const ControlItem * control_item =
    dynamixel_workbench_.getItemInfo(first_joint_id, control_item_name);
  if (not control_item) {
    return false;
  }
  control_items_[control_item_name] = control_item;
  return true;
}

bool DynamixelHardware::set_control_item(
  const char * control_item_name, const char * secondary_control_item_name)
{
  const uint8_t first_joint_id = joints_info_[0].id;
  const ControlItem * control_item =
    dynamixel_workbench_.getItemInfo(first_joint_id, control_item_name);
  if (not control_item) {
    control_item = dynamixel_workbench_.getItemInfo(first_joint_id, secondary_control_item_name);
  }
  if (not control_item) {
    return false;
  }
  control_items_[control_item_name] = control_item;
  return true;
}

bool DynamixelHardware::add_sdk_handler()
{
  const char * log{};
  if (not dynamixel_workbench_.addSyncWriteHandler(
        control_items_[GOAL_POSITION]->address, control_items_[GOAL_POSITION]->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
    return false;
  };

  if (not dynamixel_workbench_.addSyncWriteHandler(
        control_items_[GOAL_VELOCITY]->address, control_items_[GOAL_VELOCITY]->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
    return false;
  }
  return true;
}

bool DynamixelHardware::enable_torque()
{
  const char * log{};
  for (const auto & joint : joints_info_) {
    if (not joint.is_mimic() and not dynamixel_workbench_.torqueOn(joint.id, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
      return false;
    }
  }
  return true;
}

bool DynamixelHardware::disable_torque()
{
  const char * log{};
  for (const auto & joint : joints_info_) {
    if (not joint.is_mimic() and not dynamixel_workbench_.torqueOff(joint.id, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
      return false;
    }
  }
  return true;
}

bool DynamixelHardware::init_dynamixels()
{
  const char * log{};
  for (const auto & joint : info_.joints) {
    const uint8_t id = std::stoi(joint.parameters.at("id"));
    for (const auto & [name, value] : joint.parameters) {
      if (name != "id" and name != "position_multiplier" and joint.parameters.find("id_of_joint_to_mimic") == joint.parameters.cend()) {
        if (not dynamixel_workbench_.itemWrite(id, name.c_str(), std::stoi(value), &log)) {
          return false;
        }
        RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Write item with success %s %d", name.c_str(), std::stoi(value));
      }
    }
  }
  return true;
}

bool DynamixelHardware::set_position_control_mode()
{
  const char * log{};
  if (control_mode_ != ControlMode::position) {
    for (const auto & joint : joints_info_) {
      if (not joint.is_mimic() and not dynamixel_workbench_.setPositionControlMode(joint.id, &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
        return false;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Position control");
    control_mode_ = ControlMode::position;
  }
  return true;
}
bool DynamixelHardware::set_velocity_control_mode()
{
  const char * log{};
  if (control_mode_ != ControlMode::velocity) {
    for (const auto & joint : joints_info_) {
      if (not joint.is_mimic() and not dynamixel_workbench_.setVelocityControlMode(joint.id, &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
        return false;
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Velocity control");
  return true;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_info_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_info_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_info_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_info_[i].command.position));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "start");
  set_all_states_when_stub_is_used();

  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  log_current_joint_position();
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));
  return hardware_interface::CallbackReturn::SUCCESS;
}
void DynamixelHardware::log_current_joint_position()
{
  for (auto & joint : joints_info_) {
    RCLCPP_INFO(
      rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "joint id %d, position %f", joint.id,
      joint.state.position);
  }
}

void DynamixelHardware::set_all_states_when_stub_is_used()
{
  for (auto & joint : joints_info_) {
    if (use_stub_ and std::isnan(joint.state.position)) {
      joint.state = {0.0, 0.0, 0.0};
    }
  }
}

hardware_interface::CallbackReturn DynamixelHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "stop");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (use_stub_) {
    return hardware_interface::return_type::OK;
  }
  const uint16_t data_length = control_items_data_length();
  std::vector<uint32_t> position_velocity_current(data_length);

  for (auto & joint : joints_info_) {
    if (not joint.is_mimic() and not read_current_states(joint.id, data_length, position_velocity_current)) {
      return hardware_interface::return_type::ERROR;
    }
    if (not joint.is_mimic()) {
      joint.state = convert_joint_values(joint.id, position_velocity_current);
      joint.state.position /= joint.position_multiplier;
    }
    else {
      joint.state = joints_info_[joint.id_of_joint_to_mimic].state;
    }
  }
  return hardware_interface::return_type::OK;
}

JointValues DynamixelHardware::convert_joint_values(
  uint8_t id, const std::vector<uint32_t> & position_velocity_current)
{
  JointValues joint_values;
  joint_values.effort = dynamixel_workbench_.convertValue2Current(
    id, DXL_MAKEWORD(position_velocity_current[4], position_velocity_current[5]));
  joint_values.velocity = dynamixel_workbench_.convertValue2Velocity(
    id, DXL_MAKEWORD(position_velocity_current[2], position_velocity_current[3]));
  joint_values.position = dynamixel_workbench_.convertValue2Radian(
    id, DXL_MAKEWORD(position_velocity_current[0], position_velocity_current[1]));
  RCLCPP_DEBUG(
    rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "joint id %d, position %f", id,
    joint_values.position);
  return joint_values;
}

uint16_t DynamixelHardware::control_items_data_length()
{
  return control_items_[PRESENT_POSITION]->data_length +
         control_items_[PRESENT_VELOCITY]->data_length +
         control_items_[PRESENT_CURRENT]->data_length;
}

bool DynamixelHardware::read_current_states(
  uint8_t id, uint16_t data_length, std::vector<uint32_t> & position_velocity_current)
{
  const char * log{};
  if (dynamixel_workbench_.readRegister(
        id, control_items_[PRESENT_POSITION]->address, data_length,
        position_velocity_current.data(), &log)) {
    return true;
  }
  RCLCPP_ERROR(
    rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Read current values failed! %s", log);

  return false;
}

hardware_interface::return_type DynamixelHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (use_stub_) {
    set_command_to_position();
    return hardware_interface::return_type::OK;
  }
  std::vector<uint8_t> ids{get_ids()};

  if (std::any_of(joints_info_.cbegin(), joints_info_.cend(), [](const auto & joint_info) {
        return joint_info.command.position != 0.0;
      })) {
    if (not set_position_control_mode()) {
      return hardware_interface::return_type::ERROR;
    }
    return write_position_commands(ids, get_position_commands());
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "Only position control is implemented");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

std::vector<uint8_t> DynamixelHardware::get_ids() const
{
  std::vector<uint8_t> ids;
  for (const auto& joint : joints_info_) {
    if (not joint.is_mimic()) {
        ids.push_back(joint.id);
    }
  }
  return ids;
}

std::vector<int32_t> DynamixelHardware::get_velocity_commands()
{
  std::vector<int32_t> commands;
  for (const auto & joint_info : joints_info_) {
    commands.push_back(dynamixel_workbench_.convertVelocity2Value(
      joint_info.id, static_cast<double>(joint_info.command.velocity)));
  }
  return commands;
}
std::vector<int32_t> DynamixelHardware::get_position_commands()
{
  std::vector<int32_t> commands;
  for (const auto & joint_info : joints_info_) {
    if (not joint_info.is_mimic() ) {
    commands.push_back(dynamixel_workbench_.convertRadian2Value(
      joint_info.id, static_cast<double>(joint_info.command.position * joint_info.position_multiplier)));
    }
  }
  return commands;
}

hardware_interface::return_type DynamixelHardware::write_velocity_commands(
  std::vector<uint8_t> ids, std::vector<int32_t> commands)
{
  constexpr uint8_t GOAL_VELOCITYIndex = 1;
  const char * log{};
  if (not dynamixel_workbench_.syncWrite(
        GOAL_VELOCITYIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::write_position_commands(
  std::vector<uint8_t> ids, std::vector<int32_t> commands)
{
  constexpr uint8_t GOAL_POSITIONIndex = 0;
  const char * log{};
  if (not dynamixel_workbench_.syncWrite(
        GOAL_POSITIONIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(NAME_OF_HARDWARE_INTERFACE), "%s", log);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}
void DynamixelHardware::set_command_to_position()
{
  for (auto & joint : joints_info_) {
    joint.state.position = joint.command.position;
  }
}
void DynamixelHardware::reset_command()
{
  for (auto & joint_info : joints_info_) {
    joint_info.command.position = joint_info.state.position;
    joint_info.command.velocity = 0.0;
    joint_info.command.effort = 0.0;
  }
}

}  // namespace dynamixel_hardware