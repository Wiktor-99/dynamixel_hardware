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

#pragma once

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

#include "dynamixel_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

using hardware_interface::return_type;

namespace dynamixel_hardware
{
struct JointValues
{
  constexpr static auto quiet_NaN = std::numeric_limits<double>::quiet_NaN();
  double position{quiet_NaN};
  double velocity{quiet_NaN};
  double effort{quiet_NaN};
};

struct Joint
{
  Joint(int id) : id{id} {}
  int id{-1};
  double position_multiplier{1};
  int id_of_joint_to_mimic{-1};
  JointValues state{};
  JointValues command{};

  bool is_mimic() const {
    return id_of_joint_to_mimic != -1;
  }
};

enum class ControlMode { none, position, velocity };

class DynamixelHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  void set_joints_info();
  bool is_stub_used();
  bool init_dynamixel_workbench();
  bool load_dynamixels();
  bool init_dynamixels();
  bool init_control_items();
  bool add_sdk_handler();
  bool enable_torque();
  bool disable_torque();
  bool set_position_control_mode();
  bool set_velocity_control_mode();
  void reset_command();
  bool set_up_all_dynamixels_components();
  void set_all_states_when_stub_is_used();
  uint16_t control_items_data_length();
  bool read_current_states(
    uint8_t id, uint16_t data_length, std::vector<uint32_t> & position_velocity_current);
  JointValues convert_joint_values(
    uint8_t id, const std::vector<uint32_t> & position_velocity_current);
  std::vector<uint8_t> get_ids() const;
  std::vector<int32_t> get_velocity_commands();
  hardware_interface::return_type write_velocity_commands(
    std::vector<uint8_t> ids, std::vector<int32_t> commands);
  std::vector<int32_t> get_position_commands();
  hardware_interface::return_type write_position_commands(
    std::vector<uint8_t> ids, std::vector<int32_t> commands);
  void set_command_to_position();
  bool set_control_item(const char * control_item_name);
  bool set_control_item(const char * control_item_name, const char * secondary_control_item_name);
  void log_current_joint_position();

  ControlMode control_mode_{};
  std::vector<Joint> joints_info_;
  bool use_stub_{};
  bool torque_enabled_{};
  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
};

}  // namespace dynamixel_hardware
