# Package dynamixel_control

The [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation for any kind of [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/) robots.

- `dynamixel_hardware`: the [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) implementation for the multiple ROBOTIS Dynamixel servos.
- `open_manipulator_x_description`: the reference implementation of the `ros2_control` robot using [ROBOTIS OpenManipulator-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/).

The `dynamixel_hardware` package is hopefully compatible any configuration of ROBOTIS Dynamixel servos thanks to the `ros2_control`'s flexible architecture.


## This fork allows to use dynamixel_hardware package with dynamixels that use communication protocol version 1.0

At this moment fork supports only position control and only with ros-foxy.

## Set up

First [install ROS 2 Foxy on Ubuntu 20.04](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/). Then follow the instruction below.

```shell
$ source /opt/ros/foxy/setup.bash
$ mkdir -p ~/ros/foxy && cd ~/ros/foxy
$ git clone https://github.com/Wiktor-99/dynamixel_hardware.git src
$ vcs import src < src/dynamixel_control.repos
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
$ . install/setup.bash
```

# Configuration of dynamixel_control in robot description

To use dynamixel_control add following **ros2_control** block.

```xml
<ros2_control name="${name}" type="system">
  <hardware>
    <plugin>dynamixel_hardware/DynamixelHardware</plugin>
    <param name="usb_port">/dev/ttyUSB0</param>
    <param name="baud_rate">57600</param>
    <param name="joint_ids">1,2,3</param>
    <param name="use_stub">true</param> <!-- optional parameter -->
  </hardware>
  <joint name="joint_name">
  </joint>
</ros2_control>
```

It is necessary to set following parameters: **usb_port**, **baud_rate** and **joint_ids**. Use stub is optional parameter if used **commands will not forwarded to hardware**.

Also **joint** blocks inside of **ros2_control** should be used. Example of **joint** block:

```xml
  <joint name="id_1">
    <param name="id">1</param>
    <param name="position_multiplier">position_multiplier[double]</param>
    <param name="id_of_joint_to_mimic">id_of_existing_joint_to_mimic[int]</param>
    <param name="Return_Delay_Time">0</param>
    <param name="CW_Angle_Limit">0</param>
    <param name="CCW_Angle_Limit">1023</param>
    <param name="Moving_Speed">1023</param>
    <command_interface name="position" />
    <state_interface name="position" />
    <state_interface name="velocity" />
    <state_interface name="effort" />
  </joint>
```
## Remember to configure  ros2_control package with joint_trajectory controller!!

## Usage

When package is configured and you have launched both package with description of your robot and **ros2_control** you can move your robot using following command:
.
```shell
$ ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory -f "{
  trajectory: {
    joint_names: [name_of_first_joint, ..., name_of_last_joint],
    points: [
      { positions: [0.1, ..., 0], time_from_start: { sec: 2 } },
      { positions: [-0.1, ..., 0], time_from_start: { sec: 4 } },
      { positions: [0, ..., 0], time_from_start: { sec: 6 } }
    ]
  }
}"
```
