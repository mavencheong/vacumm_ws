

#include <vacumm_hw_interface.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace vacumm_ns {
VacummHWInterface::VacummHWInterface(ros::NodeHandle &nh,
                                     urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model) {
  // Load rosparams

  wheel_state_sub = nh.subscribe("/vacumm/wheel_state", 1,
                                 &VacummHWInterface::wheelStateCallback, this);

  wheel_cmd_pub =
      nh.advertise<vacumm_hardware::WheelCmd>("/vacumm/wheel_cmd", 3);
  ros::NodeHandle rpnh(nh_, "hardware_interface");
}

void VacummHWInterface::wheelStateCallback(
    const vacumm_hardware::WheelState::ConstPtr &msg) {

  for (int i = 0; i < num_joints_; i++) {
    joint_velocity_[i] = msg->vel[i];
    joint_position_[i] = msg->pos[i];
  }
}

void VacummHWInterface::init() {
  // Call parent class version of this function
  GenericHWInterface::init();

  ROS_INFO("VacummHWInterface Ready.");
}

void VacummHWInterface::read(ros::Duration &elapsed_time) {
  // No need to read since our write() command populates our state for us
}

void VacummHWInterface::write(ros::Duration &elapsed_time) {
  // Safety
  static vacumm_hardware::WheelCmd cmd;

  for (int i = 0; i < num_joints_; i++) {
    cmd.vel[i] = joint_velocity_command_[i];
    cmd.pos[i] = joint_position_command_[i];
  }
  // printCommand();
  wheel_cmd_pub.publish(cmd);
}

void VacummHWInterface::enforceLimits(ros::Duration &period) {
  // Enforces position and velocity
  //   pos_jnt_sat_interface_.enforceLimits(period);
}

} // namespace vacumm_ns