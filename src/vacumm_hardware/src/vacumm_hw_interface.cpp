

#include <vacumm_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace vacumm_ns {
VacummHWInterface::VacummHWInterface(ros::NodeHandle &nh,
                                     urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model) {
  // Load rosparams
  ros::NodeHandle rpnh(nh_, "hardware_interface");
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
}

void VacummHWInterface::enforceLimits(ros::Duration &period) {
  // Enforces position and velocity
  //   pos_jnt_sat_interface_.enforceLimits(period);
}

} // namespace vacumm_ns