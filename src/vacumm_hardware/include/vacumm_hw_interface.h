
#ifndef VACUMM_HW_INTERFACE_H
#define VACUMM_HW_INTERFACE_H

#include <generic_hw_interface.h>
#include <vacumm_hardware/WheelCmd.h>
#include <vacumm_hardware/WheelState.h>

namespace vacumm_ns {
/** \brief Hardware interface for a robot */
class VacummHWInterface : public ros_control_boilerplate::GenericHWInterface {
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  VacummHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);

protected:
  ros::Subscriber wheel_state_sub;
  ros::Subscriber wheel_encoder_sub;
  ros::Publisher wheel_cmd_pub;
  
  void wheelStateCallback(const vacumm_hardware::WheelState::ConstPtr &msg);
  void wheelEncoderCallback(const vacumm_hardware::WheelState::ConstPtr &msg);
  double ticksToAngle(const int &ticks);
  double ticksToDegree(const int &ticks);
  double ticksToAngle2(const int &ticks);
  double ticksToDegree2(const int &ticks);
  double degreeToAngles(const double &degree);
}; // class

} // namespace vacumm_ns

#endif