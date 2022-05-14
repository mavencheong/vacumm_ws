

#include <vacumm_hw_interface.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace vacumm_ns
{
  VacummHWInterface::VacummHWInterface(ros::NodeHandle &nh,
                                       urdf::Model *urdf_model)
      : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    // Load rosparams

    wheel_state_sub = nh.subscribe("/vacumm/wheel_state", 1,
                                   &VacummHWInterface::wheelStateCallback, this);


    // wheel_encoder_sub = nh.subscribe("/vacumm/wheel_encoder", 1,
    //                                &VacummHWInterface::wheelEncoderCallback, this);


    wheel_cmd_pub =
        nh.advertise<vacumm_hardware::WheelCmd>("/vacumm/wheel_cmd", 3);
    ros::NodeHandle rpnh(nh_, "hardware_interface");

    double wheelVel[2];
  }

  void VacummHWInterface::wheelStateCallback(
      const vacumm_hardware::WheelState::ConstPtr &msg)
  {
    double wheel_angles[2];
    double wheel_degrees[2];
    double wheel_angles_delta[2];
    for (int i = 0; i < num_joints_; i++)
    {
      wheel_degrees[i] = ticksToDegree2(msg->encoder[i]);
      wheel_angles[i] = degreeToAngles(wheel_degrees[i]);

      wheel_angles_delta[i] = wheel_angles[i] - joint_position_[i];  

      joint_velocity_[i] = msg->vel[i];
      joint_position_[i] = wheel_angles[i] ;
    }
  }


  void VacummHWInterface::init()
  {
    // Call parent class version of this function
    GenericHWInterface::init();

    ROS_INFO("VacummHWInterface Ready.");
  }

  void VacummHWInterface::read(ros::Duration &elapsed_time)
  {

  }

  void VacummHWInterface::write(ros::Duration &elapsed_time)
  {
    // Safety
    static vacumm_hardware::WheelCmd cmd;
    const double cmd_dt(elapsed_time.toSec());
    
    for (int i = 0; i < num_joints_; i++)
    {
      cmd.vel[i] = joint_velocity_command_[i];
      cmd.pos[i] += (joint_velocity_command_[i] * cmd_dt);

      // joint_velocity_[i] = joint_velocity_command_[i];
      // joint_position_[i] += (joint_velocity_command_[i] * cmd_dt);


      // ROS_INFO_STREAM_NAMED(
      //     name_, "Velocity" << joint_velocity_command_[i]  << ", Elapsed Time: " << cmd_dt << "Position: " << joint_position_[i]  );
    }
    // printCommand();
    wheel_cmd_pub.publish(cmd);
  }

  void VacummHWInterface::enforceLimits(ros::Duration &period)
  {
    // Enforces position and velocity
    //   pos_jnt_sat_interface_.enforceLimits(period);
  }

  double VacummHWInterface::ticksToAngle(const int &ticks)
  {
    double angle = (double)ticks * (2.0 * M_PI / 840.0);
    return angle;
  }

  double VacummHWInterface::ticksToDegree(const int &ticks)
  {
    double angle = (360/840.0)*ticks;
    return angle;
  }


  double VacummHWInterface::ticksToAngle2(const int &ticks)
  {
    double angle = (double)ticks * (2.0 * M_PI / 1205.0);
    return angle;
  }

  double VacummHWInterface::ticksToDegree2(const int &ticks)
  {
    double angle = (360/1205.0)*ticks;
    return angle;
  }


  double VacummHWInterface::degreeToAngles(const double &degree)
  {
    return degree * M_PI / 180;
  }

} // namespace vacumm_ns