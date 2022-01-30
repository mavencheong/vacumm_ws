#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <time.h>

namespace ros_control_boilerplate {
// Used to convert seconds elapsed to nanoseconds
static const double BILLION = 1000000000.0;

/**
 * \brief The control loop - repeatidly calls read() and write() to the hardware
 * interface at a specified frequency We use MONOTONIC time to ensure robustness
 * in the event of system time updates/change. See
 * http://stackoverflow.com/questions/3523442/difference-between-clock-realtime-and-clock-monotonic
 */
class GenericHWControlLoop {
public:
  /**
   * \brief Constructor
   * \param NodeHandle
   * \param hardware_interface - the robot-specific hardware interface to be use
   * with your robot
   */
  GenericHWControlLoop(
      ros::NodeHandle &nh,
      std::shared_ptr<hardware_interface::RobotHW> hardware_interface);

  // Run the control loop (blocking)
  void run();

protected:
  // Update funcion called with loop_hz_ rate
  void update();

  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  // Name of this class
  std::string name_ = "generic_hw_control_loop";

  // Settings
  ros::Duration desired_update_period_;
  double cycle_time_error_threshold_;

  // Timing
  ros::Duration elapsed_time_;
  double loop_hz_;
  struct timespec last_time_;
  struct timespec current_time_;

  /** \brief ROS Controller Manager and Runner
   *
   * This class advertises a ROS interface for loading, unloading, starting, and
   * stopping ros_control-based controllers. It also serializes execution of all
   * running controllers in \ref update.
   */
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  /** \brief Abstract Hardware Interface for your robot */
  std::shared_ptr<hardware_interface::RobotHW> hardware_interface_;

}; // end class

} // namespace ros_control_boilerplate