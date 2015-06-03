#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>

namespace reemc_controllers_tutorials{

  class HelloController : public controller_interface::Controller<hardware_interface::JointStateInterface>
  {
  public:
    bool init(hardware_interface::JointStateInterface* hw, ros::NodeHandle &n)
    {
      return true;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
      // WARNING! This will not be realtime-safe
      ROS_INFO_NAMED("hello_controller", "Hello ros_control!");
    }

    void starting(const ros::Time& time) { }
    void stopping(const ros::Time& time) { }

  private:
  };
  PLUGINLIB_DECLARE_CLASS(reemc_controllers_tutorials, HelloController, reemc_controllers_tutorials::HelloController, controller_interface::ControllerBase);

}
