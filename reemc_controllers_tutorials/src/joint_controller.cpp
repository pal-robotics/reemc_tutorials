#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace reemc_controllers_tutorials{

  class JointController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
  {
  public:
    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
    {
      cont = 0;
      controlled_joint_name_ = "head_2_joint";

      ROS_INFO_STREAM("LOADING JOINT CONTROLLER ");
      // Get a joint handle
      try
      {
        joint_ = hw->getHandle(controlled_joint_name_);  // throws on failure
        ROS_INFO_STREAM("Found joint '" << controlled_joint_name_);
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find joint '" << controlled_joint_name_);
        return false;
      }
      return true;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
      //Move the head using a sine wave
      double joint_comand = 0.2*sin(cont/1000.0);
      joint_.setCommand(joint_comand);
      cont = cont + 1;
    }

    void starting(const ros::Time& time) { }
    void stopping(const ros::Time& time) { }

  private:
    hardware_interface::JointHandle joint_;
    std::string controlled_joint_name_;
    double cont;
  };
  PLUGINLIB_DECLARE_CLASS(reemc_controllers_tutorials, JointController, reemc_controllers_tutorials::JointController, controller_interface::ControllerBase);

}
