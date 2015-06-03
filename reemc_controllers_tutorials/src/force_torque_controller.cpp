#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Wrench.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <boost/assign.hpp>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <sensor_msgs/Imu.h>

namespace reemc_controllers_tutorials{


  class ForceTorqueController : public controller_interface::Controller<hardware_interface::ForceTorqueSensorInterface>
  {
  public:
    bool init(hardware_interface::ForceTorqueSensorInterface* hw, ros::NodeHandle &n)
    {
      // get force torque sensors names
      const std::vector<std::string>& sensor_names = hw->getNames();
      for (unsigned i=0; i<sensor_names.size(); i++)
        ROS_INFO("Got sensor %s", sensor_names[i].c_str());

      for (unsigned i=0; i<sensor_names.size(); i++){
        // sensor handle
        sensors_.push_back(hw->getHandle(sensor_names[i]));
      }

      return true;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
      geometry_msgs::Wrench ft[2];

      for(unsigned int s = 0; s<2; ++s){
        ft[s].force.x  = sensors_[s].getForce()[0];
        ft[s].force.y  = sensors_[s].getForce()[1];
        ft[s].force.z   = sensors_[s].getForce()[2];
        ft[s].torque.x = sensors_[s].getTorque()[0];
        ft[s].torque.y = sensors_[s].getTorque()[1];
        ft[s].torque.z = sensors_[s].getTorque()[2];
      }

      ROS_INFO_STREAM_THROTTLE(1.0, "Force sensor left: fx = "<<ft[0].force.x<<" fy: "<<ft[0].force.y<<" fz: "<<ft[0].force.z<<
                      " tx: "<<ft[0].torque.x<<" ty: "<<ft[0].torque.y<<" tz: "<<ft[0].torque.z);
      ROS_INFO_STREAM_THROTTLE(1.0, "Force sensor right: fx = "<<ft[1].force.x<<" fy: "<<ft[1].force.y<<" fz: "<<ft[1].force.z<<
                      " tx: "<<ft[1].torque.x<<" ty: "<<ft[1].torque.y<<" tz: "<<ft[1].torque.z);
    }

    void starting(const ros::Time& time) { }
    void stopping(const ros::Time& time) { }

  private:
    std::vector<hardware_interface::ForceTorqueSensorHandle> sensors_;

  };
  PLUGINLIB_DECLARE_CLASS(reemc_controllers_tutorials, ForceTorqueController, reemc_controllers_tutorials::ForceTorqueController, controller_interface::ControllerBase);

}
