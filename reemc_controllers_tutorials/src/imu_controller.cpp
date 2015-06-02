#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <boost/assign.hpp>
#include <hardware_interface/imu_sensor_interface.h>
#include <sensor_msgs/Imu.h>

namespace reemc_tutorial_controllers{

  class ImuController : public controller_interface::Controller<hardware_interface::ImuSensorInterface>
  {
  public:
    bool init(hardware_interface::ImuSensorInterface* hw, ros::NodeHandle &n)
    {
      // get all imu sensor names
      const std::vector<std::string>& sensor_names = hw->getNames();
      for (unsigned i=0; i<sensor_names.size(); i++)
        ROS_INFO("Got sensor %s", sensor_names[i].c_str());

      for (unsigned i=0; i<sensor_names.size(); i++){
        // sensor handle
        sensor_ = hw->getHandle(sensor_names[i]);
      }
      return true;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
      sensor_msgs::Imu value;
      // Orientation
      if (sensor_.getOrientation())
      {
        value.orientation.x = sensor_.getOrientation()[0];
        value.orientation.y = sensor_.getOrientation()[1];
        value.orientation.z = sensor_.getOrientation()[2];
        value.orientation.w = sensor_.getOrientation()[3];
      }
      ROS_INFO_STREAM_THROTTLE(1.0, "IMU orientation x: "<<value.orientation.x<<" y: "<<value.orientation.y<<" z: "<<value.orientation.z<<" w: "<<value.orientation.w);

    }

    void starting(const ros::Time& time) { }
    void stopping(const ros::Time& time) { }

  private:
    hardware_interface::ImuSensorHandle sensor_;

  };
  PLUGINLIB_DECLARE_CLASS(reemc_tutorial_controllers, ImuController, reemc_tutorial_controllers::ImuController, controller_interface::ControllerBase);

}
