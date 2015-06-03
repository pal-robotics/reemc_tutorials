#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Wrench.h>
#include <eigen3/Eigen/Dense>

using namespace hardware_interface;
using namespace std;

namespace reemc_controllers_tutorials{

  class CombinedResourceController : public controller_interface::ControllerBase
  {
  public:

    bool initRequest(hardware_interface::RobotHW* robot_hw,
                     ros::NodeHandle&             root_nh,
                     ros::NodeHandle &            controller_nh,
                     std::set<std::string>&       claimed_resources)
    {

      // Check if construction finished cleanly
      if (state_ != CONSTRUCTED)
      {
        ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
        return false;
      }

      // Get a pointer to the joint position control interface
      PositionJointInterface* pos_iface = robot_hw->get<PositionJointInterface>();
      if (!pos_iface)
      {
        ROS_ERROR("This controller requires a hardware interface of type '%s'."
                  " Make sure this is registered in the hardware_interface::RobotHW class.",
                  getHardwareInterfaceType().c_str());
        return false;
      }

      // Get a pointer to the force-torque sensor interface
      ForceTorqueSensorInterface* ft_iface = robot_hw->get<ForceTorqueSensorInterface>();
      if (!ft_iface)
      {
        ROS_ERROR("This controller requires a hardware interface of type '%s'."
                  " Make sure this is registered in the hardware_interface::RobotHW class.",
                  internal::demangledTypeName<ForceTorqueSensorInterface>().c_str());
        return false;
      }

      // Get a pointer to the IMU sensor interface
      ImuSensorInterface* imu_iface = robot_hw->get<ImuSensorInterface>();
      if (!imu_iface)
      {
        ROS_ERROR("This controller requires a hardware interface of type '%s'."
                  " Make sure this is registered in the hardware_interface::RobotHW class.",
                  internal::demangledTypeName<ImuSensorInterface>().c_str());
        return false;
      }

      // Return which resources are claimed by this controller
      pos_iface->clearClaims();
      if (!init(pos_iface,
                ft_iface,
                imu_iface,
                root_nh,
                controller_nh))
      {
        ROS_ERROR("Failed to initialize the controller");
        std::cerr  << "FAILED LOADING WALKING" << std::endl;
        return false;
      }
      claimed_resources = pos_iface->getClaims();
      pos_iface->clearClaims();

      // success
      state_ = INITIALIZED;
      return true;
    }

    bool init(PositionJointInterface*     pos_iface,
              ForceTorqueSensorInterface* ft_iface,
              ImuSensorInterface*         imu_iface,
              ros::NodeHandle&            /*root_nh*/,
              ros::NodeHandle&            controller_nh)
    {

      // Hardware interfaces
      if (!initJoints(pos_iface, controller_nh)            ||
          !initForceTorqueSensors(ft_iface, controller_nh) ||
          !initImuSensors(imu_iface, controller_nh))
      {
        ROS_ERROR_STREAM("Failed to initialize controller '" << internal::demangledTypeName(*this) << "'");
        return false;
      }
      return true;
    }

    bool initJoints(PositionJointInterface* pos_iface,
                    ros::NodeHandle&        controller_nh)
    {
      // Get joint names from the parameter server
      using namespace XmlRpc;
      XmlRpcValue joint_names;
      if (!controller_nh.getParam("joints", joint_names))
      {
        ROS_ERROR_STREAM("No joints given (namespace:" << controller_nh.getNamespace() << ").");
        return false;
      }
      if (joint_names.getType() != XmlRpcValue::TypeArray)
      {
        ROS_ERROR_STREAM("Malformed joint specification (namespace:" << controller_nh.getNamespace() << ").");
        return false;
      }

      // Populate temporary container of joint handles
      vector<hardware_interface::JointHandle> joints_tmp;
      for (int i = 0; i < joint_names.size(); ++i)
      {
        XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpcValue::TypeString)
        {
          ROS_ERROR_STREAM("Array of joint names should contain all strings (namespace:" << controller_nh.getNamespace() << ").");
          return false;
        }
        const string joint_name = static_cast<string>(name_value);

        // Get a joint handle
        try
        {
          joints_tmp.push_back(pos_iface->getHandle(joint_name));
          ROS_DEBUG_STREAM("Found joint '" << joint_name << "' in '" <<
                           getHardwareInterfaceType() << "'");
        }
        catch (...)
        {
          ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in '" <<
                           getHardwareInterfaceType() << "'");
          return false;
        }
      }

      // Member list of joint handles is updated only once all resources have been claimed
      joints_ = joints_tmp;

      return true;
    }

    bool initForceTorqueSensors(ForceTorqueSensorInterface* ft_iface,
                                ros::NodeHandle&            controller_nh)
    {
      //Resize for to sensors
      ft_sensors_.resize(2);
      // Left ankle
      string left_ft_name;
      if (!controller_nh.getParam("left_ft_sensor", left_ft_name))
      {
        ROS_ERROR_STREAM("No left_ft_sensor given (namespace:" << controller_nh.getNamespace() << ").");
        return false;
      }
      try
      {
        ft_sensors_[0] = ft_iface->getHandle(left_ft_name);

        ROS_DEBUG_STREAM("Found force-torque sensor '" << left_ft_name << "' in '" <<
                         internal::demangledTypeName(*ft_iface) << "'");
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find force-torque sensor '" << left_ft_name << "' in '" <<
                         internal::demangledTypeName(*ft_iface) << "'");
        return false;
      }

      // Right ankle
      string right_ft_name;
      if (!controller_nh.getParam("right_ft_sensor", right_ft_name))
      {
        ROS_ERROR_STREAM("No right_ft_sensor given (namespace:" << controller_nh.getNamespace() << ").");
        return false;
      }
      try
      {
        ft_sensors_[1] = ft_iface->getHandle(right_ft_name);

        ROS_DEBUG_STREAM("Found force-torque sensor '" << right_ft_name << "' in '" <<
                         internal::demangledTypeName(*ft_iface) << "'");
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find force-torque sensor '" << right_ft_name << "' in '" <<
                         internal::demangledTypeName(*ft_iface) << "'");
        return false;
      }

      return true;

    }

    bool initImuSensors(ImuSensorInterface* imu_iface,
                        ros::NodeHandle&    controller_nh)
    {
      // Base IMU
      string base_imu_name;
      if (!controller_nh.getParam("base_imu_sensor", base_imu_name))
      {
        ROS_ERROR_STREAM("No base_imu_sensor given (namespace:" << controller_nh.getNamespace() << ").");
        return false;
      }
      try
      {
        imu_sensor = imu_iface->getHandle(base_imu_name);

        if (!imu_sensor.getOrientation())
        {
          ROS_ERROR_STREAM("IMU sensor '" << base_imu_name << "' does not provide orientation readings");
          return false;
        }

        ROS_DEBUG_STREAM("Found IMU sensor '" << base_imu_name << "' in '" <<
                         internal::demangledTypeName(*imu_iface) << "'");
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find IMU sensor '" << base_imu_name << "' in '" <<
                         internal::demangledTypeName(*imu_iface) << "'");
        return false;
      }

      return true;
    }


    void update(const ros::Time& time, const ros::Duration& period)
    {
      //F-T sensor
      geometry_msgs::Wrench ft[2];

      for(unsigned int s = 0; s<2; ++s){
        ft[s].force.x  = ft_sensors_[s].getForce()[0];
        ft[s].force.y  = ft_sensors_[s].getForce()[1];
        ft[s].force.z   = ft_sensors_[s].getForce()[2];
        ft[s].torque.x = ft_sensors_[s].getTorque()[0];
        ft[s].torque.y = ft_sensors_[s].getTorque()[1];
        ft[s].torque.z = ft_sensors_[s].getTorque()[2];
      }
      ROS_INFO_STREAM_THROTTLE(1.0, "Force sensor right: fx = "<<ft[0].force.x<<" fy: "<<ft[0].force.y<<" fz: "<<ft[0].force.z<<
                      " tx: "<<ft[0].torque.x<<" ty: "<<ft[0].torque.y<<" tz: "<<ft[0].torque.z);
      ROS_INFO_STREAM_THROTTLE(1.0, "Force sensor right: fx = "<<ft[1].force.x<<" fy: "<<ft[1].force.y<<" fz: "<<ft[1].force.z<<
                      " tx: "<<ft[1].torque.x<<" ty: "<<ft[1].torque.y<<" tz: "<<ft[1].torque.z);

      //IMU
      sensor_msgs::Imu value;
      if (imu_sensor.getOrientation())
      {
        value.orientation.x = imu_sensor.getOrientation()[0];
        value.orientation.y = imu_sensor.getOrientation()[1];
        value.orientation.z = imu_sensor.getOrientation()[2];
        value.orientation.w = imu_sensor.getOrientation()[3];
      }
      ROS_INFO_STREAM("IMU orientation x: "<<value.orientation.x<<" y: "<<value.orientation.y<<" z: "<<value.orientation.z<<" w: "<<value.orientation.w);
    }

    void stopping(const ros::Time& time)
    {}

    std::string getHardwareInterfaceType() const {return hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>();}

  private:

    // Hardware interfaces
    std::vector<hardware_interface::JointHandle> joints_;
    hardware_interface::ImuSensorHandle imu_sensor;
    //We will store two force torque handles,
    //0 will be the left ankle sensor and 1 the right ankle sensor
    std::vector<hardware_interface::ForceTorqueSensorHandle> ft_sensors_;

  };
  PLUGINLIB_DECLARE_CLASS(reemc_controllers_tutorials, CombinedResourceController, reemc_controllers_tutorials::CombinedResourceController, controller_interface::ControllerBase);

}
