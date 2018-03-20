#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <controller_interface/controller_base.h>
#include <pal_ros_utils/xmlrpc_helpers.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <XmlRpcException.h>
#include <pluginlib/class_list_macros.h>

using namespace ddynamic_reconfigure;

namespace reemc_effort_control{

class SimpleEffortController : public controller_interface::ControllerBase, public boost::noncopyable
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SimpleEffortController():
      realHardware_(false){

    }

    ~SimpleEffortController() {

    }

    /**
     * @brief getHardwareInterfaceType returns the hardware interface type, in this case an Effot controller
     * @return
     */
    std::string getHardwareInterfaceType() const {
        return hardware_interface::internal::demangledTypeName<hardware_interface::EffortJointInterface>();
    }

    /**
     * @brief initTorqueConstants reads from the parameter server the torque constants
     * @param root_nh
     * @return
     */
    bool initTorqueConstants(ros::NodeHandle& root_nh){
        ROS_INFO_STREAM("Parsing torque constants from param server");

        // Get joint names from the parameter server
        using namespace XmlRpc;
        XmlRpcValue joint_torque_param;
        if (!root_nh.getParam("torque_constant", joint_torque_param)){
            ROS_ERROR_STREAM("No  torque constants found in param serve");
            return false;
        }

        try{

            torqueConversions_.resize(joints_.size());
            torqueConversions_.setZero();

            ROS_INFO_STREAM("Number of torque constants : "<<joint_torque_param.size());
            /// @todo make it robust to parsing error(Wrong names, missing names, ect)
            /// @todo map the value in the torque constant vector to the right value (name), now we assume same order.
            // XmlRpc::XmlRpcValue joint_torque_constants = joint_torque_param["torque_constant"];
            XmlRpc::XmlRpcValue joint_torque_constants = joint_torque_param;

            for(size_t i=0; i<joints_.size(); ++i){
                bool found = false;
                for(XmlRpc::XmlRpcValue::ValueStruct::iterator it = joint_torque_constants.begin(); it != joint_torque_constants.end() && !found; ++it )
                {
                    std::string joint_name = static_cast<std::string>(it->first);
                    if(joint_name == joints_[i].getName()){
                        double torque_constant = static_cast<double>(it->second);
                        torqueConversions_(i) = torque_constant;
                        ROS_INFO_STREAM("parsed torque constant for joint: "<<joint_name<<" | value :"<<torque_constant);
                        found = true;
                    }
                }
                if(!found){
                    ROS_ERROR_STREAM("Could not find torque constant for joint : "<<joints_[i].getName());
                }
            }
        }
        catch (XmlRpc::XmlRpcException exception){
            ROS_ERROR_STREAM("Exception code: " + exception.getCode());
            ROS_ERROR_STREAM("Exception message: " + exception.getMessage());
            return false;
        }

        return true;
    }

    /**
     * @brief initJoints
     * @param eff_iface
     * @param controller_nh
     * @return
     */
    bool initJoints(hardware_interface::EffortJointInterface *eff_iface, ros::NodeHandle& controller_nh){
        // Get joint names from the parameter server
        using namespace XmlRpc;
        XmlRpcValue joint_names;
        if(!controller_nh.getParam("joints", joint_names) ){
            ROS_ERROR_STREAM("No joints given (namespace:" << controller_nh.getNamespace() << ").");
            return false;
        }
        if(joint_names.getType() != XmlRpcValue::TypeArray){
            ROS_ERROR_STREAM("Malformed joint specification (namespace:" << controller_nh.getNamespace() << ").");
            return false;
        }

        // Populate temporary container of joint handles
        std::vector<hardware_interface::JointHandle> joints_tmp;
        for (int i = 0; i < joint_names.size(); ++i){
            XmlRpcValue &name_value = joint_names[i];
            if (name_value.getType() != XmlRpcValue::TypeString){
                ROS_ERROR_STREAM("Array of joint names should contain all strings (namespace:" << controller_nh.getNamespace() << ").");
                return false;
            }
            const std::string joint_name = static_cast<std::string>(name_value);
            this->controllerJointNames_.push_back(joint_name);

            // Get a joint handle
            try{
                joints_tmp.push_back(eff_iface->getHandle(joint_name));
                ROS_DEBUG_STREAM("Found joint '" << joint_name << "' in '" <<
                                 getHardwareInterfaceType() << "'");
            }
            catch (...){
                ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in '" <<
                                 getHardwareInterfaceType() << "'");
                return false;
            }
        }

        // Member list of joint handles is updated only once all resources have been claimed
        joints_ = joints_tmp;

        return true;
    }



    bool init(hardware_interface::EffortJointInterface* eff_iface, ros::NodeHandle &controller_nh){
        // Hardware interfaces
        if (!initJoints(eff_iface, controller_nh) ){
            ROS_ERROR_STREAM("Failed to initialize controller '" << hardware_interface::internal::demangledTypeName(*this) << "'");
            return false;
        }

        std::vector<std::string> joints_name;
        for(size_t i=0; i<joints_.size(); ++i){
            joints_name.push_back(joints_.at(i).getName());
        }

        if(!controller_nh.hasParam("hardware")){
            ROS_ERROR_STREAM("Hardware or simulation has not been setted");
            return false;
        }
        controller_nh.param<bool>("hardware", realHardware_, true);

        if(realHardware_){
            ROS_INFO_STREAM("Configured for real hardware");
        }
        else{
            ROS_INFO_STREAM("Configured for simulation");
        }

        return true;
    }

    bool initRequest(hardware_interface::RobotHW* robot_hw,
                     ros::NodeHandle&             root_nh,
                     ros::NodeHandle &            controller_nh,
                     ClaimedResources&       claimed_resources){

        // Check if construction finished cleanly
        if (state_ != CONSTRUCTED){
            ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
            return false;
        }

        // Get a pointer to the joint position control interface
        hardware_interface::EffortJointInterface* eff_iface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (!eff_iface)
        {
            ROS_ERROR("This controller requires a hardware interface of type '%s'."
                      " Make sure this is registered in the hardware_interface::RobotHW class.",
                      hardware_interface::internal::demangledTypeName<hardware_interface::EffortJointInterface>().c_str());
            return false;
        }

        // Return which resources are claimed by this controller
        eff_iface->clearClaims();
        if (!init(eff_iface,
                  controller_nh))
        {
            ROS_ERROR("Failed to initialize the controller");
            return false;
        }
        claimed_resources.push_back(hardware_interface::InterfaceResources(hardware_interface::internal::demangledTypeName<hardware_interface::EffortJointInterface>(),
                                        eff_iface->getClaims()));
        eff_iface->clearClaims();


        //Parse torque constants
        if(!initTorqueConstants(root_nh)){
            ROS_ERROR_STREAM("Failed reading torque constants");
            return false;
        }

        Q_act_.resize(joints_.size());
        Qd_act_.resize(joints_.size());
        Tau_cmd_.resize(joints_.size());

        Q_act_.setZero();
        Qd_act_.setZero();
        Tau_cmd_.setZero();

        ddReconfigure_.reset(new DDynamicReconfigure(ros::NodeHandle(root_nh, "controller_params")) );
        for(size_t i=0; i<torqueConversions_.rows(); ++i){
            std::string joint_name = joints_[i].getName();
            ddReconfigure_->RegisterVariable(&Tau_cmd_(i), "torque_comand_" + joint_name, -20, 20);
        }
        ddReconfigure_->PublishServicesTopics();

        // success
        state_ = INITIALIZED;
        return true;
    }

    void update(const ros::Time& time, const ros::Duration& period){

        readJoints();

        writeJoints();

    }


    void stopping(const ros::Time& time) {
        for(size_t j = 0; j<joints_.size(); ++j){
            Tau_cmd_(j) = 0.0;
        }
        writeJoints();
    }

    void readJoints(){

        for(unsigned int j = 0; j<joints_.size(); ++j){
            bool found = false;
            for(unsigned int i=0; (i < controllerJointNames_.size()) && !found; ++i){
                if(controllerJointNames_[i] == joints_[j].getName()){
                    Q_act_(i) = joints_[j].getPosition();
                    Qd_act_(i) = joints_[j].getVelocity();
                    found = true;
                }
            }
            if(!found){
                std::stringstream ss;
                ss<<"Joint not found in model, cannot read "<<joints_[j].getName()<<std::endl;
                ss<<"Available joints: "<<std::endl;
                for(size_t k=0; k<controllerJointNames_.size(); ++k){
                    ss<<"   "<<controllerJointNames_[k]<<std::endl;
                }
                ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
            }
        }

    }

    void writeJoints(){

        if(!realHardware_){
            for(unsigned int j = 0; j<joints_.size(); ++j){
                bool found = false;
                for(unsigned int i=0; (i < controllerJointNames_.size()) && !found; ++i){
                    if(controllerJointNames_[i] == joints_[j].getName()){
                        joints_[j].setCommand(Tau_cmd_(i));
                        found = true;
                    }
                }
                if(!found){
                    std::stringstream ss;
                    ss<<"Joint not found in model, cannot write simulated model "<<joints_[j].getName()<<std::endl;
                    ss<<"Available joints: "<<std::endl;
                    for(size_t k=0; k<controllerJointNames_.size(); ++k){
                        ss<<"   "<<controllerJointNames_[k]<<std::endl;
                    }
                    ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
                }
            }
        }
        else{
            for(unsigned int j = 0; j<joints_.size(); ++j){
                bool found = false;
                for(unsigned int i=0; (i < controllerJointNames_.size()) && !found; ++i){
                    if(controllerJointNames_[i] == joints_[j].getName()){

                        joints_[j].setCommand(Tau_cmd_(i)*torqueConversions_(j));
                        found = true;
                    }
                }
                if(!found){
                    std::stringstream ss;
                    ss<<"Joint not found in model, cannot write real model "<<joints_[j].getName()<<std::endl;
                    ss<<"Available joints: "<<std::endl;
                    for(size_t k=0; k<controllerJointNames_.size(); ++k){
                        ss<<"   "<<controllerJointNames_[k]<<std::endl;
                    }
                    ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
                }
            }
        }
    }

protected:

    Eigen::VectorXd Q_act_;
    Eigen::VectorXd Qd_act_;
    Eigen::VectorXd Tau_cmd_;

    Eigen::VectorXd torqueConversions_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<std::string> controllerJointNames_;
    boost::shared_ptr<DDynamicReconfigure> ddReconfigure_;

    bool realHardware_;

};

PLUGINLIB_DECLARE_CLASS(reemc_effort_control, SimpleEffortController, reemc_effort_control::SimpleEffortController, controller_interface::ControllerBase);

}


