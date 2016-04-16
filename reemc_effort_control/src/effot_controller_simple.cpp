#ifndef PAL_ARM_ROS_CONTROLBASE_H
#define PAL_ARM_ROS_CONTROLBASE_H

#include <string>
#include <vector>

#include <boost/scoped_ptr.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <angles/angles.h>
#include <ros/node_handle.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
//#include <hardware_interface/joint_temperature_interface.h>
#include <controller_interface/controller_base.h>
#include <sensor_msgs/JointState.h>
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <rbdl/addons/rbdlUrdfParser.h>
#include <pal_robot_tools/reference/reference_abstract.h>
#include <force_control_core/ForceControlDebug.h>

#include <ddynamic_reconfigure/DDynamicReconfigure.h>

namespace walking_force_control{

  class PalArmRosControlBase : public controller_interface::ControllerBase, public boost::noncopyable
  {
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PalArmRosControlBase();
    ~PalArmRosControlBase() {}

    // Do not modify the implementation of this method. If you want to change something in the initialization step,
    // please change the implementation of the private init(...) method.
    bool initRequest(hardware_interface::RobotHW* robot_hw,
                     ros::NodeHandle&             root_nh,
                     ros::NodeHandle &            controller_nh,
                     std::set<std::string>&       claimed_resources);

    std::string getHardwareInterfaceType() const {return hardware_interface::internal::demangledTypeName<hardware_interface::EffortJointInterface>();}

    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping(const ros::Time& time);

    void readJoints();
    void writeJoints();

    //Controller specific
    virtual bool loading_controller(ros::NodeHandle &control_nh) = 0;
    virtual bool update_controller(const ros::Time& time, const ros::Duration& period) = 0;
    virtual bool stopping_controller() = 0;

    bool embeded_;

  protected:

    // Hardware interfaces
    std::vector<hardware_interface::JointHandle> joints_;

    bool init(hardware_interface::EffortJointInterface *eff_iface,
              ros::NodeHandle&                                controller_nh);

    bool initJoints(hardware_interface::EffortJointInterface *eff_iface, ros::NodeHandle& controller_nh);
    bool initTorqueConstants(ros::NodeHandle& root_nh);

    std::vector<std::string> controller_joint_names_;

    //Controller variables
    RigidBodyDynamics::Model rbdl_model_;
    bool floating_base_;
    int n_dof_;

    std::vector<std::string> tip_links_;

    std::vector< double > joint_position_min_; //Joint min position
    std::vector< double > joint_position_max_; //Joint max position
    std::vector< double > joint_vel_min_; //Joint min velocity
    std::vector< double > joint_vel_max_; //Joint max velocity
    std::vector< double > joint_damping_; //Joint viscous damping
    std::vector< double > joint_friction_; //Joint static friction
    std::vector< double > joint_max_effort_; //Joint maximum torque
    std::vector< std::string > joint_names_;

    //    std::vector<unsigned int> tip_body_ids; //Contains id of the tips
    //    std::vector<RigidBodyDynamics::Math::Vector3d> tip_positions;
    //    std::vector<RigidBodyDynamics::Math::Vector3d> target_tip_positions;

    Eigen::VectorXd Q_act_; //Actual joint configuration
    Eigen::VectorXd Q_des_; //Actual joint configuration

    Eigen::VectorXd Qd_act_; //Actual joint velocities
    Eigen::VectorXd Qd_des_; //Desired joint velocities

    Eigen::VectorXd Old_Qd_des_; //Desired joint velocities
    Eigen::VectorXd Qdd_des_; //Desired joint velocities
    Eigen::VectorXd Tau_cmd_; //Desired joint torques

    Eigen::VectorXd torque_conversion_; //Torque conversion for real hardware
    Eigen::VectorXd torque_scaling_;
    Eigen::VectorXd viscous_friction_;
    Eigen::VectorXd static_friction_;

    boost::shared_ptr<DDynamicReconfigure> dd_reconfigure_;
  };

}

#endif



#include <force_control_core/eigen_realtime.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <force_control_core/pal_arm_ros_control.h>
#include <XmlRpc.h>
#include <rbdl/rbdl_utils.h>
#include <dynamic_introspection/DynamicIntrospection.h>

using namespace std;
using namespace hardware_interface;
using namespace walking_force_control;

/// @todo, move out joint mode handle constructor, why isnt there an empty constructor for its class?
PalArmRosControlBase::PalArmRosControlBase():
    embeded_(false){

}

bool PalArmRosControlBase::initTorqueConstants(ros::NodeHandle& root_nh){
    ROS_INFO_STREAM("Parsing torque constants from param server");
    // Get joint names from the parameter server
    using namespace XmlRpc;
    XmlRpcValue joint_torque_param;
    if (!root_nh.getParam("torque_constant", joint_torque_param)){
        ROS_ERROR_STREAM("No  torque constants found in param serve");
        return false;
    }
    /*
    if (joint_torque_param.getType() != XmlRpcValue::TypeArray){
        ROS_ERROR_STREAM("Malformed joint specification joint torque constant especification");
        return false;
    }
    */
    try{
        torque_conversion_.resize(joints_.size());
        torque_conversion_.setZero();

        torque_scaling_.resize(joints_.size());
        torque_scaling_.setZero();

        viscous_friction_.resize(joints_.size());
        viscous_friction_.setZero();

        static_friction_.resize(joints_.size());
        static_friction_.setZero();

        ROS_INFO_STREAM("Number of torque constants : "<<joint_torque_param.size());
        /// @todo make it robust to parsing error(Wrong names, missing names, ect)
        /// @todo map the value in the torque constant vector to the right value (name), now we assume same order.
        // XmlRpc::XmlRpcValue joint_torque_constants = joint_torque_param["torque_constant"];
        XmlRpc::XmlRpcValue joint_torque_constants = joint_torque_param;

        for(unsigned int i=0; i<joints_.size(); ++i){
            bool found = false;
            for(XmlRpc::XmlRpcValue::ValueStruct::iterator it = joint_torque_constants.begin(); it != joint_torque_constants.end() && !found; ++it )
            {
                std::string joint_name = static_cast<std::string>(it->first);
                if(joint_name == joints_[i].getName()){
                    double torque_constant = static_cast<double>(it->second);
                    torque_conversion_(i) = torque_constant;
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


    dd_reconfigure_.reset(new DDynamicReconfigure(ros::NodeHandle(root_nh, "controller_params")) );
    for(size_t i=0; i<torque_scaling_.rows(); ++i){
        std::string joint_name = joints_[i].getName();
        dd_reconfigure_->RegisterVariable(&torque_scaling_(i), "torque_scaling_" + joint_name, -2, 2);
        dd_reconfigure_->RegisterVariable(&viscous_friction_(i), "viscous_friction_" + joint_name, 0, 10);
        dd_reconfigure_->RegisterVariable(&static_friction_(i), "static_friction_" + joint_name, 0, 10);
    }
    dd_reconfigure_->PublishServicesTopics();

    return true;
}


bool PalArmRosControlBase::initJoints(EffortJointInterface *eff_iface, ros::NodeHandle& controller_nh)
{
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
    vector<hardware_interface::JointHandle> joints_tmp;
    for (int i = 0; i < joint_names.size(); ++i){
        XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpcValue::TypeString){
            ROS_ERROR_STREAM("Array of joint names should contain all strings (namespace:" << controller_nh.getNamespace() << ").");
            return false;
        }
        const string joint_name = static_cast<string>(name_value);
        this->controller_joint_names_.push_back(joint_name);

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

bool PalArmRosControlBase::init(hardware_interface::EffortJointInterface* eff_iface, ros::NodeHandle &controller_nh)
{
    // Hardware interfaces
    if (!initJoints(eff_iface, controller_nh) ){
        ROS_ERROR_STREAM("Failed to initialize controller '" << internal::demangledTypeName(*this) << "'");
        return false;
    }

    std::vector<std::string> joints_name;
    for(size_t i=0; i<joints_.size(); ++i){
        joints_name.push_back(joints_.at(i).getName());
    }

    //Load dynamic model of the arm
    floating_base_ = false;
    rbdl_model_.gravity = Eigen::Vector3d (0., 0., -9.8);

    // Get joint default configuration from the parameter server
       using namespace XmlRpc;
       XmlRpcValue subtree_conf;
       if (!controller_nh.getParam("robot_model_chains", subtree_conf))
       {
           ROS_WARN_STREAM("No subtree given (namespace:" << controller_nh.getNamespace() << ").");
       }
       else{
       try{
           ROS_INFO_STREAM("Number of subtree tips : "<<subtree_conf.size());
           //for(XmlRpc::XmlRpcValue::ValueArray::iterator it = subtree_conf.begin(); it != subtree_conf.end(); ++it ){
           for(unsigned int i=0; i<subtree_conf.size(); ++i){
               std::string tip_name = static_cast<std::string>(subtree_conf[i]);
               ROS_DEBUG_STREAM("Parsed tip: "<<tip_name);
               tip_links_.push_back(tip_name);
           }
       }
       catch (XmlRpc::XmlRpcException exception) {
           ROS_ERROR_STREAM("Exception code: " + exception.getCode());
           ROS_ERROR_STREAM("Exception message: " + exception.getMessage());
           return false;
       }
    }

    if(tip_links_.size() > 0){
    // Parse the robot if subchains specified
      parseUrdfParamServerParameters(rbdl_model_, joint_names_,
                                   joint_position_min_, joint_position_max_,
                                   joint_vel_min_, joint_vel_max_,
                                   joint_damping_, joint_friction_,
                                   joint_max_effort_,
                                   floating_base_,
                                   tip_links_,
                                   false);
      ROS_INFO_STREAM(RigidBodyDynamics::Utils::GetModelDOFOverview(rbdl_model_));
    }
    else{ // Parse the full robot if there is no subchain specified
      parseUrdfParamServerParameters(rbdl_model_, joint_names_,
                                   joint_position_min_, joint_position_max_,
                                   joint_vel_min_, joint_vel_max_,
                                   joint_damping_, joint_friction_,
                                   joint_max_effort_,
                                   floating_base_,
                                   false);
    }

    ROS_INFO_STREAM(RigidBodyDynamics::Utils::GetModelDOFOverview(rbdl_model_));

    n_dof_ = rbdl_model_.dof_count;

    Q_act_.resize(n_dof_); //Actual joint configuration
    Q_des_.resize(n_dof_);
    Qd_act_.resize(n_dof_); //Actual joint velocities
    Qd_des_.resize(n_dof_);

    Qdd_des_.resize(n_dof_); //Desired joint accelerations
    Tau_cmd_.resize(n_dof_); //Desired joint torquesinitTorqueConstants

    Q_act_.setZero();
    Q_des_.setZero();
    Qd_act_.setZero();
    Qd_des_.setZero();
    Qdd_des_.setZero();
    Tau_cmd_.setZero();

    RigidBodyDynamics::UpdateKinematicsCustom(rbdl_model_, &Q_act_, NULL, NULL);

    if(!controller_nh.hasParam("hardware")){
        ROS_ERROR_STREAM("Hardware or simulation has not been setted");
        return false;
    }
    controller_nh.param<bool>("hardware", embeded_, true);
    if(embeded_) ROS_INFO_STREAM("Configured for real hardware");
    else ROS_INFO_STREAM("Configured for simulation");

    if(!controller_nh.hasParam("base_link_name")){
        ROS_INFO_STREAM("base link name has not been setted");
        return false;
    }

    //Loading specific controller
    if(!this->loading_controller(controller_nh)){
        ROS_INFO_STREAM("Failed to load controller ");
        return false;
    }

    ROS_INFO_STREAM("Finished loading");

    return true;
}

bool PalArmRosControlBase::initRequest(hardware_interface::RobotHW* robot_hw,
                                       ros::NodeHandle&             root_nh,
                                       ros::NodeHandle &            controller_nh,
                                       std::set<std::string>&       claimed_resources){

    // Check if construction finished cleanly
    if (state_ != CONSTRUCTED){
        ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
        return false;
    }

    // Get a pointer to the joint position control interface
    EffortJointInterface* eff_iface = robot_hw->get<EffortJointInterface>();
    if (!eff_iface)
    {
        ROS_ERROR("This controller requires a hardware interface of type '%s'."
                  " Make sure this is registered in the hardware_interface::RobotHW class.",
                  internal::demangledTypeName<EffortJointInterface>().c_str());
        return false;
    }

    // Return which resources are claimed by this controller
    eff_iface->clearClaims();
    if (!init(eff_iface,
              controller_nh))
    {
        ROS_ERROR("Failed to initialize the controller");
        std::cerr  << "Falided to load effort interfaces" << std::endl;
        return false;
    }
    claimed_resources = eff_iface->getClaims();
    eff_iface->clearClaims();


    //Parse torque constants
    if(!initTorqueConstants(root_nh)){
        ROS_ERROR_STREAM("Failed reading torque constants");
        return false;
    }


    REGISTER_VARIABLE(&Q_act_, "Q_act");
    REGISTER_VARIABLE(&Q_des_, "Q_des");
    REGISTER_VARIABLE(&Qd_act_, "Qd_act");
    REGISTER_VARIABLE(&Qd_des_, "Qd_des");
    REGISTER_VARIABLE(&Tau_cmd_, "Tau_cmd");

    // success
    state_ = INITIALIZED;
    return true;
}

void PalArmRosControlBase::update(const ros::Time& time, const ros::Duration& period)
{

    readJoints();

    RigidBodyDynamics::UpdateKinematicsCustom(rbdl_model_, &Q_act_, &Qd_act_, NULL);
    //Update control loop
    bool safe = this->update_controller(time,period);

    if(!safe)
    {
        ROS_ERROR_STREAM("Error in controller update loop");
        if(this->stopRequest(time))
            ROS_ERROR_STREAM("ERROR: component stopped");
    }

    writeJoints();

    PUBLISH_DEBUG_DATA_TOPIC;
}

void PalArmRosControlBase::starting(const ros::Time& time) {

}

void PalArmRosControlBase::stopping(const ros::Time& time) {
    for(size_t j = 0; j<joints_.size(); ++j){
        Tau_cmd_(j) = 0.0;
    }
    writeJoints();
}

void PalArmRosControlBase::readJoints(){

    for(unsigned int j = 0; j<joints_.size(); ++j){
        bool found = false;
        for(unsigned int i=0; (i < joint_names_.size()) && !found; ++i){
            if(joint_names_[i] == joints_[j].getName()){
                Q_act_(i) = joints_[j].getPosition();
                Qd_act_(i) = joints_[j].getVelocity();
                found = true;
            }
        }
        if(!found){
            std::stringstream ss;
            ss<<"Joint not found in model, cannot read "<<joints_[j].getName()<<std::endl;
            ss<<"Available joints: "<<std::endl;
            for(size_t k=0; k<joint_names_.size(); ++k){
                ss<<"   "<<joint_names_[k]<<std::endl;
            }
            ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
        }
    }

}

void PalArmRosControlBase::writeJoints(){
    if(!embeded_){
        for(unsigned int j = 0; j<joints_.size(); ++j){
            bool found = false;
            for(unsigned int i=0; (i < joint_names_.size()) && !found; ++i){
                if(joint_names_[i] == joints_[j].getName()){
                    joints_[j].setCommand(Tau_cmd_(i));
                    found = true;
                }
            }
            if(!found){
                std::stringstream ss;
                ss<<"Joint not found in model, cannot write simulated model "<<joints_[j].getName()<<std::endl;
                ss<<"Available joints: "<<std::endl;
                for(size_t k=0; k<joint_names_.size(); ++k){
                    ss<<"   "<<joint_names_[k]<<std::endl;
                }
                ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
            }
        }
    }
    else{
        for(unsigned int j = 0; j<joints_.size(); ++j){
            bool found = false;
            for(unsigned int i=0; (i < joint_names_.size()) && !found; ++i){
                if(joint_names_[i] == joints_[j].getName()){
                    //Friction
                    Tau_cmd_(i) += viscous_friction_(j)*Qd_act_(i);
                    if(Qd_act_(i) > 0){
                        Tau_cmd_(i) += static_friction_(j);
                    }
                    else{
                        Tau_cmd_(i) -= static_friction_(j);
                    }

                    joints_[j].setCommand(Tau_cmd_(i)*torque_conversion_(j)*torque_scaling_(j));
                    found = true;
                }
            }
            if(!found){
                std::stringstream ss;
                ss<<"Joint not found in model, cannot write real model "<<joints_[j].getName()<<std::endl;
                ss<<"Available joints: "<<std::endl;
                for(size_t k=0; k<joint_names_.size(); ++k){
                    ss<<"   "<<joint_names_[k]<<std::endl;
                }
                ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
            }
        }
    }
}

