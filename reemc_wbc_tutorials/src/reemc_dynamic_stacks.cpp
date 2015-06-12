#include <Eigen/Dense>
#include <pal_wbc_controller/task_abstract.h>
#include <pal_wbc_controller/StackOfTasksDynamic.h>
#include <pal_wbc_controller/generic_meta_task.h>

#include <wbc_tasks/constraint_dynamic_task.h>
#include <wbc_tasks/go_to_dynamic_task.h>
#include <wbc_tasks/torque_damping_dynamic_task.h>
#include <wbc_tasks/physics_constraint_dynamic_task.h>
#include <wbc_tasks/com_dynamic_task.h>
#include <wbc_tasks/reference_dynamic_posture.h>
#include <wbc_tasks/torque_limit_dynamic_task.h>
#include <wbc_tasks/unilateral_forces_dynamic.h>
#include <wbc_tasks/cop_dynamic.h>
#include <wbc_tasks/cop_dynamic.h>
#include <pluginlib/class_list_macros.h>

void setUpPhysicsContactForceTorque(StackOfTasksDynamicPtr stack, ros::NodeHandle &nh){
  ROS_ERROR_STREAM("Not implemented");
  assert(false);

}

void setUpPhysicsContactForceOnly(StackOfTasksDynamicPtr stack, ros::NodeHandle &nh){
  std::cerr<<"Setting up physics"<<std::endl;
  std::vector<TaskAbstractPtr> constraint_tasks;

  std::vector<Contact> force_constraints = stack->getContactForceDescription();
  std::vector<Contact> torque_constraints = stack->getContactTorqueDescriptoin();

  //Unilateral contact foces == ZMP in flat terrain
  UnilateralForcesDynamicsPtr unilateral_forces(new UnilateralForcesDynamics() );
  unilateral_forces->setUpTask(*stack.get(), nh);
  //constraint_tasks.push_back(unilateral_forces);

  PhysicsConstraintTaskPtr dynamic_task(new PhysicsConstraintTask() );
  PhysicsConstraintTask::PhysicsConstraintParam physics_param;

  for(unsigned int i=0; i<force_constraints.size(); ++i){
    physics_param.constraint_force_.push_back(force_constraints[i]);
  }
  for(unsigned int i=0; i<torque_constraints.size(); ++i){
    physics_param.constraint_torque_.push_back(torque_constraints[i]);
  }
  dynamic_task->setUpTask(physics_param, *stack.get(), nh);
  constraint_tasks.push_back(dynamic_task);

  // We will get the contact configuration to setup the physics and constraints
  // from what is stored in the stack

  if(force_constraints.size()>0){
    for(unsigned int i=0; i<force_constraints.size(); ++i){
      constraint_tasks.push_back(TaskAbstractPtr(
                                   new ConstraintDynamicPositionMetaTask(*stack.get(), force_constraints[i].first,
                                                                         force_constraints[i].second, nh)) );
    }
  }

  if(torque_constraints.size()>0){
    for(unsigned int i=0; i<torque_constraints.size(); ++i){
      constraint_tasks.push_back(TaskAbstractPtr(new ConstraintDynamicOrientationMetaTask(
                                                   *stack.get(), torque_constraints[i].first,
                                                   torque_constraints[i].second, nh)) );
    }
  }

  /*
  if(force_constraints.size()>0){
    for(unsigned int i=0; i<force_constraints.size(); ++i){
      constraint_tasks.push_back(TaskAbstractPtr(
                                   new GoToPositionDynamicMetaTask(*stack.get(), force_constraints[i].first, "none",
                                                                         force_constraints[i].second, nh)) );
    }
  }

  if(torque_constraints.size()>0){
    for(unsigned int i=0; i<torque_constraints.size(); ++i){
      constraint_tasks.push_back(TaskAbstractPtr(new GoToOrientationDynamicMetaTask(
                                                   *stack.get(), torque_constraints[i].first, "none", nh)) );
    }
  }
  */

  GenericMetaTaskPtr constraint_metatask(new GenericMetaTask(constraint_tasks, stack->getNumberVariables()) );
  stack->pushTask(TaskAbstractPtr(constraint_metatask));

}

class reemc_dynamic_stack1: public StackConfigurationDynamic{

  void setupStack(StackOfTasksDynamicPtr stack, ros::NodeHandle &nh){

    std::vector<std::string> joint_names = stack->getJointNames();
    Eigen::VectorXd referece_posture(stack->getNumberDof() - 6);
    referece_posture.setZero();

    /// @todo Fix the way we get joint indexes
    referece_posture(stack->getJointIndex("leg_left_3_joint")) = -0.4;
    referece_posture(stack->getJointIndex("leg_right_3_joint")) = -0.4;
    referece_posture(stack->getJointIndex("leg_left_4_joint")) = 0.8;
    referece_posture(stack->getJointIndex("leg_right_4_joint")) = 0.8;
    referece_posture(stack->getJointIndex("leg_left_5_joint")) = -0.4;
    referece_posture(stack->getJointIndex("leg_right_5_joint")) = -0.4;
    referece_posture(stack->getJointIndex("arm_left_4_joint")) = 0.4;
    referece_posture(stack->getJointIndex("arm_right_4_joint")) = 0.4;
    referece_posture(stack->getJointIndex("arm_left_2_joint")) = 0.3;
    referece_posture(stack->getJointIndex("arm_right_2_joint")) = 0.3;

    setUpPhysicsContactForceOnly(stack, nh);

    ConstraintFIXC0MDynamicMetaTaskPtr com_constraint (new ConstraintFIXC0MDynamicMetaTask(*stack.get(), nh) );
    //TorqueDampingDynamicTaskAllJointsMetaTaskPtr damping_task(new TorqueDampingDynamicTaskAllJointsMetaTask(*stack.get(), joint_names, nh) );
    ReferenceDynamicPostureTaskAllJointsMetaTaskPtr reference_task(new ReferenceDynamicPostureTaskAllJointsMetaTask(*stack.get(), joint_names, referece_posture, nh) );

    std::vector<TaskAbstractPtr> constraint_tasks;
    //constraint_tasks.push_back(com_constraint);
    GenericMetaTaskPtr constraint_metatask(new GenericMetaTask(constraint_tasks, stack->getNumberVariables()) );
    //stack->pushTask(TaskAbstractPtr(constraint_metatask));


    // 4. Position Target Reference for right and left arm
    GoToPositionDynamicMetaTaskPtr go_to_position_left_arm(
          new GoToPositionDynamicMetaTask(*stack.get(), "arm_left_7_link", "interactive_marker", Eigen::Vector3d::Zero(), nh) );

    GoToPositionDynamicMetaTaskPtr go_to_position_right_arm(
          new GoToPositionDynamicMetaTask(*stack.get(), "arm_right_7_link","interactive_marker", Eigen::Vector3d::Zero(), nh) );

    //GoToPositionDynamicMetaTaskPtr go_to_head(new GoToPositionDynamicMetaTask(&stack, "head_2_link"));

    GoToPositionDynamicMetaTaskPtr go_to_base_link(
          new GoToPositionDynamicMetaTask(*stack.get(), "base_link","interactive_marker", Eigen::Vector3d::Zero(), nh) );

    std::vector<TaskAbstractPtr> go_to_position_tasks;
    go_to_position_tasks.push_back(go_to_position_left_arm);
    go_to_position_tasks.push_back(go_to_position_right_arm);
    //go_to_position_tasks.push_back(go_to_base_link);
    //go_to_position_tasks.push_back(damping);
    GenericMetaTaskPtr go_to_position_metatasks(new GenericMetaTask(go_to_position_tasks, stack->getNumberVariables()) );
    stack->pushTask(TaskAbstractPtr(go_to_position_metatasks));

    // 6. Try to keep COM Z up
    ConstraintFIXC0MHeightDynamicMetaTaskPtr com_height_constraint (new ConstraintFIXC0MHeightDynamicMetaTask(*stack.get(), nh) );
    // stack.pushTask(TaskAbstractPtr(com_height_constraint));

    // 7. Fix orientation waist and base_link
    ConstraintDynamicOrientationMetaTaskPtr base_orientation_constraint(
          new  ConstraintDynamicOrientationMetaTask(*stack.get(), "base_link", Eigen::Vector3d::Zero(), nh) );

    ConstraintDynamicOrientationMetaTaskPtr trunk_orientation_constraint(
          new ConstraintDynamicOrientationMetaTask(*stack.get(), "torso_2_link",Eigen::Vector3d::Zero(), nh) );

    std::vector<TaskAbstractPtr> orientation_metatask_constraints;
    orientation_metatask_constraints.push_back(trunk_orientation_constraint);
    orientation_metatask_constraints.push_back(base_orientation_constraint);

    GenericMetaTaskPtr orientation_metatasks(new GenericMetaTask(orientation_metatask_constraints, stack->getNumberVariables()) );
    //stack.pushTask(orientation_metatasks);

    // 7. Damping task
    std::vector<TaskAbstractPtr> damping_tasks;
    // damping_tasks.push_back(damping_task);
    // damping_tasks.push_back(reference_task);
    //go_to_position_tasks.push_back(damping);
    GenericMetaTaskPtr damping_metatask(new GenericMetaTask(damping_tasks, stack->getNumberVariables()) );
    //stack.pushTask(damping_metatask);
    stack->pushTask(reference_task);
    //stack.pushTask(damping_task);

  }
};

PLUGINLIB_EXPORT_CLASS(reemc_dynamic_stack1, StackConfigurationDynamic);


/*
class reemc_dynamic_stack2: public StackConfiguration{
  void setupStack(StackOfTasksPtr stack){

    std::vector<std::string> joint_names = stack->getJointNames();
    RigidBodyDynamics::Model &rbdl_model = stack->rbdl_model_;
    Eigen::VectorXd referece_posture(rbdl_model.dof_count - 6);
    referece_posture.setZero();

    /// @todo Fix the way we get joint indexes
    int id;
    id = rbdl_model.GetBodyId("leg_left_3_link");
    referece_posture(id -1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("leg_right_3_link");
    referece_posture(id -1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("leg_left_4_link");
    referece_posture(id - 1 - 6) = 0.8;
    id = rbdl_model.GetBodyId("leg_right_4_link");
    referece_posture(id - 1 - 6 ) = 0.8;
    id = rbdl_model.GetBodyId("leg_left_5_link");
    referece_posture(id -1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("leg_right_5_link");
    referece_posture(id - 1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("arm_left_4_link");
    referece_posture(id - 1 - 6 ) = 0.4;
    id = rbdl_model.GetBodyId("arm_right_4_link");
    referece_posture(id - 1 - 6 ) = 0.4;
    id = rbdl_model.GetBodyId("arm_left_2_link");
    referece_posture(id - 1 - 6 ) = 0.3;
    id = rbdl_model.GetBodyId("arm_right_2_link");
    referece_posture(id - 1 - 6) = 0.3;

    // 2. Constraint the COM and left, right foot and Dynamics
    TorqueLimitDynamicAllJointsMetaTaskPtr torque_limits(new TorqueLimitDynamicAllJointsMetaTask(stack.get(), stack->joint_max_effort, joint_names));

    setUpPhysics(stack);

    ConstraintFIXC0MDynamicMetaTaskPtr com_constraint (new ConstraintFIXC0MDynamicMetaTask(stack.get()));
    ReferenceDynamicPostureTaskAllJointsMetaTaskPtr reference_task(new ReferenceDynamicPostureTaskAllJointsMetaTask(stack.get(), joint_names, referece_posture) );

    std::vector<TaskAbstractPtr> constraint_tasks;
    constraint_tasks.push_back(com_constraint);
    //constraint_tasks.push_back(torque_limits);
    GenericMetaTaskPtr constraint_metatask(new GenericMetaTask(stack.get(), constraint_tasks));
    stack->pushTask(TaskAbstractPtr(constraint_metatask));

    // 4. Position Target Reference for right and left arm
    GoToPositionDynamicMetaTaskPtr go_to_position_left_arm(new GoToPositionDynamicMetaTask(stack.get(), "arm_left_7_link","interactive_marker"));
    GoToPositionDynamicMetaTaskPtr go_to_position_right_arm(new GoToPositionDynamicMetaTask(stack.get(), "arm_right_7_link","interactive_marker"));
    //GoToPositionDynamicMetaTaskPtr go_to_head(new GoToPositionDynamicMetaTask(&stack, "head_2_link"));
    GoToPositionDynamicMetaTaskPtr go_to_base_link(new GoToPositionDynamicMetaTask(stack.get(), "base_link", "interactive_marker"));

    std::vector<TaskAbstractPtr> go_to_position_tasks;
    go_to_position_tasks.push_back(go_to_position_left_arm);
    go_to_position_tasks.push_back(go_to_position_right_arm);
    //go_to_position_tasks.push_back(go_to_base_link);
    //go_to_position_tasks.push_back(damping);
    GenericMetaTaskPtr go_to_position_metatasks(new GenericMetaTask(stack.get(), go_to_position_tasks));
    stack->pushTask(TaskAbstractPtr(go_to_position_metatasks));

    // 6. Try to keep COM Z up
    ConstraintFIXC0MHeightDynamicMetaTaskPtr com_height_constraint (new ConstraintFIXC0MHeightDynamicMetaTask(stack.get()));
    // stack.pushTask(TaskAbstractPtr(com_height_constraint));

    // 7. Fix orientation waist and base_link
    ConstraintDynamicOrientationMetaTaskPtr base_orientation_constraint(new  ConstraintDynamicOrientationMetaTask(stack.get(), "base_link"));
    ConstraintDynamicOrientationMetaTaskPtr trunk_orientation_constraint(new ConstraintDynamicOrientationMetaTask(stack.get(), "torso_2_link"));
    std::vector<TaskAbstractPtr> orientation_metatask_constraints;
    orientation_metatask_constraints.push_back(trunk_orientation_constraint);
    orientation_metatask_constraints.push_back(base_orientation_constraint);
    GenericMetaTaskPtr orientation_metatasks(new GenericMetaTask(stack.get(), orientation_metatask_constraints));
    //stack.pushTask(orientation_metatasks);

    // 7. Damping task
    std::vector<TaskAbstractPtr> damping_tasks;
    // damping_tasks.push_back(damping_task);
    // damping_tasks.push_back(reference_task);
    //go_to_position_tasks.push_back(damping);
    GenericMetaTaskPtr damping_metatask(new GenericMetaTask(stack.get(), damping_tasks));
    //stack.pushTask(damping_metatask);
    stack->pushTask(reference_task);
    //stack.pushTask(damping_task);

  }
};
COMPONENT_REGISTER(StackConfiguration, reemc_dynamic_stack2, "reemc_dynamic_stack2")

//Taken from the kinematic stack that works at acceleration level
class reemc_dynamic_stack3: public StackConfiguration{
  void setupStack(StackOfTasksPtr stack){

    std::vector<std::string> joint_names = stack->getJointNames();
    RigidBodyDynamics::Model &rbdl_model = stack->rbdl_model_;
    Eigen::VectorXd referece_posture(rbdl_model.dof_count - 6);
    referece_posture.setZero();

    /// @todo Fix the way we get joint indexes
    int id;
    id = rbdl_model.GetBodyId("leg_left_3_link");
    referece_posture(id -1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("leg_right_3_link");
    referece_posture(id -1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("leg_left_4_link");
    referece_posture(id - 1 - 6) = 0.8;
    id = rbdl_model.GetBodyId("leg_right_4_link");
    referece_posture(id - 1 - 6 ) = 0.8;
    id = rbdl_model.GetBodyId("leg_left_5_link");
    referece_posture(id -1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("leg_right_5_link");
    referece_posture(id - 1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("arm_left_4_link");
    referece_posture(id - 1 - 6 ) = 0.4;
    id = rbdl_model.GetBodyId("arm_right_4_link");
    referece_posture(id - 1 - 6 ) = 0.4;
    id = rbdl_model.GetBodyId("arm_left_2_link");
    referece_posture(id - 1 - 6 ) = 0.3;
    id = rbdl_model.GetBodyId("arm_right_2_link");
    referece_posture(id - 1 - 6) = 0.3;


    // 0. Constraint the COM and left, right foot and Dynamics
    TorqueLimitDynamicAllJointsMetaTaskPtr torque_limits(
          new TorqueLimitDynamicAllJointsMetaTask(stack.get(), stack->joint_max_effort, joint_names));

    setUpPhysics(stack);

//
//  // 1. Joint and velocity limits
//  JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
//        new JointPositionLimitKinematicAllJointsMetaTask(stack.get(),
//                                                         stack->joint_position_min, stack->joint_positoin_max,
//                                                         stack->joint_vel_min, stack->joint_vel_max,
//                                                         stack->joint_names));
//  stack->pushTask(joint_position_limit_task);
//

    ConstraintFIXC0MDynamicMetaTaskPtr com_constraint (new ConstraintFIXC0MDynamicMetaTask(stack.get()));
    stack->pushTask(com_constraint);

    // 4. Position Target Reference for right and left arm
    GoToPositionDynamicMetaTaskPtr go_to_position_left_arm(new GoToPositionDynamicMetaTask(stack.get(), "arm_left_7_link", "interactive_marker"));
    GoToPositionDynamicMetaTaskPtr go_to_position_right_arm(new GoToPositionDynamicMetaTask(stack.get(), "arm_right_7_link", "interactive_marker"));

    std::vector<TaskAbstractPtr> go_to_position_tasks;
    go_to_position_tasks.push_back(go_to_position_left_arm);
    go_to_position_tasks.push_back(go_to_position_right_arm);
    GenericMetaTaskPtr go_to_position_metatasks(new GenericMetaTask(stack.get(), go_to_position_tasks));
    stack->pushTask(TaskAbstractPtr(go_to_position_metatasks));

    GoToOrientationDynamicMetaTaskPtr base_link_orientation(new GoToOrientationDynamicMetaTask(stack.get(), "base_link", "interactive_marker"));
    stack->pushTask(base_link_orientation);
    GoToOrientationDynamicMetaTaskPtr torso_orientation(new GoToOrientationDynamicMetaTask(stack.get(), "torso_2_link", "interactive_marker"));
    stack->pushTask(torso_orientation);

    ReferenceDynamicPostureTaskAllJointsMetaTaskPtr reference_task(new ReferenceDynamicPostureTaskAllJointsMetaTask(stack.get(),
                                                                                                                    joint_names,
                                                                                                                    referece_posture));
    stack->pushTask(reference_task);

  }
};
COMPONENT_REGISTER(StackConfiguration, reemc_dynamic_stack3, "reemc_dynamic_stack3")

//Taken from the kinematic stack that works at acceleration level
class reemc_dynamic_friction_stack: public StackConfiguration{
  void setupStack(StackOfTasksPtr stack){

    std::vector<std::string> joint_names = stack->getJointNames();
    RigidBodyDynamics::Model &rbdl_model = stack->rbdl_model_;
    Eigen::VectorXd referece_posture(rbdl_model.dof_count - 6);
    referece_posture.setZero();

    /// @todo Fix the way we get joint indexes
    int id;
    id = rbdl_model.GetBodyId("leg_left_3_link");
    referece_posture(id -1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("leg_right_3_link");
    referece_posture(id -1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("leg_left_4_link");
    referece_posture(id - 1 - 6) = 0.8;
    id = rbdl_model.GetBodyId("leg_right_4_link");
    referece_posture(id - 1 - 6 ) = 0.8;
    id = rbdl_model.GetBodyId("leg_left_5_link");
    referece_posture(id -1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("leg_right_5_link");
    referece_posture(id - 1 - 6) = -0.4;
    id = rbdl_model.GetBodyId("arm_left_4_link");
    referece_posture(id - 1 - 6 ) = 0.4;
    id = rbdl_model.GetBodyId("arm_right_4_link");
    referece_posture(id - 1 - 6 ) = 0.4;
    id = rbdl_model.GetBodyId("arm_left_2_link");
    referece_posture(id - 1 - 6 ) = 0.3;
    id = rbdl_model.GetBodyId("arm_right_2_link");
    referece_posture(id - 1 - 6) = 0.3;

    // 0. Constraint the COM and left, right foot and Dynamics
    TorqueLimitDynamicAllJointsMetaTaskPtr torque_limits(
          new TorqueLimitDynamicAllJointsMetaTask(stack.get(), stack->joint_max_effort, joint_names));

    setUpPhysics(stack);

    ConstraintFIXC0MDynamicMetaTaskPtr com_constraint (new ConstraintFIXC0MDynamicMetaTask(stack.get()));
    stack->pushTask(com_constraint);

    // 4. Position Target Reference for right and left arm
    GoToPositionDynamicMetaTaskPtr go_to_position_left_arm(new GoToPositionDynamicMetaTask(stack.get(), "arm_left_7_link", "interactive_marker"));
    GoToPositionDynamicMetaTaskPtr go_to_position_right_arm(new GoToPositionDynamicMetaTask(stack.get(), "arm_right_7_link", "interactive_marker"));

    std::vector<TaskAbstractPtr> go_to_position_tasks;
    go_to_position_tasks.push_back(go_to_position_left_arm);
    go_to_position_tasks.push_back(go_to_position_right_arm);

    GenericMetaTaskPtr go_to_position_metatasks(new GenericMetaTask(stack.get(), go_to_position_tasks));
    stack->pushTask(TaskAbstractPtr(go_to_position_metatasks));

    GoToOrientationDynamicMetaTaskPtr base_link_orientation(new GoToOrientationDynamicMetaTask(stack.get(), "base_link", "interactive_marker"));
    stack->pushTask(base_link_orientation);
    GoToOrientationDynamicMetaTaskPtr torso_orientation(new GoToOrientationDynamicMetaTask(stack.get(), "torso_2_link", "interactive_marker"));
    stack->pushTask(torso_orientation);

    ReferenceDynamicPostureTaskAllJointsMetaTaskPtr reference_task(new ReferenceDynamicPostureTaskAllJointsMetaTask(stack.get(),
                                                                                                                    joint_names,
                                                                                                                    referece_posture));
    stack->pushTask(reference_task);

  }
};

COMPONENT_REGISTER(StackConfiguration, reemc_dynamic_friction_stack, "reemc_dynamic_friction_stack")
*/

class reemc_dynamic_reduced_test: public StackConfigurationDynamic{
  void setupStack(StackOfTasksDynamicPtr stack, ros::NodeHandle &nh){

    std::vector<std::string> joint_names = stack->getJointNames();
    Eigen::VectorXd referece_posture(stack->getNumberDof() - 6);
    referece_posture.setZero();

    referece_posture(stack->getJointIndex("leg_left_3_joint")) = -0.4;
    referece_posture(stack->getJointIndex("leg_right_3_joint")) = -0.4;
    referece_posture(stack->getJointIndex("leg_left_4_joint")) = 0.8;
    referece_posture(stack->getJointIndex("leg_right_4_joint")) = 0.8;
    referece_posture(stack->getJointIndex("leg_left_5_joint")) = -0.4;
    referece_posture(stack->getJointIndex("leg_right_5_joint")) = -0.4;
    referece_posture(stack->getJointIndex("arm_left_4_joint")) = 0.4;
    referece_posture(stack->getJointIndex("arm_right_4_joint")) = 0.4;
    referece_posture(stack->getJointIndex("arm_left_2_joint")) = 0.3;
    referece_posture(stack->getJointIndex("arm_right_2_joint")) = 0.3;

    //    // 0. Constraint the COM and left, right foot and Dynamics
    //    TorqueLimitDynamicAllJointsMetaTaskPtr torque_limits(
    //          new TorqueLimitDynamicAllJointsMetaTask(stack.get(), stack->joint_max_effort, joint_names));

    setUpPhysicsContactForceOnly(stack, nh);

    /*
    //ConstraintFIXC0MDynamicMetaTaskPtr com_constraint (new ConstraintFIXC0MDynamicMetaTask(stack.get()));
    GoToCOMtDynamicMetaTaskPtr com_constraint (new GoToCOMtDynamicMetaTask(*stack.get(), "interactive_marker", nh));
    stack->pushTask(com_constraint);

    // 4. Position Target Reference for right and left arm
    GoToPositionDynamicMetaTaskPtr go_to_position_left_arm(
          new GoToPositionDynamicMetaTask(*stack.get(), "arm_left_7_link", "interactive_marker", Eigen::Vector3d::Zero(), nh));
    GoToPositionDynamicMetaTaskPtr go_to_position_right_arm(
          new GoToPositionDynamicMetaTask(*stack.get(), "arm_right_7_link", "interactive_marker", Eigen::Vector3d::Zero(), nh));

    std::vector<TaskAbstractPtr> go_to_position_tasks;
    go_to_position_tasks.push_back(go_to_position_left_arm);
    go_to_position_tasks.push_back(go_to_position_right_arm);
    GenericMetaTaskPtr go_to_position_metatasks(
          new GenericMetaTask(go_to_position_tasks, stack->getNumberVariables()) );
    //stack->pushTask(TaskAbstractPtr(go_to_position_metatasks));

    */

//    GoToOrientationDynamicMetaTaskPtr base_link_orientation(
//          new GoToOrientationDynamicMetaTask(*stack.get(), "base_link", "interactive_marker", nh));
//    //stack->pushTask(base_link_orientation);

//    GoToOrientationDynamicMetaTaskPtr torso_orientation(
//          new GoToOrientationDynamicMetaTask(*stack.get(), "torso_2_link", "interactive_marker", nh));
//    //stack->pushTask(torso_orientation);


    //COM
    //ConstraintFIXC0MDynamicMetaTaskPtr com_constraint (new ConstraintFIXC0MDynamicMetaTask(*stack.get(), nh) );
//    GoToCOMtDynamicMetaTaskPtr com_constraint (new GoToCOMtDynamicMetaTask(*stack.get(), "interactive_marker", nh));
//    stack->pushTask(com_constraint);

//    // COP
//    COPTranckingDynamicMetaTaskPtr cop_tracking(new COPTranckingDynamicMetaTask(*stack.get(), "interactive_marker", nh));
//    stack->pushTask(TaskAbstractPtr(cop_tracking));

//    std::vector<TaskAbstractPtr> balance_tasks;
//    balance_tasks.push_back(com_constraint);
//  //  balance_tasks.push_back(cop_tracking);
//    GenericMetaTaskPtr balance_metatasks(
//          new GenericMetaTask(balance_tasks, stack->getNumberVariables()) );
//    //stack->pushTask(TaskAbstractPtr(balance_metatasks));


    ReferenceDynamicPostureTaskAllJointsMetaTaskPtr reference_task(
          new ReferenceDynamicPostureTaskAllJointsMetaTask(*stack.get(),
                                                           joint_names,
                                                           referece_posture,
                                                           nh));
    stack->pushTask(reference_task);

  }
};

PLUGINLIB_EXPORT_CLASS(reemc_dynamic_reduced_test, StackConfigurationDynamic);

