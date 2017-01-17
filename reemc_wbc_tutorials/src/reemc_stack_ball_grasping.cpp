/*
* Software License Agreement (Modified BSD License)
*
* Copyright (c) 2014, 2015 PAL Robotics, S.L.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of PAL Robotics, S.L. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/*
* Author: Hilario Tomé
*/

#include <Eigen/Dense>
#include <pal_wbc_controller/StackOfTasksKinematic.h>
#include <pal_wbc_controller/generic_meta_task.h>

#include <wbc_tasks/joint_pos_limit_kinematic_task.h>
#include <wbc_tasks/constraint_kinematic_task.h>
#include <wbc_tasks/go_to_kinematic_task.h>
#include <wbc_tasks/go_to_spline_kinematic_task.h>
#include <wbc_tasks/go_to_relative_kinematic_task.h>
#include <wbc_tasks/com_kinematic_task.h>
#include <wbc_tasks/com_stabilizer_kinematic_task.h>
#include <wbc_tasks/reference_kinematic_task.h>
#include <wbc_tasks/gaze_kinematic_task.h>
#include <wbc_tasks/gaze_spline_kinematic_task.h>
#include <wbc_tasks/self_collision_kinematic_task.h>
#include <wbc_tasks/self_collision_safety_kinematic_task.h>
#include <pluginlib/class_list_macros.h>

/**
 * This stack gives and example of how to create a goto task for only one arm and leave a reference configuration
 * in the other arm. The idea is to emulate the robot grasping a ball
 */

using namespace pal_wbc;

class reemc_ball_grasping_stack: public StackConfigurationKinematic{
public:

  bool setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh){

    std::vector<std::string> default_reference_joints;
    default_reference_joints.push_back("arm_left_1_joint");
    default_reference_joints.push_back("arm_left_2_joint");
    default_reference_joints.push_back("arm_left_3_joint");
    default_reference_joints.push_back("arm_left_4_joint");
    default_reference_joints.push_back("arm_left_5_joint");
    default_reference_joints.push_back("arm_left_6_joint");
    default_reference_joints.push_back("arm_left_7_joint");
    default_reference_joints.push_back("arm_right_1_joint");
    default_reference_joints.push_back("arm_right_2_joint");
    default_reference_joints.push_back("arm_right_3_joint");
    default_reference_joints.push_back("arm_right_4_joint");
    default_reference_joints.push_back("arm_right_5_joint");
    default_reference_joints.push_back("arm_right_6_joint");
    default_reference_joints.push_back("arm_right_7_joint");
    default_reference_joints.push_back("leg_left_4_joint");
    default_reference_joints.push_back("leg_right_4_joint");

    Eigen::VectorXd default_reference_posture(default_reference_joints.size());
    default_reference_posture.setZero();
    default_reference_posture(3) = 0.5;
    default_reference_posture(10) = 0.5;
    default_reference_posture(1) = 0.5;
    default_reference_posture(8) = 0.5;
    default_reference_posture(14) = 0.8;
    default_reference_posture(15) = 0.8;

    std::vector< double > joint_pos_min_override = stack->getJointPositionLimitMin();

    joint_pos_min_override[stack->getJointIndex("torso_1_joint")] = -1.10;
    joint_pos_min_override[stack->getJointIndex("torso_2_joint")] = -0.2;
    joint_pos_min_override[stack->getJointIndex("leg_left_3_joint")] = -1.6;
    joint_pos_min_override[stack->getJointIndex("leg_right_3_joint")] = -1.6;
    joint_pos_min_override[stack->getJointIndex("leg_left_4_joint")] = 0.3;
    joint_pos_min_override[stack->getJointIndex("leg_right_4_joint")] = 0.3;
    joint_pos_min_override[stack->getJointIndex("leg_left_5_joint")] = -1.4;
    joint_pos_min_override[stack->getJointIndex("leg_right_5_joint")] = -1.4;

    joint_pos_min_override[stack->getJointIndex("arm_left_2_joint")] = 0.2;
    joint_pos_min_override[stack->getJointIndex("arm_right_2_joint")] = 0.2;
    joint_pos_min_override[stack->getJointIndex("arm_left_4_joint")] = 0.2;
    joint_pos_min_override[stack->getJointIndex("arm_right_4_joint")] = 0.2;

    std::vector< double > joint_pos_max_override = stack->getJointPositionLimitMax();
    joint_pos_max_override[stack->getJointIndex("torso_1_joint")] = 1.10;
    joint_pos_max_override[stack->getJointIndex("torso_2_joint")] = 0.5;
    joint_pos_max_override[stack->getJointIndex("arm_left_3_joint")] = 0.8;
    joint_pos_max_override[stack->getJointIndex("arm_right_3_joint")] = 0.8;
    joint_pos_max_override[stack->getJointIndex("leg_left_4_joint")] = 2.0;
    joint_pos_max_override[stack->getJointIndex("leg_right_4_joint")] = 2.0;


    // Joint and velocity limits
    JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
          new JointPositionLimitKinematicAllJointsMetaTask(*stack.get(),
                                                           joint_pos_min_override, joint_pos_max_override,
                                                           stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
                                                           stack->getJointNames(),
                                                           0.5,
                                                           false,
                                                           nh));
    stack->pushTask(joint_position_limit_task);

    // Constraint the COM and left, right foot
    GoToPoseMetaTaskPtr left_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_left_6_link", "topic", nh));
    GoToPoseMetaTaskPtr right_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_right_6_link", "topic", nh));


    std::vector<TaskAbstractPtr> constraint_tasks;
    constraint_tasks.push_back(left_foot_constraint);
    constraint_tasks.push_back(right_foot_constraint);
    GenericMetaTaskPtr constraint_metatask(new GenericMetaTask(constraint_tasks, stack->getStateSize()));
    stack->pushTask(TaskAbstractPtr(constraint_metatask));

    assert(fts_.size() == 2);
    ConstraintStabilizedFIXC0MMetaTaskPtr com_constraint (
          new ConstraintStabilizedFIXC0MMetaTask(*stack.get(),
                                                 fts_[0],
          fts_[1],
        nh) );
    stack->pushTask(com_constraint);

    // Self collision
    SelfCollisionSafetyParameters sc_params;
    sc_params.min_distance = 0.08;
    sc_params.influence_distance = 0.08;
    sc_params.epsison = 0.01;
    sc_params.safety_distance = 0;
    sc_params.number_collisions = 10;
    SelfCollisionSafetyKinematicTaskPtr self_collision(new SelfCollisionSafetyKinematicTask());
    self_collision->setUpTask(sc_params, *stack.get(), nh);
    self_collision->setDamping(0.1);
    stack->pushTask(self_collision);

    GoToOrientationMetaTaskPtr base_link_orientation(
          new GoToOrientationMetaTask(*stack.get(), "base_link", "topic", nh));
    base_link_orientation->setDamping(0.1);
    stack->pushTask(base_link_orientation);

    GazeSplinePointKinematicMetaTaskPtr gaze_task(
          new GazeSplinePointKinematicMetaTask(*stack.get(), "stereo_optical_frame", "topic", nh));
    gaze_task->setDamping(0.1);
    stack->pushTask(gaze_task);

    std::vector<std::string> default_reference_left_arm;
    default_reference_left_arm.push_back("arm_left_1_joint");
    default_reference_left_arm.push_back("arm_left_2_joint");
    default_reference_left_arm.push_back("arm_left_3_joint");
    default_reference_left_arm.push_back("arm_left_4_joint");
    default_reference_left_arm.push_back("arm_left_5_joint");
    default_reference_left_arm.push_back("arm_left_6_joint");
    default_reference_left_arm.push_back("arm_left_7_joint");
    Eigen::VectorXd default_reference_left_arm_posture(default_reference_left_arm.size()); default_reference_left_arm_posture.setZero();
    default_reference_left_arm_posture(1) = 0.5;
    default_reference_left_arm_posture(3) = 0.5;
    ReferenceKinematicTaskAllJointsMetaTaskPtr left_arm_reference_posture(
          new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(),
                                                                                                                      default_reference_left_arm,
                                                                                                                      default_reference_left_arm_posture,
                                                                                                                      nh, 2.));
    left_arm_reference_posture->setDamping(0.1);

    GoToSplinePositionMetaTaskPtr go_to_position_right_arm(
          new GoToSplinePositionMetaTask(*stack.get(), "arm_right_7_link", "topic", nh));

    std::vector<TaskAbstractPtr> go_to_position_tasks;
    go_to_position_tasks.push_back(go_to_position_right_arm);
    go_to_position_tasks.push_back(left_arm_reference_posture);
    GenericMetaTaskPtr go_to_position_metatasks(new GenericMetaTask(go_to_position_tasks, stack->getStateSize()));
    go_to_position_metatasks->setDamping(0.1);
    stack->pushTask(TaskAbstractPtr(go_to_position_metatasks));

    //Reference configurations, be aware that it seems that the task*nullspaces jacobians are also bad conditioned and need damping
    //First torso + left arm
    //Rest of reference joints

    std::vector<std::string> torso_reference_joints;
    torso_reference_joints.push_back("torso_1_joint");
    torso_reference_joints.push_back("torso_2_joint");

    Eigen::VectorXd torso_posture(torso_reference_joints.size());
    torso_posture.setZero();

    ReferenceKinematicTaskAllJointsMetaTaskPtr torso_reference_task(
          new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(),
                                                      torso_reference_joints,
                                                      torso_posture,
                                                      nh, 2.0));
    torso_reference_task->setDamping(5.0);

    stack->pushTask(torso_reference_task);


    ReferenceKinematicTaskAllJointsMetaTaskPtr default_reference(
          new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(),
                                                      default_reference_joints,
                                                      default_reference_posture,
                                                      nh, 2.0));
    default_reference->setDamping(5.0);
    stack->pushTask(default_reference);

    return true;
  }
};

PLUGINLIB_EXPORT_CLASS(reemc_ball_grasping_stack, StackConfigurationKinematic);

