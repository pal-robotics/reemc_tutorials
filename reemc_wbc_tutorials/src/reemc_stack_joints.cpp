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
#include <wbc_tasks/com_kinematic_task.h>
#include <wbc_tasks/com_stabilizer_kinematic_task.h>
#include <wbc_tasks/reference_kinematic_task.h>
#include <wbc_tasks/gaze_kinematic_task.h>
#include <wbc_tasks/gaze_spline_kinematic_task.h>
#include <wbc_tasks/self_collision_kinematic_task.h>
#include <wbc_tasks/self_collision_safety_kinematic_task.h>
#include <pluginlib/class_list_macros.h>

/**
 * The following stack is an example to command the joints directly of the robot, you can
 * set the joints of the robot through dynamic reconfigure while maintaining balance and
 * avoid self collision
 */

using namespace pal_wbc;

class reemc_stack_joints: public StackConfigurationKinematic{
public:

  bool setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh){

    //Joint and velocity limits
    JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
          new JointPositionLimitKinematicAllJointsMetaTask(*stack.get(),
                                                           stack->getJointPositionLimitMin(), stack->getJointPositionLimitMax(),
                                                           stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
                                                           stack->getJointNames(),
                                                           1.0,
                                                           false,
                                                           nh));
    stack->pushTask("joint_limits", joint_position_limit_task);

    GoToPoseMetaTaskPtr left_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_left_6_link", "none", nh));
    GoToPoseMetaTaskPtr right_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_right_6_link", "none", nh));

    task_container_vector constraint_tasks;
    constraint_tasks.push_back({"left_foot_constraint", left_foot_constraint});
    constraint_tasks.push_back({"right_foot_constraint", right_foot_constraint});
    //Constraint both feet
    GenericMetaTaskPtr constraint_metatask(new GenericMetaTask(constraint_tasks, stack->getStateSize()));
    stack->pushTask("constraints", constraint_metatask);
    //Constraint the X-Y coordinates of the COM
    ConstraintFIXC0MMetaTaskPtr com_constraint (new ConstraintFIXC0MMetaTask(*stack.get(), nh) );
    stack->pushTask("com_xy", com_constraint);

    //Avoid self collision
    SelfCollisionSafetyParameters sc_params;
    sc_params.min_distance = 0.08;
    sc_params.influence_distance = 0.08;
    sc_params.epsison = 0.02;
    sc_params.safety_distance = 0;
    sc_params.number_collisions = 10;

    SelfCollisionSafetyKinematicTaskPtr self_collision(new SelfCollisionSafetyKinematicTask);
    self_collision->setUpTask(sc_params, *stack.get(), nh);
    self_collision->setDamping(0.1);
    stack->pushTask("selft_collision", self_collision);

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
    default_reference_joints.push_back("torso_1_joint");
    default_reference_joints.push_back("torso_2_joint");

    Eigen::VectorXd default_reference_posture(default_reference_joints.size());
    default_reference_posture.setZero();
    default_reference_posture(3) = 0.5;
    default_reference_posture(10) = 0.5;
    default_reference_posture(1) = 0.5;
    default_reference_posture(8) = 0.5;
    default_reference_posture(14) = 0.8;
    default_reference_posture(15) = 0.8;
    default_reference_posture(16) = 0.0;
    default_reference_posture(17) = 0.0;
    //Create the last task with to achieve a reference posture
    ReferenceKinematicTaskAllJointsMetaTaskPtr default_reference(
          new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(),
                                                      default_reference_joints,
                                                      default_reference_posture,
                                                      nh, 2.));
    stack->pushTask("default_reference", default_reference);

    return true;
  }
};

PLUGINLIB_EXPORT_CLASS(reemc_stack_joints, StackConfigurationKinematic);
