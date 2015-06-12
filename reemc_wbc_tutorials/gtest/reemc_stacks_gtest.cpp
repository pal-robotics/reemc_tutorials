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

#include "reemc_stacks_gtest.h"

void reemc_wbc_velocity_gtest_stack::setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh){

    eVector3 r; r.setZero();
    eQuaternion E; E.setIdentity();
    right_arm_target = ReferenceAbstract::create("none", nh, "right_arm", "/world", r, E);
    left_arm_target = ReferenceAbstract::create("none", nh, "left_arm", "/world", r, E);
    head_target = ReferenceAbstract::create("none", nh, "head", "/world", r, E);

    std::vector<std::string> joint_names = stack->getJointNames();
    Eigen::VectorXd referece_posture(joint_names.size());
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

    // 1. Joint and velocity limits
    JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
                new JointPositionLimitKinematicAllJointsMetaTask(*stack.get(),
                                                                 stack->getJointPositionLimitMin(), stack->getJointPositionLimitMax(),
                                                                 stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
                                                                 stack->getJointNames(),
                                                                 1.0,
                                                                 false,
                                                                 nh));
    stack->pushTask(joint_position_limit_task);

    // 2. Constraint the COM and left, right foot
    GoToPoseMetaTaskPtr left_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_left_6_link", "none", nh));
    GoToPoseMetaTaskPtr right_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_right_6_link", "none", nh));


    std::vector<TaskAbstractPtr> constraint_tasks;
    constraint_tasks.push_back(left_foot_constraint);
    constraint_tasks.push_back(right_foot_constraint);
    GenericMetaTaskPtr constraint_metatask(new GenericMetaTask(constraint_tasks, stack->getNumberVariables()));
    stack->pushTask(TaskAbstractPtr(constraint_metatask));

    ConstraintFIXC0MMetaTaskPtr com_constraint (new ConstraintFIXC0MMetaTask(*stack.get(), nh));
    stack->pushTask(com_constraint);

    GazePointKinematicMetaTaskPtr gaze_task(new GazePointKinematicMetaTask(*stack.get(), "stereo_optical_frame", head_target, nh));
    stack->pushTask(gaze_task);

    // 4. Position Target Reference for right and left arm
    GoToPositionMetaTaskPtr go_to_position_left_arm(new GoToPositionMetaTask(*stack.get(), "arm_left_7_link", left_arm_target, nh));
    GoToPositionMetaTaskPtr go_to_position_right_arm(new GoToPositionMetaTask(*stack.get(), "arm_right_7_link", right_arm_target, nh));

    std::vector<TaskAbstractPtr> go_to_position_tasks;
    go_to_position_tasks.push_back(go_to_position_left_arm);
    go_to_position_tasks.push_back(go_to_position_right_arm);
    GenericMetaTaskPtr go_to_position_metatasks(new GenericMetaTask(go_to_position_tasks, stack->getNumberVariables()));
    stack->pushTask(TaskAbstractPtr(go_to_position_metatasks));

    GoToOrientationMetaTaskPtr base_link_orientation(new GoToOrientationMetaTask(*stack.get(), "base_link", "none", nh));
    //stack->pushTask(base_link_orientation);

    ReferenceKinematicTaskAllJointsMetaTaskPtr reference(new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(),
                                                                                                     joint_names,
                                                                                                     referece_posture,
                                                                                                     nh));
   stack->pushTask(reference);
}


