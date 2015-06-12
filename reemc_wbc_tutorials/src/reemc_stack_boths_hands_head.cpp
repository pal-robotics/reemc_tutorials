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
#include <wbc_tasks/momentum_kinematic_task.h>
#include <wbc_tasks/reference_kinematic_task.h>
#include <wbc_tasks/gaze_kinematic_task.h>
#include <wbc_tasks/gaze_spline_kinematic_task.h>
#include <wbc_tasks/self_collision_kinematic_task.h>
#include <wbc_tasks/self_collision_safety_kinematic_task.h>
#include <pluginlib/class_list_macros.h>

/**
 * The reemc_stack_both_hands_head class is an example class to command the gaze and hands of the robot.
 * It shows how to use the interactive marker reference instead of the topic. Be aware that using
 * interactive markers makes this controller not real time, to make it real time, they should be changed by
 * topics and the commands to the task should be sent through the topics.
 */

class reemc_stack_both_hands_head: public StackConfigurationKinematic{
public:

    void setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh){

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

        //Joint and velocity limits
        JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
                    new JointPositionLimitKinematicAllJointsMetaTask(*stack.get(),
                                                                     stack->getJointPositionLimitMin(), stack->getJointPositionLimitMax(),
                                                                     stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
                                                                     stack->getJointNames(),
                                                                     1.0,
                                                                     false,
                                                                     nh));
        stack->pushTask(joint_position_limit_task);

        //Constraint left, right foot
        GoToPoseMetaTaskPtr left_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_left_6_link", "none", nh) );
        GoToPoseMetaTaskPtr right_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_right_6_link", "none", nh) );

        std::vector<TaskAbstractPtr> constraint_tasks;
        constraint_tasks.push_back(left_foot_constraint);
        constraint_tasks.push_back(right_foot_constraint);
        GenericMetaTaskPtr constraint_metatask(new GenericMetaTask(constraint_tasks, stack->getNumberVariables()));
        stack->pushTask(TaskAbstractPtr(constraint_metatask));

        //Constraint the COM
        ConstraintFIXC0MMetaTaskPtr com_constraint (new ConstraintFIXC0MMetaTask(*stack.get(), nh));
        stack->pushTask(com_constraint);

        //Self collision
        SelfCollisionSafetyKinematicTask::SelfCollisionSafetyParameters sc_params;
        sc_params.min_distance = 0.08;
        sc_params.influence_distance = 0.08;
        sc_params.epsison = 0.02;
        sc_params.safety_distance = 0;
        sc_params.number_collisions = 10;
        SelfCollisionSafetyKinematicTaskPtr self_collision(new SelfCollisionSafetyKinematicTask() );
        self_collision->setUpTask(sc_params, *stack.get(), nh);
        self_collision->setDamping(0.1);
        stack->pushTask(self_collision);

        GazePointKinematicMetaTaskPtr gaze_task(new GazePointKinematicMetaTask(*stack.get(), "stereo_optical_frame", "interactive_marker", nh));
        gaze_task->setDamping(0.02);
        stack->pushTask(gaze_task);

        //Position Target Reference for right and left arm
        GoToPositionMetaTaskPtr go_to_position_left_arm(new GoToPositionMetaTask(*stack.get(), "arm_left_7_link", "interactive_marker", nh));
        GoToPositionMetaTaskPtr go_to_position_right_arm(new GoToPositionMetaTask(*stack.get(), "arm_right_7_link", "interactive_marker", nh));
        std::vector<TaskAbstractPtr> go_to_position_tasks;
        go_to_position_tasks.push_back(go_to_position_left_arm);
        go_to_position_tasks.push_back(go_to_position_right_arm);
        GenericMetaTaskPtr go_to_position_metatasks(new GenericMetaTask(go_to_position_tasks, stack->getNumberVariables()));
        go_to_position_metatasks->setDamping(0.1);
        stack->pushTask(TaskAbstractPtr(go_to_position_metatasks));

        //Try to keep base orientation
        GoToOrientationMetaTaskPtr base_link_orientation(new GoToOrientationMetaTask(*stack.get(), "base_link", "interactive_marker", nh));
        base_link_orientation->setDamping(0.02);
        stack->pushTask(base_link_orientation);

        //Try to keep torso orientation
        GoToOrientationMetaTaskPtr torso_link_orientation(new GoToOrientationMetaTask(*stack.get(), "torso_2_link", "interactive_marker", nh));
        torso_link_orientation->setDamping(0.02);
        stack->pushTask(torso_link_orientation);

        //Reference posture
        ReferenceKinematicTaskAllJointsMetaTaskPtr reference(new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(),
                                                                                                         default_reference_joints,
                                                                                                         default_reference_posture,
                                                                                                         nh));
        stack->pushTask(reference);
    }
};

PLUGINLIB_EXPORT_CLASS(reemc_stack_both_hands_head, StackConfigurationKinematic);


/**
 * Same class as above with the COM task replace by stabilized center of mass task
 */
class reemc_stack_both_hands_head_stabilized: public StackConfigurationKinematic{
public:

    void setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh){

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

        JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
                    new JointPositionLimitKinematicAllJointsMetaTask(*stack.get(),
                                                                     stack->getJointPositionLimitMin(), stack->getJointPositionLimitMax(),
                                                                     stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
                                                                     stack->getJointNames(),
                                                                     1.0,
                                                                     false,
                                                                     nh));
        stack->pushTask(joint_position_limit_task);

        GoToPoseMetaTaskPtr left_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_left_6_link", "none", nh) );
        GoToPoseMetaTaskPtr right_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_right_6_link", "none", nh) );

        std::vector<TaskAbstractPtr> constraint_tasks;
        constraint_tasks.push_back(left_foot_constraint);
        constraint_tasks.push_back(right_foot_constraint);
        GenericMetaTaskPtr constraint_metatask(new GenericMetaTask(constraint_tasks, stack->getNumberVariables()));
        stack->pushTask(TaskAbstractPtr(constraint_metatask));

        ConstraintStabilizedFIXC0MMetaTaskPtr com_constraint (new ConstraintStabilizedFIXC0MMetaTask(*stack.get(),
                                                                                                     left_force_, right_force_,
                                                                                                     left_torque_, right_torque_,
                                                                                                     "leg_left_6_link",
                                                                                                     "leg_right_6_link",
                                                                                                     nh));
        stack->pushTask(com_constraint);

        SelfCollisionSafetyKinematicTask::SelfCollisionSafetyParameters sc_params;
        sc_params.min_distance = 0.08;
        sc_params.influence_distance = 0.08;
        sc_params.epsison = 0.02;
        sc_params.safety_distance = 0;
        sc_params.number_collisions = 10;
        SelfCollisionSafetyKinematicTaskPtr self_collision(new SelfCollisionSafetyKinematicTask() );
        self_collision->setUpTask(sc_params, *stack.get(), nh);
        self_collision->setDamping(0.1);
        stack->pushTask(self_collision);

        GazePointKinematicMetaTaskPtr gaze_task(new GazePointKinematicMetaTask(*stack.get(), "stereo_optical_frame", "interactive_marker", nh));
        gaze_task->setDamping(0.02);
        stack->pushTask(gaze_task);

        GoToPositionMetaTaskPtr go_to_position_left_arm(new GoToPositionMetaTask(*stack.get(), "arm_left_7_link", "interactive_marker", nh));
        GoToPositionMetaTaskPtr go_to_position_right_arm(new GoToPositionMetaTask(*stack.get(), "arm_right_7_link", "interactive_marker", nh));
        std::vector<TaskAbstractPtr> go_to_position_tasks;
        go_to_position_tasks.push_back(go_to_position_left_arm);
        go_to_position_tasks.push_back(go_to_position_right_arm);
        GenericMetaTaskPtr go_to_position_metatasks(new GenericMetaTask(go_to_position_tasks, stack->getNumberVariables()));
        go_to_position_metatasks->setDamping(0.1);
        stack->pushTask(TaskAbstractPtr(go_to_position_metatasks));

        GoToOrientationMetaTaskPtr base_link_orientation(new GoToOrientationMetaTask(*stack.get(), "base_link", "interactive_marker", nh));
        base_link_orientation->setDamping(0.02);
        stack->pushTask(base_link_orientation);

        GoToOrientationMetaTaskPtr torso_link_orientation(new GoToOrientationMetaTask(*stack.get(), "torso_2_link", "interactive_marker", nh));
        torso_link_orientation->setDamping(0.02);
        stack->pushTask(torso_link_orientation);

        ReferenceKinematicTaskAllJointsMetaTaskPtr reference(new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(),
                                                                                                         default_reference_joints,
                                                                                                         default_reference_posture,
                                                                                                         nh));
        stack->pushTask(reference);
    }
};

PLUGINLIB_EXPORT_CLASS(reemc_stack_both_hands_head_stabilized, StackConfigurationKinematic);

