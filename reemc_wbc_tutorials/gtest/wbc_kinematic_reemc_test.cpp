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

#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <pal_wbc_controller/StackOfTasksKinematic.h>
#include <hqp_solvers/hqp_solver.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <pal_robot_tools/conversions.h>
#include <pluginlib/class_loader.h>
#include "reemc_stacks_gtest.h"

int argc_;
char** argv_;
ros::Publisher joint_state_pub;

void debug_publish_joints(std::vector<std::string> joint_names, Eigen::VectorXd Q){
    sensor_msgs::JointState joints;
    joints.header.stamp = ros::Time::now();
    joints.name = joint_names;
    for(unsigned int i=6; i<Q.rows(); ++i){
        joints.position.push_back(Q(i));
    }
    joint_state_pub.publish(joints);
}

void debug_publish_base_tf(StackOfTasksPtr stack, Eigen::VectorXd Q){
    static   tf::TransformBroadcaster br;
    //Publish Floating base transform
    Eigen::Vector3d worldBodyPos = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, 6,  Eigen::Vector3d (0., 0., 0.), false);
    Eigen::Matrix3d worldBodyRot = RigidBodyDynamics::CalcBodyWorldOrientation(*stack->getModel(), Q, 6, false);
    tf::Transform base_tf;
    base_tf.setOrigin(tf::Vector3(worldBodyPos(0) ,worldBodyPos(1), worldBodyPos(2)));
    Eigen::Quaterniond quat(worldBodyRot.transpose());
    base_tf.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
    br.sendTransform(tf::StampedTransform(base_tf, ros::Time::now(), "/world", "/base_link"));
}

/**
 WBC control of reemc kinematic velocity level,
 Stack:
  JointLimits,
  Foot in fixed ground
  COM Fixed
  SelfCollision
  Gaze
  Goto left and right arm
  Trunk orientation
Solver:
  uquadprog
*/

TEST(wbc_test, wbc_reemc_kinematic_velocity) {

    ros::init(argc_, argv_, "wbc_reemc_kinematic_velocity");
    ros::NodeHandle nh;
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Duration dt(0.005);
    nh.setParam("dt", dt.toSec()); /// @todo this should be allowed to be a parameter c++ not rely on param server
    bool floating_base = true;
    Eigen::VectorXd solution;

    std::string urdf_path = ros::package::getPath("reemc_wbc");
    urdf_path = urdf_path + "/gtest/reemc_full.urdf";
     StackOfTasksKinematicPtr stack(new StackOfTasksKinematic(formulation_t::velocity, solution, dt.toSec(), floating_base, urdf_path));
    //StackOfTasksKinematicPtr stack(new StackOfTasksKinematic(formulation_t::velocity, solution, dt.toSec(), floating_base));

    Eigen::VectorXd Q(solution.size()); Q.setZero();
    Eigen::VectorXd QDot(solution.size()); QDot.setZero();

    eVector3 left_arm;
    eVector3 right_arm;
    unsigned int left_id = stack->getModel()->GetBodyId("arm_left_7_link");
    unsigned int right_id = stack->getModel()->GetBodyId("arm_right_7_link");

    /// @todo intial joint conf should be removed
    Q(6 + stack->getJointIndex("leg_left_3_joint")) = -0.4;
    Q(6 + stack->getJointIndex("leg_right_3_joint")) = -0.4;
    Q(6 + stack->getJointIndex("leg_left_4_joint")) = 0.8;
    Q(6 + stack->getJointIndex("leg_right_4_joint")) = 0.8;
    Q(6 + stack->getJointIndex("leg_left_5_joint")) = -0.4;
    Q(6 + stack->getJointIndex("leg_right_5_joint")) = -0.4;
    Q(6 + stack->getJointIndex("arm_left_4_joint")) = 0.4;
    Q(6 + stack->getJointIndex("arm_right_4_joint")) = 0.4;
    Q(6 + stack->getJointIndex("arm_left_2_joint")) = 0.3;
    Q(6 + stack->getJointIndex("arm_right_2_joint")) = 0.3;

    stack->updateTasks(Q, QDot, ros::Time::now());

    boost::shared_ptr<reemc_wbc_velocity_gtest_stack> stack_conf(new reemc_wbc_velocity_gtest_stack());

    stack_conf->setupStack(stack, nh);

    /// @todo if this is not called after setup task there is no pointers to the levels, how can this be??
    stack->updateTasks(Q, QDot, ros::Time::now());

    boost::shared_ptr<pluginlib::ClassLoader<HQPSolver> > hqpSolverLoader
            (new pluginlib::ClassLoader<HQPSolver>("hqp_solvers", "HQPSolver") );
    HQPSolverPtr  hqp_solver;

    try{
        hqp_solver = hqpSolverLoader->createInstance("QpReductionuEqualitiesQuadprogHeapAllocation");
    }
    catch(pluginlib::PluginlibException& ex){
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        EXPECT_TRUE(false);
    }

    std::vector<Level*> levels;
    levels.reserve(100);
    stack->getLevels(levels);

    hqp_solver->configureSolver(levels, stack->getNumberVariables());

    stack_conf->head_target->setPositionTarget(eVector3(0.5, 0.0, 0.4));
    stack_conf->left_arm_target->setPositionTarget(eVector3(0.2, 0.3, 0.4));
    stack_conf->right_arm_target->setPositionTarget(eVector3(0.2, -0.3, 0.4));
    stack->updateTasks(Q, QDot, ros::Time::now());

    ros::Time duration(10.0);
    for(ros::Time t(0); t<duration; t+=dt){
        stack->updateTasks(Q, QDot, ros::Time::now());
        hqp_solver->solve_hirarchy(solution);
        Q += solution*dt.toSec();
        debug_publish_joints(stack->getJointNames(), Q);
        debug_publish_base_tf(stack, Q);
    }

    left_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, left_id,  Eigen::Vector3d::Zero(), false);
    right_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, right_id,  Eigen::Vector3d::Zero(), false);
    ASSERT_NEAR(pal::squared_difference(stack_conf->left_arm_target->getPositionTarget(), left_arm), 0.0, 1e-2);
    ASSERT_NEAR(pal::squared_difference(stack_conf->right_arm_target->getPositionTarget(), right_arm), 0.0, 1e-2);

    stack_conf->head_target->setPositionTarget(eVector3(0.5, 0.0, 0.4));
    stack_conf->left_arm_target->setPositionTarget(eVector3(0.2, 0.3, -0.4));
    stack_conf->right_arm_target->setPositionTarget(eVector3(0.2, -0.3, -0.4));

    for(ros::Time t(0); t<duration; t+=dt){
        stack->updateTasks(Q, QDot, ros::Time::now());
        hqp_solver->solve_hirarchy(solution);
        Q += solution*dt.toSec();
        debug_publish_joints(stack->getJointNames(), Q);
        debug_publish_base_tf(stack, Q);
    }

    left_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, left_id,  Eigen::Vector3d::Zero(), false);
    right_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, right_id,  Eigen::Vector3d::Zero(), false);
    ASSERT_NEAR(pal::squared_difference(stack_conf->left_arm_target->getPositionTarget(), left_arm), 0.0, 1e-2);
    ASSERT_NEAR(pal::squared_difference(stack_conf->right_arm_target->getPositionTarget(), right_arm), 0.0, 1e-2);

    stack_conf->head_target->setPositionTarget(eVector3(0.5, 0.0, 0.4));
    stack_conf->left_arm_target->setPositionTarget(eVector3(0.2, 0.3, -0.0));
    stack_conf->right_arm_target->setPositionTarget(eVector3(0.2, -0.0, -0.0));

    for(ros::Time t(0); t<duration; t+=dt){
        stack->updateTasks(Q, QDot, ros::Time::now());
        hqp_solver->solve_hirarchy(solution);
        Q += solution*dt.toSec();
        debug_publish_joints(stack->getJointNames(), Q);
        debug_publish_base_tf(stack, Q);
    }

    left_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, left_id,  Eigen::Vector3d::Zero(), false);
    right_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, right_id,  Eigen::Vector3d::Zero(), false);
    ASSERT_NEAR(pal::squared_difference(stack_conf->left_arm_target->getPositionTarget(), left_arm), 0.0, 1e-2);
    ASSERT_NEAR(pal::squared_difference(stack_conf->right_arm_target->getPositionTarget(), right_arm), 0.0, 1e-2);

    stack_conf->head_target->setPositionTarget(eVector3(0.5, -0.3, 0.4));
    stack_conf->left_arm_target->setPositionTarget(eVector3(0.2, 0.5, -0.0));
    stack_conf->right_arm_target->setPositionTarget(eVector3(0.2, -0.3, -0.0));

    for(ros::Time t(0); t<duration; t+=dt){
        stack->updateTasks(Q, QDot, ros::Time::now());
        hqp_solver->solve_hirarchy(solution);
        Q += solution*dt.toSec();
        debug_publish_joints(stack->getJointNames(), Q);
        debug_publish_base_tf(stack, Q);
    }

    left_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, left_id,  Eigen::Vector3d::Zero(), false);
    right_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, right_id,  Eigen::Vector3d::Zero(), false);
    ASSERT_NEAR(pal::squared_difference(stack_conf->left_arm_target->getPositionTarget(), left_arm), 0.0, 1e-2);
    ASSERT_NEAR(pal::squared_difference(stack_conf->right_arm_target->getPositionTarget(), right_arm), 0.0, 1e-2);

    stack_conf->head_target->setPositionTarget(eVector3(0.5, -0.3, 0.4));
    stack_conf->left_arm_target->setPositionTarget(eVector3(0.2, 0.5, -0.3));
    stack_conf->right_arm_target->setPositionTarget(eVector3(0.2, -0.3, -0.3));

    for(ros::Time t(0); t<duration; t+=dt){
        stack->updateTasks(Q, QDot, ros::Time::now());
        hqp_solver->solve_hirarchy(solution);
        Q += solution*dt.toSec();
        debug_publish_joints(stack->getJointNames(), Q);
        debug_publish_base_tf(stack, Q);
    }

    left_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, left_id,  Eigen::Vector3d::Zero(), false);
    right_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, right_id,  Eigen::Vector3d::Zero(), false);
    ASSERT_NEAR(pal::squared_difference(stack_conf->left_arm_target->getPositionTarget(), left_arm), 0.0, 1e-2);
    ASSERT_NEAR(pal::squared_difference(stack_conf->right_arm_target->getPositionTarget(), right_arm), 0.0, 1e-2);

    stack_conf->head_target->setPositionTarget(eVector3(0.5, -0.3, 0.4));
    stack_conf->left_arm_target->setPositionTarget(eVector3(0.2, -0.2, -0.2));
    stack_conf->right_arm_target->setPositionTarget(eVector3(0.2, -0.3, -0.0));

    for(ros::Time t(0); t<duration; t+=dt){
        stack->updateTasks(Q, QDot, ros::Time::now());
        hqp_solver->solve_hirarchy(solution);
        Q += solution*dt.toSec();
        debug_publish_joints(stack->getJointNames(), Q);
        debug_publish_base_tf(stack, Q);
    }

    left_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, left_id,  Eigen::Vector3d::Zero(), false);
    right_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, right_id,  Eigen::Vector3d::Zero(), false);
    ASSERT_NEAR(pal::squared_difference(stack_conf->left_arm_target->getPositionTarget(), left_arm), 0.0, 1e-2);
    ASSERT_NEAR(pal::squared_difference(stack_conf->right_arm_target->getPositionTarget(), right_arm), 0.0, 1e-2);

    stack_conf->head_target->setPositionTarget(eVector3(0.5, -0.0, 0.4));
    stack_conf->left_arm_target->setPositionTarget(eVector3(0.2, 0.3, 0.7));
    stack_conf->right_arm_target->setPositionTarget(eVector3(0.2, -0.3, 0.7));

    for(ros::Time t(0); t<duration; t+=dt){
        stack->updateTasks(Q, QDot, ros::Time::now());
        hqp_solver->solve_hirarchy(solution);
        Q += solution*dt.toSec();
        debug_publish_joints(stack->getJointNames(), Q);
        debug_publish_base_tf(stack, Q);
    }

    left_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, left_id,  Eigen::Vector3d::Zero(), false);
    right_arm = RigidBodyDynamics::CalcBodyToBaseCoordinates(*stack->getModel(), Q, right_id,  Eigen::Vector3d::Zero(), false);
    ASSERT_NEAR(pal::squared_difference(stack_conf->left_arm_target->getPositionTarget(), left_arm), 0.0, 1e-2);
    ASSERT_NEAR(pal::squared_difference(stack_conf->right_arm_target->getPositionTarget(), right_arm), 0.0, 1e-2);

   // pal_wbc_controller::stackDebug debug_msg;
   // stack->copyDebugMsg(debug_msg, Q, ros::Time::now());

    //cleaning up
    hqp_solver.reset();
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    argc_ = argc;
    argv_ = argv;
    return RUN_ALL_TESTS();
}
