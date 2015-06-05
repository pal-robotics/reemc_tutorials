/*
 *  ik_example.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 6/5/2015
 *      Author: luca
 */

#include <walking_controller/ik/pair_ik_leg_analytic_kajita.h>
#include <walking_controller/reemc_definitions.h>

int main(int argc, char **argv)
{

 /// Instantiating an object IK for legs
 /// Rototranslation matrix from base_link to first joint, femur and tibia length are provided
 LegsPairIKAnalyticKajita legs_kinematics(pal::center_to_left_matrix, pal::center_to_right_matrix, pal::FEMUR_LENGTH, pal::TIBIA_LENGTH);

 /// Left foot pose
 eVector3 leftFootPos(0.0, pal::HIP_SPACING, 0.0);
 eVector3 leftFootRPY(0.0, 0.0, 0.0);

 /// Right foot pose
 eVector3 rightFootPos(0.0, -pal::HIP_SPACING, 0.0);
 eVector3 rightFootRPY(0.0, 0.0, 0.0);

 /// CoM pose
 eVector3 hipPos = eVector3(0.0, 0.0, pal::TIBIA_LENGTH + pal::FEMUR_LENGTH + pal::FOOT_HEIGHT - 0.001);
 eVector3 hipRPY = eVector3(0.0, 0.0, 0.0 );

 /// Setting IK with CoM and feet poses
 legs_kinematics.setBaseFrameCoord(createMatrix(hipRPY, hipPos));
 legs_kinematics.setFootCoord(pal::LEFT, createMatrix(leftFootRPY, leftFootPos), pal::ankle_to_foot_center[pal::LEFT]);
 legs_kinematics.setFootCoord(pal::RIGHT, createMatrix(rightFootRPY, rightFootPos), pal::ankle_to_foot_center[pal::RIGHT]);

 std::vector<double> leftAngles(6,0.0);
 std::vector<double> rightAngles(6,0.0);

 /// Compute inverse kinematics for left leg joints
 legs_kinematics.ik_analytic(pal::LEFT, &leftAngles[0]);

  /// Compute inverse kinematics for right leg joints
 legs_kinematics.ik_analytic(pal::RIGHT, &rightAngles[0]);

 std::cout << "CoM        pos (" << hipPos[X] << ", " << hipPos[Y] << ", "  << hipPos[Z] << ")" <<
 std::cout << "           rpy (" << hipRPY[ROLL] <<", "<< hipRPY[PITCH] << ", " << hipRPY[YAW] << ")" << std::endl;

 std::cout << "Left foot  pos (" << leftFootPos[X] << ", " << leftFootPos[Y]<< ", " << leftFootPos[Z] << ")" <<
 std::cout << "           rpy (" << leftFootRPY[ROLL]<< ", " << leftFootRPY[PITCH] << ", " << leftFootRPY[YAW] << ")" <<  std::endl;

 std::cout << "Right foot pos (" << rightFootPos[X]    << ", " << rightFootPos[Y]      << ", " << rightFootPos[Z] << ")" <<
 std::cout << "           rpy (" << rightFootRPY[ROLL] << ", " <<  rightFootRPY[PITCH] << ", " <<  rightFootRPY[YAW] << ")" <<  std::endl;

 std::cout << "Left leg joints  (" << leftAngles[0] << ", " <<  leftAngles[1] << ", " <<  leftAngles[2] << ", " <<
                                 leftAngles[3] << ", " <<  leftAngles[4] << ", " <<  leftAngles[5] << ")" <<  std::endl;
 std::cout << "Right leg joints (" << rightAngles[0] << ", " <<  rightAngles[1] << ", " <<  rightAngles[2] << ", " <<
                                 rightAngles[3] << ", " <<  rightAngles[4] << ", " <<  rightAngles[5] << ")" <<  std::endl;
 return 0;
}
