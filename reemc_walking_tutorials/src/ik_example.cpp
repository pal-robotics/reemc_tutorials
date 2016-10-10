/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2015, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ik_example.cpp
 *  Created on: 6/5/2015
 *  Author: Luca Marchionni
 */

#include <walking_controller/ik/pair_ik_leg_analytic_kajita.h>
#include <walking_controller/reemc_definitions.h>

namespace pal{

const double HIP_SPACING = (0.145 / 2.0);

const double FEMUR_LENGTH = 0.300;
const double TIBIA_LENGTH = 0.300;
const double FOOT_HEIGHT  = 0.110;
const eVector3 ankle_to_foot_center_left(   0,  0, -FOOT_HEIGHT);
const eVector3 ankle_to_foot_center_right(  0,  0, -FOOT_HEIGHT);

const eVector3 ankle_to_foot_center[2] = {ankle_to_foot_center_left, ankle_to_foot_center_right};

const eVector3 foot_center_to_calcaneus_left(-0.075,-0.06,0);
const eVector3 foot_center_to_calcaneus_right(-0.075,0.06,0);
const eVector3 ankle_to_foot_calcaneus[2] = {foot_center_to_calcaneus_left, foot_center_to_calcaneus_right};

const eMatrixHom center_to_left_matrix  =  initMatrixHom(  eMatrixRot::Identity(), eVector3(0,  HIP_SPACING, 0) );
const eMatrixHom center_to_right_matrix =  initMatrixHom(  eMatrixRot::Identity(), eVector3(0, -HIP_SPACING, 0) );

}


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

 std::cout << "CoM        pos (" << hipPos[X] << ", " << hipPos[Y] << ", "  << hipPos[Z] << ")  ";
 std::cout << "           rpy (" << hipRPY[ROLL] <<", "<< hipRPY[PITCH] << ", " << hipRPY[YAW] << ")" << std::endl;

 std::cout << "Left foot  pos (" << leftFootPos[X] << ", " << leftFootPos[Y]<< ", " << leftFootPos[Z] << ") ";
 std::cout << "           rpy (" << leftFootRPY[ROLL]<< ", " << leftFootRPY[PITCH] << ", " << leftFootRPY[YAW] << ")" <<  std::endl;

 std::cout << "Right foot pos (" << rightFootPos[X]    << ", " << rightFootPos[Y]      << ", " << rightFootPos[Z] << ")";
 std::cout << "           rpy (" << rightFootRPY[ROLL] << ", " <<  rightFootRPY[PITCH] << ", " <<  rightFootRPY[YAW] << ")" <<  std::endl;

 std::cout << "Left leg joints  (" << leftAngles[0] << ", " <<  leftAngles[1] << ", " <<  leftAngles[2] << ", " <<
                                 leftAngles[3] << ", " <<  leftAngles[4] << ", " <<  leftAngles[5] << ")" <<  std::endl;
 std::cout << "Right leg joints (" << rightAngles[0] << ", " <<  rightAngles[1] << ", " <<  rightAngles[2] << ", " <<
                                 rightAngles[3] << ", " <<  rightAngles[4] << ", " <<  rightAngles[5] << ")" <<  std::endl;
 return 0;
}
