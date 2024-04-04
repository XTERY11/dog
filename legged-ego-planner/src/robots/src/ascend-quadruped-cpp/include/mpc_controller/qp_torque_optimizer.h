/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: compute the force for stance controller. 
* Author: Zang Yaohua
* Create: 2021-10-25
* Notes: now this cotroller is based on floating-base dynamics, with lineard equations and MPC.
* Modify: init the file. @ Zang Yaohua
*/

#ifndef ASCEND_QUADRUPED_CPP_QP_TORQUE_OPTIMIZER_H
#define ASCEND_QUADRUPED_CPP_QP_TORQUE_OPTIMIZER_H

#include <tuple>
#include <Eigen/Dense>
#include "robots/robot.h"
#include "mpc_controller/qp_torque_optimizer.h"

namespace Quadruped {
Eigen::Matrix<float, 6, 12> ComputeMassMatrix(float robotMass,
                                              Eigen::Matrix<float, 3, 3> robotInertia,
                                              Eigen::Matrix<float, 4, 3> footPositions);
/** @brief
 * @param float total mass of robot for MPC computing
 * @param Eigen::Matrix<int, 4, 1> 4-length array indicating whether feet is contact with ground.
 * @param float frictionCoef defines the interaction force effect between foot and env.
 * @param float min force that applys 
 * @param float max force that applys
 * @return Constraint matrix.
 */
std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
        float mpcBodyMass,
        Eigen::Matrix<int, 4, 1> contacts,
        float frictionCoef=0.8f,
        float fMinRatio=0.1f,
        float fMaxRatio=10.f);

std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> ComputeObjectiveMatrix(
        Eigen::Matrix<float, 6, 12> massMatrix,
        Eigen::Matrix<float, 6, 1> desiredAcc,
        Eigen::Matrix<float, 6, 1> accWeight,
        float regWeight);

Eigen::Matrix<float, 3, 4> ComputeContactForce(Robot *robot,
                                                Eigen::Matrix<float, 6, 1> desiredAcc,
                                                Eigen::Matrix<int,4,1> contacts,
                                                Eigen::Matrix<float, 6, 1> accWeight,
                                                float regWeight=1e-4,
                                                float frictionCoef=0.45,
                                                float fMinRatio=0.1,
                                                float fMaxRatio=10.);

Eigen::Matrix<float, 3, 4> ComputeContactForceTest(float bodyMass,
                                                   Eigen::Matrix<float, 3, 3> bodyInertia,
                                                   Eigen::Matrix<float, 4, 3> footPositions,
                                                   Eigen::Matrix<float, 6, 1> desiredAcc,
                                                   Eigen::Matrix<int,4,1> contacts,
                                                   Eigen::Matrix<float, 6, 1> accWeight,
                                                   float regWeight,
                                                   float frictionCoef,
                                                   float fMinRatio=0.1,
                                                   float fMaxRatio=10.);

} // namespace Quadruped
#endif //ASCEND_QUADRUPED_CPP_QP_TORQUE_OPTIMIZER_H
