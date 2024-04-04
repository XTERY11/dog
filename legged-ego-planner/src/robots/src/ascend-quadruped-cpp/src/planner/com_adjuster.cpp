/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: a interface of robot locomotion controller.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "planner/com_adjuster.h"

namespace Quadruped {
ComAdjuster::ComAdjuster(Robot *robotIn,
                         OpenloopGaitGenerator *gaitGeneratorIn,
                         RobotEstimator *robotEstimatorIn)
    : robot(robotIn), gaitGenerator(gaitGeneratorIn), robotEstimator(robotEstimatorIn), delta(1.0f)
{
    // todo: update the pose in world frame by estimator
    basePosition = robot->GetBasePosition(); // base to  world frame
    baseOrientation = robot->GetBaseOrientation();  // base to world frame

    // world to base
    auto inv = robotics::math::invertTransform(basePosition, baseOrientation);
    inverseTranslation = inv.block<3, 1>(0, 3);
    inverseRotation = inv.block<3, 3>(0, 0);

    legState = gaitGenerator->legState;
    normalizedPhase = gaitGenerator->normalizedPhase;
    footPosition = robot->GetFootPositionsInBaseFrame();
    for (int i = 0; i < 4; ++i) {
        contactK[i] = 0.f;
        swingK[i] = 0.f;
        weightFactor[i] = 0.f;
    }
    supportPolygonVertices = Eigen::Matrix<float, 3, 4>::Zero();
    comPosInBaseFrame = Eigen::Matrix<float, 3, 1>::Zero();
    comPosInWorldFrame << basePosition[0], basePosition[1], 0.f;
}

void ComAdjuster::Reset(float current_time)
{
    // todo: update the pose in world frame by estimator
    basePosition = robot->GetBasePosition();
    baseOrientation = robot->GetBaseOrientation();
    // world to base
    auto inv = robotics::math::invertTransform(basePosition, baseOrientation);
    inverseTranslation = inv.block<3, 1>(0, 3);
    inverseRotation = inv.block<3, 3>(0, 0);

    legState = gaitGenerator->legState;
    normalizedPhase = gaitGenerator->normalizedPhase;
    footPosition = robot->GetFootPositionsInBaseFrame();
    for (int i = 0; i < 4; ++i) {
        contactK[i] = 0.f;
        swingK[i] = 0.f;
        weightFactor[i] = 0.f;
    }
    supportPolygonVertices = Eigen::Matrix<float, 3, 4>::Zero();
    comPosInBaseFrame = Eigen::Matrix<float, 3, 1>::Zero();
    comPosInWorldFrame << basePosition[0], basePosition[1], 0.f;
}

Eigen::Matrix<float, 3, 1> ComAdjuster::Update(float current_time)
{

    legState = gaitGenerator->legState;
    normalizedPhase = gaitGenerator->normalizedPhase;
    footPosition = robot->GetFootPositionsInBaseFrame();

    // Compute prior contact probility and stance probility
    for (int legId = 0; legId < legState.rows(); ++legId) {
        if (legState[legId] == LegState::STANCE || legState[legId] == LegState::EARLY_CONTACT) {
            contactK[legId] = 0.5 * (std::erf(normalizedPhase[legId] / (delta * sqrt(2)))
                + std::erf((1.0 - normalizedPhase[legId]) / (delta * sqrt(2)))
            );
            swingK[legId] = 0.f;
        } else {
            swingK[legId] = 0.5 * (2.0 + std::erf(-normalizedPhase[legId] / (delta * sqrt(2)))
                + std::erf((normalizedPhase[legId] - 1.0) / (delta * sqrt(2)))
            );
            contactK[legId] = 0.f;
        }
        weightFactor[legId] = contactK[legId] + swingK[legId];
    }

    for (int legId = 0; legId < legState.rows(); ++legId) {
        auto p = footPosition.col(legId);
        auto footMap = ADJEST_LEG[legId];
        auto pCw = footPosition.col(footMap["cw"]);
        auto pCcw = footPosition.col(footMap["ccw"]);

        auto phi = weightFactor[legId];
        auto phiCw = weightFactor[footMap["cw"]];
        auto phiCcw = weightFactor[footMap["ccw"]];

        Eigen::Matrix<float, 6, 2> pMat = Eigen::Matrix<float, 6, 2>::Zero();
        pMat.block<3, 1>(0, 0) = p;
        pMat.block<3, 1>(3, 0) = p;
        pMat.block<3, 1>(0, 1) = pCw;
        pMat.block<3, 1>(3, 1) = pCcw;
        Eigen::Matrix<float, 2, 1> phiMat = {phi, 1 - phi};
        auto virtualPointAdj = pMat * phiMat; // shape = 6 * 1
        Eigen::Matrix<float, 3, 1> virtualPointCw = virtualPointAdj.head(3);
        Eigen::Matrix<float, 3, 1> virtualPointCcw = virtualPointAdj.tail(3);
        supportPolygonVertices.col(legId) = (phi * p + phiCcw * virtualPointCcw + phiCw * virtualPointCw)
            / (phi + phiCcw + phiCw);
    }

    comPosInBaseFrame = supportPolygonVertices.rowwise().mean(); // shape = (3,1)

    return comPosInBaseFrame;
}
} // namespace Quadruped