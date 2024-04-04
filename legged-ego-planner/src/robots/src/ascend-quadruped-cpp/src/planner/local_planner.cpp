/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "planner/local_planner.h"

namespace Quadruped {
LocalPlanner::LocalPlanner(Robot *robotIn) : robot(robotIn),
                                             resetTime(robot->GetTimeSinceReset()),
                                             timeSinceReset(0.f),
                                             terrainType(TerrainType::PLUM_PILES),
                                             footholdPlanner(FootStepper(terrainType, 0.14f, 0.10f, "optimal")),
                                             stepCount(0) {
    costMap = Eigen::Matrix<float, N, N>::Zero();
    comPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
    desiredComPose = Eigen::Matrix<float, 6, 1>::Zero();
    // footholds
    desiredFootholdsOffset = Eigen::Matrix<float, 3, 4>::Zero();
//    desiredFootholdsOffset = robot->GetFootPositionsInWorldFrame();//todo reset by 0?
////    desiredFootholdsOffset.row(2) << 0.f, 0.f, 0.f, 0.f;
//    std::cout << "[LocalPlanner] init footholds in base frame: \n" << robot->GetFootPositionsInBaseFrame() << std::endl;
//    std::cout << "[LocalPlanner] init basePosition: \n" << robot->basePosition << std::endl;
//    std::cout << "[LocalPlanner] init baseOrientation: \n" << robot->baseOrientation << std::endl;
//    std::cout << "[LocalPlanner] set desiredFootholdsOffset by FootPositionsInWorldFrame: \n" << desiredFootholdsOffset << std::endl;
//    float stepL = 0.01;
//    desiredFootholdsOffset.row(0) << stepL, stepL, stepL, stepL;
    Reset();
}

void LocalPlanner::Reset() {
    resetTime = robot->GetTimeSinceReset();
    timeSinceReset = 0.f;
    footholdPlanner.Reset(timeSinceReset);
    comPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
    desiredComPose = Eigen::Matrix<float, 6, 1>::Zero();
    // footholds
    desiredFootholdsOffset = Eigen::Matrix<float, 3, 4>::Zero();
//    desiredFootholdsOffset = robot->GetFootPositionsInWorldFrame(); //todo reset by 0?
////    desiredFootholdsOffset.row(2) << 0.f, 0.f, 0.f, 0.f;
//    std::cout << "[LocalPlanner Reset] reset desiredFootholdsOffset by FootPositionsInWorldFrame: \n" << desiredFootholdsOffset << std::endl;
}

void LocalPlanner::UpdateOnce(Eigen::Matrix<float, 3, 4> currentFootholds, Eigen::Matrix<float, 6, 1> currentComPose) {
    std::cout << "------------------------[LocalPlanner::UpdateOnce]------------------------" << std::endl;
    std::cout << "current footholds: \n" << currentFootholds << std::endl;
    comPose = GetComPose();
    desiredComPose = GetComGoal(comPose);
    std::cout << "current com pose: \n" << comPose.transpose() << std::endl;
    std::cout << "desired com pose: \n" << desiredComPose.transpose() << std::endl;
    ComputeFootholdsOffset(currentFootholds, comPose, desiredComPose);

    std::cout << "desired footholds offset: \n" << desiredFootholdsOffset << std::endl;
    std::cout << "------------------------[LocalPlanner::UpdateOnce Finished]------------------------" << std::endl;

    stepCount += 1;
    if (stepCount > 20) {
        // self.rst = True
        stepCount = 0;
        printf("**********************************************************************\n");
    }
}

Eigen::Matrix<float, 3, 4> LocalPlanner::ComputeFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds,
                                                                Eigen::Matrix<float, 6, 1> currentComPose,
                                                                Eigen::Matrix<float, 6, 1> desiredComPose) {
    desiredFootholdsOffset = footholdPlanner.GetOptimalFootholdsOffset(currentFootholds);
    return desiredFootholdsOffset;
}

Eigen::Matrix<float, 3, 4> LocalPlanner::ComputeNextFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds) {
    desiredFootholdsOffset = footholdPlanner.GetOptimalFootholdsOffset(currentFootholds);
    return desiredFootholdsOffset;
}
} // namespace Quadruped