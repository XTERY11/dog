/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "state_estimator/robot_pose_estimator.h"

namespace Quadruped {

    RobotPoseEstimator::RobotPoseEstimator(Robot *robotIn, OpenloopGaitGenerator *gaitGeneratorIn)
    : robot(robotIn), gaitGenerator(gaitGeneratorIn)
    {
        lastTimestamp = 0.f;
        estimatedPose << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f ;
        printf("RobotPoseEstimator finish constructing!\n");
    }

    void RobotPoseEstimator::Reset(float currentTime)
    {
        lastTimestamp = 0.f;
        estimatedPose << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f ;
    }

    float RobotPoseEstimator::ComputeDeltaTime(const LowState *robotState)
    {
        float deltaTime;
        if (std::abs(lastTimestamp) < 1e-5) {
            // First timestamp received, return an estimated delta_time.
            deltaTime = robot->timeStep;
        } else {
            deltaTime = (robotState->tick - lastTimestamp) / 1000.;
        }
        lastTimestamp = robotState->tick;
        return deltaTime;
    }

    void RobotPoseEstimator::Update(float currentTime)
    {
        const LowState &robotState = robot->lowState;
        // Propagate current state estimate with new accelerometer reading."""
        float deltaTime = ComputeDeltaTime(&robotState);
        float height = EstimateRobotHeight();
        estimatedPose[2] = height;
        robot->basePosition[2] = height;
            
    }

    float RobotPoseEstimator::EstimateRobotHeight()
    {
        Eigen::Matrix<float, 4, 1> baseOrientation;
        Eigen::Matrix<float, 3, 3> rotMat;
        Eigen::Matrix<float, 4, 3> footPositions;
        Eigen::Matrix<float, 4, 3> footPositionsWorldFrame;
        Eigen::Matrix<float, 4, 1> usefulHeights;

        Eigen::Matrix<int, 4, 1> contacts;
        for (int legId = 0; legId < 4; legId++) {
            int legState = gaitGenerator->desiredLegState[legId];
            if (legState == LegState::STANCE || legState == LegState::EARLY_CONTACT) {
                contacts[legId] = true;
            } else {
                contacts[legId] = false;
            }
        }

        if (contacts.sum() == 0) {
            // All foot in air, no way to estimate
            return robot->bodyHeight;
        } else {
            baseOrientation = this->robot->GetBaseOrientation();
            rotMat = robotics::math::quaternionToRotationMatrix(baseOrientation);
            footPositions = this->robot->GetFootPositionsInBaseFrame().transpose();
            footPositionsWorldFrame = (rotMat * footPositions.transpose()).transpose();
            usefulHeights = -footPositionsWorldFrame.block<4, 1>(0, 2).cwiseProduct(contacts.cast<float>());
            return usefulHeights.sum() / contacts.sum();
        }
    }
} // Quadruped
