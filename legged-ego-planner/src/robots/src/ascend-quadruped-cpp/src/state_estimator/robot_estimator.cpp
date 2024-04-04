/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "state_estimator/robot_estimator.h"

namespace Quadruped {
    RobotEstimator::RobotEstimator(Robot *robotIn,
                                   OpenloopGaitGenerator *gaitGeneratorIn)
        : robot(robotIn),velocityEstimator(robotIn, gaitGeneratorIn),
          poseEstimator(robotIn, gaitGeneratorIn)

    {
        estimatedPosition << 0,0,0;
        estimatedRPY << 0,0,0;
        estimatedVelocity << 0,0,0;
        lastTimestamp = 0.0;
    }

    void RobotEstimator::Reset(float currentTime)
    {
        
        timeSinceReset = robot->GetTimeSinceReset();
        estimatedPosition << 0,0,0;
        estimatedRPY << 0,0,0;
        estimatedVelocity << 0,0,0;
        lastTimestamp = 0.0;
        velocityEstimator.Reset(currentTime);
        poseEstimator.Reset(currentTime);
        std::cout<< "reset pos= "<<GetEstimatedPosition()<<std::endl;
    }

    float RobotEstimator::ComputeDeltaTime(const LowState *robotState)
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

    void RobotEstimator::Update(float currentTime)
    {
        velocityEstimator.Update(currentTime);
        poseEstimator.Update(currentTime);

        estimatedVelocity = velocityEstimator.GetEstimatedVelocity();
        const Vec6<float> &pose = poseEstimator.GetEstimatedPose();
        estimatedPosition = pose.head(3);
        estimatedRPY = pose.tail(3);
    }
} // namespace Quadruped
