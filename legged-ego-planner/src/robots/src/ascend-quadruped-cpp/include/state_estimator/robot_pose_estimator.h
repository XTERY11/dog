/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_ROBOT_POSE_ESTIMATOR_H
#define ASCEND_QUADRUPED_CPP_ROBOT_POSE_ESTIMATOR_H

#include "robots/robot.h"
#include "mpc_controller/openloop_gait_generator.h"

namespace Quadruped {
    class RobotPoseEstimator {
    public:
        RobotPoseEstimator(Robot *robotIn, OpenloopGaitGenerator *gaitGeneratorIn);

        void Reset(float currentTime);

        float ComputeDeltaTime(const LowState *robotState);

        void Update(float currentTime);

        float EstimateRobotHeight();

        const Vec6<float> &GetEstimatedPose() const
        {
            return estimatedPose;
        }

    private:
        Robot *robot;
        float lastTimestamp;
        Vec6<float> estimatedPose;
        OpenloopGaitGenerator *gaitGenerator;
        
    };

} // namespace Quadruped

#endif//ASCEND_QUADRUPED_CPP_ROBOT_VELOCITY_ESTIMATOR_H