/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_ROBOT_VELOCITY_ESTIMATOR_H
#define ASCEND_QUADRUPED_CPP_ROBOT_VELOCITY_ESTIMATOR_H

#include <deque>
#include <numeric>
#include "utils/se3.h"
#include "robots/robot.h"
#include "mpc_controller/openloop_gait_generator.h"
#include "state_estimator/filter.h"
#include "TinyEKF.h"

namespace Quadruped {
class RobotVelocityEstimator {
public:
    /**
     * @brief Estimates base velocity of A1 robot.
     * The velocity estimator consists of 2 parts:
     * 1) A state estimator for CoM velocity.
     * 
     * Two sources of information are used:
     * The integrated reading of accelerometer and the velocity estimation from
     * contact legs. The readings are fused together using a Kalman Filter.
     * 
     * 2) A moving average filter to smooth out velocity readings
     */
    RobotVelocityEstimator(Robot *robot,
                           OpenloopGaitGenerator *gaitGeneratorIn,
                           float accelerometerVarianceIn = 0.1f,
                           float sensorVarianceIn = 0.1f,
                           float initialVarianceIn = 0.1f,
                           int movingWindowFilterSizeIn = 120);

    void Reset(float currentTime);

    /** @brief compute the time period between two adjacent imu message */
    float ComputeDeltaTime(const LowState *robotState);

    /** @brief Estimate the velocity */
    void Update(float currentTime);

    const Vec3<float> &GetEstimatedVelocity() const
    {
        return estimatedVelocity;
    }

private:
    Robot *robot;
    OpenloopGaitGenerator *gaitGenerator;
    int windowSize;
    float initialVariance;
    float lastTimestamp;
    Vec3<float> estimatedVelocity;
    MovingWindowFilter velocityFilterX;
    MovingWindowFilter velocityFilterY;
    MovingWindowFilter velocityFilterZ;

    TinyEKF *filter;
};
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_ROBOT_VELOCITY_ESTIMATOR_H
