/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "state_estimator/robot_velocity_estimator.h"

/** @brief Initiates the velocity estimator.
 *  See filterpy documentation in the link below for more details.
 *  https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
 *
 * @param robot: the robot class for velocity estimation.
 * @param accelerometer_variance: noise estimation for accelerometer reading.
 * @param sensor_variance: noise estimation for motor velocity reading.
 * @param initial_covariance: covariance estimation of initial state.
 */
namespace Quadruped {
RobotVelocityEstimator::RobotVelocityEstimator(Robot *robotIn,
                                               OpenloopGaitGenerator *gaitGeneratorIn,
                                               float accelerometerVarianceIn,
                                               float sensorVarianceIn,
                                               float initialVarianceIn,
                                               int movingWindowFilterSizeIn)
{
    robot = robotIn;
    gaitGenerator = gaitGeneratorIn;
    initialVariance = initialVarianceIn;
    lastTimestamp = 0.f;
    estimatedVelocity << 0.f, 0.f, 0.f;
    windowSize = movingWindowFilterSizeIn;
    filter = new TinyEKF(0.f, initialVariance, accelerometerVarianceIn, sensorVarianceIn);
    velocityFilterX = MovingWindowFilter(windowSize);
    velocityFilterY = MovingWindowFilter(windowSize);
    velocityFilterZ = MovingWindowFilter(windowSize);
}

void RobotVelocityEstimator::Reset(float currentTime)
{
    // filter->Reset(0., initialVariance);
    velocityFilterX = MovingWindowFilter(windowSize);
    velocityFilterY = MovingWindowFilter(windowSize);
    velocityFilterZ = MovingWindowFilter(windowSize);

    lastTimestamp = 0.f;
    estimatedVelocity << 0.f, 0.f, 0.f;
}

float RobotVelocityEstimator::ComputeDeltaTime(const LowState *robotState)
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

void RobotVelocityEstimator::Update(float currentTime)
{
    const LowState &robotState = robot->lowState;
    // Propagate current state estimate with new accelerometer reading."""
    float deltaTime = ComputeDeltaTime(&robotState);
    const auto &acc = robotState.imu.accelerometer;
    Vec3<float> sensorAcc(acc[0], acc[1], acc[2]);
    Quat<float> baseOrientation = robot->GetBaseOrientation(); // w,x,y,z
    Mat3<float> rotMat = robotics::math::quaternionToRotationMatrix(baseOrientation); //.transpose();
    Vec3<float> calibratedAcc = rotMat * sensorAcc;
    calibratedAcc[2] -= 9.81;
    double deltaV[3] = {calibratedAcc[0] * deltaTime, calibratedAcc[1] * deltaTime, calibratedAcc[2] * deltaTime};
    // filter.Predict(deltaTime, calibratedAcc * deltaTime);

    // Correct estimation using contact legs
    std::vector<Vec3<float>> observedVelocities;
    auto footContact(robot->GetFootContacts());
    for (int leg_id = 0; leg_id < 4; ++leg_id) {
        if (footContact[leg_id]) {
            Mat3<float> jacobian = robot->ComputeJacobian(leg_id);
            // Only pick the jacobian related to joint motors
            Vec3<float> jointVelocities = robot->GetMotorVelocities().segment(leg_id * 3, 3);
            Vec3<float> legVelocityInBaseFrame = -(jacobian * jointVelocities);
            observedVelocities.push_back(rotMat * legVelocityInBaseFrame);
        }
    }
    int num = observedVelocities.size();
    if (num > 0) {
        Vec3<float> MeanObservedVelocity = Vec3<float>::Zero();
        for (auto &v: observedVelocities) {
            MeanObservedVelocity += v;    // std::mean(observedVelocities);
        }
        MeanObservedVelocity /= num;
        // filter.Update(deltaTime, MeanObservedVelocity);
        double z[3] = {MeanObservedVelocity[0], MeanObservedVelocity[1], MeanObservedVelocity[2]};
        filter->step(deltaV, z);
    } else {
        double z[3] = {estimatedVelocity[0], estimatedVelocity[1], estimatedVelocity[2]};
        filter->step(deltaV, z);
    }
    float x[3] = {(float)filter->getX(0), (float)filter->getX(1), (float)filter->getX(2)};
    float vx = velocityFilterX.CalculateAverage(x[0]);
    float vy = velocityFilterY.CalculateAverage(x[1]);
    float vz = velocityFilterZ.CalculateAverage(x[2]);
    estimatedVelocity << vx, vy, vz; // world frame
    estimatedVelocity = rotMat.transpose() * estimatedVelocity; // base frame
}
} //namespace Quadruped

