/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Publishing odometry information over ROS.
* Author: Zhu Linsen & Zhao Yao
* Create: 2021-12-03
* Notes: None.
* Modify: init the file. @ Zhu Linsen
*/


#ifndef ASCEND_QUADRUPED_CPP_INCLUDE_ROS_ROBOT_ODOM_ESTIMATOR_H_
#define ASCEND_QUADRUPED_CPP_INCLUDE_ROS_ROBOT_ODOM_ESTIMATOR_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "mpc_controller/locomotion_controller.h"

namespace Quadruped {
//namespace Quadruped {
class RobotOdometryEstimator {
public:
    RobotOdometryEstimator(Robot *robotIn, LocomotionController *locomotionController, ros::NodeHandle &nhIn);
    void PublishOdometry();
private:
    // RobotVelocityEstimator *stateEstimator;
    Robot *robot;
    RobotEstimator *robotEstimator;
    LocomotionController *locomotionController;

    ros::NodeHandle nh;
    ros::Publisher pubOdometry;
    // broadcast odom to base_link
    tf::TransformBroadcaster odomBroadcaster;
    ros::Time currentTime;
    ros::Time lastTime;
    // the position of the base
    float x = 0.f;
    float y = 0.f;
    float theta = 0.f;

    // velocity of the base
    Eigen::Matrix<float, 3, 1> estimatedVelocity = Eigen::Matrix<float, 3, 1>::Zero();
    // current angle of the base
    Eigen::Matrix<float, 3, 1> baseRollPitchYaw = Eigen::Matrix<float, 3, 1>::Zero();
    // angular Velocity of the base
    Eigen::Matrix<float, 3, 1> baseRollPitchYawRate = Eigen::Matrix<float, 3, 1>::Zero();
    // current position of the base
    Eigen::Matrix<float, 3, 1> basePosition = Eigen::Matrix<float, 3, 1>::Zero();
};
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_INCLUDE_ROS_ROBOT_ODOM_ESTIMATOR_H_
