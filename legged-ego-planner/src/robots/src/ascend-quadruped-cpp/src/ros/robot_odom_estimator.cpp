/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Publishing odometry information over ROS.
* Author: Zhu Linsen & Zhao Yao
* Create: 2021-12-03
* Notes: None.
* Modify: init the file. @ Zhu Linsen
*/

#include "ros/robot_odom_estimator.h"

namespace Quadruped {
// init constructor
RobotOdometryEstimator::RobotOdometryEstimator(Robot *robotIn,
                                               LocomotionController *locomotionControllerIn,
                                               ros::NodeHandle &nhIn)
: robot(robotIn), nh(nhIn)
{
    robotEstimator = locomotionControllerIn->GetRobotEstimator();
    baseRollPitchYaw = robot->baseRollPitchYaw;
    baseRollPitchYawRate = robot->baseRollPitchYawRate;
    pubOdometry = nh.advertise<nav_msgs::Odometry>("legOdom", 1);
    lastTime = ros::Time::now();
    x = basePosition[0];
    y = basePosition[1];
    theta = baseRollPitchYaw[2];
    ROS_INFO("robot_odom_estimator init success...");
}

void RobotOdometryEstimator::PublishOdometry()
{
    currentTime = ros::Time::now();
    estimatedVelocity = robotEstimator->GetEstimatedVelocity();
    baseRollPitchYawRate = robot->GetBaseRollPitchYawRate();
    float vX = estimatedVelocity[0];
    float vY = estimatedVelocity[1];
    float vTheta = baseRollPitchYawRate[2];
//    std::cout << "vTheta:" << vTheta << std::endl;

    // compute odometry in a typical way given the velocities of the robot
    float deltaT = (currentTime - lastTime).toSec();
    float deltaX = (vX * cos(theta) - vY * sin(theta)) * deltaT;
    float deltaY = (vX * sin(theta) + vY * cos(theta)) * deltaT;
    float deltaTheta = vTheta * deltaT;
//    std::cout << "vTheta:" << vTheta << std::endl;

    x += deltaX * 1.1f;
    y += deltaY * 1.1f;
    theta += deltaTheta;
//    ROS_INFO("dT: %f, x: %f, y: %f, theta: %f", deltaT, x, y, theta);

    //update robot position
    robot->basePosition[0] = x;
    robot->basePosition[1] = y;

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);

    // publish the transform over tf
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.stamp = currentTime;
    odomTrans.header.frame_id = "odom";
    odomTrans.child_frame_id = "base";

    odomTrans.transform.translation.x = x;
    odomTrans.transform.translation.y = y;
    odomTrans.transform.translation.z = 0.0;
    odomTrans.transform.rotation = odomQuat;

    // send the transform
    odomBroadcaster.sendTransform(odomTrans);

    // publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odomQuat;

    //set the velocity
    odom.child_frame_id = "base";
    odom.twist.twist.linear.x = vX;
    odom.twist.twist.linear.y = vY;
    odom.twist.twist.angular.z = vTheta;

    //publish the message
    pubOdometry.publish(odom);

    lastTime = currentTime;
}
} // namespace Quadruped
