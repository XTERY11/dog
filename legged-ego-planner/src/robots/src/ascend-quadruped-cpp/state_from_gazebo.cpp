
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
ros::Publisher robotVelocity_BASE_frame_pub;
void callback_BASE(const gazebo_msgs::LinkStates::ConstPtr &msg) {
    int index = 0;
    for (auto &linkName : msg->name) {
        if (linkName == "a1_gazebo::base")
            break;
        ++index;
    }

    static tf::TransformBroadcaster bf2;
    tf::Transform transform2;
    transform2.setRotation(tf::Quaternion(0,
                                         0,
                                         0,
                                         1));
    transform2.setOrigin(tf::Vector3(0,
                                    0,
                                    0));
    bf2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "odom"));


    static tf::TransformBroadcaster bf;
    tf::Transform transform;
    transform.setRotation(tf::Quaternion(msg->pose[index].orientation.x,
                                         msg->pose[index].orientation.y,
                                         msg->pose[index].orientation.z,
                                         msg->pose[index].orientation.w));
    transform.setOrigin(tf::Vector3(msg->pose[index].position.x,
                                    msg->pose[index].position.y,
                                    msg->pose[index].position.z));
    bf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base"));


    nav_msgs::Odometry legOdom;
    legOdom.header.stamp = ros::Time::now();
    legOdom.header.frame_id = "odom";
    legOdom.child_frame_id = "base";

    // set the position
    legOdom.pose.pose.position.x = msg->pose[index].position.x;
    legOdom.pose.pose.position.y = msg->pose[index].position.y;
    legOdom.pose.pose.position.z = msg->pose[index].position.z;
    legOdom.pose.pose.orientation.w = msg->pose[index].orientation.w ;
    legOdom.pose.pose.orientation.x = msg->pose[index].orientation.x ;
    legOdom.pose.pose.orientation.y = msg->pose[index].orientation.y ;
    legOdom.pose.pose.orientation.z = msg->pose[index].orientation.z ;

    // set the velocity

    legOdom.twist.twist.linear.x = msg->twist[index].linear.x;
    legOdom.twist.twist.linear.y = msg->twist[index].linear.y;
    legOdom.twist.twist.linear.z = msg->twist[index].linear.z;

    legOdom.twist.twist.angular.x =msg->twist[index].angular.x;
    legOdom.twist.twist.angular.y =msg->twist[index].angular.y;
    legOdom.twist.twist.angular.z =msg->twist[index].angular.z;

    robotVelocity_BASE_frame_pub.publish(legOdom);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "state_from_gazebo");
    ros::NodeHandle nh;
    ros::Subscriber tfState_BASE_sub;
    ros::Subscriber robotVelocity_sub;

    tfState_BASE_sub = nh.subscribe("/gazebo/link_states", 1, &callback_BASE);

    robotVelocity_BASE_frame_pub = nh.advertise<nav_msgs::Odometry>("/legOdom", 10);

    ros::spin();
    return 0;
}
