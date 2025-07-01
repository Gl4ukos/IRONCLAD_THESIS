#pragma once


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

class command_publishers
{
public:
    command_publishers(ros::NodeHandle& nh);

    void publishVelocity(double vel);
    void publishSteering(double steer);
    float clip_vel(float val);
    float clip_steer(float val);

private:
    ros::Publisher velocity_pub;
    ros::Publisher steering_pub;

    std_msgs::Float64MultiArray vel_cmd;
    std_msgs::Float64MultiArray steer_cmd;

    double abs_max_vel = 10;
    double abs_max_steer = 0.9;
};
