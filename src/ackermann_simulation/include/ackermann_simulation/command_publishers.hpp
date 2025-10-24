#pragma once


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

class command_publishers
{
public:
    command_publishers(ros::NodeHandle& nh);

    void publishVelocity(double vel);
    void publishSteering(double steer);

    void reset_position();
    double get_published_speed();
    double get_published_steering();

    ros::ServiceClient set_model_state_client;

private:
    ros::Publisher velocity_pub;
    ros::Publisher steering_pub;

    double speed;
    double steering;

    std_msgs::Float64MultiArray vel_cmd;
    std_msgs::Float64MultiArray steer_cmd;

};
