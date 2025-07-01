#include "../include/command_publishers.hpp"

command_publishers::command_publishers(ros::NodeHandle& nh)
{
    steering_pub = nh.advertise<std_msgs::Float64MultiArray>("/ack/front_steering_controller/command", 10);
    velocity_pub = nh.advertise<std_msgs::Float64MultiArray>("/ack/rear_drive_controller/command", 10);

    vel_cmd.data.resize(2);
    steer_cmd.data.resize(2);
}


float command_publishers::clip_vel(float val) {
    if (val < -abs_max_vel) return -abs_max_vel;
    else if (val > abs_max_vel) return abs_max_vel;
    else return val;
}

float command_publishers::clip_steer(float val) {
    if (val < -abs_max_steer) return -abs_max_steer;
    else if (val > abs_max_steer) return abs_max_steer;
    else return val;
}

void command_publishers::publishSteering(double steer)
{  
    steer_cmd.data[0] = steer;
    steer_cmd.data[1] = steer;
    steering_pub.publish(steer_cmd);
}

void command_publishers::publishVelocity(double vel)
{
    vel_cmd.data[0] = vel;
    vel_cmd.data[1] = vel;
    velocity_pub.publish(vel_cmd);
}
