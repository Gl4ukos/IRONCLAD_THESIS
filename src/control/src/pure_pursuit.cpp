#include <vector>
#include <cmath>
#include <iostream>
#include "control/pure_pursuit.hpp"



Pure_pursuit::Pure_pursuit(double wheelbase, double max_speed, double max_steering)
    : wheelbase(wheelbase), max_speed(max_speed), max_steering(max_steering) {
    }


// expects the DISTANCE to target and DIFF in yaw
int Pure_pursuit::set_target(double x, double y){
        //if target unachievable return -1
        target_x = x;
        target_y = y;
        curr_dist_sq = (target_x*target_x) + (target_y * target_y);
        return 0;
}

double Pure_pursuit::calc_speed(){
    curr_dist = sqrt(curr_dist_sq);
    speed = std::min(max_speed, Kp*curr_dist - Kd*curr_dist); 
    return speed;
}

double Pure_pursuit::calc_steering(){
    curvature = (2*target_y)/curr_dist_sq;
    return atan(wheelbase * curvature);
}

int Pure_pursuit::get_trajectory(nav_msgs::Path *path_msg, double resolution){
    path_msg->header.stamp = ros::Time::now();

    double step = curr_dist / resolution;
    double radius = 1/curvature;

    double d_theta = curvature*step;

    //center of circle
    double xc = 0.0;
    double yc = radius * 1; 

    for (int i=0; i< resolution; i++){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = path_msg->header.stamp;
        pose.header.frame_id = path_msg->header.frame_id;


        double angle = d_theta * i/step;
        double x = xc + radius * sin(angle);
        double y = yc - radius * cos(angle);
        
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0;
        
        path_msg->poses.push_back(pose);

    }

    return 0;
}
