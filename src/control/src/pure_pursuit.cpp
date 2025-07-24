#include <vector>
#include <cmath>
#include <iostream>
#include "control/pure_pursuit.hpp"
#include <tf2/utils.h>



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
    speed = std::min(max_speed, Kp*curr_dist - Kd*speed); 
    return speed;
}

double Pure_pursuit::calc_steering(){
    curvature = (2*target_y)/curr_dist_sq;
    return atan(wheelbase * curvature);
}



int Pure_pursuit::get_trajectory(nav_msgs::Path *path_msg, double resolution, double  robot_x, double  robot_y, double  robot_yaw){
    
    path_msg->header.stamp = ros::Time::now();
    path_msg->poses.clear();
    
    double local_x,local_y; // local trajectory coordinates

    if (std::abs(curvature) < 1e-6) {
        // Straight line approximation
        for (int i = 0; i < resolution; i++) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = path_msg->header.stamp;
            pose.header.frame_id = path_msg->header.frame_id;

            double dist = curr_dist * i / (resolution - 1);

            local_x = dist;
            local_y = 0;
            pose.pose.orientation.w= 1.0;

            pose.pose.position.x = robot_x + cos(robot_yaw)*local_x - sin(robot_yaw)*local_y; //global
            pose.pose.position.y = robot_y + sin(robot_yaw)*local_x + cos(robot_yaw)*local_y;
            pose.pose.orientation.w = 1.0;

            path_msg->poses.push_back(pose);
        }
        return 0;
    }

    double radius = 1.0 / curvature;
    double total_angle = curvature * curr_dist; // total arc angle
    double angle_increment = total_angle / (resolution - 1);

    double xc = 0.0;
    double yc = radius;

    if (curvature < 0) {
        yc = -radius;
    }

    for (int i = 0; i < resolution; i++){
        double angle = (angle_increment * i);

        local_x = xc + radius * sin(angle);
        local_y = yc - radius * cos(angle);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = path_msg->header.stamp;
        pose.header.frame_id = path_msg->header.frame_id;

        pose.pose.position.x = robot_x + cos(robot_yaw) *local_x -sin(robot_yaw)*local_y;
        pose.pose.position.y = robot_y + sin(robot_yaw)*local_x + cos(robot_yaw) * local_y;
        pose.pose.position.z = 0;

        // Optionally, compute orientation facing tangent to path
        double heading = angle + (curvature > 0 ? M_PI/2 : -M_PI/2) + robot_yaw;
        tf2::Quaternion q;
        q.setRPY(0, 0, heading);
        pose.pose.orientation = tf2::toMsg(q);

        path_msg->poses.push_back(pose);
    }

    return 0;
}
