#include <vector>
#include <cmath>
#include <iostream>
#include "control/lateral.hpp"
#include <tf2/utils.h>


Lateral::Lateral(double wheelbase, double max_speed, double max_steering)
    : wheelbase(wheelbase), max_speed(max_speed), max_steering(max_steering) {
    }

int Lateral::set_target(double x_diff, double y_diff, double yaw_diff){
        target_x = x_diff;
        target_y = y_diff;
        target_yaw = yaw_diff;
        curr_dist_sq = (target_x*target_x) + (target_y * target_y);
        return 0;
}

double Lateral::calc_speed(){
    curr_dist = sqrt(curr_dist_sq);
    speed = std::min(max_speed, Kp*curr_dist - Kd*speed); 
    return speed;
}

double Lateral::calc_steering(){
    double k=2.5;
    if(speed>0){
        steering = target_yaw + atan2(k*target_y,speed);
        std::cout<<"e_d:"<< target_y << "   er:" << target_yaw <<"\n";
    }else{
        steering =0;
    }

    return steering;
}

int Lateral::getErrorMarkers(visualization_msgs::Marker target_yaw_marker, visualization_msgs::Marker target_y_marker, double r_x, double r_y, double r_yaw){
    

    return 0;
}


int Lateral::get_trajectory(nav_msgs::Path *path_msg, double resolution, double  robot_x, double  robot_y, double  robot_yaw){
    
    path_msg->header.stamp = ros::Time::now();
    path_msg->poses.clear();
    double curvature = tan(steering)/wheelbase;
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

    double yc = radius;

    if (curvature < 0) {
        yc = -radius;
    }

    for (int i = 0; i < resolution; i++){
        double angle = (angle_increment * i);

        local_x = radius * sin(angle);
        local_y = radius * (1-cos(angle));

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
