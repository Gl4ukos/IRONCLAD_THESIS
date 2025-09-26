#include <vector>
#include <cmath>
#include <iostream>
#include "control/lateral.hpp"
#include <tf2/utils.h>


Lateral::Lateral(double wheelbase, double max_speed, double max_steering)
    : wheelbase(wheelbase), max_speed(max_speed), max_steering(max_steering), speed(0.1) {}

int Lateral::set_target(double x_diff, double y_diff, double yaw_diff){
        target_x = x_diff;
        target_y = y_diff;
        target_yaw = yaw_diff;
        curr_dist_sq = (target_x*target_x) + (target_y * target_y);
        return 0;
}

void Lateral::set_max_speed(double val){
    max_speed = val;
}
void Lateral::set_max_steering(double val){
    max_steering = val;
}

double Lateral::clip_speed(double val){
    return std::max(std::min(val,max_speed),0.0);
}

double Lateral::calc_speed(){
    curr_dist = sqrt(curr_dist_sq);
    speed = std::min(max_speed, Kp*curr_dist - Kd*speed); 
    return clip_speed(speed);
}

double Lateral::clip_steering(double val){
    return std::max(std::min(val, max_steering), -max_steering);
}

double Lateral::calc_steering(){
    double k= 0.5;
    if(speed>0){
        steering = target_yaw + atan2(k*target_y,speed);
    }else{
        steering =0;
    }
    return clip_steering(steering);
}

int Lateral::getErrorMarkers(visualization_msgs::Marker &target_yaw_marker, visualization_msgs::Marker &target_y_marker, double r_x, double r_y, double r_yaw){
    ros::Time now = ros::Time::now();

    // ----- Lateral error vector -----
    target_y_marker.header.frame_id = "world"; 
    target_y_marker.header.stamp = now;
    target_y_marker.ns = "lateral_error";
    target_y_marker.id = 0;
    target_y_marker.type = visualization_msgs::Marker::ARROW;
    target_y_marker.action = visualization_msgs::Marker::ADD;
    
    // Start at robot position
    geometry_msgs::Point start;
    start.x = r_x;
    start.y = r_y;
    start.z = 0;

    // End at lateral error along Y-axis in robot frame
    geometry_msgs::Point end;
    double x_global = r_x + 0;          // along x = 0 (forward)
    double y_global = r_y + target_y;   // lateral error
    end.z = 0;


    //rotating for r_yaw, around the robot
    end.x = r_x + target_y * (-sin(r_yaw));
    end.y = r_y + target_y * ( cos(r_yaw));

    target_y_marker.points.clear();
    target_y_marker.points.push_back(start);
    target_y_marker.points.push_back(end);

    target_y_marker.scale.x = 0.05; // shaft diameter
    target_y_marker.scale.y = 0.1;  // head diameter
    target_y_marker.scale.z = 0.1;  // head length
    target_y_marker.color.r = 0.8;
    target_y_marker.color.g = 0.9;
    target_y_marker.color.b = 0.2;
    target_y_marker.color.a = 1.0;

    // ----- Target orientation vector -----
    target_yaw_marker.header.frame_id = "world";
    target_yaw_marker.header.stamp = now;
    target_yaw_marker.ns = "target_yaw";
    target_yaw_marker.id = 1;
    target_yaw_marker.type = visualization_msgs::Marker::ARROW;
    target_yaw_marker.action = visualization_msgs::Marker::ADD;

    // Start at robot
    geometry_msgs::Point start_yaw = start;
    geometry_msgs::Point end_yaw;

    double length = 1.0; // scale for visualization
    end_yaw.x = r_x + length * cos(target_yaw + r_yaw);
    end_yaw.y = r_y + length * sin(target_yaw + r_yaw);
    end_yaw.z = 0;

    target_yaw_marker.points.clear();
    target_yaw_marker.points.push_back(start_yaw);
    target_yaw_marker.points.push_back(end_yaw);

    target_yaw_marker.scale.x = 0.05;
    target_yaw_marker.scale.y = 0.1;
    target_yaw_marker.scale.z = 0.1;
    target_yaw_marker.color.r = 1.0;
    target_yaw_marker.color.g = 1.0;
    target_yaw_marker.color.b = 0.0;
    target_yaw_marker.color.a = 1.0;

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
