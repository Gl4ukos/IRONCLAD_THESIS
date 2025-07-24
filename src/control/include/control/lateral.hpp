#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Lateral{
    private:
    double wheelbase; //distance from rear to front wheels
    double target_x;
    double target_y;
    double target_yaw;
    double max_speed;
    double max_steering;

    double curr_dist_sq;
    double curr_dist;
    double curr_yaw;
    double speed;
    double steering;

    double Kp = 1;
    double Kd = 0;
    double Ki = 0;

    public:
    Lateral(double wheelbase, double max_speed, double max_steering);
    // expects the DISTANCE to target
    int set_target(double x, double y, double yaw);
    double calc_speed();
    double calc_steering();
    int get_trajectory(nav_msgs::Path *path_msg, double resolution,double  double_x,double  double_y ,double double_yaw);
    int getErrorMarkers(visualization_msgs::Marker target_yaw_marker, visualization_msgs::Marker target_y_marker, double r_x, double r_y, double r_yaw);
};