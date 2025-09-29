#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    double prev_error;

    double Kp = 8;
    double Kd = 1;
    double Ki = 0;

    public:
    Lateral(double wheelbase, double max_speed, double max_steering);
    // expects the DISTANCE to target
    int set_target(double x, double y, double yaw);
    void set_max_speed(double val);
    void set_max_steering(double val);
    double clip_speed(double val);
    double clip_steering(double val);
    double calc_speed(double dt);
    double calc_steering();
    int get_trajectory(nav_msgs::Path *path_msg, double resolution,double  double_x,double  double_y ,double double_yaw);
    int getErrorMarkers(visualization_msgs::Marker &target_yaw_marker, visualization_msgs::Marker &target_y_marker, double r_x, double r_y, double r_yaw);
};