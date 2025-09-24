#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>

class Pure_pursuit{
    private:
    
    // parameters
    double wheelbase; //distance from rear to front wheels
    double max_speed;
    double max_steering;
    double Kp = 10;
    double Kd = 1;
    double Ki = 0;

    // variables
    double target_x;
    double target_y;
    double curr_dist_sq;
    double curr_dist;
    double curr_yaw;
    double curvature;
    double speed;



    public:
    Pure_pursuit(double wheelbase, double max_speed, double max_steering);
    // expects the DISTANCE to target
    int set_target(double x, double y);
    void set_max_speed(double new_max);
    void set_max_steer(double new_max);
    double clip_speed(double speed);
    double clip_steering(double steering);
    double calc_speed();
    double calc_steering();
    int get_trajectory(nav_msgs::Path *path_msg, double resolution,double  double_x,double  double_y ,double double_yaw);
    
};