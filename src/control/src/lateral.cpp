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



int Lateral::get_trajectory(nav_msgs::Path *path_msg, double resolution, double  robot_x, double  robot_y, double  robot_yaw){
 return 0;
}
