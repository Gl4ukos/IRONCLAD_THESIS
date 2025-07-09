#include <vector>
#include <cmath>
#include <iostream>
#include "control/pure_pursuit.hpp"



Pure_pursuit::Pure_pursuit(double wheelbase, double max_speed, double max_steering)
    : wheelbase(wheelbase), max_speed(max_speed), max_steering(max_steering) {}


// expects the DISTANCE to target
int Pure_pursuit::set_target(double x, double y){
        //if target unachievable return -1
        target_x = x;
        target_y = y;
        curr_dist_sq = (target_x*target_x) + (target_y * target_y);
        return 0;
}

double Pure_pursuit::calc_speed(){
        double dist = sqrt(curr_dist_sq);
        return std::min(max_speed, Kp*dist - Kd*dist);
}

double Pure_pursuit::calc_steering(){
    double curvature = (2*target_y)/curr_dist_sq;
    return atan(wheelbase * curvature);
}

