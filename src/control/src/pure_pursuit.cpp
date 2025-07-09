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
        return 0;
}

double Pure_pursuit::calc_speed(){
        return max_speed;        

}

double Pure_pursuit::calc_steering(){
    double dx = target_x;
    double dy = target_y;

    double curvature = (2*dy)/(dx*2 + dy*2);
    return atan(wheelbase * curvature);
}

