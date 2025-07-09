#pragma once

class Pure_pursuit{
    private:
    double wheelbase; //distance from rear to front wheels
    double target_x;
    double target_y;
    double max_speed;
    double max_steering;


    public:
    Pure_pursuit(double wheelbase, double max_speed, double max_steering);
    // expects the DISTANCE to target
    int set_target(double x, double y);
    double calc_speed();
    double calc_steering();
};