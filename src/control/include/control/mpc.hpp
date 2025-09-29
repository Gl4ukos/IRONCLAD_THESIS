#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense> 
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

struct State{
    double x;
    double y;
    double yaw;
};

struct Command{
    double vel;
    double steer;
};

struct Weights{
    double position=1;
    double orientation = 0.2;
    double steering = 0;
    double effort = 0;
};

class Mpc{
    private:
    //variables
    State curr_state;
    State target;
    double curr_dist_sq;
    double speed_pid;
    Weights weights;
    std::vector<Command> controls;
    std::vector<State> resulting_states;
    std::vector<State> current_states;
    
    //hyperparameters
    size_t horizon=3;
    double dt =0.2;
    double wheelbase;
    double max_speed;
    double max_steer;

    int max_iterations = 40;
    double Kp = 10;
    double Kd = 1;
    double Ki = 0;
    double pid_prev_error;



    public: 
    Mpc(double wheelbase, double max_speed, double max_steering);
    State predict_next_state(State& curr, double vel, double steering, double dt);
    int set_target(double x, double y, double yaw);
    double compute_cost(std::vector<double>& steerings, std::vector<double>& velocities);
    double evaluate_cost(std::vector<Command>& control_sequence, State& start);
    Command get_command(State& start);
    double calc_speed_pid(double dt);
    void set_max_speed(double val);
    void generate_controls();
    double get_dt();
    void print_controls();
    void get_trajectory(nav_msgs::Path *path_msg, double  robot_x, double  robot_y, double  robot_yaw);
};
