#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense> 

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
    double orientation = 0;
    double steering = 0;
    double effort = 0;
};

class Mpc{
    private:
    //variables
    State curr_state;
    State target;
    Weights weights;
    std::vector<Command> controls;
    
    //hyperparameters
    size_t horizon=2;
    double dt =0.2;
    double wheelbase;
    double max_speed;
    double max_steer;
    int max_iterations = 3;



    public: 
    Mpc(double wheelbase, double max_speed, double max_steering);
    State predict_next_state(State& curr, double vel, double steering, double dt);
    int set_target(double x, double y, double yaw);
    double compute_cost(std::vector<State>& states,
                        std::vector<double>& steerings,
                        std::vector<double>& velocities
    );
    double evaluate_cost(std::vector<Command>& control_sequence, State& start);
    Command get_command(State& start);
    void generate_controls();
    double get_dt();
    void print_controls();
};
