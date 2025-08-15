#include "control/mpc.hpp"

Mpc::Mpc(double wheelbase, double max_speed, double max_steering){
    this->wheelbase = wheelbase;
    this->max_speed = max_speed;
    this->max_steer = max_steering;
    
}

double Mpc::get_dt(){
    return dt;
}


State Mpc::predict_next_state(State& curr, double vel, double steering, double dt){
    State next;
    next.x = curr.x + vel * cos(curr.yaw) * dt;
    next.y = curr.y + vel * sin(curr.yaw) * dt;
    next.yaw = curr.yaw + (vel/wheelbase) * tan(steering) * dt;
    return next;
}

double Mpc::compute_cost(std::vector<State>& states,
                        std::vector<double>& steerings,
                        std::vector<double>& velocities)
{
    double cost = 0.0;

    for (size_t i =0; i<states.size(); i++){
        State s = states[i];

        //position error
        double dx = s.x - target.x;
        double dy = s.y - target.y;
        cost += weights.position * (dx*dx + dy*dy);

        //orientation error
        double d_yaw = s.yaw - target.yaw;

        //wrap around
        while (d_yaw > M_PI) d_yaw -= 2 * M_PI;
        while (d_yaw < -M_PI) d_yaw += 2 * M_PI;

        cost += weights.orientation * d_yaw*d_yaw;
        //control effort avoiding sharp turns if possible
        cost += weights.steering * steerings[i] * steerings[i];

    }
        return cost;
}



double Mpc::evaluate_cost(std::vector<Command>& control_sequence, State& start){
    std::vector<State> states;
    std::vector<double> steers;
    std::vector<double> vels;

    State s = start;
    for(size_t i=0;  i<control_sequence.size(); i++){
        s = predict_next_state(s, control_sequence[i].vel, control_sequence[i].steer, dt);
        states.push_back(s);
        steers.push_back(control_sequence[i].steer);
        vels.push_back(control_sequence[i].vel);
    }
    return compute_cost(states,steers,vels);
}

int Mpc::set_target(double x_diff, double y_diff, double yaw_diff){
        target.x = x_diff;
        target.y = y_diff;
        target.yaw = yaw_diff;
        return 0;
}

void Mpc::print_controls() {
    for (size_t i = 0; i < controls.size(); i++) {
        std::cout << "Step " << i
                  << " | vel: " << controls[i].vel
                  << " | steer: " << controls[i].steer
                  << "\n";
    }
}

Command Mpc::get_command(State& start) {
    controls.clear();
    controls.resize(horizon, {max_speed*0.5, 0.0});

    const double vel_d = 2;    // Finite difference step for velocity
    const double steer_d = 0.05;   // Finite difference step for steering
    double alpha = 0.1;           // Initial learning rate
    const double tol = 1e-6;      // Convergence tolerance
    const double min_alpha = 0.01; // Minimum step size

    double prev_cost = evaluate_cost(controls, start);
    int n_params = horizon * 2;

    double max_grad_norm = 1.0;
    double step;

    Eigen::VectorXd grad(n_params);
    
    for (int iter = 0; iter < max_iterations; iter++) {
        // 1. Compute gradient
        for (int p = 0; p < n_params; p++) {
            std::vector<Command> temp = controls;
            step = (p % 2 == 0) ? vel_d : steer_d;
            (p % 2 == 0) ? temp[p/2].vel += step : temp[p/2].steer += step;
            grad[p] = (evaluate_cost(temp, start) - prev_cost) / step;
        }
        
        //normalizing gradient vector (SUPER IMPORTANT)
        double grad_norm = grad.norm();
        if (grad_norm > max_grad_norm) {
            grad *= (max_grad_norm / grad_norm);
        }
        //std::cout<<"GRAD : "<<grad<<"\n";

        // 2. Update controls with gradient descent
        for (int p = 0; p < n_params; p++) {
            step = (p % 2 == 0) ? vel_d : steer_d;
            int sign = (grad[p] > 0) - (grad[p] < 0);
            double update = -(sign * step);  // needs tweaking
            if (p % 2 == 0) {
                controls[p/2].vel = std::max(0.0, 
                    std::min(max_speed, controls[p/2].vel + update));
            } else {
                controls[p/2].steer = std::max(-max_steer,
                    std::min(max_steer, controls[p/2].steer + update));
            }
        }

        // std::cout<<"\niter: "<<iter<<" controls:\n";
        // print_controls();

        // 3. Check convergence
        double new_cost = evaluate_cost(controls, start);
        if (fabs(prev_cost - new_cost) < tol) break;
        prev_cost = new_cost;

        // 4. Simple step size adaptation
        alpha = std::max(min_alpha, alpha * (new_cost < prev_cost ? 1.1 : 0.9));
    }

    return controls.front();
}

void Mpc::generate_controls(){
    controls.clear();
    for (size_t i=0; i<horizon; i++){
        Command temp;
        temp.vel=max_speed;
        temp.steer=0.0;

        controls.push_back(temp);
    }
}

