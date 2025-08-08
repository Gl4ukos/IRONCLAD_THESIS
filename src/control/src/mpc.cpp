#include "control/mpc.hpp"

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
Command Mpc::get_command(State& start){
    controls.clear();
    controls.resize(horizon, {max_speed*0.8, 0.0}); // initial guess

    double eps = 1e-4;
    double alpha = 1.0;
    double tol = 1e-6;

    double prev_cost = evaluate_cost(controls, start);

    int n_params = horizon * 2;
    for (int iter = 0; iter < max_iterations; iter++){
        Eigen::VectorXd grad(n_params);
        Eigen::MatrixXd H(n_params, n_params);

        // Compute gradient by finite differences
        for (int p = 0; p < n_params; p++){
            std::vector<Command> temp_controls = controls;
            if (p % 2 == 0) {
                temp_controls[p / 2].vel += eps;
            } else {
                temp_controls[p / 2].steer += eps;
            }
            double cost_plus = evaluate_cost(temp_controls, start);
            grad[p] = (cost_plus - prev_cost) / eps;
        }

        // Compute Hessian by finite differences (second derivatives)
        for (int i = 0; i < n_params; i++){
            for (int j = 0; j < n_params; j++){
                std::vector<Command> temp_controls = controls;

                if (i % 2 == 0) temp_controls[i / 2].vel += eps;
                else temp_controls[i / 2].steer += eps;

                if (j % 2 == 0) temp_controls[j / 2].vel += eps;
                else temp_controls[j / 2].steer += eps;

                double cost_pp = evaluate_cost(temp_controls, start);

                temp_controls = controls;
                if (i % 2 == 0) temp_controls[i / 2].vel += eps;
                else temp_controls[i / 2].steer += eps;

                double cost_p = evaluate_cost(temp_controls, start);

                temp_controls = controls;
                if (j % 2 == 0) temp_controls[j / 2].vel += eps;
                else temp_controls[j / 2].steer += eps;

                double cost_p2 = evaluate_cost(temp_controls, start);

                double value = (cost_pp - cost_p - cost_p2 + prev_cost) / (eps * eps);
                H(i, j) = value;
            }
        }

        // Solve Newton step: H * delta_u = -grad
        Eigen::VectorXd delta_u = H.ldlt().solve(-grad);

        // Update controls using step size alpha
        for (int p = 0; p < n_params; p++){
            if (p % 2 == 0){
                controls[p / 2].vel += alpha * delta_u[p];
                // Optionally clamp velocity to allowed bounds here
            } else {
                controls[p / 2].steer += alpha * delta_u[p];
                // Optionally clamp steering to allowed bounds here
            }
        }

        double new_cost = evaluate_cost(controls, start);
        if (fabs(prev_cost - new_cost) < tol) {
            break; // Converged
        }
        prev_cost = new_cost;
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

