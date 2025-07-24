#include "ackermann_simulation/command_publishers.hpp"

command_publishers::command_publishers(ros::NodeHandle& nh)
{
    steering_pub = nh.advertise<std_msgs::Float64MultiArray>("/ack/front_steering_controller/command", 10);
    velocity_pub = nh.advertise<std_msgs::Float64MultiArray>("/ack/rear_drive_controller/command", 10);

    vel_cmd.data.resize(2);
    steer_cmd.data.resize(2);

    ros::service::waitForService("/gazebo/set_model_state");
    set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

}


float command_publishers::clip_vel(float val) {
    if (val < -abs_max_vel) return -abs_max_vel;
    else if (val > abs_max_vel) return abs_max_vel;
    else return val;
}

float command_publishers::clip_steer(float val) {
    if (val < -abs_max_steer) return -abs_max_steer;
    else if (val > abs_max_steer) return abs_max_steer;
    else return val;
}

void command_publishers::publishSteering(double steer)
{  
    steer = clip_steer(steer);
    steer_cmd.data[0] = steer;
    steer_cmd.data[1] = steer;
    steering_pub.publish(steer_cmd);
}

void command_publishers::publishVelocity(double vel)
{
    vel = clip_vel(vel);
    vel_cmd.data[0] = vel;
    vel_cmd.data[1] = vel;
    velocity_pub.publish(vel_cmd);
}


double command_publishers::get_max_speed(){
    return abs_max_vel;
}

double command_publishers::get_max_steer(){
    return abs_max_steer;
}


void command_publishers::reset_position(){
    gazebo_msgs::SetModelState set_model_state_srv;
    gazebo_msgs::ModelState &state = set_model_state_srv.request.model_state;

    // Replace with your actual model name from /gazebo/model_states
    state.model_name = "ackermann_car";
    state.reference_frame = "world";  // Important for correct positioning

    // Set desired position
    state.pose.position.x = 0.0;
    state.pose.position.y = 0.0;
    state.pose.position.z = 0.0;

    // Set desired orientation (no rotation)
    state.pose.orientation.x = 0.0;
    state.pose.orientation.y = 0.0;
    state.pose.orientation.z = 0.0;
    state.pose.orientation.w = 1.0;

    // Zero velocity
    state.twist.linear.x = 0.0;
    state.twist.linear.y = 0.0;
    state.twist.linear.z = 0.0;
    state.twist.angular.x = 0.0;
    state.twist.angular.y = 0.0;
    state.twist.angular.z = 0.0;

    // Call the service
    if (set_model_state_client.call(set_model_state_srv)){
        ROS_INFO("CAR POSITION RESET.");
    }else{
        ROS_ERROR("FAILED TO RESET CAR.");
    }
    return;
}


