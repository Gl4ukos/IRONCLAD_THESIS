#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "ackermann_simulation/command_publishers.hpp"
#include "control/pure_pursuit.hpp"
#include "control/lateral.hpp"
#include "control/mpc.hpp"
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <fstream>
#include <vector>
#include <gazebo_msgs/ModelStates.h>
#include "command_transmitter.cpp"


int BOOST = 1;
double MAX_SPEED = 10.0;
double MAX_STEER = 0.75;
double LOOKAHEAD = 1.0;
double wheelbase = 1.0; //! probably not true  
int controller_mode =0;
double curr_x, curr_y, curr_z, curr_yaw;

double MAX_SPEED_PP = 10.0;
double MIN_SPEED_PP = 2.0;

double MAX_SPEED_LAT = 10.0;
double MIN_SPEED_LAT = 2.0;

double MAX_SPEED_MPC = 10.0;
double MIN_SPEED_MPC = 2.0;


int load_trajectory(nav_msgs::Path &trajectory, std::string filepath){
    std::ifstream file(filepath);
    if (!file.is_open()) {
        ROS_ERROR_STREAM("Failed to open CSV file: " << filepath);
        return -1;
    }

    trajectory.poses.clear();
    trajectory.header.frame_id = "world";
    trajectory.header.stamp = ros::Time::now();

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double x, y, z, qx, qy, qz, qw;
        char comma;

        if (!(ss >> x >> comma >> y >> comma >> z >> comma >> qx >> comma >> qy >> comma >> qz >> comma >> qw)) {
            ROS_WARN_STREAM("Skipping invalid line: " << line);
            continue;
        }

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;

        trajectory.poses.push_back(pose);
    }

    return 0;
}


double eucl_dist(double t_x,double t_y){
    return sqrt(t_x*t_x + t_y*t_y);
}

void update_pose(nav_msgs::Odometry msg){
    curr_x = msg.pose.pose.position.x;
    curr_y = msg.pose.pose.position.y;
    curr_z = msg.pose.pose.position.z; 

    curr_yaw = tf2::getYaw(msg.pose.pose.orientation);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;

    ros::Rate rate(10);  // 10 Hz

    ros::Publisher spawn_target_pub =  nh.advertise<geometry_msgs::PoseStamped>("/target_spawner/target_pose", 10);
    ros::Publisher target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("commander/target",1);
    ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("/commander/predicted_path", 1);
    ros::Publisher trajectory_publisher = nh.advertise<nav_msgs::Path>("/commander/goal_trajectory", 1, true);

    ros::Subscriber odometry = nh.subscribe("/pose", 10, update_pose);

    CommandTransmitter transmitter("139.91.62.145",5005);


    //setting up pose estimate msg
    geometry_msgs::PoseStamped target_posest_msg;
    target_posest_msg.header.frame_id="world";
    target_posest_msg.header.stamp = ros::Time::now();
    target_posest_msg.pose.orientation.w=1.0;
    target_posest_msg.pose.position.z=0.25;

    //setting up executed path msg
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";

    //setting up sampled path (contains the points from the initial given plan that were used as targets, not all points are used bc of lookahead distance)
    std::vector<double> sampled_path_x;
    std::vector<double> sampled_path_y;
    std::vector<double> sampled_path_yaw;

    //setting up total executed path msg analytical
    std::vector<double> anal_total_path_x;
    std::vector<double> anal_total_path_y;
    std::vector<double> anal_total_path_yaw;
    


    //setting up goal trajectory msg
    nav_msgs::Path goal_trajectory_msg;
    goal_trajectory_msg.header.frame_id = "world";

    //setting up target_pose_msg
    geometry_msgs::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "world";
    target_pose_msg.header.stamp = ros::Time::now();
    target_pose_msg.pose.orientation.w=1.0;

    while(ros::ok() && !(controller_mode>=1 && controller_mode<=3)){
        std::cout<<"TYPE TO SELECT CONTROLLER: \n-> 1 (Pure Pursuit)\n-> 2 (Lateral/Stanley)\n-> 3 (MPC Gradient Descend)\n$";
        std::cin>>controller_mode;
    }

    double target_x,target_y,target_yaw;
    double speed, steering;
    
    Command mpc_command;
    State mpc_start_state;
    mpc_start_state.x = 0.0;
    mpc_start_state.y =0.0;
    mpc_start_state.yaw =0.0;

    double x_diff,y_diff,yaw_diff;
    tf2::Quaternion quaternion;

    command_publishers sim_pubs(nh);
    
    Pure_pursuit ctr_pure_pursuit(wheelbase, MAX_SPEED, MAX_STEER);
    Lateral ctr_lateral(wheelbase, MAX_SPEED, MAX_STEER);
    Mpc ctr_mpc(wheelbase, MAX_SPEED, MAX_STEER);

    sim_pubs.reset_position();

    //loading the test trajectory
    load_trajectory(goal_trajectory_msg, "src/informatics/pose_sequences/PLAN.csv");
    goal_trajectory_msg.header.stamp = ros::Time::now();
    trajectory_publisher.publish(goal_trajectory_msg);
    
    ros::Time start_time = ros::Time::now();

    for (int i=0; i<goal_trajectory_msg.poses.size(); i++){
        //selecting current pose-target
        goal_trajectory_msg.poses[i].header.frame_id = "world";
        goal_trajectory_msg.poses[i].header.stamp = ros::Time::now();
    
        //publishing current pose-target
        //spawn_target_pub.publish(goal_trajectory_msg.poses[i]);
        target_pose_pub.publish(goal_trajectory_msg.poses[i]);

        //acuiring target_x, target_y, target_yaw from current pose-target
        target_x = goal_trajectory_msg.poses[i].pose.position.x;
        target_y = goal_trajectory_msg.poses[i].pose.position.y;
        tf2::Quaternion q;
        double roll, pitch;
        tf2::fromMsg(goal_trajectory_msg.poses[i].pose.orientation, q);
        tf2::Matrix3x3(q).getRPY(roll, pitch, target_yaw);
        
        ros::spinOnce();

        //displaying target as object in gazebo (redundant)
        target_posest_msg.pose.position.x=target_x;
        target_posest_msg.pose.position.y=target_y;
        //spawn_target_pub.publish(target_posest_msg);
        //displaying target as vector in rviz 
        target_pose_msg.pose.position.x = target_x;
        target_pose_msg.pose.position.y = target_y;
        target_pose_msg.pose.orientation = tf2::toMsg(q);
        target_pose_pub.publish(target_pose_msg);

        //updating target position (difference)
        yaw_diff = target_yaw - curr_yaw;
        x_diff = std::cos(-curr_yaw) * (target_x - curr_x) - std::sin(-curr_yaw) * (target_y - curr_y);
        y_diff = std::sin(-curr_yaw) * (target_x - curr_x) + std::cos(-curr_yaw) * (target_y - curr_y);
    

        ctr_pure_pursuit.set_target(x_diff, y_diff);
        ctr_lateral.set_target(x_diff, y_diff, curr_yaw);
        ctr_mpc.set_target(x_diff, y_diff, curr_yaw);
        
        
        std::cout<<"TARGET["<< i<<"/"<< goal_trajectory_msg.poses.size() <<"] AQCUIRED\nMOVING...\n";

        // if last move, make sure to achieve it with good accuracy
        if(i+1 == (int)goal_trajectory_msg.poses.size()){
            std::cout<<"APPROACHING FINAL TARGET\nADJUSTING SPEED...\n";
            LOOKAHEAD = 0.2;
            MAX_SPEED_PP = 5.0;
            MIN_SPEED_PP = 1.0;
            MAX_SPEED_LAT = 5.0;
            MIN_SPEED_LAT = 1.0;
            MAX_SPEED_MPC = 5.0;
            MIN_SPEED_MPC = 1.0;
        }

        // executing the move
        while((abs(x_diff)>LOOKAHEAD || abs(y_diff)>LOOKAHEAD) && (ros::ok())){

            anal_total_path_x.push_back(curr_x);
            anal_total_path_y.push_back(curr_y);
            anal_total_path_yaw.push_back(curr_yaw);

            switch(controller_mode){
                case 3:
                    // MPC
                    ctr_mpc.set_target(x_diff, y_diff,yaw_diff);
                    sleep(ctr_mpc.get_dt());
                    mpc_command = ctr_mpc.get_command(mpc_start_state);
                    steering = mpc_command.steer;
                    if(abs(steering) == MAX_STEER){
                        speed = MAX_SPEED; // to move with max steering the car will need a lot of torgue
                    }else{
                        speed= std::min(std::max(mpc_command.vel, MIN_SPEED_MPC),MAX_SPEED_MPC);
                    }
                    ctr_mpc.get_trajectory(&path_msg, curr_x, curr_y, curr_yaw);
                    break;
                case 1:
                    // PURE PURSUIT
                    ctr_pure_pursuit.set_target(x_diff, y_diff);
                    steering = ctr_pure_pursuit.calc_steering();
                    if(abs(steering) == MAX_STEER){
                        speed = std::min(std::max(ctr_pure_pursuit.calc_speed(), 0.5), 10.0);
                    }else{
                        speed = std::min(std::max(ctr_pure_pursuit.calc_speed(), MIN_SPEED_PP), MAX_SPEED_PP); //applying upper and lower bound to speed, so its speedy enough and doesnt stall
                    }
                    ctr_pure_pursuit.get_trajectory(&path_msg, 20, curr_x, curr_y, curr_yaw);
                    break;
                case 2:
                    // STANLEY 
                    ctr_lateral.set_target(x_diff, y_diff, yaw_diff);
                    steering = ctr_lateral.calc_steering();
                    if(steering == MAX_STEER){
                        speed = MAX_SPEED;
                    }else{
                        speed = std::min(std::max(ctr_lateral.calc_speed(), MIN_SPEED_LAT), MAX_SPEED_LAT);
                    }
                    ctr_lateral.get_trajectory(&path_msg, 20, curr_x, curr_y, curr_yaw);
                    break;
            }

        
            //publishing command
            sim_pubs.publishVelocity(speed*BOOST);
            sim_pubs.publishSteering(steering);
            //transmitting command
            transmitter.send_command(speed, steering);
        
            // this one may be redundant
            target_pose_msg.header.stamp = ros::Time::now();
            target_pose_pub.publish(target_pose_msg);
            path_publisher.publish(path_msg);
        
            // (redundant) for debugging
            // std::cout<<"coords: "<<curr_x<<","<<curr_y<<" yaw: "<<curr_yaw<<"\n";
            //std::cout<<"target: "<<x_diff<<", "<<y_diff<<"\n";
            //std::cout<<"published velocity: "<<speed<<" steering:"<<steering<<"\n\n";

            // updating target position
            //x and y distances rotated so the car is like heading to 0 angle
            x_diff = std::cos(-curr_yaw) * (target_x - curr_x) - std::sin(-curr_yaw) * (target_y - curr_y);
            y_diff = std::sin(-curr_yaw) * (target_x - curr_x) + std::cos(-curr_yaw) * (target_y - curr_y);
            yaw_diff = target_yaw - curr_yaw;
            ros::spinOnce();
        }
    }
    ros::Time end_time = ros::Time::now();
    double duration = (end_time - start_time).toSec(); 
    std::cout<<"TRAJECTORY COMPLETED!\n";
    sim_pubs.publishVelocity(0.0);
    sim_pubs.publishSteering(0.0);
    sleep(0.5);
    std::cout<<"ZEROING SPEED\n";

    for(int o=0; o<4; o++){ //will this thing fucking stop?
        transmitter.send_command(0, 0);
        sleep(0.5);
    }
    transmitter.send_command(0, 0);
    sleep(0.2);
    std::cout<<"TIME: ["<<duration<<"sec]\n";
    sleep(0.2);
    std::cout<<"RESETTING...\n";
    sleep(0.5);
    
    std::cout<<"SAVING TRAJECTORY...\n";
    
    std::string final_anal_traj_filename= "";
    
    switch(controller_mode){
        case 1:
            final_anal_traj_filename = "src/informatics/pose_sequences/PP_TRAJ_ANAL.csv";
            break;
        case 2:
            final_anal_traj_filename = "src/informatics/pose_sequences/LAT_TRAJ_ANAL.csv";
            break;
        case 3:
            final_anal_traj_filename = "src/informatics/pose_sequences/MPC_TRAJ_ANAL.csv";
            break;            
    }

    std::ofstream traj_file_anal(final_anal_traj_filename);
    traj_file_anal<<std::fixed<<std::setprecision(6);
    for( size_t i=0; i<anal_total_path_x.size(); i++){
        tf2::Quaternion q;
        q.setRPY(0,0,anal_total_path_yaw[i]);
        geometry_msgs::Quaternion q_msg = tf2::toMsg(q);
        traj_file_anal<<anal_total_path_x[i]<<","<<anal_total_path_y[i]<<",0.000000,"<<q_msg.x<<","<<q_msg.y<<","<<q_msg.z<<","<<q_msg.w<<"\n";
    }
    traj_file_anal.close();

    sleep(1);
    std::cout<<"DISPLAYING RESULTS...\n";
    std::string cmd = "python3 src/informatics/src/plotter.py "+ std::to_string(controller_mode);
    std::system(cmd.c_str());

    sim_pubs.reset_position();
    curr_x=0;
    curr_y=0;

    //make sure to remove the analytics bc they are too big for github
    cmd = "rm -rf "+ final_anal_traj_filename;
    std::system(cmd.c_str());

    return 0;
}
