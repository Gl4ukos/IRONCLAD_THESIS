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



double wheelbase = 1.0; //! probably not true  

double curr_x, curr_y, curr_z, curr_yaw;



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

    //setting up pose estimate msg
    geometry_msgs::PoseStamped target_posest_msg;
    target_posest_msg.header.frame_id="world";
    target_posest_msg.header.stamp = ros::Time::now();
    target_posest_msg.pose.orientation.w=1.0;
    target_posest_msg.pose.position.z=0.25;

    //setting up executed path msg
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";

    //setting up goal trajectory msg
    nav_msgs::Path goal_trajectory_msg;
    goal_trajectory_msg.header.frame_id = "world";

    //setting up target_pose_msg
    geometry_msgs::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "world";
    target_pose_msg.header.stamp = ros::Time::now();
    target_pose_msg.pose.orientation.w=1.0;

    
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
    
    Pure_pursuit ctr_pure_pursuit(wheelbase, sim_pubs.get_max_speed(), sim_pubs.get_max_steer());
    Lateral ctr_lateral(wheelbase, sim_pubs.get_max_speed(), sim_pubs.get_max_steer());
    Mpc ctr_mpc(wheelbase, sim_pubs.get_max_speed(), sim_pubs.get_max_steer());

    sim_pubs.reset_position();
    
    //loading the test trajectory
    load_trajectory(goal_trajectory_msg, "src/brain/utils/trajectory_test.csv");
    trajectory_publisher.publish(goal_trajectory_msg);

    while(ros::ok()){
        std::cout<<"Enter Target Coords\n";
        std::cin >> target_x >> target_y >> target_yaw;
        ros::spinOnce();

        goal_trajectory_msg.header.stamp = ros::Time::now();
        trajectory_publisher.publish(goal_trajectory_msg);
        //displaying target as object in gazebo
        target_posest_msg.pose.position.x=target_x;
        target_posest_msg.pose.position.y=target_y;
        spawn_target_pub.publish(target_posest_msg);

        quaternion.setRPY(0,0,target_yaw);
        target_pose_msg.pose.position.x = target_x;
        target_pose_msg.pose.position.y = target_y;
        target_pose_msg.pose.orientation = tf2::toMsg(quaternion);
        target_pose_pub.publish(target_pose_msg);    

        std::cout<<"TARGET SET\n";

        //x and y distances rotated so the car is like heading to 0 angle
        yaw_diff = target_yaw - curr_yaw;
        x_diff = std::cos(-curr_yaw) * (target_x - curr_x) - std::sin(-curr_yaw) * (target_y - curr_y);
        y_diff = std::sin(-curr_yaw) * (target_x - curr_x) + std::cos(-curr_yaw) * (target_y - curr_y);
        
        
        ctr_pure_pursuit.set_target(x_diff, y_diff);
        ctr_lateral.set_target(x_diff, y_diff, curr_yaw);
        ctr_mpc.set_target(x_diff, y_diff, curr_yaw);

        ros::Duration(1).sleep();
        
        std::cout<<"MOVING...\n";

        int valid_move =0;
        while(abs(target_x-curr_x)>0.2 || abs(target_y-curr_y)>0.2 ){
            
            mpc_command = ctr_mpc.get_command(mpc_start_state);
            speed = std::max(mpc_command.vel, 2.0); //applying lower limit to speed so the car doesnt stall when steering aggresively
            steering = mpc_command.steer;

            //speed = ctr_pure_pursuit.calc_speed();
            //steering = ctr_pure_pursuit.calc_steering();
            //ctr_pure_pursuit.get_trajectory(&path_msg, 20, curr_x, curr_y, curr_yaw);


            // STANLEY 
            // speed = ctr_lateral.calc_speed();
            // steering = ctr_lateral.calc_steering();
            // ctr_lateral.get_trajectory(&path_msg, 20, curr_x, curr_y, curr_yaw);

            //publish
            sim_pubs.publishVelocity(speed);
            sim_pubs.publishSteering(steering);

            target_pose_msg.header.stamp = ros::Time::now();
            target_pose_pub.publish(target_pose_msg);

            path_msg.header.stamp = ros::Time::now();
            path_publisher.publish(path_msg);
        
            // std::cout<<"coords: "<<curr_x<<","<<curr_y<<" yaw: "<<curr_yaw<<"\n";
            std::cout<<"target: "<<x_diff<<", "<<y_diff<<"\n";
            std::cout<<"published velocity: "<<speed<<" steering:"<<steering<<"\n\n";
            ros::spinOnce();

            //x and y distances rotated so the car is like heading to 0 angle
            x_diff = std::cos(-curr_yaw) * (target_x - curr_x) - std::sin(-curr_yaw) * (target_y - curr_y);
            y_diff = std::sin(-curr_yaw) * (target_x - curr_x) + std::cos(-curr_yaw) * (target_y - curr_y);
            yaw_diff = target_yaw - curr_yaw;
           
            ctr_mpc.set_target(x_diff, y_diff,yaw_diff);
            sleep(ctr_mpc.get_dt());
            //ctr_pure_pursuit.set_target(x_diff, y_diff);
            //ctr_lateral.set_target(x_diff, y_diff, yaw_diff);
            
        }

        std::cout<<"POSITION ACHIEVED!\n";
        sim_pubs.publishVelocity(0.0);
        sim_pubs.publishSteering(0.0);
        sleep(2);
        std::cout<<"RESETTING...\n";
        sleep(1);
        sim_pubs.reset_position();
        curr_x=0;
        curr_y=0;

    }

    return 0;
}
