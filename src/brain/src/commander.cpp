#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "ackermann_simulation/command_publishers.hpp"
#include "control/pure_pursuit.hpp"
#include <ros/console.h>
#include <nav_msgs/Odometry.h>

double wheelbase = 1.0; //! probably not true  

double curr_x, curr_y, curr_theta;

double eucl_dist(double t_x,double t_y){
    return sqrt(t_x*t_x + t_y*t_y);
}

void update_pose(nav_msgs::Odometry msg){
    curr_x = msg.pose.pose.position.x;
    curr_y = msg.pose.pose.position.y; 
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;

    ros::Rate rate(10);  // 10 Hz

    ros::Publisher spawn_target_pub =  nh.advertise<geometry_msgs::PoseStamped>("/target_spawner/target_pose", 10);
    ros::Subscriber odometry = nh.subscribe("/pose", 10, update_pose);


    //setting up pose msg
    geometry_msgs::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id="map";
    target_pose_msg.header.stamp = ros::Time::now();
    target_pose_msg.pose.orientation.w=1.0;
    target_pose_msg.pose.position.z=0.25;
        
    
    double target_x,target_y;
    double speed, steering;
    command_publishers sim_pubs(nh);

    Pure_pursuit ctr_pure_pursuit(wheelbase, sim_pubs.get_max_speed(), sim_pubs.get_max_steer());


    
    while(ros::ok()){
        std::cout<<"Enter Target Coords\n";
        std::cin >> target_x >> target_y;
    
        ros::spinOnce();

        //displaying target as object in gazebo
        target_pose_msg.pose.position.x=target_x;
        target_pose_msg.pose.position.y=target_y;
        spawn_target_pub.publish(target_pose_msg);

        std::cout<<"TARGET SET\n";
        ctr_pure_pursuit.set_target(target_x-curr_x, target_y-curr_y);
        ros::Duration(1).sleep();

        
        std::cout<<"MOVING...\n";

        while(abs(target_x-curr_x)>0.2 || abs(target_y-curr_y)>0.2 ){
            speed = ctr_pure_pursuit.calc_speed();
            steering = ctr_pure_pursuit.calc_steering();
            
            
            //sim_pubs.publishVelocity(speed);
            //sim_pubs.publishSteering(steering);
            std::cout<<"command published: "<<speed<<" "<<steering<<"\n";
            sleep(1);
            ros::spinOnce();
            std::cout<<"Pos: "<<curr_x<< " - " <<curr_y<<"\n";
            ctr_pure_pursuit.set_target(target_x-curr_x, target_y-curr_y);
        }

        std::cout<<"POSITION ACHIEVED!\n";
        sleep(2);
        std::cout<<"RESETTING...\n";
        sleep(1);
        sim_pubs.reset_position();

    }


    return 0;
}
