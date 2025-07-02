#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;

    ros::Rate rate(10);  // 10 Hz

    ros::Publisher spawn_target_pub =  nh.advertise<geometry_msgs::PoseStamped>("/target_spawner/target_pose", 10);
    
    //setting up pose msg
    geometry_msgs::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id="map";
    target_pose_msg.header.stamp = ros::Time::now();
    target_pose_msg.pose.orientation.w=1.0;
    target_pose_msg.pose.position.z=0.25;
        
    
    double x,y;

    
    while(ros::ok()){
        std::cout<<"Enter Target Coords\n";
        std::cin >> x >>y;
    
        target_pose_msg.pose.position.x=x;
        target_pose_msg.pose.position.y=y;
        
        spawn_target_pub.publish(target_pose_msg);
        ros::Duration(1).sleep();
    }


    return 0;
}
