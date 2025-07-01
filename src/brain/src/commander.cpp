#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/control_command", 10);

    ros::Rate rate(10);  // 10 Hz

    while (ros::ok())
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 1.0;   // Forward speed
        cmd.angular.z = 0.5;  // Turn rate

        cmd_pub.publish(cmd);
        rate.sleep();
    }

    return 0;
}
