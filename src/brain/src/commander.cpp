#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;

    ros::Rate rate(10);  // 10 Hz



    return 0;
}
