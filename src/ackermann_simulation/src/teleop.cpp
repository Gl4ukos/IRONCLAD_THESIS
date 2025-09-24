#include <ros/ros.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include "ackermann_simulation/command_publishers.hpp"

double MAX_SPEED = 30.0;
double MAX_STEER = 1.0;

char getKey()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);            // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);          // disable buffering & echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);   // apply new settings
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);   // restore old settings
    return ch;
}

double clip_vel(double val){
    if (val < -MAX_SPEED) return -MAX_SPEED;
    else if (val > MAX_SPEED) return MAX_SPEED;
    else return val;
}
double clip_steer(double val){
    if (val < -MAX_STEER) return -MAX_STEER;
    else if (val > MAX_STEER) return MAX_STEER;
    else return val;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop");
    ros::NodeHandle nh;

    command_publishers sim_pubs(nh);

    double curr_speed = 0;
    double curr_steer = 0;

    std::cout << "Press keys to send float commands (wasd), 'q' to quit:\n";

    ros::Rate rate(100);  // 10 Hz loop
    while (ros::ok())
    {
        if (isatty(STDIN_FILENO)) // check if input is from terminal
        {
            char c = getKey();

            switch (c)
            {
                case 'w':{
                    curr_speed = clip_vel(curr_speed+5);
                    sim_pubs.publishVelocity(curr_speed); break;
                } 
                case 's': {
                    curr_speed = clip_vel(curr_speed-5);
                    sim_pubs.publishVelocity(curr_speed); break;
                }
                case 'a': { 
                    curr_steer = clip_steer(curr_steer+=0.1);
                    sim_pubs.publishSteering(curr_steer); break;
                }
                case 'd': {
                    curr_steer = clip_steer(curr_steer-0.1);
                    sim_pubs.publishSteering(curr_steer); break;
                }
                case 'x':{
                    sim_pubs.publishVelocity(0.0); 
                    sim_pubs.publishSteering(0.0);
                    curr_speed = 0.0;
                    curr_steer = 0.0;
                    break;
                } 

                case 'q':{
                    std::cout << "Exiting.\n";  
                    sim_pubs.publishVelocity(0.0); 
                    sim_pubs.publishSteering(0.0);
                    return 0;

                }
                default: break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
