Requirements:

    ->ROS NOETIC
        -> ros-noetic-ackermann-msgs
        -> ros-noetic-ros-controllers
        -> ros-noetic-ros-control
        -> ros-noetic-gazebo-ros-control
        -> ros-noetic-gazebo-msgs
    -> UBUNTU FOCAL
    
TODO:

    BASELINE:

    -> fix: the car model's movement in rviz is completely fucked the pose to tf is bad
    -> Improve architecture: Split into vehicle functions & data/config,  simulation utils

    OPTIONAL:
    -> Make model more realistic, use urdf from Fusion
    -> Make realistic city like environment
    -> Adjust displayed odometry vector length in rviz, dynamically to the speed
    

NOTES:

    -> Should not give target directly behind the car (angle pi), the car will  never turn into place