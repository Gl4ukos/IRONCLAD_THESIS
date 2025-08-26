Requirements:

    ->ROS NOETIC
        -> ros-noetic-ackermann-msgs
        -> ros-noetic-ros-controllers
        -> ros-noetic-ros-control
        -> ros-noetic-gazebo-ros-control
        -> ros-noetic-gazebo-msgs
        -> libeigen3-dev

    -> UBUNTU FOCAL
    
TODO:

    BASELINE:
        -> Add display trajectory to mpc
        -> Add dynamic target assignment to intergrate trajectory
        -> Code MPC Newton

    OPTIONAL:
    -> make markers for Lateral that display the rot_error and lateral_error on rviz
    -> Tune/tweak Lateral (maybe add some other control parameter to steering)
    -> fix: the car model's movement in rviz is completely fucked the pose to tf is bad
    -> Improve architecture: Split into vehicle functions & data/config,  simulation utils
    -> Make model more realistic, use urdf from Fusion
    -> Make realistic city like environment
    

NOTES:
    -> Should not give target directly behind the car (angle pi), the car will  never turn into place