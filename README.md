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
        -> print mounts for rpi5, motor driver, cable organizers
        -> make some cable for battery connection to rpi5, motordriver and servo
        -> Add fishing line to servo and steer knuckles
        -> Add mount for led screen to display  or at least some led lamps



        -> Improve target acquisition
        -> Add mpc prediction display
        -> Fix prediction display on Pure Pursuit
        -> Add trajectory scenarios

    OPTIONAL:
    -> Tune/tweak Lateral (maybe add some other control parameter to steering)
    -> fix: the car model's movement in rviz is completely fucked the pose to tf is bad
    -> Make model more realistic, use urdf from Fusion
    -> Make realistic city like environment
    