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
        -> ! BEFORE TESTING HARDWARE: Add some system to check for speed and steer value compatibility
        -> Get a battery & charger
        -> Solder a switch on the power distributor
        -> Fix servo jitter - hook in servo driver (WITHOUT BURNING IT THIS TIME)


        -> Fix lateral error display orientation (in lateral.cpp getErrorMarkers) 
        -> Graphs are very computationally expensive
        -> Add mpc prediction display
        -> Fix prediction display on Pure Pursuit
        -> Add trajectory scenarios

    OPTIONAL:
    -> Print some cable organizer to screw on the elecmount2
    -> Tune/tweak Lateral (maybe add some other control parameter to steering)
    -> fix: the car model's movement in rviz is completely fucked the pose to tf is bad
    -> Make model more realistic, use urdf from Fusion

    ! NOTES:
    -> MPC struggles A LOT when target is in bad position. It needs some straying to "find" the target
    -> Pure pursuit: LOOKAHEAD>> & MAX_SPEED>> -> faster & less accurate (and the opposite)
    -> Stanley: SPEED<< -> VERY AGRESSIVE STEERING 
    -> Stanley: REAAALLY does not perform well with high LOOKAHEAD, it oscillates a lot due to lateral error
    -> Stanley: Oscilalates a lot in sharp turns, bc lateral and yaw error may become contradicting