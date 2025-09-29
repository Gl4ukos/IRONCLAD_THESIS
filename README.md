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
        -> Get a battery & charger
        -> Solder a switch on the power distributor
        -> Add some stabilizer on the servo and steer knuckles

        -> Play with speeds-PID params-Lookaheads and trajectory, get some results on the paper

    OPTIONAL:
    -> Graphs are very computationally expensive
    -> Print some cable organizer to screw on the elecmount2
    -> fix: the car model's movement in rviz is completely fucked the pose to tf is bad
    -> Make model more realistic, use urdf from Fusion
    -> Make a second tcp connection to take the time and calibrate the ironclad's speed

    ! NOTES:
    -> MPC:
        -> struggles A LOT when target is in bad position. It needs some straying to "find" the target
        -> Speed calculation is kinda problematic sometimes, bc the error gets calculated for the final state and not the first one (so it gets low speed results, eventhough it could be higher)
    
    -> Pure pursuit: 
        ->LOOKAHEAD>> & MAX_SPEED>> -> faster & less accurate (and the opposite)
        -> When a semicomplete circle is displayed on RVIZ it actually means that there is no possible trajectory for the current target, bc the angle needed surpasses the steering cap.
    
    -> Stanley: 
        ->SPEED<< -> VERY AGRESSIVE STEERING 
        -> REAAALLY does not perform well with high LOOKAHEAD, it oscillates a lot due to lateral error
        -> Oscilalates a lot in sharp turns, bc lateral and yaw error may become contradicting
    