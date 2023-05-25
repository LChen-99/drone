
roslaunch vins fast_drone_250.launch & sleep 3;
roslaunch apriltag_ros xtdrone_detecetion_indoor1.launch & sleep 3;
rosrun detectAndLanding pub_pose & sleep 3;


wait;

