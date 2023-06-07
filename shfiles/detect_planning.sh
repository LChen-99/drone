
roslaunch vins fast_drone_250.launch & sleep 3;
roslaunch vins rviz.launch  & sleep 2;
roslaunch px4ctrl run_ctrl.launch  & sleep 7;
roslaunch ego_planner single_run_in_exp.launch & sleep 5;
rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"  & sleep 8;

roslaunch apriltag_ros xtdrone_detection_indoor1.launch & sleep 3;



wait;

