#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include "collector.h"
#include "neucontroller.h"
#include "input.h"
#include "tic_toc.h"
// #include "ThrustCurve.h"
#include "controller.h"

struct AutoTakeoffLand_t
{
	bool landed{true};
	ros::Time toggle_takeoff_land_time;
	std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};
	Eigen::Vector4d start_pose;
	
	static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // motors idle running for 3 seconds before takeoff
	static constexpr double DELAY_TRIGGER_TIME = 2.0;  // Time to be delayed when reach at target height
};

class PX4CtrlFSM
{
public:
	Parameter_t &param;
	bool start_collecting_;
	RC_Data_t rc_data;
	State_Data_t state_data;
	ExtendedState_Data_t extended_state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Pwm_Data_t pwm_data;
	Command_Data_t cmd_data;
	Battery_Data_t bat_data;
	Takeoff_Land_Data_t takeoff_land_data;
	// 控制器，可以选择线性pid或者se3几何控制
	Controller* controller;
	//NeuControl controller_2;
	//LinearControl controller_1;
	Collector collector;
	ros::Publisher traj_start_trigger_pub;
	ros::Publisher ctrl_FCU_pub;
	ros::Publisher debug_pub; //debug
	ros::ServiceClient set_FCU_mode_srv;
	ros::ServiceClient arming_client_srv;
	ros::ServiceClient reboot_FCU_srv;
	ros::ServiceServer collect_trigger;
	quadrotor_msgs::Px4ctrlDebug debug_msg; //debug

	Eigen::Vector4d hover_pose;
	ros::Time last_set_hover_pose_time;

	enum State_t
	{
		MANUAL_CTRL = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER, // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL,	// px4ctrl is actived, and controling the drone.
		AUTO_TAKEOFF,
		AUTO_LAND
	};
	PX4CtrlFSM(Parameter_t &);
	// PX4CtrlFSM(Parameter_t &, LinearControl &, NeuControl &);
	void process();
	bool rc_is_received(const ros::Time &now_time);
	bool cmd_is_received(const ros::Time &now_time);
	bool odom_is_received(const ros::Time &now_time);
	bool imu_is_received(const ros::Time &now_time);
	bool bat_is_received(const ros::Time &now_time);
	bool recv_new_odom();
	State_t get_state() { return state; }
	bool get_landed() { return takeoff_land.landed; }
	// server : start collecting
	bool doCollectReq(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
	void writeCurState (const Controller_Output_t u, const ros::Time& now_time){
		Eigen::Vector3d pos(odom_data.p);
		Eigen::Vector3d vel(odom_data.v);
		Eigen::Quaterniond q(odom_data.q);
		Eigen::Vector4d pwm(pwm_data.pwm[0], pwm_data.pwm[1], pwm_data.pwm[2], pwm_data.pwm[3]);
		collector.outfile << now_time << ",";
		collector.outfile << "\"[" << pos(0) << "," << pos(1) << "," << pos(2) << "]\",";
		collector.outfile << "\"[" << vel(0) << "," << vel(1) << "," << vel(2) << "]\",";
		collector.outfile << "\"[" << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "]\",";
		collector.outfile << "\"[" << pwm(0) << "," << pwm(1) << "," << pwm(2) <<  "," << pwm(3) << "]\",";
		collector.outfile << "\"[" << controller->disturbance_obs(0) << "," << controller->disturbance_obs(1) << "," << controller->disturbance_obs(2) << "]\",";
		collector.outfile << "\"[" << controller->disturbance_mea(0) << "," << controller->disturbance_mea(1) << "," << controller->disturbance_mea(2) << "]\",";
		collector.outfile  << u.thrust << "," << controller->thr2acc_  << "," << bat_data.volt << "," << bat_data.percentage << std::endl;

	}
private:
	State_t state; // Should only be changed in PX4CtrlFSM::process() function!
	AutoTakeoffLand_t takeoff_land;

	// ---- control related ----
	Desired_State_t get_hover_des();
	Desired_State_t get_cmd_des();

	// ---- auto takeoff/land ----
	void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
	void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing 
	void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
	Desired_State_t get_rotor_speed_up_des(const ros::Time now);
	Desired_State_t get_takeoff_land_des(const double speed);

	// ---- tools ----
	void set_hov_with_odom();
	void set_hov_with_rc();

	bool toggle_offboard_mode(bool on_off); // It will only try to toggle once, so not blocked.
	bool toggle_arm_disarm(bool arm); // It will only try to toggle once, so not blocked.
	void reboot_FCU();
	
	void publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_trigger(const nav_msgs::Odometry &odom_msg);
};

#endif