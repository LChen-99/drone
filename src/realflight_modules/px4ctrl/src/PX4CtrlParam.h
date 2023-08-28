#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
class Parameter_t
{
public:
	std::string model_path;
	std::string csv_filename;
	std::string prefix;
	struct Gain
	{
		double Kp0, Kp1, Kp2;
		double Kv0, Kv1, Kv2;
		double Kvi0, Kvi1, Kvi2;
		double Kvd0, Kvd1, Kvd2;
		double KAngR, KAngP, KAngY;
	};

	struct RotorDrag
	{
		double x, y, z;
		double k_thrust_horz;
	};

	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool land_on_target;
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
		double fly_speed;
	};

	struct DisturbanceObs{
		bool constant;
		bool use;
		double Q;
		double R;
		double t;
		double lamda;
		double P;
	};
	Eigen::Matrix4d T;
	std::string body_T_cam;
	Gain gain;
	RotorDrag rt_drag;
	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;
	DisturbanceObs disturbance_obs;
	int pose_solver;
	double mass;
	double gra;
	double max_angle;
	double ctrl_freq_max;
	double max_manual_vel;
	double low_voltage;
	
	bool use_bodyrate_ctrl;
	// bool print_dbg;

	Parameter_t();
	void config_from_ros_handle(const ros::NodeHandle &nh);
	void config_full_thrust(double hov);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};

	

	// config_file是输入，R和t是返回值
	void readParameters(std::string config_file, Eigen::Matrix4d& T)
	{
		cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
		if(!fsSettings.isOpened())
		{
			std::cerr << "ERROR: Wrong path to settings" << std::endl;
		}
		cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
		
        cv::cv2eigen(cv_T, T);
		std::cout << T << std::endl;
		fsSettings.release();
	};

};

#endif