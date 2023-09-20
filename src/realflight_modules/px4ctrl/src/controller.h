/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>
#include "tic_toc.h"
#include "torch_model.h"
#include "input.h"
#include <Eigen/Dense>
#include "Kalman_filter.h"


struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	//Eigen::Vector3d des_v_real;
};


class LinearControl
{
public:
  LinearControl(Parameter_t &);
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      const Pwm_Data_t &pwm,
      Controller_Output_t &u);
  bool estimateThrustModel(const Eigen::Vector3d &est_v,
      const Parameter_t &param);
  void resetThrustMapping(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
  double fromQuaternion2yaw(Eigen::Quaterniond q);
};



class Controller{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Controller(Parameter_t & param);
	virtual quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      const Pwm_Data_t &pwm,
      Controller_Output_t &u);
  virtual void updateAdapt(double t, const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    const Pwm_Data_t &pwm,
    const double &u){}
    
	bool estimateThrustModel(const Eigen::Vector3d &est_v,
      const Parameter_t &param);
	void resetThrustMapping(void);
	Parameter_t param_;
	quadrotor_msgs::Px4ctrlDebug debug_msg_;
	static double thr2acc_;
	static double P_;

	
	
	std::queue<std::pair<ros::Time, double>> timed_thrust_;
	static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

	// Thrust-accel mapping params
	static constexpr double rho2_ = 0.998; // do not change
	

	double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
	double fromQuaternion2yaw(Eigen::Quaterniond q);
  Vector3d disturbance_obs;
  Vector3d disturbance_mea;
  KalmanAdaptive* Kalman;
private:
};


class LinearController : public Controller
{
public:
  LinearController(Parameter_t& param) : Controller(param){}
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      const Pwm_Data_t &pwm,
      Controller_Output_t &u);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

};
class SE3Controller : public Controller
{
public:
  SE3Controller(Parameter_t& param) : Controller(param){}
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      const Pwm_Data_t &pwm,
      Controller_Output_t &u);
 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	
};


class Neural_Fly_Control : public Controller
{
public:
  Neural_Fly_Control(Parameter_t& param) : Controller(param){
	  model_ = new NetworkModel(param.model_path);
    Kalman = new KalmanAdaptive(0.02, param);
    prev_vel = Vector3d(0, 0, 0);
    binit_ = false;
    prev_t = -1.0;
  }
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      const Pwm_Data_t &pwm,
      Controller_Output_t &u);
  void updateAdapt(double t, const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    const Pwm_Data_t &pwm,
    const double &u);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	//TODO:自适应率
  bool binit_;
  
private:
  Vector3d integral = Vector3d::Zero();
  double prev_t;
  Vector3d prev_vel;
  Vector3d prev_f_;
	NetworkModel* model_;
};
#endif
