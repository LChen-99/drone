#include "controller.h"
#include "tic_toc.h"
using namespace std;



double Controller::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}
double Controller::thr2acc_ = 0;
double Controller::P_ = 1e6;
Controller::Controller(Parameter_t &param) : param_(param)
{
  disturbance_obs(0) = 0;
  disturbance_obs(1) = 0;
  disturbance_obs(2) = 0;
  resetThrustMapping();
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
Controller::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    const Pwm_Data_t &pwm,
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
	// 计算网络输出
    TicToc calcu;
      //compute disired acceleration
      
      Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
      Eigen::Vector3d Kp,Kv;
      Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
      Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
      des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
      des_acc += Eigen::Vector3d(0,0,param_.gra);

      u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
      double roll,pitch,yaw,yaw_imu;
      double yaw_odom = fromQuaternion2yaw(odom.q);
      double sin = std::sin(yaw_odom);
      double cos = std::cos(yaw_odom);
      roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
      pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;
      // yaw = fromQuaternion2yaw(des.q);
      yaw_imu = fromQuaternion2yaw(imu.q);
      // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
      //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
      //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
      Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
      // /vins_fusion/imu_propagate 和 /mavros/imu/data 的坐标系定义不同，
      //  Rw1_imu * Rimu_w2 * Rw2_des = Rw1_des
      u.q = imu.q * odom.q.inverse() * q;
    //   TicToc calcu;
    // ROS_INFO("calcu time: %f\n", calcu.toc());

  /* WRITE YOUR CODE HERE */

  //used for debug
  // debug_msg_.des_p_x = des.p(0);
  // debug_msg_.des_p_y = des.p(1);
  // debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}



quadrotor_msgs::Px4ctrlDebug
LinearController::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    const Pwm_Data_t &pwm,
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
      //compute disired acceleration
      Odom_Data_t cur = odom;
      // if((des.p - odom.p).norm() > 3){
      //   ROS_WARN("des.p - odom.p  = %4lf is large!", (des.p - odom.p).norm());
      //   cur.v = des.v;
      //   cur.p = des.p;
      // }
      Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
      Eigen::Vector3d Kp,Kv;
      Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
      Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;

      des_acc = des.a + Kv.asDiagonal() * (des.v - cur.v) + Kp.asDiagonal() * (des.p - cur.p);

      des_acc += Eigen::Vector3d(0,0,param_.gra);

      u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
 
      double roll,pitch,yaw,yaw_imu;
      double yaw_odom = fromQuaternion2yaw(cur.q);
      double sin = std::sin(yaw_odom);
      double cos = std::cos(yaw_odom);
      roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
      pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;

      yaw_imu = fromQuaternion2yaw(imu.q);
 
      Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
      // /vins_fusion/imu_propagate 和 /mavros/imu/data 的坐标系定义不同，
      //  Rw1_imu * Rimu_w2 * Rw2_des = Rw1_des
      u.q = imu.q * cur.q.inverse() * q;


  /* WRITE YOUR CODE HERE */

  //used for debug
  // debug_msg_.des_p_x = des.p(0);
  // debug_msg_.des_p_y = des.p(1);
  // debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}
/*
  compute throttle percentage 
*/
double 
Controller::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
Controller::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr*/
    /***********************************/
    // 悬停和静止时imu [0, 0, g]
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    static size_t tick  = 0;
    if(param.thr_map.print_val){
      tick++;
      if(tick % 100 == 0){
        ROS_DEBUG("thr2acc = %6.3f", thr2acc_);
        ROS_DEBUG("hover_percentage = %6.3f", param_.gra / thr2acc_);
        
      }
      
      //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    }
    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
Controller::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}

quadrotor_msgs::Px4ctrlDebug
SE3Controller::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    const Pwm_Data_t &pwm,
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
      //compute disired acceleration
      Odom_Data_t cur = odom;
      // if((des.p - odom.p).norm() > 2){
      //   ROS_WARN("des.p - odom.p  = %4lf is too large!", (des.p - odom.p).norm());
      //   cur.v = des.v;
      //   cur.p = des.p;
      // }
      Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
      Eigen::Vector3d Kp,Kv;
      Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
      Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
      des_acc = des.a + Kv.asDiagonal() * (des.v - cur.v) + Kp.asDiagonal() * (des.p - cur.p);
      des_acc += Eigen::Vector3d(0,0,param_.gra);
      Eigen::Matrix3d rotation_q = cur.q.toRotationMatrix();
      Eigen::Matrix3d rotation_des = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rotation_z = rotation_q.col(2);
      //cout << des_acc << endl;
      double acc_z = des_acc.dot(rotation_z);
      Eigen::Vector3d acc(0, 0, acc_z);
      u.thrust = computeDesiredCollectiveThrustSignal(acc);
      //double roll,pitch,yaw,yaw_imu;
      Eigen::Vector3d zb_des, xc_des, yb_des, xb_des;
      zb_des = des_acc / des_acc.norm();
      xc_des = {cos(des.yaw), sin(des.yaw), 0};
      yb_des = zb_des.cross(xc_des) / zb_des.cross(xc_des).norm();
      xb_des = yb_des.cross(zb_des);
      xb_des = xb_des / xb_des.norm();
      rotation_des.col(0) = xb_des;
      rotation_des.col(1) = yb_des;
      rotation_des.col(2) = zb_des;
      Eigen::Quaterniond q_des(rotation_des);
      
      // /vins_fusion/imu_propagate 和 /mavros/imu/data 的坐标系定义不同，
      //  Rw1_imu * Rimu_w2 * Rw2_des = Rw1_des
      u.q = imu.q * cur.q.inverse() * q_des;


  /* WRITE YOUR CODE HERE */

  //used for debug
  // debug_msg_.des_p_x = des.p(0);
  // debug_msg_.des_p_y = des.p(1);
  // debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}





quadrotor_msgs::Px4ctrlDebug
Neural_Fly_Control::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    const Pwm_Data_t &pwm,
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
  
  TicToc calcu;
  Eigen::Vector3d Kp, Kv, Kvi;
  Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
  Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
  Kvi << param_.gain.Kvi0, param_.gain.Kvi1, param_.gain.Kvi2;
	Odom_Data_t cur = odom; 
	Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
  
  Eigen::Vector3d s = (cur.v - des.v) + Kp.asDiagonal() * (cur.p - des.p);
  Eigen::Vector3d v_r = des.v -  Kp.asDiagonal() * (cur.p - des.p);
  Eigen::Vector3d dv_r = des.a -  Kp.asDiagonal() * (cur.v - des.v);
  
  integral += 1.0 / param_.ctrl_freq_max * s;
  des_acc = dv_r - Kv.asDiagonal() * s - Kvi.asDiagonal() * integral;
  // des_acc = dv_r - Kv.asDiagonal() * s;
  // des_acc = des.a + Kp.asDiagonal() * (des.v - cur.v) + Kp.asDiagonal() * (des.p - cur.p) + Kvi.asDiagonal() * integral;
  // ROS_INFO("integral: %f, %f, %f\n", integral.x(), integral.y(), integral.z());
  // ROS_INFO("Kv.asDiagonal() * s: %f, %f, %f\n", (Kv.asDiagonal() * s.x()), (Kv.asDiagonal() * s.y()), (Kv.asDiagonal() * s.z()));
  // ROS_INFO("des.p - cur.p: %f, %f, %f\n", (des.p.x() - cur.p.x()), (des.p.y() - cur.p.y()), (des.p.z() - cur.p.z()));
  des_acc += Eigen::Vector3d(0,0,param_.gra);
  // ROS_INFO("des_acc: %f, %f, %f\n", des_acc.x(), des_acc.y(), des_acc.z());
  if(param_.disturbance_obs.use && binit_){
    
    if(!param_.disturbance_obs.constant){
      Matrix<double, 11, 1> feature;
      feature.block(0, 0, 3, 1) = odom.v;
      feature(3, 0) = odom.q.x();
      feature(4, 0) = odom.q.y();
      feature(5, 0) = odom.q.z();
      feature(6, 0) = odom.q.w();
      double hover_ = 910.0 / (1400.0 * 1000.0);

      feature(7, 0) = pwm.pwm[0] * hover_;
      feature(8, 0) = pwm.pwm[1] * hover_;
      feature(9, 0) = pwm.pwm[2] * hover_;
      feature(10, 0) = pwm.pwm[3] * hover_;
      Vector3d phi_output = model_->forward(feature);
      auto a = Kalman->get_a();
      
      f(0) = phi_output.transpose() * a.block<3, 1>(0, 0);
      f(1) = phi_output.transpose() * a.block<3, 1>(3, 0);
      f(2) = phi_output.transpose() * a.block<3, 1>(6, 0);
      //只对z轴限制
      if(abs(f(2)) > 1.0){
        f(2) = 1.0 * f(2) / abs(f(2));
      }
      
      // f(2) = 0;
      des_acc -= f;
    }else{
      auto a = Kalman->get_a();
      Vector3d phi_output = Eigen::Vector3d(1, 1, 1);
      
      f(0) = phi_output.transpose() * a.block<3, 1>(0, 0);
      f(1) = phi_output.transpose() * a.block<3, 1>(3, 0);
      f(2) = phi_output.transpose() * a.block<3, 1>(6, 0);
      // if(abs(f(2)) > 1.0){
      //   f(2) = 1.0 * f(2) / abs(f(2));
      // }
      f(2) = 0;  
      des_acc -= f; 
    }
    disturbance_obs = f;
    std::cout << "f = " << f.transpose() << std::endl;
  }
  // std::cout << "des_acc = " << des_acc.transpose() << endl;
  // PID
  if(0){
    Eigen::Matrix3d rotation_q = cur.q.toRotationMatrix();
    Eigen::Matrix3d rotation_des = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rotation_z = rotation_q.col(2);
    Eigen::Vector3d zb_des, xc_des, yb_des, xb_des;
    double acc_z = des_acc.dot(rotation_z);
    Eigen::Vector3d acc(0, 0, acc_z);
    
    u.thrust = computeDesiredCollectiveThrustSignal(acc);  
    zb_des = des_acc / des_acc.norm();
    xc_des = {cos(des.yaw), sin(des.yaw), 0};
    yb_des = zb_des.cross(xc_des) / zb_des.cross(xc_des).norm();
    xb_des = yb_des.cross(zb_des);
    xb_des = xb_des / xb_des.norm();
    rotation_des.col(0) = xb_des;
    rotation_des.col(1) = yb_des;
    rotation_des.col(2) = zb_des;
    Eigen::Quaterniond q_des(rotation_des);
    u.q = imu.q * cur.q.inverse() * q_des;
  }else{
    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
 
    double roll,pitch,yaw,yaw_imu;
    double yaw_odom = fromQuaternion2yaw(cur.q);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
    pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;

    yaw_imu = fromQuaternion2yaw(imu.q);

    Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
    // /vins_fusion/imu_propagate 和 /mavros/imu/data 的坐标系定义不同，
    //  Rw1_imu * Rimu_w2 * Rw2_des = Rw1_des
    u.q = imu.q * cur.q.inverse() * q;
  }
  /* WRITE YOUR CODE HERE */

  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  ROS_DEBUG("calcu time: %f\n", calcu.toc());
  return debug_msg_;
}

void Neural_Fly_Control::updateAdapt(double t, const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    const Pwm_Data_t &pwm,
    const double &u){

	
	Odom_Data_t cur = odom;    
	Matrix<double, 11, 1> feature;
	feature.block(0, 0, 3, 1) = odom.v;
	feature(3, 0) = odom.q.x();
	feature(4, 0) = odom.q.y();
	feature(5, 0) = odom.q.z();
	feature(6, 0) = odom.q.w();
  // 1564是悬停PWM值
	double hover_ = 910.0 / (1400.0 * 1000.0);

	feature(7, 0) = pwm.pwm[0] * hover_;
	feature(8, 0) = pwm.pwm[1] * hover_;
	feature(9, 0) = pwm.pwm[2] * hover_;
	feature(10, 0) = pwm.pwm[3] * hover_;

	// cur.q --- R^w(vins)_imu 
	// imu.q --- R^w(px4)_imu
  Vector3d phi_output;
  if(param_.disturbance_obs.constant){
    phi_output = Eigen::Vector3d(1, 1, 1);
  }else{
    phi_output = model_->forward(feature);
  }

  Vector3d a_w;
  if(prev_t == -1){
    prev_t = t;
    prev_vel = cur.v;
    a_w = odom.q * imu.a - Eigen::Vector3d(0, 0, param_.gra);
  }else{
    double delta_t = t - prev_t;
	  a_w = (cur.v - prev_vel) / delta_t;
    // cout << "delta_t = " << delta_t << endl;
  }
	
	//计算网络输出
  // cout << "a_w = " << a_w << endl;
	Vector3d fu(0, 0, u * thr2acc_);
	Vector3d f_measurement = (a_w - Eigen::Vector3d(0, 0, -param_.gra) - cur.q * fu);
  disturbance_mea = f_measurement;
  ROS_INFO("disturbance_mea: %lf, %lf, %lf", disturbance_mea.x(), disturbance_mea.y(), disturbance_mea.z());
	Vector3d s = (cur.v - des.v) + (cur.p - des.p);
	// 更新a
	Kalman->update(f_measurement, s, phi_output);
	auto a = Kalman->get_a();
	// cout << "a = " << a << endl;
	Vector3d f_;
	f_(0) = phi_output.transpose() * a.block<3, 1>(0, 0);
	f_(1) = phi_output.transpose() * a.block<3, 1>(3, 0);
	f_(2) = phi_output.transpose() * a.block<3, 1>(6, 0);
  prev_f_ = f_;
  if(!binit_ && (prev_f_ - f_).norm() < 0.1){
		binit_ = true;
    ROS_INFO("obs init!");
	}
  
  prev_t = t;
	prev_vel = cur.v;
	// cout << "f_measurement = " << f_measurement << endl;
	// cout << "f = " << f_ << endl;
}
