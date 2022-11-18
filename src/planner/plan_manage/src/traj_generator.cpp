#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <plan_manage/generator.h>
#include <math.h>
ros::Publisher pos_cmd_pub;
ros::Subscriber initial_pos_sub;
quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;
using namespace std;
bool receive_traj_ = true;
vector<UniformBspline> traj_;
double traj_duration_ = 10;
ros::Time start_time_;
int traj_id_;
int times = 0;
double T = 0.01;
// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
vector<Eigen::Vector3d> traj_pos, traj_vel, traj_acc;
bool get_initialpos = false;
Eigen::Vector3d initial_pos;
double initial_yaw;
Eigen::Vector3d cur_pos;
double cur_yaw;
TrajGenerator generator;
void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();
  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);
  int points_size = generator.traj_pos_.size();
  static ros::Time time_last = ros::Time::now();
  if (times < points_size)
  {
    pos = generator.traj_pos_[times];
    vel = generator.traj_vel_[times];
    acc = generator.traj_acc_[times];
    // pos = traj_pos[times];
    // vel = traj_vel[times];
    // acc = traj_acc[times];
    /*** calculate yaw ***/
    //yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    //double tf = min(traj_duration_, t_cur + 2.0);
   // pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (times >= points_size)
  {
    /* hover when finish traj_ */
    pos = generator.traj_pos_[points_size - 1];
    vel.setZero();
    acc.setZero();
    
    //yaw_yawdot.first = last_yaw_;
    //yaw_yawdot.second = 0;

    //pos_f = pos;
    //return;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  //ROS_INFO("initial_pos = %lf, initial_pos = %lf, initial_pos = %lf", initial_pos(0), initial_pos(1), initial_pos(2));
  cmd.position.x = pos(0) + initial_pos(0);
  cmd.position.y = pos(1) + initial_pos(1);
  cmd.position.z = pos(2) + initial_pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = initial_yaw;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;
  //ROS_INFO("initial_pos = %lf, initial_pos = %lf, initial_pos = %lf", initial_pos(0), initial_pos(1), initial_pos(2));
  //ROS_INFO("des_x = %lf, des_y = %lf, des_z = %lf", pos(0), pos(1), pos(2));
  if(get_initialpos){
    times++;
    pos_cmd_pub.publish(cmd);
  }
  
}
double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}
void posCallback(const nav_msgs::Odometry::ConstPtr &msg){
  cur_pos(0) = msg->pose.pose.position.x;
  cur_pos(1) = msg->pose.pose.position.y;
  cur_pos(2) = msg->pose.pose.position.z;
  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  cur_yaw = fromQuaternion2yaw(q);
  if(get_initialpos == false){
    initial_pos = Eigen::Vector3d::Zero();
    initial_pos(0) = msg->pose.pose.position.x;
    initial_pos(1) = msg->pose.pose.position.y;
    initial_pos(2) = msg->pose.pose.position.z;
    
    initial_yaw = fromQuaternion2yaw(q);
    get_initialpos = true;
    //ROS_INFO("x = %lf, y = %lf, z = %lf", initial_pos(0), initial_pos(1), initial_pos(2));
  }
 
}
void circle_generate(){
  int sample_time = traj_duration_ / T;
  double radius = 0.5;
  
  traj_pos = vector<Eigen::Vector3d>(sample_time, Eigen::Vector3d::Zero());
  traj_vel = vector<Eigen::Vector3d>(sample_time, Eigen::Vector3d::Zero());
  traj_acc = vector<Eigen::Vector3d>(sample_time, Eigen::Vector3d::Zero());

  for(int i = 0; i < sample_time; i++){
    Eigen::Vector3d p(radius * sin((2 * M_PI / sample_time) * i), radius * cos((2 * M_PI / sample_time) * i) - radius, 0);
    Eigen::Vector3d v(radius * cos((2 * M_PI / sample_time) * i), - radius * sin((2 * M_PI / sample_time) * i), 0);
    Eigen::Vector3d a(-radius * sin((2 * M_PI / sample_time) * i), - radius * cos((2 * M_PI / sample_time) * i), 0);
    traj_pos[i] = p;
    traj_vel[i] = v;
    traj_acc[i] = a;
  }
  
  
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_generator");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");
  
  initial_pos_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 100, posCallback);
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  ros::Duration(0.5).sleep();
  Matrix<double, 3, 3> waypoints;
  waypoints << 0, 0, 0,\
               -1, 0, -1,\
               -2, -1, -2;
  generator.Generator(waypoints.transpose());
  // circle_generate();
  ros::Timer cmd_timer = nh.createTimer(ros::Duration(T), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}