#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include "plan_manage/collector.h"
#include <plan_manage/generator.h>
#include <nav_msgs/Path.h>
// #include <nav_msgs/Odometry.h>
#include <math.h>
ros::Publisher pos_cmd_pub;
ros::Publisher pub_des_path;
ros::Publisher pub_groundtruth_path;
ros::Subscriber initial_pos_sub;
quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using namespace std;
bool receive_traj_ = true;
double traj_duration_ = 5;
ros::Time start_time_;
int traj_id_;
int times = 0;
double T = 0.01;
// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
vector<Eigen::Vector3d> traj_pos, traj_vel, traj_acc;

nav_msgs::Path des_path;
nav_msgs::Path groundtruth_path;
Collector collector("/home/luochen/drone-project/drone-project/flylog/data.csv");
Eigen::Quaterniond initial_q;
bool get_initialpos = false;
Eigen::Vector3d initial_pos;
double initial_yaw;
Eigen::Vector3d cur_pos;
Eigen::Vector3d cur_vel;
double cur_yaw;
TrajGenerator generator;
void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  //double t_cur = (time_now - start_time_).toSec();
  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);
  int points_size = generator.traj_pos_.size();
  static ros::Time time_last = ros::Time::now();
  if (times < points_size)
  {
    pos = generator.traj_pos_[times];
    vel = generator.traj_vel_[times];
    acc = generator.traj_acc_[times];
  }
  else if (times >= points_size)
  {
    pos = generator.traj_pos_[points_size - 1];
    vel.setZero();
    acc.setZero();
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
  cmd.yaw_dot = 0;

  geometry_msgs::PoseStamped despose_stamped;
  despose_stamped.header = cmd.header;
  despose_stamped.header.frame_id = "map";
  pos = pos + initial_pos;

  despose_stamped.pose.position.x = pos(0);
  despose_stamped.pose.position.y = pos(1);
  despose_stamped.pose.position.z = pos(2);
  des_path.header = cmd.header;
  des_path.header.frame_id = "map";
  des_path.poses.push_back(despose_stamped);
  pub_des_path.publish(des_path);
  // groundtruth
  geometry_msgs::PoseStamped groundtruth_stamped;
  groundtruth_stamped.header = cmd.header;
  groundtruth_stamped.header.frame_id = "map";

  collector.write(time_now.toNSec(), cur_pos, cur_vel, pos, vel);
  groundtruth_stamped.pose.position.x = cur_pos(0);
  groundtruth_stamped.pose.position.y = cur_pos(1);
  groundtruth_stamped.pose.position.z = cur_pos(2);
  groundtruth_path.header = cmd.header;
  groundtruth_path.header.frame_id = "map";
  groundtruth_path.poses.push_back(groundtruth_stamped);
  pub_groundtruth_path.publish(groundtruth_path);
  last_yaw_ = cmd.yaw;

  if(get_initialpos){
    times++;
    if(times == points_size) times = 0;
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
  cur_vel(0) = msg->twist.twist.linear.x;
  cur_vel(1) = msg->twist.twist.linear.y;
  cur_vel(2) = msg->twist.twist.linear.z;
  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  cur_yaw = fromQuaternion2yaw(q);
  if(get_initialpos == false){
    initial_pos = Eigen::Vector3d::Zero();
    initial_pos(0) = msg->pose.pose.position.x;
    initial_pos(1) = msg->pose.pose.position.y;
    initial_pos(2) = msg->pose.pose.position.z;
    initial_q = q;
    initial_yaw = fromQuaternion2yaw(q);
    get_initialpos = true;
    //ROS_INFO("x = %lf, y = %lf, z = %lf", initial_pos(0), initial_pos(1), initial_pos(2));
  }
}
void circle_generate(){
  int sample_time = traj_duration_ / T;
  double radius = 2;
  
  generator.traj_pos_ = vector<Eigen::Vector3d>(sample_time, Eigen::Vector3d::Zero());
  generator.traj_vel_ = vector<Eigen::Vector3d>(sample_time, Eigen::Vector3d::Zero());
  generator.traj_acc_ = vector<Eigen::Vector3d>(sample_time, Eigen::Vector3d::Zero());

  for(int i = 0; i < sample_time; i++){
    Eigen::Vector3d p(radius * cos((2 * M_PI / sample_time) * i + M_PI) + radius, radius * sin((2 * M_PI / sample_time) * i + M_PI), 0);
    Eigen::Vector3d v(- radius * 2 * M_PI / traj_duration_ * sin((2 * M_PI / sample_time) * i + M_PI), radius * 2 * M_PI / traj_duration_  * cos((2 * M_PI / sample_time) * i + M_PI),  0);
    // Eigen::Vector3d a = Vector3d::Zero();
    Eigen::Vector3d a( - radius * pow(2 * M_PI / traj_duration_ , 2)* cos((2 * M_PI / sample_time) * i + M_PI), -radius * pow(2 * M_PI / traj_duration_ , 2) * sin((2 * M_PI / sample_time) * i + M_PI), 0);
    generator.traj_pos_[i] = p;
    generator.traj_vel_[i] = v;
    generator.traj_acc_[i] = a;
  }
  
  
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_generator");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");
  
  initial_pos_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 100, posCallback);
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  pub_des_path = nh.advertise<nav_msgs::Path>("des_path", 1000);
  pub_groundtruth_path = nh.advertise<nav_msgs::Path>("groundtruth_path", 1000);
  ros::Duration(0.5).sleep();
  // Matrix<double, 3, 3> waypoints;
  // waypoints << 0, 0, 0,\
  //              1, 0, 1,\
  //              2, 1, 2;
  // generator.Generator(waypoints.transpose());
  //generator.RandomGenerator();
  generator.circle_generate(5, 2);
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