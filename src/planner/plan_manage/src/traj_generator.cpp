#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <plan_manage/generator.h>
#include <nav_msgs/Path.h>
// #include <nav_msgs/Odometry.h>
#include <math.h>
ros::Publisher pos_cmd_pub;
ros::Publisher pub_des_path;
ros::Subscriber initial_pos_sub;
ros::Publisher pub_groundtruth_path;
quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;
using namespace std;
bool receive_traj_ = true;
vector<UniformBspline> traj_;
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
Eigen::Quaterniond initial_q;
bool get_initialpos = false;
Eigen::Vector3d initial_pos;
double initial_yaw;
Eigen::Vector3d cur_pos;
Eigen::Vector3d cur_vel;
double cur_yaw;
TrajGenerator generator(0.02);
void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if(!get_initialpos){
    return;
  }
  if(generator.traj_pos_.size() == 0){
    times = 0;
    generator.last_pos = cur_pos;
    generator.RandomGenerator();
    
  }
  ros::Time time_now = ros::Time::now();
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

    /* hover when finish traj_ */
    pos = generator.traj_pos_[points_size - 1];
    vel.setZero();
    acc.setZero();
    if(times - points_size > 100){
      generator.last_pos = cur_pos;
      generator.RandomGenerator();
      // des_path.poses.clear();
      times = 0;
    }

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
  cmd.position.x = pos(0) ;
  cmd.position.y = pos(1) ;
  cmd.position.z = pos(2) ;
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
  despose_stamped.pose.position.x = pos(0);
  despose_stamped.pose.position.y = pos(1);
  despose_stamped.pose.position.z = pos(2);
  des_path.header = cmd.header;
  des_path.header.frame_id = "map";
  des_path.poses.push_back(despose_stamped);
  pub_des_path.publish(des_path);

  geometry_msgs::PoseStamped groundtruth_stamped;
  groundtruth_stamped.header = cmd.header;
  groundtruth_stamped.header.frame_id = "map";

  groundtruth_stamped.pose.position.x = cur_pos(0);
  groundtruth_stamped.pose.position.y = cur_pos(1);
  groundtruth_stamped.pose.position.z = cur_pos(2);
  groundtruth_path.header = cmd.header;
  groundtruth_path.header.frame_id = "map";
  groundtruth_path.poses.push_back(groundtruth_stamped);
  pub_groundtruth_path.publish(groundtruth_path);
  last_yaw_ = cmd.yaw;
  // 获得初始的pose，在初始的pose为起点产生轨迹
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
    generator.initial_pos = initial_pos;
    initial_q = q;
    initial_yaw = fromQuaternion2yaw(q);
    generator.initial_yaw = initial_yaw;
    get_initialpos = true;
    
  }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_generator");
 
  ros::NodeHandle nh("~");
  
  initial_pos_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 100, posCallback);
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  pub_des_path = nh.advertise<nav_msgs::Path>("des_path", 1000);
  pub_groundtruth_path = nh.advertise<nav_msgs::Path>("groundtruth_path", 1000);
  ros::Duration(0.5).sleep();
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