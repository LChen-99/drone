
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include <iostream>
using namespace std;
int main(int argc, char **argv)
{
    // std::cout << "here";
    ros::init(argc, argv, "pub_pose");
    if(argc > 4){
        cout << "error xyz" << endl;
        return 0;
    }
    ros::NodeHandle n("~");
    // ros::Subscriber sub_tag = ros::Subscriber();
    ros::Publisher pub_tag_waypoint = n.advertise<geometry_msgs::PoseStamped>("/tag_goal", 10);
    // tf2_ros::Buffer buffer;
    // tf2_ros::TransformListener listener(buffer);
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time();
    pose.header.frame_id = "world";
    pose.pose.position.x = stod(argv[1]);
    pose.pose.position.y = stod(argv[2]);
    pose.pose.position.z = stod(argv[3]);
    ROS_INFO("x = %lf, y = %lf, z = %lf", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    pose.pose.orientation.w  = 1;
    pose.pose.orientation.x  = 0;
    pose.pose.orientation.y  = 0;
    pose.pose.orientation.z  = 0;
    // pub_tag_waypoint.publish(pose);
    ros::Rate r(10);
    // size_t tick = 0;
    for(int i = 0; i < 1; i++){
        r.sleep();
        ros::spinOnce();
        pub_tag_waypoint.publish(pose);
        
    }
    
    ROS_INFO("publish pose");
    
    
    return 0;
}