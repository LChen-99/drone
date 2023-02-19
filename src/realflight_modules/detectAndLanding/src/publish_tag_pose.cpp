
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include <iostream>

int main(int argc, char **argv)
{
    // std::cout << "here";
    ros::init(argc, argv, "pub_pose");
    ros::NodeHandle n("~");
    // ros::Subscriber sub_tag = ros::Subscriber();
    ros::Publisher pub_tag_waypoint = n.advertise<geometry_msgs::PoseStamped>("/tag_goal", 1);
    // tf2_ros::Buffer buffer;
    // tf2_ros::TransformListener listener(buffer);
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time();
    pose.pose.position.x = 3.65;
    pose.pose.position.y = -0.42;
    pose.pose.position.z = 1.5;
    pose.pose.orientation.w  = 1;
    pose.pose.orientation.x  = 0;
    pose.pose.orientation.y  = 0;
    pose.pose.orientation.z  = 0;
    // pub_tag_waypoint.publish(pose);
    ros::Rate r(1);
    // size_t tick = 0;
    while (ros::ok())
    {   
        pub_tag_waypoint.publish(pose);
        ROS_INFO("publish pose");
        r.sleep();  
        ros::spinOnce();
    }
    return 0;
}