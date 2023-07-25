
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

int main(int argc, char **argv)
{
    // std::cout << "here";
    ros::init(argc, argv, "detect_landing");
    ros::NodeHandle n("~");
    // ros::Subscriber sub_tag = ros::Subscriber();
    ros::Publisher pub_tag_waypoint = n.advertise<geometry_msgs::PoseStamped>("/tag_goal", 10);
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Rate r(10);
    Eigen::Vector3d position(0, 0, 0);
    Eigen::Vector3d last_position(0, 0, 0);
    Eigen::Quaterniond q(0, 0, 0,0);
    size_t tick = 0;
    int times = 0;
    while (ros::ok())
    {   

        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {   
            // buffer.lookupTransform()
            geometry_msgs::PointStamped point_base;
            geometry_msgs::PoseStamped tag_pose;
            geometry_msgs::TransformStamped pose = buffer.lookupTransform("world", "tag_0", ros::Time());
            last_position = position;
            position = (position * times + Eigen::Vector3d(pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z)) / (times + 1);
            // q += Eigen::Quaterniond(pose.transform.rotation.w, pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z);
            times++;
            r.sleep();
            if((last_position - position).norm() < 0.01){
                pose.header.stamp = ros::Time();
                pose.header.frame_id = "world";
                tag_pose.pose.orientation = pose.transform.rotation;
                tag_pose.pose.position.x = pose.transform.translation.x;
                tag_pose.pose.position.y = pose.transform.translation.y;
                tag_pose.pose.position.z = pose.transform.translation.z;
                ROS_INFO("tag_0:(%.2f,%.2f,%.2f)",tag_pose.pose.position.x,tag_pose.pose.position.y,tag_pose.pose.position.z);
                ros::spinOnce();
                pub_tag_waypoint.publish(tag_pose);
                break;
            }
           
            tick++;
            
            // break;
        }

        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO(":%s",e.what());
        }
        
        
    }
    return 0;
}