
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include <iostream>

int main(int argc, char **argv)
{
    // std::cout << "here";
    ros::init(argc, argv, "detect_landing");
    ros::NodeHandle n("~");
    // ros::Subscriber sub_tag = ros::Subscriber();
    ros::Publisher pub_tag_waypoint = n.advertise<geometry_msgs::PoseStamped>("/tag_goal", 10);
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Rate r(1);
    size_t tick = 0;
    while (ros::ok())
    {   

        geometry_msgs::PointStamped point;
        point.header.frame_id = "tag_0";
        point.header.stamp = ros::Time();
        point.point.x = 0;
        point.point.y = 0;
        point.point.z = 0;
        
        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {   
            // buffer.lookupTransform()
            geometry_msgs::PointStamped point_base;
            geometry_msgs::PoseStamped pose;
            point_base = buffer.transform(point,"world");
            point_base.point.z = 1;
            ROS_INFO("tag_0:(%.2f,%.2f,%.2f)",point_base.point.x,point_base.point.y,point_base.point.z);
            if(tick == 3){
                pose.header.stamp = ros::Time();
                pose.pose.position.x = point_base.point.x;
                pose.pose.position.y = point_base.point.y;
                pose.pose.position.z = 0;
                pose.pose.orientation.w  = 1;
                pose.pose.orientation.x  = 0;
                pose.pose.orientation.y  = 0;
                pose.pose.orientation.z  = 0;
                pub_tag_waypoint.publish(pose);
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
        r.sleep();  
        ros::spinOnce();
    }
    return 0;
}