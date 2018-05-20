#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    std::cout << "Height = " << cloud->height << std::endl; 
}


int main (int argc, char** argv) 
{
     ros::init (argc, argv, "cloud_sub");
     ros::NodeHandle n;
     ros::Rate loop_rate(10);
     ros::Subscriber sub;
     sub = n.subscribe ("/camera/depth/points", 1, cloud_callback);
     ros::spin();
 }
