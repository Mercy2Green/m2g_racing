#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


ros::Publisher camera_info_pub, image_pub;
ros::Subscriber camera_info_sub, image_sub;


void camera_info_cb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    camera_info_pub.publish(*msg);

}

void image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    image_pub.publish(*msg);
}



int main (int argc ,char* argv[])
{

    ros::init(argc, argv, "scene_2_image_raw");
    ros::NodeHandle nh;

    camera_info_sub = nh.subscribe("/Scene/camera_info", 1, camera_info_cb);
    image_sub = nh.subscribe("/Scene", 1, image_cb);
        
    camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    image_pub = nh.advertise<sensor_msgs::Image>("image_raw", 1);

    ros::Rate rate(20.0);

    while(ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }


    return 0;
}