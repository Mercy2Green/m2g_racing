//This function trans CirclePoses to vector of waypoints

#include <iostream>

# include <ros/ros.h>

# include <geometry_msgs/PoseStamped.h>
# include <geometry_msgs/PoseWithCovarianceStamped.h>

# include <sensor_msgs/Imu.h>

using namespace std;

ros::Publisher correct_pose_pub, correct_posewithcov_pub, correct_imu_pub;

geometry_msgs::PoseStamped correct_pose;

void pose_correct_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    geometry_msgs::PoseWithCovarianceStamped out_msg;
    out_msg.pose.covariance[0] = 0.05;
    out_msg.pose.covariance[7] = 0.05;
    out_msg.pose.covariance[14] = 0.05;
    out_msg.pose.covariance[21] = 0.01;
    out_msg.pose.covariance[28] = 0.01;
    out_msg.pose.covariance[35] = 0.01;

    out_msg.pose.pose = msg->pose;
    out_msg.pose.pose.position.y = - out_msg.pose.pose.position.y;
    out_msg.pose.pose.position.z = - out_msg.pose.pose.position.z;

    out_msg.header = msg->header;

    correct_pose = *msg;
    correct_pose.pose.position.y = -correct_pose.pose.position.y;
    correct_pose.pose.position.z = -correct_pose.pose.position.z;
    //correct_pose_pub.publish(correct_pose);
    correct_posewithcov_pub.publish(out_msg);
}

void imu_correct_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu out_msg;
    out_msg = *msg;
    out_msg.angular_velocity.y = -out_msg.angular_velocity.y;
    out_msg.angular_velocity.z = -out_msg.angular_velocity.z;
    out_msg.linear_acceleration.y = -out_msg.linear_acceleration.y;
    out_msg.linear_acceleration.z = -out_msg.linear_acceleration.z;

    correct_imu_pub.publish(out_msg);

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pose_correct");
    ros::NodeHandle nh;


    //geometry_msgs::Pose waypoint_pose;

    geometry_msgs::PoseStamped correct_pose;

    ros::Rate rate(100.0);

    ros::Subscriber origin_pose_sub = nh.subscribe("/airsim_node/drone_1/pose", 10, pose_correct_cb);

    correct_posewithcov_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose", 10);

    ros::Subscriber origin_imu_sub = nh.subscribe("/airsim_node/drone_1/imu/imu", 10, imu_correct_cb);

    correct_imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 10);

    //correct_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose", 10);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

}