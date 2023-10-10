//This function trans CirclePoses to vector of waypoints

#include <iostream>

# include <ros/ros.h>

# include "airsim_ros/CirclePoses.h"

# include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>

#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>

#include <airsim_ros/Takeoff.h>

using namespace std;

airsim_ros::CirclePoses circle_pose;
int circle_num = 0; 
airsim_ros::Circle* circle_list;
airsim_ros::Circle circle;

nav_msgs::Path waypoints;


void circle_poses_cb(const airsim_ros::CirclePoses::ConstPtr& msg)
{
    circle_pose = *msg;
    circle_num = circle_pose.poses.size();
}

void circle_cb(const airsim_ros::Circle::ConstPtr& msg)
{
    circle = *msg;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "circle_2_waypoint");
    ros::NodeHandle nh;
    geometry_msgs::Pose waypoint_pose;
    airsim_ros::Takeoff takeoff;

    bool takeoffflag = false;
    //takeoffflag = true;

    ros::Rate rate(1.0);

    ros::ServiceClient takeoff_client = nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    if(!takeoffflag)
    {
        takeoff_client.call(takeoff);
        takeoffflag = true;
        ros::spinOnce();
    }


    //ros::Subscriber circle_poses_sub = nh.subscribe("/airsim_node/drone_1/circle_poses", 1, circle_poses_cb);
    ros::Subscriber circle_poses_sub = nh.subscribe("/airsim_node/drone_1/circle_poses", 1, circle_poses_cb);

    ros::Publisher waypoint_pub = nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 1);


    while(ros::ok() && circle_num==0){

        ros::spinOnce();
        rate.sleep();
    }

    



    if (circle_num != 0)
    {
        // geometry_msgs::PoseStamped init_pose;
        // init_pose.pose.position.x = 0;
        // init_pose.pose.position.y = 0;
        // init_pose.pose.position.z = 1;
        // waypoints.poses.push_back(init_pose);
        // waypoints.header.frame_id = std::string("world");
        // waypoints.header.stamp = ros::Time::now();

        
        vector<int> not_circle = {7, 8, 9, 10, 11};

        //for (int i=0;i<circle_num;i++)
        for (int i=0;i<7;i++)
        {
            if (std::find(not_circle.begin(), not_circle.end(), i-1) != not_circle.end())
            {
                continue;
            }
            else
            {
                cout << "circle_num: " << i << endl;
                circle.position.x = circle_pose.poses[i].position.x;
                circle.position.y = circle_pose.poses[i].position.y;
                circle.position.z = circle_pose.poses[i].position.z;

                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.pose.position.x = circle.position.x;
                pose_stamped.pose.position.y = -circle.position.y;
                pose_stamped.pose.position.z = -circle.position.z;
                waypoints.poses.push_back(pose_stamped);
                waypoints.header.frame_id = std::string("world");
                waypoints.header.stamp = ros::Time::now();
            }

        }
    while (ros::ok())
    {
        waypoint_pub.publish(waypoints);
        ros::spinOnce();
        rate.sleep();
    }

    }
}


//-------------V1
    // if (circle_num != 0)
    // {
    //     for (int i=0;i<circle_num;i++)
    //     {
    //         cout << "circle_num: " << i << endl;
    //         circle.position.x = circle_pose.poses[i].position.x;
    //         circle.position.y = circle_pose.poses[i].position.y;
    //         circle.position.z = circle_pose.poses[i].position.z;
    //         geometry_msgs::PoseStamped pose_stamped;
    //         pose_stamped.pose.position.x = circle.position.x;
    //         pose_stamped.pose.position.y = circle.position.y;
    //         pose_stamped.pose.position.z = circle.position.z;
    //         waypoints.poses.push_back(pose_stamped);
    //         waypoints.header.frame_id = std::string("world");
    //         waypoints.header.stamp = ros::Time::now();
    //     }
    // while (ros::ok())
    // {
    //     waypoint_pub.publish(waypoints);
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // }





// void publish_waypoints() {
//     waypoints.header.frame_id = std::string("world");
//     waypoints.header.stamp = ros::Time::now();
//     pub1.publish(waypoints);
//     // geometry_msgs::PoseStamped init_pose;
//     // init_pose.header = odom.header;
//     // init_pose.pose = odom.pose.pose;
//     // waypoints.poses.insert(waypoints.poses.begin(), init_pose);
//     // // pub2.publish(waypoints);
//     // waypoints.poses.clear();
// }

// void circle_2_waypoint(airsim_ros::Circle::ConstPtr& circle, nav_msgs::Path::ConstPtr& waypoint)
// {
//     geometry_msgs::PoseStamped poses_waypoint;
//     poses_waypoint = waypoint->poses;
//     poses_waypoint.pose.position.x = ;
// }

// void circle_2_waypoint(airsim_ros::Circle::ConstPtr& circle, geometry_msgs::Pose::Ptr& waypoint_pose)
// {
//     waypoint_pose->position.x = circle->position.x;
//     waypoint_pose->position.y = circle->position.y;
//     waypoint_pose->position.z = circle->position.z;

//     //This needs yaw....
// }



//waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);

//This is I want

// #include <iostream>
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/Vector3.h>
// #include <nav_msgs/Path.h>
// #include "sample_waypoints.h"
// #include <vector>
// #include <deque>
// #include <boost/format.hpp>
// #include <eigen3/Eigen/Dense>

// using namespace std;
// using bfmt = boost::format;

// ros::Publisher pub1;
// ros::Publisher pub2;
// ros::Publisher pub3;
// string waypoint_type = string("manual");
// bool is_odom_ready;
// nav_msgs::Odometry odom;
// nav_msgs::Path waypoints;

// // series waypoint needed
// std::deque<nav_msgs::Path> waypointSegments;
// ros::Time trigged_time;

// void load_seg(ros::NodeHandle& nh, int segid, const ros::Time& time_base) {
//     std::string seg_str = boost::str(bfmt("seg%d/") % segid);
//     double yaw;
//     double time_of_start = 0.0;
//     ROS_INFO("Getting segment %d", segid);
//     ROS_ASSERT(nh.getParam(seg_str + "yaw", yaw));
//     ROS_ASSERT_MSG((yaw > -3.1499999) && (yaw < 3.14999999), "yaw=%.3f", yaw);
//     ROS_ASSERT(nh.getParam(seg_str + "time_of_start", time_of_start));
//     ROS_ASSERT(time_of_start >= 0.0);

//     std::vector<double> ptx;
//     std::vector<double> pty;
//     std::vector<double> ptz;

//     ROS_ASSERT(nh.getParam(seg_str + "x", ptx));
//     ROS_ASSERT(nh.getParam(seg_str + "y", pty));
//     ROS_ASSERT(nh.getParam(seg_str + "z", ptz));

//     ROS_ASSERT(ptx.size());
//     ROS_ASSERT(ptx.size() == pty.size() && ptx.size() == ptz.size());

//     nav_msgs::Path path_msg;

//     path_msg.header.stamp = time_base + ros::Duration(time_of_start);

//     double baseyaw = tf::getYaw(odom.pose.pose.orientation);
    
//     for (size_t k = 0; k < ptx.size(); ++k) {
//         geometry_msgs::PoseStamped pt;
//         pt.pose.orientation = tf::createQuaternionMsgFromYaw(baseyaw + yaw);
//         Eigen::Vector2d dp(ptx.at(k), pty.at(k));
//         Eigen::Vector2d rdp;
//         rdp.x() = std::cos(-baseyaw-yaw)*dp.x() + std::sin(-baseyaw-yaw)*dp.y();
//         rdp.y() =-std::sin(-baseyaw-yaw)*dp.x() + std::cos(-baseyaw-yaw)*dp.y();
//         pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
//         pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
//         pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;
//         path_msg.poses.push_back(pt);
//     }

//     waypointSegments.push_back(path_msg);
// }

// void load_waypoints(ros::NodeHandle& nh, const ros::Time& time_base) {
//     int seg_cnt = 0;
//     waypointSegments.clear();
//     ROS_ASSERT(nh.getParam("segment_cnt", seg_cnt));
//     for (int i = 0; i < seg_cnt; ++i) {
//         load_seg(nh, i, time_base);
//         if (i > 0) {
//             ROS_ASSERT(waypointSegments[i - 1].header.stamp < waypointSegments[i].header.stamp);
//         }
//     }
//     ROS_INFO("Overall load %zu segments", waypointSegments.size());
// }

// void publish_waypoints() {
//     waypoints.header.frame_id = std::string("world");
//     waypoints.header.stamp = ros::Time::now();
//     pub1.publish(waypoints);
//     geometry_msgs::PoseStamped init_pose;
//     init_pose.header = odom.header;
//     init_pose.pose = odom.pose.pose;
//     waypoints.poses.insert(waypoints.poses.begin(), init_pose);
//     // pub2.publish(waypoints);
//     waypoints.poses.clear();
// }

// void publish_waypoints_vis() {
//     nav_msgs::Path wp_vis = waypoints;
//     geometry_msgs::PoseArray poseArray;
//     poseArray.header.frame_id = std::string("world");
//     poseArray.header.stamp = ros::Time::now();

//     {
//         geometry_msgs::Pose init_pose;
//         init_pose = odom.pose.pose;
//         poseArray.poses.push_back(init_pose);
//     }

//     for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
//         geometry_msgs::Pose p;
//         p = it->pose;
//         poseArray.poses.push_back(p);
//     }
//     pub2.publish(poseArray);
// }

// void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
//     is_odom_ready = true;
//     odom = *msg;

//     if (waypointSegments.size()) {
//         ros::Time expected_time = waypointSegments.front().header.stamp;
//         if (odom.header.stamp >= expected_time) {
//             waypoints = waypointSegments.front();

//             std::stringstream ss;
//             ss << bfmt("Series send %.3f from start:\n") % trigged_time.toSec();
//             for (auto& pose_stamped : waypoints.poses) {
//                 ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
//                           pose_stamped.pose.position.x % pose_stamped.pose.position.y %
//                           pose_stamped.pose.position.z % pose_stamped.pose.orientation.w %
//                           pose_stamped.pose.orientation.x % pose_stamped.pose.orientation.y %
//                           pose_stamped.pose.orientation.z << std::endl;
//             }
//             ROS_INFO_STREAM(ss.str());

//             publish_waypoints_vis();
//             publish_waypoints();

//             waypointSegments.pop_front();
//         }
//     }
// }

// void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
// /*    if (!is_odom_ready) {
//         ROS_ERROR("[waypoint_generator] No odom!");
//         return;
//     }*/

//     trigged_time = ros::Time::now(); //odom.header.stamp;
//     //ROS_ASSERT(trigged_time > ros::Time(0));

//     ros::NodeHandle n("~");
//     n.param("waypoint_type", waypoint_type, string("manual"));
    
//     if (waypoint_type == string("circle")) {
//         waypoints = circle();
//         publish_waypoints_vis();
//         publish_waypoints();
//     } else if (waypoint_type == string("eight")) {
//         waypoints = eight();
//         publish_waypoints_vis();
//         publish_waypoints();
//     } else if (waypoint_type == string("points")) {
//         waypoints = point();
//         publish_waypoints_vis();
//         publish_waypoints();
//     } else if (waypoint_type == string("series")) {
//         load_waypoints(n, trigged_time);
//     } else if (waypoint_type == string("manual-lonely-waypoint")) {
//         if (msg->pose.position.z > -0.1) {
//             // if height > 0, it's a valid goal;
//             geometry_msgs::PoseStamped pt = *msg;
//             waypoints.poses.clear();
//             waypoints.poses.push_back(pt);
//             publish_waypoints_vis();
//             publish_waypoints();
//         } else {
//             ROS_WARN("[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
//         }
//     } else {
//         if (msg->pose.position.z > 0) {
//             // if height > 0, it's a normal goal;
//             geometry_msgs::PoseStamped pt = *msg;
//             if (waypoint_type == string("noyaw")) {
//                 double yaw = tf::getYaw(odom.pose.pose.orientation);
//                 pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
//             }
//             waypoints.poses.push_back(pt);
//             publish_waypoints_vis();
//         } else if (msg->pose.position.z > -1.0) {
//             // if 0 > height > -1.0, remove last goal;
//             if (waypoints.poses.size() >= 1) {
//                 waypoints.poses.erase(std::prev(waypoints.poses.end()));
//             }
//             publish_waypoints_vis();
//         } else {
//             // if -1.0 > height, end of input
//             if (waypoints.poses.size() >= 1) {
//                 publish_waypoints_vis();
//                 publish_waypoints();
//             }
//         }
//     }
// }

// void traj_start_trigger_callback(const geometry_msgs::PoseStamped& msg) {
//     if (!is_odom_ready) {
//         ROS_ERROR("[waypoint_generator] No odom!");
//         return;
//     }

//     ROS_WARN("[waypoint_generator] Trigger!");
//     trigged_time = odom.header.stamp;
//     ROS_ASSERT(trigged_time > ros::Time(0));

//     ros::NodeHandle n("~");
//     n.param("waypoint_type", waypoint_type, string("manual"));

//     ROS_ERROR_STREAM("Pattern " << waypoint_type << " generated!");
//     if (waypoint_type == string("free")) {
//         waypoints = point();
//         publish_waypoints_vis();
//         publish_waypoints();
//     } else if (waypoint_type == string("circle")) {
//         waypoints = circle();
//         publish_waypoints_vis();
//         publish_waypoints();
//     } else if (waypoint_type == string("eight")) {
//         waypoints = eight();
//         publish_waypoints_vis();
//         publish_waypoints();
//    } else if (waypoint_type == string("point")) {
//         waypoints = point();
//         publish_waypoints_vis();
//         publish_waypoints();
//     } else if (waypoint_type == string("series")) {
//         load_waypoints(n, trigged_time);
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "waypoint_generator");
//     ros::NodeHandle n("~");
//     n.param("waypoint_type", waypoint_type, string("manual"));
//     ros::Subscriber sub1 = n.subscribe("odom", 10, odom_callback);
//     ros::Subscriber sub2 = n.subscribe("goal", 10, goal_callback);
//     ros::Subscriber sub3 = n.subscribe("traj_start_trigger", 10, traj_start_trigger_callback);
//     pub1 = n.advertise<nav_msgs::Path>("waypoints", 50);
//     pub2 = n.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);

//     trigged_time = ros::Time(0);

//     ros::spin();
//     return 0;
// }
