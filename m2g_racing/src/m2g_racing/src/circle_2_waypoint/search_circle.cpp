/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-10-06 18:19:19
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-10-22 20:54:56
 * @FilePath: /version_red_circle/red_circle.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Odometry.h>

//static const std::string OPENCV_WINDOW = "Image window";

using namespace std;

#define COLOR_LIMIT_LOW 80
#define COLOR_LIMIT_HIGHT 200

#define DISTANCE_LIMIT 10

#define x1 1200
#define x2 1280
#define y1 650
#define y2 720

float bias_x = 0.5;
float bias_y = 0.3;
float bias_z = 0.15;

cv::Mat rvec(3,1,cv::DataType<double>::type);
cv::Mat tvec(3,1,cv::DataType<double>::type);

geometry_msgs::PointStamped circle_location;

ros::Publisher circle_location_pub;

std::string float_string(float num)
{
    std::ostringstream oss;
    oss<<num;
    std::string str(oss.str());
    return str;
}

//比较轮廓面积(USB_Port_Lean用来进行轮廓排序)
bool Contour_Area(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2)
{
  return contourArea(contour1) > contourArea(contour2);
}

//比较轮廓面积(USB_Port_Lean用来进行轮廓排序)
bool Point_2d_x(cv::Point2d point1, cv::Point2d point2)
{
  return point1.x > point2.x;
}

bool Point_2d_y(cv::Point2d point1, cv::Point2d point2)
{
  return point1.y > point2.y;
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/airsim_node/drone_1/front_left/Scene", 1,
      &ImageConverter::imageCb, this);


    //image_pub_ = it_.advertise("/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::Mat src = cv_ptr->image;

    // cv::namedWindow("red_window");

    //cv::Mat src = cv::imread("../left_camera/1.png");

    if (src.empty())
    {
        //std::cout << "No Image" << std::endl;
    }
    else
    {
        //std::cout << "Have Image" << std::endl;
    }

    cv::Mat src_copy, src_a;
    //src.copyTo(src_copy);
    src.copyTo(src_a);

//////////
    //去除右下角的固定红色部分
    cv::Mat roi;
    roi.create(src_a.size(),CV_8UC1);
    roi.setTo(255);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> pts;
    
    //cv::rectangle();
    pts.push_back(cv::Point(x1,y1));
    pts.push_back(cv::Point(x1,y2));
    pts.push_back(cv::Point(x2,y2));
    pts.push_back(cv::Point(x2,y1));
    contours.push_back(pts);
    cv::drawContours(roi,contours,0,cv::Scalar(0),-1);
    src.copyTo(src_copy,roi);

////////

    //HSV
    cv::Mat src_hsv,src_hsv_red,src_hsv_yellow;
    cv::cvtColor(src_copy,src_hsv,cv::COLOR_RGB2HSV_FULL);

    src_hsv.create(src_a.size(), src_a.type());
    cv::Scalar red_hsv_scalar_l = cv::Scalar(156,115,80);
    cv::Scalar red_hsv_scalar_h = cv::Scalar(180,255,255);
    cv::inRange(src_hsv,red_hsv_scalar_l,red_hsv_scalar_h,src_hsv_red);

    cv::Scalar yellow_hsv_scalar_l = cv::Scalar(100,60,220);
    cv::Scalar yellow_hsv_scalar_h = cv::Scalar(150,255,255);
    cv::inRange(src_hsv,yellow_hsv_scalar_l,yellow_hsv_scalar_h,src_hsv_yellow);

    cv::Mat element_hsv,src_hsv_open,src_hsv_erode,src_hsv_open2,src_hsv_delate;
    cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::morphologyEx(src_hsv_red,src_hsv_open,cv::MORPH_OPEN,element_hsv);
    cv::morphologyEx(src_hsv_yellow,src_hsv_open2,cv::MORPH_OPEN,element_hsv);

    std::vector<std::vector<cv::Point>> hsv_red_countours;
    std::vector<cv::Vec4i> hsv_red_hierarchy;
    cv::findContours(src_hsv_open,hsv_red_countours,hsv_red_hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> hsv_yellow_countours;
    std::vector<cv::Vec4i> hsv_yellow_hierarchy;
    cv::findContours(src_hsv_open2,hsv_yellow_countours,hsv_yellow_hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> find_circle;
    cv::Mat hsv_draw_counts;
    hsv_draw_counts.create(src_hsv.size(),src_hsv.type());
    if(hsv_red_countours.size() == 0)
    {
        if(hsv_yellow_countours.size() == 0)
        {
            //std::cout<<"No Find Circle"<<std::endl;
            return;
        }
        else
        {
            //std::cout<<"yellow:"<<hsv_yellow_countours.size()<<std::endl;
            std::sort(hsv_yellow_countours.begin(), hsv_yellow_countours.end(), Contour_Area);
            find_circle = hsv_yellow_countours[0];
        }
    }
    else
    {
        //std::cout<<hsv_red_countours.size()<<std::endl;
        std::sort(hsv_red_countours.begin(), hsv_red_countours.end(), Contour_Area);
        if(hsv_yellow_countours.size() == 0)
        {
            find_circle = hsv_red_countours[0];
        }
        else
        {
            //std::cout<<"yellow:"<<hsv_yellow_countours.size()<<std::endl;
            std::sort(hsv_yellow_countours.begin(), hsv_yellow_countours.end(), Contour_Area);
            if(cv::contourArea(hsv_red_countours[0]) > cv::contourArea(hsv_yellow_countours[0]))
            {
                find_circle = hsv_red_countours[0];
            }
            else
            {
                find_circle = hsv_yellow_countours[0];
            }
        }
    }
    cv::drawContours(hsv_draw_counts,hsv_yellow_countours,0,cv::Scalar(255,255,255),2,8,hsv_yellow_hierarchy,0,cv::Point());

    //画框
    cv::Rect rect;
    rect = cv::boundingRect(find_circle);

    cv::rectangle(src_a,rect,cv::Scalar(0,255,0),2);
    //写坐标
    std::string position,p_x,p_y,p_width,p_height,width,height;
    p_x = float_string(rect.x+rect.width/2.0);
    p_y = float_string(rect.y + rect.height/2.0);
    p_width = float_string(rect.width);
    p_height = float_string(rect.height);

    position = "center:(" + p_x + "," + p_y + ")";
    width = "width:" + p_width ;
    height = "height:" + p_height;
    //std::cout<<position<<std::endl;
    cv::putText(src_a,position,cv::Point(rect.x+rect.width,rect.y),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1,8,false);
    cv::putText(src_a,width,cv::Point(rect.x+rect.width,rect.y + 16),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1,8,false);
    cv::putText(src_a,height,cv::Point(rect.x+rect.width,rect.y + 32),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1,8,false);

    std::vector<cv::Point2d> point2D;
    std::vector<cv::Point3f> circle_position;
    std::vector<cv::Point> aaa;
    std::vector<std::vector<cv::Point>> circle_point;
    std::vector<cv::Vec4i> circle_position_hierarchy;

    std::sort(find_circle.begin(), find_circle.end(),Point_2d_x);
    point2D.push_back(find_circle.at(0));
    point2D.push_back(find_circle.at(find_circle.size()-1));

    std::sort(find_circle.begin(), find_circle.end(),Point_2d_y);
    point2D.push_back(find_circle.at(0));
    point2D.push_back(find_circle.at(find_circle.size()-1));

    //std::cout<<point2D<<std::endl;

    cv::Mat circle_picture,abc;
    circle_picture.create(src_a.size(),CV_8U);
    cv::circle(circle_picture,cv::Point(circle_picture.cols/ 2,circle_picture.rows/ 2),100,cv::Scalar(255,255,255));//100 is d (d is 2 * r)
    cv::findContours(circle_picture,circle_point,circle_position_hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    std::sort(circle_point.begin(), circle_point.end(), Contour_Area);
    aaa = circle_point[0];

    std::sort(aaa.begin(), aaa.end(),Point_2d_x);
    circle_position.push_back(cv::Point3f(aaa.at(0).x,aaa.at(0).y,0));
    circle_position.push_back(cv::Point3f(aaa.at(aaa.size()-1).x,aaa.at(aaa.size()-1).y,0));

    std::sort(aaa.begin(), aaa.end(),Point_2d_y);
    circle_position.push_back(cv::Point3f(aaa.at(0).x,aaa.at(0).y,0));
    circle_position.push_back(cv::Point3f(aaa.at(aaa.size()-1).x,aaa.at(aaa.size()-1).y,0));

    //std::cout<<circle_position<<std::endl;

    double fx = 319.095644; //focal length x
    double fy = 319.567799;//focal le
 
    double cx = 319.296424; //optical centre x
    double cy = 239.651267; //optical centre y

    cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
    cameraMatrix.at<double>(0,0)=fx;
    cameraMatrix.at<double>(1,1)=fy;
    cameraMatrix.at<double>(2,2)=1;
    cameraMatrix.at<double>(0,2)=cx;
    cameraMatrix.at<double>(1,2)=cy;
    cameraMatrix.at<double>(0,1)=0;
    cameraMatrix.at<double>(1,0)=0;
    cameraMatrix.at<double>(2,0)=0;
    cameraMatrix.at<double>(2,1)=0;

    cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = -0.001091;
    distCoeffs.at<double>(1) = 0.000990;
    distCoeffs.at<double>(2) = 0.000026;
    distCoeffs.at<double>(3) = -0.000359;
    distCoeffs.at<double>(4) = 0.000000;
    // cv::Mat rvec(3,1,cv::DataType<double>::type);
    // cv::Mat tvec(3,1,cv::DataType<double>::type);
 
    cv::solvePnP(circle_position, point2D, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

    tvec.at<double>(0, 0) = tvec.at<double>(0, 0)/1000 + bias_y;
    tvec.at<double>(1, 0) = tvec.at<double>(1, 0)/1000 + bias_z;
    tvec.at<double>(2, 0) = tvec.at<double>(2, 0)/1000 + bias_x;

    // std::cout<<rvec<<std::endl;
    //std::cout<<tvec<<std::endl;

    cv::Mat mix;
    src.copyTo(mix);
    cv::circle(mix,cv::Point(circle_picture.cols/ 2,circle_picture.rows/ 2),100,cv::Scalar(255,0,255));

    //显示
    //cv::imshow("window", mix);
    //cv::imshow("hsv", hsv_draw_counts);
    //cv::imshow("hsv_open", src_hsv_open);
    //cv::imshow("after_draw",src_a);
    cv::waitKey(5);
    // Output modified video stream
    //cv_ptr->image = src_hsv_open;
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};


void uav_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{


    geometry_msgs::Point uav_point = msg->pose.pose.position;

    circle_location.header.frame_id = std::string("world");
    circle_location.header.stamp = ros::Time::now();


    circle_location.point.x = uav_point.x + (tvec.at<double>(2, 0) + bias_x);
    circle_location.point.y = uav_point.y - (tvec.at<double>(0, 0) + bias_y);
    circle_location.point.z = uav_point.z - (tvec.at<double>(1, 0) + bias_z);

    circle_location_pub.publish(circle_location);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");

    ros::NodeHandle nh;
    circle_location_pub = nh.advertise<geometry_msgs::PointStamped>("/circle_location", 10);

    ros::Subscriber uav_pose_sub = nh.subscribe("/odometry", 1, uav_pose_cb);

    ImageConverter ic;
    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    // ros::spin();
    // rate.sleep();

    return 0;
}

//shou lian zai fa