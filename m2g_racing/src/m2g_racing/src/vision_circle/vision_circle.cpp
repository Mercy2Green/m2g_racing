/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-10-06 18:19:19
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-10-11 20:48:38
 * @FilePath: /version_red_circle/red_circle.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define COLOR_LIMIT_LOW 80
#define COLOR_LIMIT_HIGHT 200

#define DISTANCE_LIMIT 10

#define x1 1200
#define x2 1280
#define y1 650
#define y2 720

std::string float_string(float num)
{
    std::ostringstream oss;
    oss<<num;
    std::string  str(oss.str());
    return str;
}

//比较轮廓面积(USB_Port_Lean用来进行轮廓排序)
bool Contour_Area(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2)
{
  return contourArea(contour1) > contourArea(contour2);
}



int main()
{
    // cv::namedWindow("red_window");

    cv::Mat src = cv::imread("../Screenshot/12.png");
    if (src.empty())
    {
        std::cout << "No Image" << std::endl;
    }
    else
    {
        std::cout << "Have Image" << std::endl;
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

    // cv::imshow("src_a",src_a);
    // cv::imshow("src_copy",src_copy);
    // cv::imshow("roi",roi);
    // cv::waitKey(0);
////////

    //HSV
    cv::Mat src_hsv,src_hsv_red,src_hsv_yellow;
    cv::cvtColor(src_copy,src_hsv,cv::COLOR_RGB2HSV_FULL);

    src_hsv.create(src_a.size(), src_a.type());
    cv::Scalar red_hsv_scalar_l = cv::Scalar(156,115,200);
    cv::Scalar red_hsv_scalar_h = cv::Scalar(180,255,255);
    cv::inRange(src_hsv,red_hsv_scalar_l,red_hsv_scalar_h,src_hsv_red);

    cv::Scalar yellow_hsv_scalar_l = cv::Scalar(100,60,220);
    cv::Scalar yellow_hsv_scalar_h = cv::Scalar(150,255,255);
    cv::inRange(src_hsv,yellow_hsv_scalar_l,yellow_hsv_scalar_h,src_hsv_yellow);

    

    cv::Mat element_hsv,src_hsv_open,src_hsv_erode,src_hsv_open2,src_hsv_delate;
    cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::morphologyEx(src_hsv_red,src_hsv_open,cv::MORPH_OPEN,element_hsv);
    cv::morphologyEx(src_hsv_yellow,src_hsv_open2,cv::MORPH_OPEN,element_hsv);
    // cv::morphologyEx(src_hsv_open,src_hsv_erode,cv::MORPH_ERODE,element_hsv);
    // cv::morphologyEx(src_hsv_erode,src_hsv_open2,cv::MORPH_ERODE,element_hsv);
    // cv::subtract(src_hsv_open,src_hsv_blackhat,src_hsv_delate);
    // cv::Mat src_hsv_blur;
    // cv::blur(src_hsv_open,src_hsv_blur,cv::Size(3,3));
    
    // cv::Mat src_hsv_canny;
    // cv::Canny(src_hsv_open,src_hsv_canny,300,300*2,3);

    // cv::imshow("hsv", src_hsv_open2);
    // cv::waitKey(0);

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
            std::cout<<"No Find Circle"<<std::endl;
            abort();
        }
        else
        {
            std::cout<<"yellow:"<<hsv_yellow_countours.size()<<std::endl;
            std::sort(hsv_yellow_countours.begin(), hsv_yellow_countours.end(), Contour_Area);
            find_circle = hsv_yellow_countours[0];
        }
    }
    else
    {
        std::cout<<hsv_red_countours.size()<<std::endl;
        std::sort(hsv_red_countours.begin(), hsv_red_countours.end(), Contour_Area);
        if(hsv_yellow_countours.size() == 0)
        {
            find_circle = hsv_red_countours[0];
        }
        else
        {
            std::cout<<"yellow:"<<hsv_yellow_countours.size()<<std::endl;
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
    // cv::Mat hsv_draw_counts;
    // hsv_draw_counts.create(src_hsv.size(),src_hsv.type());
    // std::cout<<hsv_red_countours.size()<<std::endl;
    // std::sort(hsv_red_countours.begin(), hsv_red_countours.end(), Contour_Area);
    // for(int i = 0; i < 1; i++)
    // {
    //     cv::drawContours(hsv_draw_counts,hsv_red_countours,i,cv::Scalar(255,255,255),2,8,hsv_red_hierarchy,0,cv::Point());
    // }

    
    // cv::HoughCircles(src_hsv_canny,);

    //RGB
    // cv::Mat src_rgb, src_red;

    // src_rgb.create(src_a.size(), src_a.type());

    // cv::Scalar red_scalar_l = cv::Scalar(0, 0, COLOR_LIMIT_HIGHT);
    // cv::Scalar red_scalar_h = cv::Scalar(COLOR_LIMIT_LOW, COLOR_LIMIT_LOW, 255);
    // cv::inRange(src_copy, red_scalar_l, red_scalar_h, src_rgb);

    // cv::Mat element,src_open;
    // cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    // cv::morphologyEx(src_rgb,src_open,cv::MORPH_OPEN,element);

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
    std::cout<<position<<std::endl;
    cv::putText(src_a,position,cv::Point(rect.x+rect.width,rect.y),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1,8,false);
    cv::putText(src_a,width,cv::Point(rect.x+rect.width,rect.y + 16),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1,8,false);
    cv::putText(src_a,height,cv::Point(rect.x+rect.width,rect.y + 32),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0),1,8,false);
    
    //显示
    cv::imshow("window", src_copy);
    cv::imshow("hsv", hsv_draw_counts);
    //cv::imshow("hsv_draw", src_hsv_open2);
    // cv::imshow("hsv_black", src_hsv_blackhat);
    // cv::imshow("hsv_open", src_hsv_open);
    //cv::imshow("rgb", src_rgb);
    //cv::imshow("morph",src_open);
    cv::imshow("after_draw",src_a);
    // cv::imshow("roi",roi);
    cv::waitKey(0);
    // cv::imshow("window",src_red);
    // cv::waitKey(0);

    // cv::Mat src_dilated;
    // cv::dilate(src_red,src_dilated,);

    return 0;
}