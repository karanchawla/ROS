#include "imagerepublisher.h"
#include <iostream>

static const std::string OPENCV_WINDOW = "Image Window";

ImageRepublisher::ImageRepublisher()
  :it(nh)
{
  sub = it.subscribe("/callisto/camera1/image_raw",1,&ImageRepublisher::imageCallback,this);
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  cv::namedWindow(OPENCV_WINDOW);
}

ImageRepublisher::~ImageRepublisher()
{}

void ImageRepublisher::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  int rows = cv_ptr->image.rows;
  int cols = cv_ptr->image.cols;

  cv::Mat hsv;
  cv_ptr->image.copyTo(hsv);
  cv::cvtColor(cv_ptr->image,hsv,cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(10,10,10), cv::Scalar(255, 255, 250), mask);

  int h = cv_ptr->image.size().height;
  int w = cv_ptr->image.size().width;

  int search_top = 3*h/4;
  int search_bottom = 3*h/4;


  cv::Moments m = cv::moments(mask);

  if(m.m00>0)
  {
    int cx = int(m.m10/m.m00);
    int cy = int(m.m01/m.m00);
    cv::circle(cv_ptr->image,cv::Point(cx,cy), 20, CV_RGB(0,0,255), -1);
    float err = cx - w/2;
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.2;
    vel_msg.angular.z = - float(err)/100;
    pub.publish(vel_msg);
  }

  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

}

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"image_republisher");
  ImageRepublisher ir;
  ros::spin();
  return 0;
}
