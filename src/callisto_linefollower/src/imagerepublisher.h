#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>

class ImageRepublisher
{
private:
  ros::NodeHandle nh;

  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  ros::Publisher pub;

  void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
public:
  ImageRepublisher();
  ~ImageRepublisher();
};
