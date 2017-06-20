#include "ros/ros.h"
#include "std_msgs/String.h"

class Listener
{
public:
  Listener();
  ~Listener();
  ros::NodeHandle nh;
private:
  ros::Subscriber listener_sub;
  void listenerCallback(const std_msgs::String::ConstPtr&);
};

Listener::Listener()
{
  listener_sub = nh.subscribe("chatter", 1000, &Listener::listenerCallback, this);
}

Listener::~Listener()
{}

void Listener::listenerCallback(const std_msgs::String::ConstPtr& str_msg)
{
  ROS_INFO("%s", str_msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  Listener l;

  ros::spin();

  return(0);
}
