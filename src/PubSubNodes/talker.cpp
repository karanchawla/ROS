#include "ros/ros.h"
#include "std_msgs/String.h"

class Publisher
{
public:
  Publisher();
  ~Publisher();
  ros::NodeHandle nh;
  void chatterLoop();
private:
  ros::Publisher chatter_pub;
};

Publisher::Publisher()
{
  chatter_pub = nh.advertise<std_msgs::String>("chatter",1000);
}

Publisher::~Publisher()
{}

void Publisher::chatterLoop()
{
  std_msgs::String msg;
  msg.data = "Basic ROS Node class example";

  chatter_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  Publisher p;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    p.chatterLoop();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
