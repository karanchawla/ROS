#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class Odometry
{
public:
  Odometry();
  ~Odometry();
  ros::NodeHandle nh;
private:
  ros::Subscriber odom_sub;
  tf::TransformBroadcaster odom_broadcaster;
  void odomCallback(const nav_msgs::Odometry& msg);
};

Odometry::Odometry()
{
  odom_sub = nh.subscribe("odom",50, &Odometry::odomCallback,this);
}

Odometry::~Odometry()
{}

void Odometry::odomCallback(const nav_msgs::Odometry &msg)
{
  float x = msg.pose.pose.position.x;
  float y = msg.pose.pose.position.y;
  float z = msg.pose.pose.position.z;

  float alpha = msg.pose.pose.orientation.x;
  float beta = msg.pose.pose.orientation.y;
  float gamma = msg.pose.pose.orientation.z;
  float delta = msg.pose.pose.orientation.w;

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "/world";
  odom_trans.child_frame_id = "/base_footprint";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = z;

  odom_trans.transform.rotation.x = alpha;
  odom_trans.transform.rotation.y = beta;
  odom_trans.transform.rotation.z = gamma;
  odom_trans.transform.rotation.w = delta;

  odom_broadcaster.sendTransform(odom_trans);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "odometry_node");

  ros::NodeHandle  nh;

  ros::spin();

  return 0;
}
