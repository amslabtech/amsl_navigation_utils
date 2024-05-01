// Copyright 2023 amsl

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class OdomBroadcaster
{
public:
  OdomBroadcaster(void);
  void process();

protected:
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  tf::TransformBroadcaster *odom_broadcaster = nullptr;
};

OdomBroadcaster::OdomBroadcaster()
{
  odom_sub_ = nh_.subscribe("odom", 1, &OdomBroadcaster::odom_callback, this);
  odom_broadcaster = new tf::TransformBroadcaster;
  ROS_INFO_STREAM("=== Odom Broadcaster ===");
}

void OdomBroadcaster::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  geometry_msgs::TransformStamped odom_trans;

  odom_trans.header.stamp = msg->header.stamp;
  odom_trans.header.frame_id = msg->header.frame_id;
  odom_trans.child_frame_id = msg->child_frame_id;
  odom_trans.transform.translation.x = msg->pose.pose.position.x;
  odom_trans.transform.translation.y = msg->pose.pose.position.y;
  odom_trans.transform.translation.y = msg->pose.pose.position.z;
  odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
  odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
  odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
  odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;

  odom_broadcaster->sendTransform(odom_trans);
}

void OdomBroadcaster::process() { ros::spin(); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_broadcaster");
  OdomBroadcaster odom_broadcaster;
  odom_broadcaster.process();

  return 0;
}
