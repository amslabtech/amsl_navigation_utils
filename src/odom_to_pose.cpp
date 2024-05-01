// Copyright 2024 amsl

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class OdomToPose
{
public:
  OdomToPose(void);
  void process(void);

private:
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);

  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::Subscriber odom_sub_;
};

OdomToPose::OdomToPose(void)
{
  odom_sub_ =
      nh_.subscribe("odom", 1, &OdomToPose::odom_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
}

void OdomToPose::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
  geometry_msgs::PoseWithCovarianceStamped p;
  p.header = msg->header;
  p.pose = msg->pose;
  pose_pub_.publish(p);
}

void OdomToPose::process(void) { ros::spin(); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_to_pose");
  OdomToPose odom_to_pose;
  odom_to_pose.process();
  return 0;
}
