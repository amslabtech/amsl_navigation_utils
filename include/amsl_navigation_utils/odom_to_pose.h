#ifndef AMSL_NAVIGATION_UTILS_ODOM_TO_POSE_H
#define AMSL_NAVIGATION_UTILS_ODOM_TO_POSE_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

class OdomToPose
{
public:
    OdomToPose(void);
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    void process(void);

private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Subscriber odom_sub_;
};

#endif
