#include "amsl_navigation_utils/odom_to_pose.h"

OdomToPose::OdomToPose(void)
{
    odom_sub_ = nh_.subscribe("odom", 1, &OdomToPose::odom_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
}

void OdomToPose::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStamped p;
    p.header = msg->header;
    p.pose = msg->pose;
    pose_pub_.publish(p);
}

void OdomToPose::process(void)
{
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_pose");
    OdomToPose odom_to_pose;
    odom_to_pose.process();
    return 0;
}
