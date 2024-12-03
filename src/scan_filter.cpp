// Copyright 2024 amsl

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class ScanFilter
{
public:
  ScanFilter(void);
  void process();

private:
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan);

  double start_ignore_angle_;
  double end_ignore_angle_;
  double ignore_distance_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher filtered_scan_pub_;
  ros::Subscriber scan_sub_;
};

ScanFilter::ScanFilter(void) : private_nh_("~")
{
  private_nh_.param<double>("start_ignore_angle", start_ignore_angle_, 0.0);
  private_nh_.param<double>("end_ignore_angle", end_ignore_angle_, 360.0);
  private_nh_.param<double>("ignore_distance", ignore_distance_, 0.1);
  start_ignore_angle_ *= M_PI / 180.0;
  end_ignore_angle_   *= M_PI / 180.0;
  filtered_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan/filtered", 1);
  scan_sub_ = nh_.subscribe("/scan", 1, &ScanFilter::scan_callback, this);

  ROS_INFO_STREAM("=== Scan Filter ===");
  ROS_INFO_STREAM("start_ignore_angle: " << start_ignore_angle_);
  ROS_INFO_STREAM("end_ignore_angle: " << end_ignore_angle_);
  ROS_INFO_STREAM("ignore_distance: " << ignore_distance_);
}

void ScanFilter::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  sensor_msgs::LaserScan filtered_scan = *msg;
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    double angle = msg->angle_min + msg->angle_increment * i + M_PI;

    if (start_ignore_angle_ < end_ignore_angle_)
    {
      if (angle > start_ignore_angle_ && angle < end_ignore_angle_)
      {
        if (msg->ranges[i] < ignore_distance_)
        {
          filtered_scan.ranges[i] = msg->range_max;
          ROS_INFO_STREAM("distance: " << msg->ranges[i]);
          ROS_INFO_STREAM("angle: " << angle*180/M_PI);
        }
      }
    }
    else
    {
      if ((angle > start_ignore_angle_ && angle < msg->angle_max+M_PI) ||
          (angle > msg->angle_min+M_PI && angle < end_ignore_angle_))
      {
        if (msg->ranges[i] < ignore_distance_)
        {
          filtered_scan.ranges[i] = msg->range_max;
          ROS_INFO_STREAM("distance2: " << msg->ranges[i]);
          ROS_INFO_STREAM("angle: " << angle*180/M_PI);
        }
      }
    }
  }

  filtered_scan_pub_.publish(filtered_scan);
}

void ScanFilter::process() { ros::spin(); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_filter");
  ScanFilter scan_filter;
  scan_filter.process();

  return 0;
}
