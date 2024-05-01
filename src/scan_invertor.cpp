// Copyright 2024 amsl

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class ScanInvertor
{
public:
  ScanInvertor(void);
  void process();

private:
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan);

  ros::NodeHandle nh_;
  ros::Publisher inverted_scan_pub_;
  ros::Subscriber scan_sub_;
};

ScanInvertor::ScanInvertor()
{
  inverted_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan/inverse", 1);
  scan_sub_ = nh_.subscribe("/scan", 1, &ScanInvertor::scan_callback, this);
  ROS_INFO_STREAM("=== Scan Invertor ===");
}

void ScanInvertor::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  sensor_msgs::LaserScan inverted_scan = *msg;
  for (int i = 0; i < msg->ranges.size(); i++)
    inverted_scan.ranges[i] = msg->ranges[msg->ranges.size() - 1 - i];
  inverted_scan_pub_.publish(inverted_scan);
}

void ScanInvertor::process() { ros::spin(); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_invertor");
  ScanInvertor scan_invertor;
  scan_invertor.process();

  return 0;
}
