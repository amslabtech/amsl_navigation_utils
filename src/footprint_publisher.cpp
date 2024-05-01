// Copyright 2023 amsl

#include <string>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>

class FootprintPublisher
{
public:
  FootprintPublisher(void);
  void process();

protected:
  void set_corners_of_rectangle();
  void publish_footprint();

  int hz_;
  std::string robot_frame_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher footprint_pub_;
  geometry_msgs::PolygonStamped footprint_;

  // each side length from origin of robot_frame
  // (for Rectangular Robot)
  double front_side_distance_;
  double rear_side_distance_;
  double right_side_distance_;
  double left_side_distance_;
};

FootprintPublisher::FootprintPublisher() : private_nh_("~")
{
  // param
  private_nh_.param<int>("hz", hz_, 1);
  private_nh_.param<std::string>("robot_frame", robot_frame_, {"base_link"});
  private_nh_.param<double>("front_side_distance", front_side_distance_, 0.5);
  private_nh_.param<double>("rear_side_distance", rear_side_distance_, 0.5);
  private_nh_.param<double>("right_side_distance", right_side_distance_, 0.5);
  private_nh_.param<double>("left_side_distance", left_side_distance_, 0.5);

  // publisher
  footprint_pub_ = private_nh_.advertise<geometry_msgs::PolygonStamped>("/footprint", 1);

  // frame id
  footprint_.header.frame_id = robot_frame_;

  set_corners_of_rectangle();

  ROS_INFO_STREAM("=== Footprint Publisher ===");
  ROS_INFO_STREAM("hz: " << hz_);
  ROS_INFO_STREAM("robot_frame: " << robot_frame_);
  ROS_INFO_STREAM("front_side_distance: " << front_side_distance_);
  ROS_INFO_STREAM("rear_side_distance: " << rear_side_distance_);
  ROS_INFO_STREAM("right_side_distance: " << right_side_distance_);
  ROS_INFO_STREAM("left_side_distance: " << left_side_distance_);
}

void FootprintPublisher::set_corners_of_rectangle()
{
  geometry_msgs::Point32 point;

  // right front
  point.x = front_side_distance_;
  point.y = -right_side_distance_;
  footprint_.polygon.points.push_back(point);

  // right rear
  point.x = -rear_side_distance_;
  footprint_.polygon.points.push_back(point);

  // left rear
  point.y = left_side_distance_;
  footprint_.polygon.points.push_back(point);

  // left front
  point.x = front_side_distance_;
  footprint_.polygon.points.push_back(point);
}

void FootprintPublisher::publish_footprint()
{
  footprint_.header.stamp = ros::Time::now();
  footprint_pub_.publish(footprint_);
}

void FootprintPublisher::process()
{
  ros::Rate loop_rate(hz_);

  while (ros::ok())
  {
    publish_footprint();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "footprint_publisher");
  FootprintPublisher footprint_publisher;
  footprint_publisher.process();

  return 0;
}
