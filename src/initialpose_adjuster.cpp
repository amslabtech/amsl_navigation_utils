// Copyright 2023 amsl

#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

class InitialPoseAdjuster
{
public:
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> CloudType;
  typedef CloudType::Ptr CloudTypePtr;
  InitialPoseAdjuster(void);
  void process();

private:
  void map_cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  double search_ground_height(double center_x, double center_y, double range);

  bool map_cloud_received_;
  double reference_range_;
  CloudTypePtr received_map_ptr_;

  ros::NodeHandle nh_;
  ros::NodeHandle local_nh_;
  ros::Subscriber map_cloud_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Publisher adjusted_pose_pub_;
};

InitialPoseAdjuster::InitialPoseAdjuster(void) : local_nh_("~")
{
  local_nh_.param<double>("reference_range", reference_range_, 5.0);
  map_cloud_sub_ = nh_.subscribe("map_cloud", 1, &InitialPoseAdjuster::map_cloud_callback, this);
  init_pose_sub_ = nh_.subscribe("initialpose", 1, &InitialPoseAdjuster::initialpose_callback, this);
  adjusted_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose/adjusted", 1, true);

  received_map_ptr_ = CloudTypePtr(new CloudType);
  map_cloud_received_ = false;
}

void InitialPoseAdjuster::map_cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (!map_cloud_received_)
  {
    map_cloud_received_ = true;
    pcl::fromROSMsg(*msg, *received_map_ptr_);
  }
}

void InitialPoseAdjuster::initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  if (map_cloud_received_)
  {
    geometry_msgs::PoseWithCovarianceStamped pub_pose_msg = *msg;
    double search_center_x = msg->pose.pose.position.x;
    double search_center_y = msg->pose.pose.position.y;
    pub_pose_msg.pose.pose.position.z = search_ground_height(search_center_x, search_center_y, reference_range_);
    pub_pose_msg.header.stamp = ros::Time::now();
    adjusted_pose_pub_.publish(pub_pose_msg);
  }
}

double InitialPoseAdjuster::search_ground_height(double center_x, double center_y, double range)
{
  double height = 0;
  CloudTypePtr filtered_cloud_ptr(new CloudType);
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud(received_map_ptr_);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(center_x - range, center_x + range);
  pass.filter(*filtered_cloud_ptr);
  pass.setInputCloud(filtered_cloud_ptr);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(center_y - range, center_y + range);
  pass.filter(*filtered_cloud_ptr);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;
  seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
  seg.setEpsAngle(M_PI * 15 / 180);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.30);
  seg.setInputCloud(filtered_cloud_ptr);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(filtered_cloud_ptr);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*filtered_cloud_ptr);

  for (auto const &p : filtered_cloud_ptr->points)
  {
    height += p.z;
  }
  height /= (*filtered_cloud_ptr).size() + 1e-10;

  return height;
}

void InitialPoseAdjuster::process() { ros::spin(); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initialpose_adjuster");
  InitialPoseAdjuster initialpose_adjuster;
  initialpose_adjuster.process();
  return 0;
}
