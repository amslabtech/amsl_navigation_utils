// Copyright 2023 amsl

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pclomp/ndt_omp.h"

#include <Eigen/Dense>

class MapCloudDownsampler
{
public:
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> CloudType;
  typedef pcl::PointCloud<PointType>::Ptr CloudTypePtr;

  MapCloudDownsampler();
  void map_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void apply_voxel_grid_filter(double leaf_size_, CloudTypePtr &cloud_ptr);
  void process();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber map_sub_;

  ros::Publisher downsampled_map_pub_;

  CloudTypePtr map_cloud_ptr_;
  double leaf_size_;
};

MapCloudDownsampler::MapCloudDownsampler() : private_nh_("~")
{
  map_sub_ = nh_.subscribe(
      "map_cloud", 1, &MapCloudDownsampler::map_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
  downsampled_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_cloud/downsampled", 1, true);

  private_nh_.param<double>("leaf_size", leaf_size_, 0.5);
  map_cloud_ptr_ = CloudTypePtr(new CloudType);
}

void MapCloudDownsampler::map_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  ROS_INFO("received map");
  pcl::fromROSMsg(*msg, *map_cloud_ptr_);
  ROS_INFO_STREAM("size: " << map_cloud_ptr_->points.size());
  apply_voxel_grid_filter(leaf_size_, map_cloud_ptr_);
  ROS_INFO_STREAM("downsampled size: " << map_cloud_ptr_->points.size());
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*map_cloud_ptr_, output);
  downsampled_map_pub_.publish(output);
}

void MapCloudDownsampler::apply_voxel_grid_filter(double leaf_size, CloudTypePtr &cloud_ptr)
{
  pcl::VoxelGrid<PointType> vg;
  vg.setInputCloud(cloud_ptr);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  CloudTypePtr output_cloud_ptr(new CloudType);
  vg.filter(*output_cloud_ptr);
  *cloud_ptr = *output_cloud_ptr;
}

void MapCloudDownsampler::process(void) { ros::spin(); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_cloud_downsampler");
  MapCloudDownsampler mapclouddownsampler;
  mapclouddownsampler.process();
  return 0;
}
