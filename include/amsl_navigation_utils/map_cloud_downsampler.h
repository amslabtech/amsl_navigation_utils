#ifndef MAP_CLOUD_DOWNSAMPLER_H
#define MAP_CLOUD_DOWNSAMPLER_H

#include <memory>
#include <thread>

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
        void apply_voxel_grid_filter(double leaf_size_, CloudTypePtr& cloud_ptr);
        void process();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber map_sub_;

        ros::Publisher downsampled_map_pub_;

        CloudTypePtr map_cloud_ptr_;
        double leaf_size_;
};

#endif
