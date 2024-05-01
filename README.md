# amsl_navigation_utils
## Environment
- Ubuntu 20.04
- ROS Noetic

## Requirements
- PCL 1.8
  - Confirmed this package works with PCL 1.10 (libpcl-dev)
- [ndt_omp](https://github.com/koide3/ndt_omp.git)
- amsl_navigation_msgs
  - contained in [amsl_navigation_manager](https://github.com/amslabtech/amsl_navigation_managers.git) package

## Nodes
### footprint_publisher
This node publishes robot footprint.
#### Published Topics
- /footprint (`geometry_msgs/PolygonStamped`)
  - rectangular robot footprint

### initialpose_adjuster
#### Published Topics
- ~\<name>/initialpose/adjusted (`geometry_msgs/PoseWithCovarianceStamped`)
  - adjusted initial pose
#### Subscribed Topics
- ~\<name>/initialpose (`geometry_msgs/PoseWithCovarianceStamped`)
  - initial pose from rviz
- ~\<name>/map_cloud (`sensor_msgs/PointCloud2`)
  - map cloud

### initialpose_publisher
#### Published Topics
- ~\<name>/initialpose (`geometry_msgs/PoseWithCovarianceStamped`)
#### Subscribed Topics
- ~\<name>/checkpoint (`std_msgs/Int32MultiArray`)
- ~\<name>/map_cloud (`sensor_msgs/PointCloud2`)
- ~\<name>/node_edge_map (`amsl_navigation_msgs::NodeEdgeMap`)

### map_cloud_downsampler
#### Published Topics
- ~\<name>/map_cloud/downsampled (`sensor_msgs/PointCloud2`)
#### Subscribed Topics
- ~\<name>/map_cloud (`sensor_msgs/PointCloud2`)

### odom_broadcaster
This node broadcasts odom transform for playing bag.
#### Published Topics
- /tf (`tf2_msgs/TFMessage`)
#### Subscribed Topics
- ~\<name>/odom (`nav_msgs/Odometry`)

### odom_to_pose
This node converts odom into pose.
#### Published Topics
- ~\<name>/pose (`geometry_msgs/PoseWithCovarianceStamped`)
#### Subscribed Topics
- ~\<name>/odom (`nav_msgs/Odometry`)

### scan_invertor
This node inverts the index of scan data.
#### Published Topics
- /scan/inverse (`sensor_msgs/LaserScan`)
#### Subscribed Topics
- /scan (`sensor_msgs/LaserScan`)
