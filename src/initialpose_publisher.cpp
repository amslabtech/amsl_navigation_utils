#include <ros/ros.h>
#include <ros/duration.h>
#include <tf/tf.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include "amsl_navigation_msgs/NodeEdgeMap.h"

class InitialPosePublisher
{
public:
    InitialPosePublisher(void);
    void process();

private:
    void map_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void node_edge_map_callback(const amsl_navigation_msgs::NodeEdgeMap::ConstPtr& msg);
    void checkpoint_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void publish_init_pose();
    int get_index_from_node_id(int target_id);

    bool map_cloud_received_;
    bool node_edge_map_received_;
    bool checkpoint_received_;
    bool init_pose_published_;
    int hz_;
    int init_node_id_;
    int target_node_id_;
    double init_delay_time_;
    amsl_navigation_msgs::NodeEdgeMap node_edge_map_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Subscriber map_cloud_sub_;
    ros::Subscriber node_edge_map_sub_;
    ros::Subscriber checkpoint_sub_;
    ros::Publisher init_pose_pub_;
};

InitialPosePublisher::InitialPosePublisher(void) : local_nh_("~")
{
    local_nh_.param<int>("hz", hz_, 10);
    local_nh_.param<int>("init_node", init_node_id_, 0);
    local_nh_.param<double>("set_delay", init_delay_time_, 0.5);

    map_cloud_sub_ = nh_.subscribe("map_cloud", 1, &InitialPosePublisher::map_cloud_callback, this);
    node_edge_map_sub_ = nh_.subscribe("node_edge_map", 1, &InitialPosePublisher::node_edge_map_callback, this);
    checkpoint_sub_ = nh_.subscribe("checkpoint", 1, &InitialPosePublisher::checkpoint_callback, this);
    init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    map_cloud_received_ = false;
    node_edge_map_received_ = false;
    checkpoint_received_ = false;
    init_pose_published_ = false;
}

void InitialPosePublisher::map_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!map_cloud_received_)
    {
        map_cloud_received_ = true;
    }
}

void InitialPosePublisher::node_edge_map_callback(const amsl_navigation_msgs::NodeEdgeMap::ConstPtr &msg)
{
    if (!node_edge_map_received_)
    {
        node_edge_map_ = *msg;
        node_edge_map_received_ = true;
    }
}

void InitialPosePublisher::checkpoint_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (!checkpoint_received_)
    {
        const std_msgs::Int32MultiArray checkpoint = *msg;
        const auto init_node_itr = find(checkpoint.data.begin(), checkpoint.data.end(), init_node_id_);
        target_node_id_ = *next(init_node_itr);
        checkpoint_received_ = true;
    }
}

void InitialPosePublisher::publish_init_pose()
{
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    const int init_node_idx = get_index_from_node_id(init_node_id_);
    const int target_node_idx = get_index_from_node_id(target_node_id_);
    const geometry_msgs::Point init_node_pose = node_edge_map_.nodes[init_node_idx].point;
    const geometry_msgs::Point target_node_pose = node_edge_map_.nodes[target_node_idx].point;
    const double init_direction = atan2(target_node_pose.y - init_node_pose.y, target_node_pose.x - init_node_pose.x);
    init_pose.header.frame_id = "map";
    init_pose.header.stamp = ros::Time::now();
    init_pose.pose.pose.position.x = init_node_pose.x;
    init_pose.pose.pose.position.y = init_node_pose.y;
    init_pose.pose.pose.position.z = 0;
    init_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_direction);

    init_pose_pub_.publish(init_pose);
    init_pose_published_ = true;
}

int InitialPosePublisher::get_index_from_node_id(int target_id)
{
    int index = 0;
    for (const auto node : node_edge_map_.nodes)
    {
        if (node.id == target_id)
            return index;

        index++;
    }
    return -1;
}

void InitialPosePublisher::process()
{
    ros::Rate rate(hz_);
    while (ros::ok() && !init_pose_published_)
    {
        if (map_cloud_received_ && node_edge_map_received_ && checkpoint_received_)
        {
            ros::Duration(init_delay_time_).sleep();
            publish_init_pose();
            init_pose_published_ = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initialpose_publisher");
    InitialPosePublisher initialpose_publisher;
    initialpose_publisher.process();
    return 0;
}
