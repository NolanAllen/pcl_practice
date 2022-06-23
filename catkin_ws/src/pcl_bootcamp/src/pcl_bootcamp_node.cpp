#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

float minFilterLtd, maxFilterLtd;
std::string topic_name, output_name, axis;
int disTh, ptColorTh, regColorTh, clustersize;
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // setup input data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *raw_cloud);

  // downsample data from raw_cloud to transformed_cloud
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(raw_cloud);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*transformed_cloud);

  // color-based segmentation
  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*transformed_cloud, *indices);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(raw_cloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(minFilterLtd, maxFilterLtd);
  pass.filter(*indices);
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud(raw_cloud);
  reg.setIndices(indices);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(disTh);
  reg.setPointColorThreshold(ptColorTh);
  reg.setRegionColorThreshold(regColorTh);
  reg.setMinClusterSize(clustersize);
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);
  transformed_cloud = reg.getColoredCloud();

  // publish output data
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*transformed_cloud, output_msg);
  output_msg.header.frame_id = cloud_msg->header.frame_id;
  output_msg.header.stamp = cloud_msg->header.stamp;
  pub.publish(output_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_tutorial_cloud");
  ros::NodeHandle n;
  n.param<std::string>("topic_name", topic_name, "/camera/depth/points");
  n.param<std::string>("output_name", output_name, "output");
  ros::Subscriber sub =
      n.subscribe<sensor_msgs::PointCloud2>(topic_name, 1, cloud_cb);
  pub = n.advertise<sensor_msgs::PointCloud2>(output_name, 1);
  ros::Rate rate(10);
  while (ros::ok()) {
    n.param<float>("minFilterLtd", minFilterLtd, 0.0);
    n.param<float>("maxFilterLtd", maxFilterLtd, 1.0);
    n.param<std::string>("axis", axis, "z");
    n.param<int>("disTh", disTh, 10);
    n.param<int>("ptColorTh", ptColorTh, 6);
    n.param<int>("regColorTh", regColorTh, 5);
    n.param<int>("clustersize", clustersize, 600);
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();
}
/*
#include <cstring>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
float minFilterLtd, maxFilterLtd;
int disTh, ptColorTh, regColorTh, clustersize;
std::string topic_name, axis;
std::string output_name;
ros::Publisher pub;
void callback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::fromROSMsg<pcl::PointXYZRGB>(*msg, *raw_cloud);
  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(raw_cloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(minFilterLtd, maxFilterLtd);
  pass.filter(*indices);
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud(raw_cloud);
  reg.setIndices(indices);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(disTh);
  reg.setPointColorThreshold(ptColorTh);
  reg.setRegionColorThreshold(regColorTh);
  reg.setMinClusterSize(clustersize);
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);
  transformed_cloud = reg.getColoredCloud();
  // pcl::fromROSMsg(*msg, *raw_cloud);
  // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  //   sor.setInputCloud (raw_cloud);
  //   sor.setLeafSize (0.01f, 0.001f, 0.01f);
  //   sor.filter (*transformed_cloud);
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*transformed_cloud, output_msg);
  output_msg.header.frame_id = msg->header.frame_id;
  output_msg.header.stamp = msg->header.stamp;
  pub.publish(output_msg);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_tutorial_cloud");
  ros::NodeHandle nh("~");
  nh.param<std::string>("topic_name", topic_name, "/camera/depth/points");
  nh.param<std::string>("output_name", output_name, "output");
  ros::Subscriber sub =
      nh.subscribe<sensor_msgs::PointCloud2>(topic_name, 1, callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>(output_name, 1);
  ros::Rate rate(10);
  while (ros::ok()) {
    nh.param<float>("minFilterLtd", minFilterLtd, 0.0);
    nh.param<float>("maxFilterLtd", maxFilterLtd, 0.0);
    nh.param<std::string>("axis", axis, "z");
    nh.param<int>("disTh", disTh, 10);
    nh.param<int>("ptColorTh", ptColorTh, 6);
    nh.param<int>("regColorTh", regColorTh, 5);
    nh.param<int>("clustersize", clustersize, 600);
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();
}*/