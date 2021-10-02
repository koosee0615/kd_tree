#pragma once
#include <vector>

// #include <pcl/octree/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>

namespace kd_tree_node {

class KD_Tree {
public:
  KD_Tree();
  ~KD_Tree();

  // callback function
  void planning_pc_callback(const sensor_msgs::PointCloud2ConstPtr &p_pc_msg);
  void depth_pc_callback(const sensor_msgs::PointCloud2ConstPtr &d_pc_msg);
  void z_depth_pc_callback(const sensor_msgs::PointCloud2ConstPtr &zd_pc_msg);
  void depth_pc_callback2(const sensor_msgs::PointCloud2ConstPtr &d_pc_msg2);

private:
  // node handle
  ros::NodeHandle nh_;
  // Subscriber
  ros::Subscriber sub_planning_pc_;
  // for one depthsensor
  ros::Subscriber sub_depth_pc_;
  // for zed
  ros::Subscriber sub_z_depth_pc_;
  // for two depthsensor
  ros::Subscriber sub_depth_pc2_;
  // Publisher
  ros::Publisher pub_pc_;
  ros::Publisher pub_z_pc_;
  ros::Publisher pub_pc2_;

  // object
  pcl::PointCloud<pcl::PointXYZRGB> p_pc_in_;
  pcl::PointCloud<pcl::PointXYZRGB> z_p_pc_in_;
  pcl::PointCloud<pcl::PointXYZRGB> p_pc_in2_;
  pcl::PointCloud<pcl::PointXYZ> d_pc_in_;
  pcl::PointCloud<pcl::PointXYZ> z_d_pc_in_;
  pcl::PointCloud<pcl::PointXYZ> d_pc_in2_;
  pcl::PointCloud<pcl::PointXYZ> d_pc_out_;
  pcl::PointCloud<pcl::PointXYZ> z_d_pc_out_;
  pcl::PointCloud<pcl::PointXYZ> d_pc_out2_;

  // object for kdtree
  pcl::PointCloud<pcl::PointXY> p_pc_xy_in_;
  pcl::PointCloud<pcl::PointXY> z_p_pc_xy_in_;
  pcl::PointCloud<pcl::PointXY> p_pc_xy_in2_;
  pcl::PointCloud<pcl::PointXY> d_pc_xy_in_;
  pcl::PointCloud<pcl::PointXY> z_d_pc_xy_in_;
  pcl::PointCloud<pcl::PointXY> d_pc_xy_in2_;
};
} // namespace kd_tree_node