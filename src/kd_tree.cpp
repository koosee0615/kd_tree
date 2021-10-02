#include <kd_tree/kd_tree.h>

#define node_count 8
#define voxel_size 0.03

namespace kd_tree_node {
KD_Tree::KD_Tree() {
  // subscribe lattice point
  sub_planning_pc_ = nh_.subscribe("/set_pc_for_planning_node/pc_for_planning",
                                   10, &KD_Tree::planning_pc_callback, this);
  // you should subscribe your point cloud topic here
  sub_depth_pc_ = nh_.subscribe("/pc_change_coordinates/converted_PC", 10,
                                &KD_Tree::depth_pc_callback, this);

  sub_depth_pc_ = nh_.subscribe("/pc_change_coordinates/converted_PC", 10,
                                &KD_Tree::depth_pc_callback, this);

  sub_z_depth_pc_ = nh_.subscribe("/pc_change_coordinates/z_converted_PC", 10,
                                  &KD_Tree::z_depth_pc_callback, this);

  sub_depth_pc2_ = nh_.subscribe("/merge_pc_node/merged_pc", 10,
                                 &KD_Tree::depth_pc_callback2, this);
  pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/kd_tree_node/kd_tree_filtered_pc", 10);
  pub_z_pc_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/kd_tree_node/z_kd_tree_filtered_pc", 10);
  pub_pc2_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/kd_tree_node/kd_tree_filtered_merged_pc", 10);
}
KD_Tree::~KD_Tree() {}

void KD_Tree::planning_pc_callback(
    const sensor_msgs::PointCloud2ConstPtr &p_pc_msg) {

  // get XYZRGB object
  pcl::fromROSMsg<pcl::PointXYZRGB>(*p_pc_msg, p_pc_in_);
  pcl::fromROSMsg<pcl::PointXYZRGB>(*p_pc_msg, p_pc_in2_);
  // for kd tree, get xy object
  pcl::fromROSMsg(*p_pc_msg, p_pc_xy_in_);
  pcl::fromROSMsg(*p_pc_msg, p_pc_xy_in2_);
}

void KD_Tree::depth_pc_callback(
    const sensor_msgs::PointCloud2ConstPtr &d_pc_msg) {

  // PC pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr d_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(d_pc_in_));
  pcl::PointCloud<pcl::PointXY>::Ptr d_pc_xy_ptr(
      new pcl::PointCloud<pcl::PointXY>(d_pc_xy_in_));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>(p_pc_in_));

  // Planning pc pointer
  pcl::PointCloud<pcl::PointXY>::Ptr p_pc_xy_ptr(
      new pcl::PointCloud<pcl::PointXY>(p_pc_xy_in_));

  // KD tree object
  pcl::search::KdTree<pcl::PointXY>::Ptr kdtree_ptr(
      new pcl::search::KdTree<pcl::PointXY>);

  // This vector will store the output neighbors.
  std::vector<int> pointIndices;
  // This vector will store their squared distances to the search point.
  std::vector<float> squaredDistances;

  // get XYZ object
  pcl::fromROSMsg(*d_pc_msg, *d_pc_ptr);
  // get XY object for kd tree
  pcl::fromROSMsg(*d_pc_msg, *d_pc_xy_ptr);

  // set xy pc in kd tree ptr
  kdtree_ptr->setInputCloud(d_pc_xy_ptr);

  // Search neighbors at every planning pcs
  for (int i = 0; i < p_pc_ptr->points.size(); i++) {
    // Search (node_count) nodes in (voxel_size) radius from points
    if (kdtree_ptr->radiusSearch(p_pc_xy_ptr->points[i], voxel_size,
                                 pointIndices, squaredDistances,
                                 node_count) == node_count) {
      // If (node count) nodes are detected, set height(z) in planning
      // pc(xyzrgb)
      float sum = 0;
      for (size_t j = 0; j < pointIndices.size(); j++) {
        sum += d_pc_ptr->points[pointIndices[j]].z;
      }
      // get average z of all detected pc in voxel_size
      p_pc_ptr->points[i].z = sum / node_count;
      p_pc_ptr->points[i].r = 0;
      p_pc_ptr->points[i].g = 0;
      p_pc_ptr->points[i].b = 200;
    } else {
      p_pc_ptr->points[i].z = NAN;
    }
  }
  // ROS_INFO("kd_tree filtered");
  pub_pc_.publish(*p_pc_ptr);
}

void KD_Tree::z_depth_pc_callback(
    const sensor_msgs::PointCloud2ConstPtr &zd_pc_msg) {

  // PC pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr z_d_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(z_d_pc_in_));
  pcl::PointCloud<pcl::PointXY>::Ptr z_d_pc_xy_ptr(
      new pcl::PointCloud<pcl::PointXY>(z_d_pc_xy_in_));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>(p_pc_in_));

  // Planning pc pointer
  pcl::PointCloud<pcl::PointXY>::Ptr p_pc_xy_ptr(
      new pcl::PointCloud<pcl::PointXY>(p_pc_xy_in_));

  // KD tree object
  pcl::search::KdTree<pcl::PointXY>::Ptr kdtree_ptr(
      new pcl::search::KdTree<pcl::PointXY>);
  // This vector will store the output neighbors.
  std::vector<int> pointIndices;
  // This vector will store their squared distances to the search point.
  std::vector<float> squaredDistances;

  pcl::fromROSMsg(*zd_pc_msg, *z_d_pc_ptr);
  pcl::fromROSMsg(*zd_pc_msg, *z_d_pc_xy_ptr);

  // set xy pc in kd tree ptr
  kdtree_ptr->setInputCloud(z_d_pc_xy_ptr);

  // Search neighbors at every planning pcs
  for (int i = 0; i < p_pc_ptr->points.size(); i++) {
    // Search (node_count) nodes in (voxel_size) radius from points
    if (kdtree_ptr->radiusSearch(p_pc_xy_ptr->points[i], voxel_size,
                                 pointIndices, squaredDistances,
                                 node_count) == node_count) {
      // If node count nodes are detected, set height(z) inplanningpc(xyzrgb)
      float sum = 0;
      for (size_t j = 0; j < pointIndices.size(); j++) {
        sum += z_d_pc_ptr->points[pointIndices[j]].z;
      }

      p_pc_ptr->points[i].z = sum / node_count;
      p_pc_ptr->points[i].r = 0;
      p_pc_ptr->points[i].g = 0;
      p_pc_ptr->points[i].b = 200;
    } else {
      p_pc_ptr->points[i].z = NAN;
    }
  }
  // ROS_INFO("kd_tree filtered");
  pub_z_pc_.publish(*p_pc_ptr);
}

void KD_Tree::depth_pc_callback2(
    const sensor_msgs::PointCloud2ConstPtr &d_pc_msg2) {
  // PC pointer
  // ROS_INFO("start convert");

  pcl::PointCloud<pcl::PointXYZ>::Ptr d_pc_ptr2(
      new pcl::PointCloud<pcl::PointXYZ>(d_pc_in2_));
  pcl::PointCloud<pcl::PointXY>::Ptr d_pc_xy_ptr2(
      new pcl::PointCloud<pcl::PointXY>(d_pc_xy_in2_));

  // Planning pc pointer
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pc_ptr2(
      new pcl::PointCloud<pcl::PointXYZRGB>(p_pc_in2_));
  pcl::PointCloud<pcl::PointXY>::Ptr p_pc_xy_ptr2(
      new pcl::PointCloud<pcl::PointXY>(p_pc_xy_in2_));

  // KD tree object
  pcl::search::KdTree<pcl::PointXY>::Ptr kdtree_ptr2(
      new pcl::search::KdTree<pcl::PointXY>);

  // This vector will store the output neighbors.
  std::vector<int> pointIndices2;
  // This vector will store their squared distances to the search point.
  std::vector<float> squaredDistances2;

  pcl::fromROSMsg(*d_pc_msg2, *d_pc_ptr2);
  pcl::fromROSMsg(*d_pc_msg2, *d_pc_xy_ptr2);

  // set xy pc in kd tree ptr

  kdtree_ptr2->setInputCloud(d_pc_xy_ptr2);

  // Search neighbors at every planning pcs
  for (size_t i = 0; i < p_pc_ptr2->points.size(); i++) {
    // Search (node_count) nodes in (voxel_size) radius from points
    if (kdtree_ptr2->radiusSearch(p_pc_xy_ptr2->points[i], voxel_size,
                                  pointIndices2, squaredDistances2,
                                  node_count) == node_count) {
      // If node count nodes are detected, set height(z) in
      // planningpc(xyzrgb)
      float sum = 0;
      for (size_t j = 0; j < pointIndices2.size(); j++) {
        sum += d_pc_ptr2->points[pointIndices2[j]].z;
      }

      p_pc_ptr2->points[i].z = sum / node_count;
    } else {
      p_pc_ptr2->points[i].z = NAN;
    }
  }
  pub_pc2_.publish(*p_pc_ptr2);
}
} // namespace kd_tree_node
