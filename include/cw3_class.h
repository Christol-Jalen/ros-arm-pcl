/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW3_CLASS_H_
#define CW3_CLASS_H_

// system includes
#include <ros/ros.h>

// moveit libraries
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// geometry libraries
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// pcl libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <sensor_msgs/PointCloud2.h>

// standard cpp libraries
#include <mutex>
#include <algorithm>
#include <map>

// octomap helper library
#include <std_srvs/Empty.h>

// include services from the spawner package - we will be responding to these
#include "cw3_world_spawner/Task1Service.h"
#include "cw3_world_spawner/Task2Service.h"
#include "cw3_world_spawner/Task3Service.h"

class cw3
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw3(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw3_world_spawner::Task1Service::Request &request,
    cw3_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw3_world_spawner::Task2Service::Request &request,
    cw3_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw3_world_spawner::Task3Service::Request &request,
    cw3_world_spawner::Task3Service::Response &response);

  // point cloud callback to get point cloud data
  void
  point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

  // octomap callback to get full map data
  void
  octomap_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

  // t1_callback run this function
  bool 
  solve_task1(geometry_msgs::Point object_point,
  geometry_msgs::Point target_point, std::string shape_type);

  // t2_callback run this function
  int64_t
  solve_task2(std::vector<geometry_msgs::PointStamped> ref_points, 
    geometry_msgs::PointStamped mystery_point);

  // t3_callback run this function
  void
  solve_task3();

  // calculate the orientation of a cross or nought
  double
  task1_getOrient(std::string shape_type);

  // calculate the width of a cross or nought
  void
  calculateWidth(std::string shape_type, double obj_orient);

  // check the type of the object: cross, nought, basket or obstacle
  std::string
  checkShape(geometry_msgs::Point point);

  // scanning the ground to get the full map point cloud
  void
  scanGround();

  // cluster the full map point cloud to get the location of every object
  void
  clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input);

  // find one object whose type occurs most
  geometry_msgs::Point 
  findMostCommonPoint(const std::map<std::tuple<float, float, float>, std::string>& point_shape_map);

  // transfer a (x,y,z) point to a Pose type
  geometry_msgs::Pose
  setPose(float x, float y, float z);

  // helper function: move the gripper to the target_pose
  bool
  moveArm(geometry_msgs::Pose target_pose);

  // helper function: open or close the gripper
  bool
  moveGripper(float width);

  // helper function: pick and place the object to a new location
  bool
  pickAndPlace(geometry_msgs::Point pointCube, geometry_msgs::Point pointBask, double angle_offset);

  // helper function: add the collision information
  void
  addCollisionObject(std::string object_name, geometry_msgs::Point centre, 
    geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation);

  // helper function: remove the collision information
  void 
  removeCollisionObject(std::string object_name);

  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  ros::Subscriber sub_pointCloud;
  ros::Subscriber sub_octomap; 

  ros::Publisher pub_octomap; 
  ros::Publisher pub_octomap_filtered;

  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr octomap_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr octomap_filtered_;
  sensor_msgs::PointCloud2 octomap_filtered_msg_;
  sensor_msgs::PointCloud2 pcl_cloud_filtered_octo_msg_; 
  
  pcl::PassThrough<pcl::PointXYZ> octomap_pointCloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_filtered_octo_;
  pcl::PCLPointCloud2 cloud_octomap_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_crop_filtered_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pass_filtered_;

  pcl::CropBox<pcl::PointXYZRGB> crop_filter_;
  pcl::PassThrough<pcl::PointXYZRGB> pass_filter_;

  geometry_msgs::Point basket_point_;

  std::map<std::tuple<float, float, float>, pcl::PointCloud<pcl::PointXYZ>::Ptr> centroid_cluster_map_;
  std::tuple<float, float, float> tuple_key_;

  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  bool activate_octomap_ = false;

  std::vector<pcl::PointXYZ> centroids_vector_;

  int64_t total_num_shapes_;
  int64_t num_most_common_shape_;

  std::mutex cloud_mutex_;

  ros::ServiceClient client;
  std_srvs::Empty srv;

  double obj_width_ = 0.04;
  double obj_height_ = 0.04;
  double pi_ = 3.14159;
  double angle_offset_ = 3.14159 / 4.0;

  double camera_offset_ = 0.04;
  double check_height_ = 0.5;

  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.01;
  double gripper_offset_ = 0.145;
  double approach_distance_ = 0.2;

  std::string base_frame_ = "panda_link0";
};

#endif // end of include guard for cw3_CLASS_H_