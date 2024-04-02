/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW3_CLASS_H_
#define CW3_CLASS_H_

// system includes
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <geometry_msgs/PoseStamped.h>pcl_cloud_
#include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
// #include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/PointCloud2.h>
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

#include <mutex>
#include <map>

// include services from the spawner package - we will be responding to these
#include "cw3_world_spawner/Task1Service.h"
#include "cw3_world_spawner/Task2Service.h"
#include "cw3_world_spawner/Task3Service.h"

// // include any services created in this package
// #include "cw3_team_x/example.h"

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

  // helper functions for tasks 1, 2, and 3
  void
  point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

  void
  octomap_callback(const sensor_msgs::PointCloud2ConstPtr& msg);////

  void 
  applyFilterTask3(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud_ptr,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_cloud_ptr);////

  bool 
  solve_task1(geometry_msgs::Point object_point, 
    geometry_msgs::Point target_point, std::string shape_type, double obj_width);

  int64_t
  solve_task2(std::vector<geometry_msgs::PointStamped> ref_points, 
    geometry_msgs::PointStamped mystery_point);

  void
  solve_task3();

  bool
  moveArm(geometry_msgs::Pose target_pose);

  bool
  moveGripper(float width);

  bool
  pickAndPlace(geometry_msgs::Point pointCube, geometry_msgs::Point pointBask, double angle_offset);

  geometry_msgs::Pose
  setPose(float x, float y, float z);

  double
  task1_getOrient(std::string shape_type);

  std::string
  task2_checkShape(geometry_msgs::Point point);

  void
  scanEnvironment();

  std::vector<pcl::PointXYZ> 
  clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input);

  geometry_msgs::Point 
  findMostCommonPoint(const std::map<std::tuple<float, float, float>, std::string>& point_shape_map);

  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;
  ros::Subscriber sub_pointCloud;

  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pcl_cloud_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr octomap_;
  pcl::PassThrough<pcl::PointXYZ> octomap_pointCloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr octomap_filtered_;
  sensor_msgs::PointCloud2 octomap_filtered_msg_;
  ros::Publisher pub_octomap_filtered;
  ros::Subscriber sub_octomap; 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_filtered_octo_;
  ros::Publisher pub_octomap; 
  pcl::PassThrough<pcl::PointXYZRGB> g_pt_; 
  sensor_msgs::PointCloud2 pcl_cloud_filtered_octo_msg_; 
  pcl::PCLPointCloud2 cloud_octomap_;


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_crop_filtered_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pass_filtered_;

  pcl::CropBox<pcl::PointXYZRGB> crop_filter_;
  pcl::PassThrough<pcl::PointXYZRGB> pass_filter_;

  geometry_msgs::Point basket_point_;


  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  double obj_width_ = 0.04;
  double pi_ = 3.14159;
  double z_offset_ = 0.125;
  double angle_offset_ = 3.14159 / 4.0;

  double camera_offset_ = 0.04;
  double check_height_ = 0.5;

  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.03;
  double gripper_offset_ = 0.145;
  double approach_distance_ = 0.2;

  bool activate_octomap_ = false;

  std::mutex cloud_mutex_;

};

#endif // end of include guard for cw3_CLASS_H_
