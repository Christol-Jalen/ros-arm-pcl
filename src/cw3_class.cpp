/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

#include <cw3_class.h> // change to your team name here!

///////////////////////////////////////////////////////////////////////////////

cw3::cw3(ros::NodeHandle nh)
{
  /* class constructor */

  nh_ = nh;
  sub_pointCloud = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw3::point_cloud_callback, this);
  sub_octomap = nh_.subscribe("/octomap_point_cloud_centers", 1, &cw3::octomap_callback, this);
  

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw3::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw3::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw3::t3_callback, this);

  // pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl_crop_filtered_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_pass_filtered_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_cloud_filtered_octo_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  octomap_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  octomap_filtered_.reset(new pcl::PointCloud<pcl::PointXYZ>); 

  pub_octomap = nh.advertise<sensor_msgs::PointCloud2> ("octomap_cloud", 1, true);
  pub_octomap_filtered = nh.advertise<sensor_msgs::PointCloud2> ("filtered_octomap_cloud", 1, true);

  ROS_INFO("cw3 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t1_callback(cw3_world_spawner::Task1Service::Request &request,
  cw3_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */
  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  bool success = solve_task1(request.object_point.point, 
    request.goal_point.point, request.shape_type);
  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t2_callback(cw3_world_spawner::Task2Service::Request &request,
  cw3_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");
  response.mystery_object_num = solve_task2(request.ref_object_points,
    request.mystery_object_point);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t3_callback(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  solve_task3();

  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
cw3::point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  std::lock_guard<std::mutex> guard(cloud_mutex_);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud_);

  if (activate_octomap_)
  {
    pcl_cloud_ = pcl_cloud_filtered_octo_;
    pcl::toROSMsg(*pcl_cloud_filtered_octo_, pcl_cloud_filtered_octo_msg_);
    pub_octomap.publish (pcl_cloud_filtered_octo_msg_);
  }

}

///////////////////////////////////////////////////////////////////////////////

void
cw3::octomap_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //std::lock_guard<std::mutex> guard(cloud_mutex_);
  pcl::fromROSMsg(*cloud_msg, *octomap_);

  // Create the filtering object
  octomap_pointCloud_.setInputCloud(octomap_);
  octomap_pointCloud_.setFilterFieldName("z");
  octomap_pointCloud_.setFilterLimits(0.03, 1);
  octomap_pointCloud_.filter(*octomap_filtered_);

  // Convert PCL point cloud to ROS message
  pcl::toROSMsg(*octomap_filtered_, octomap_filtered_msg_);
  octomap_filtered_msg_.header = cloud_msg->header;  // Copy the header to maintain the timestamp and frame_id
  pub_octomap_filtered.publish(octomap_filtered_msg_);
}

///////////////////////////////////////////////////////////////////////////////

void
cw3::solve_task3()
{
  scanEnvironment();

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_octomap(new pcl::PointCloud<pcl::PointXYZ>);
  
  // Get the current camera points cloud
  *current_octomap = *octomap_filtered_;

  // Print out the number of points in current_octomap
  ROS_INFO("Number of points in current_octomap: %lu", current_octomap->size());

  std::vector<pcl::PointXYZ> centroids_vector = clustering(current_octomap);
  ROS_INFO("==============================");
  ROS_INFO("The centroids are:");
  ROS_INFO("==============================");
  for(const auto& point : centroids_vector) 
  { 
    ROS_INFO("Centroid: (%f, %f, %f)", point.x, point.y, point.z);
  }

  for(const auto& point : centroids_vector) 
  { 
    geometry_msgs::Pose pose = setPose(point.x - camera_offset_, point.y, point.z+0.5);
    moveArm(pose);
    ROS_INFO("This is centroid: (%f, %f, %f)", point.x, point.y, point.z);
  }

}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::solve_task1(geometry_msgs::Point object_point,
  geometry_msgs::Point target_point, std::string shape_type)
{
  ROS_INFO("object_point: [%f, %f, %f]\n", object_point.x, object_point.y, object_point.z);
  
  geometry_msgs::Pose object_pose = setPose(object_point.x - camera_offset_, object_point.y, 0.5);

  bool success = moveArm(object_pose);

  // determine the orientation of the object
  double obj_theta = task1_getOrient(shape_type);
  ROS_INFO("### The object orient as (%f) degree", obj_theta);

  // determine the grasp position and place position
  geometry_msgs::Point pick_obj_point = object_point;
  geometry_msgs::Point plac_tar_point = target_point;

  // add offset according the orientation
  if (shape_type == "nought") {
    pick_obj_point.x += 2 * obj_width_ * sin(obj_theta);
    pick_obj_point.y += 2 * obj_width_ * cos(obj_theta);
    plac_tar_point.y += 2 * obj_width_;
    ROS_INFO("Get nought object");
  } 
  else if (shape_type == "nought") {
    pick_obj_point.x += obj_width_ * cos(obj_theta);
    pick_obj_point.y += obj_width_ * sin(obj_theta);
    plac_tar_point.x += obj_width_;
    ROS_INFO("Get cross object");
  } 
  else {
    ROS_INFO("Invalid Type for object type, check your input!");
  }

  ROS_INFO("### object_point: [%f, %f, %f]\n", pick_obj_point.x, pick_obj_point.y, pick_obj_point.z);
  // arm pick obj then place to tag
  success = success * pickAndPlace(pick_obj_point, plac_tar_point);
  return true;
}

///////////////////////////////////////////////////////////////////////////////

double
cw3::task1_getOrient(std::string shape_type) 
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_points(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_points(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // Get the current camera points cloud
  *current_points = *pcl_cloud_;

  // filter the points cloud from type
  if (shape_type == "nought") {
    crop_filter_.setInputCloud(current_points);
    crop_filter_.setMin(Eigen::Vector4f(-0.13, 0.05, 0.35, 1.0));
    crop_filter_.setMax(Eigen::Vector4f(0.13, 0.4, 0.41, 1.0));
    crop_filter_.filter(*pcl_crop_filtered_);
    *obj_points = *pcl_crop_filtered_;
    ROS_INFO("Get nought object, apply crop filter");
  } 
  else if (shape_type == "cross") {
    pass_filter_.setInputCloud(current_points);
    pass_filter_.setFilterFieldName("z");
    pass_filter_.setFilterLimits(0.35, 0.41);
    pass_filter_.filter(*pcl_pass_filtered_);
    *obj_points = *pcl_pass_filtered_;
    ROS_INFO("Get cross object, apply pass filter");
  } 
  else {
    ROS_INFO("Invalid Type for object type, check your input!");
  } 
  
  // Compute the centroid of the point cloud
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*obj_points, centroid);

  // apply PCA to get 
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(obj_points);

  // Get the orientation of the first principal component
  Eigen::Vector3f eigenVectors = pca.getEigenVectors().col(0);
  double dot = eigenVectors(1); // y-component
  double det = eigenVectors(0); // x-component

  // Compute angle between the major axis and the global x-axis
  double obj_orient = atan2(dot, det); 

  // Normalize the orientation to be within -PI/4 to PI/4
  if (obj_orient > pi_ / 4) {
      obj_orient -= pi_ / 2;
  } 
  else if (obj_orient < -pi_ / 4) {
      obj_orient += pi_ / 2;
  }

  // return the result
  return obj_orient;
}

////////////////////////////////////////////////////////////////////

bool
cw3::pickAndPlace(geometry_msgs::Point pointCube, geometry_msgs::Point pointBask)
{
  // This function pick a cube from given position and drop it at another given position

  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose = setPose(pointCube.x, pointCube.y, pointCube.z + gripper_offset_);

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose = grasp_pose;
  approach_pose.position.z += approach_distance_;

  /* Now perform the pick */

  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the object
  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
  }

  // open the gripper
  success *= moveGripper(gripper_open_);
  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
  }

  // approach to grasping pose
  ROS_INFO("### grasp pose %f, %f, %f\n", grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);
  success *= moveArm(grasp_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
  }

  // grasp!
  success *= moveGripper(gripper_closed_);
  if (not success) 
  {
    ROS_ERROR("Closing gripper to grasp failed");
  }

  // retreat with object
  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed");
  }

  // move over the basket
  approach_pose.position.x = pointBask.x; // set up position that over the basket
  approach_pose.position.y = pointBask.y;

  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Moving over the basket failed");
  }

  // place the cube
  approach_pose.position.z = pointBask.z + approach_distance_;
  success *= moveArm(approach_pose);
  ROS_INFO("### place pose %f, %f, %f\n", approach_pose.position.x, approach_pose.position.y, approach_pose.position.z);
  success *= moveGripper(gripper_open_);
  if (not success)
  {
    ROS_ERROR("Placing the cube failed");
  }

  ROS_INFO("Placing successful");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

int64_t
cw3::solve_task2(std::vector<geometry_msgs::PointStamped> ref_points, 
  geometry_msgs::PointStamped mystery_point)
{
  ROS_INFO("---------- task 2 start ----------\n");
  geometry_msgs::Pose init_pose = setPose(check_height_ * 0.75, 0, check_height_);

  std::string res_ref = task2_checkShape(ref_points[0].point);
  ROS_INFO("%s\n", res_ref.c_str());

  std::string res_mystery = task2_checkShape(mystery_point.point);
  ROS_INFO("%s %s\n", res_ref.c_str(), res_mystery.c_str());

  bool success = moveArm(init_pose);

  int64_t res;
  if (res_ref == res_mystery) {
    ROS_INFO("shape of mystery is the same as ref 1");
    res = 1;
  } else {
    ROS_INFO("shape of mystery is the same as ref 2");
    res = 2;
  }

  ROS_INFO("---------- task 2 end ----------\n");
  return res;
}

///////////////////////////////////////////////////////////////////////////////

std::string
cw3::task2_checkShape(geometry_msgs::Point point) {
  geometry_msgs::Pose pose = setPose(point.x - camera_offset_, point.y, check_height_);
  bool success = moveArm(pose);

  // extract RGB value of the central data
  // camera size: 480x640
  std::lock_guard<std::mutex> guard(cloud_mutex_);
  uint32_t height = pcl_cloud_->height;
  uint32_t width = pcl_cloud_->width;
  ROS_INFO("height: %u, width: %u\n", height, width);
  pcl::PointXYZRGB center_point = pcl_cloud_->points[height * 0.5 * width + width * 0.5];
  uint32_t rgb = *reinterpret_cast<int*>(&center_point.rgb);
  const uint8_t red = (rgb >> 16) & 0x0000ff;
  const uint8_t green = (rgb >> 8) & 0x0000ff;
  const uint8_t blue = (rgb) & 0x0000ff;

  // judge shape
  std::string res;
  if (green > red && green > blue)
    res = "nought";
  else
    res = "cross";
  return res;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

/////////////////////////////////////////////////////////////////////
bool
cw3::moveGripper(float width)
{
  /* this function moves the gripper fingers to a new position. Joints are:
      - panda_finger_joint1
      - panda_finger_joint2 */

  // safety checks
  if (width > gripper_open_) width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}

/////////////////////////////////////////////////////////////////////

geometry_msgs::Pose
cw3::setPose(float x, float y, float z)
{
  // This function return a gripper pose for 
  // given xyz and oriention straight down to the floor

  // determine the grasping orientation
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // set the desired Pose
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = grasp_orientation;

  return pose;
}


////////////////////////////////////////////////////////////////////////
void
cw3::scanEnvironment()
{
  geometry_msgs::Point reset_point;
  reset_point.x = 0.5;
  reset_point.y = 0.0;
  reset_point.z = 0.5;
  geometry_msgs::Pose reset_pose = setPose(reset_point.x, reset_point.y, reset_point.z);
  bool reset_success = moveArm(reset_pose);

  // set speed of arm
  arm_group_.setMaxVelocityScalingFactor(0.05);

  // Create corners of scan area
  geometry_msgs::Point corner1;
  corner1.x = -0.50;
  corner1.y = -0.40;
  corner1.z = 0.6;
  geometry_msgs::Point corner2;
  corner2.x = 0.50;
  corner2.y = -0.40;
  corner2.z = 0.6;
  geometry_msgs::Point corner3;
  corner3.x = 0.50;
  corner3.y = 0.30;
  corner3.z = 0.6;
  geometry_msgs::Point corner4;
  corner4.x = -0.50;
  corner4.y = 0.30;
  corner4.z = 0.6;

  // Add corners to a vector
  std::vector<geometry_msgs::Point> corners;
  corners.push_back(corner1);
  corners.push_back(corner2);
  corners.push_back(corner3);
  corners.push_back(corner4);
  corners.push_back(corner1);

  // Set constant gripper angle
  double angle = -1.5708;
  int num_steps = 4;

  geometry_msgs::Pose pose;
  geometry_msgs::Point distance;
  geometry_msgs::Point step;
  bool success;

  for (int i = 0; i < corners.size() - 1; i++)
  { 
    // Move arm to corner position
    pose = setPose(corners.at(i).x, corners.at(i).y, corners.at(i).z);
    success = moveArm(pose);

    // Enable filter when arm is in position
    if (i == 0)
      activate_octomap_ = true;

    // Initialise variable to store distance between points
    distance.x = corners.at(i).x - corners.at(i+1).x;
    distance.y = corners.at(i).y - corners.at(i+1).y;
    distance.z = 0;
    
    for (int j = 1; j < num_steps - 1; j++)
    {
      // Calculate step distance
      step.x = corners.at(i).x - (j * distance.x / num_steps);
      step.y = corners.at(i).y - (j * distance.y / num_steps);
      step.z = corners.at(i).z;

      ROS_INFO("Step: (%f, %f, %f)", step.x, step.y, step.z);

      // Move arm to step position
      pose = setPose(step.x, step.y, step.z);
      success = moveArm(pose);

      if (i == 3 && j == 2)
        j++;
    }
  }
  ros::Duration(1.0).sleep();

  // Disable filter when scan is complete
  activate_octomap_ = false;
  // Reset the arm velocity
  arm_group_.setMaxVelocityScalingFactor(0.1);

}

////////////////////////////////////////////////////////////////////////

std::vector<pcl::PointXYZ> 
cw3::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input)
{
    // Vector to store the centroids of clusters
    std::vector<pcl::PointXYZ> centroids_vector;

    // Create a KDTree for searching nearest neighbors
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_input);

    // Cluster indices in extraction
    std::vector<pcl::PointIndices> cluster_indices;

    // Define parameters for clustering
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; // extracting clusters
    ec.setClusterTolerance(0.02); // Adjust as per your point cloud's characteristics
    ec.setMinClusterSize(100);    // Minimum points in a cluster
    ec.setMaxClusterSize(10000);  // Maximum points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_input); // point cloud data to be clustered
    ec.extract(cluster_indices); // store the cluster result

    // Get the number of clusters
    size_t num_clusters = cluster_indices.size();

    // Print or use the number of clusters
    ROS_INFO("Number of clusters found: %lu", num_clusters);

    // Process each cluster to compute centroids
    for (const auto& cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto & idx : cluster.indices) 
        {
            cluster_cloud->push_back((*cloud_input)[idx]);
        }
        cluster_cloud->width = cluster_cloud->size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;

        // Compute the centroid of the cluster
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster_cloud, centroid);

        if (std::abs(centroid[0]) > 0.1 || std::abs(centroid[1]) > 0.1) // get rid of the cluster at the robot base
        {
          // Add the centroid to the centroids vector
          pcl::PointXYZ simple_centroid(centroid[0], centroid[1], centroid[2]);
          centroids_vector.push_back(simple_centroid);
        }
    }

    return centroids_vector;
}