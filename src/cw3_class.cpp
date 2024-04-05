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

  // subscribe related topics to get the information
  sub_pointCloud = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw3::point_cloud_callback, this);
  sub_octomap = nh_.subscribe("/octomap_point_cloud_centers", 1, &cw3::octomap_callback, this);
  
  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw3::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw3::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw3::t3_callback, this);

  // Initialise a client for octomap reset
  client = nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");

  // Initialise pcl pointers
  pcl_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl_crop_filtered_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_pass_filtered_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_cloud_filtered_octo_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  octomap_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  octomap_filtered_.reset(new pcl::PointCloud<pcl::PointXYZ>); 

  // Initialise octomap publishers
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

  // Return the task 3 response
  ROS_INFO("================Task 3 Return Message================");
  ROS_INFO("totoal_num_shapes = %ld", total_num_shapes_);
  ROS_INFO("num_most_common_shape = %ld", num_most_common_shape_);

  response.total_num_shapes = total_num_shapes_;
  response.num_most_common_shape = num_most_common_shape_;

  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
cw3::point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  std::lock_guard<std::mutex> guard(cloud_mutex_); // lock the mutex for pcl_cloud_
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

bool
cw3::solve_task1(geometry_msgs::Point object_point,
  geometry_msgs::Point target_point, std::string shape_type)
{
  /* core function to solve task 1 */

  // print the position of the object
  ROS_INFO("====== object_point: [%f, %f, %f] ======", object_point.x, object_point.y, object_point.z);
  
  // move above the object
  geometry_msgs::Pose object_pose = setPose(object_point.x - camera_offset_, object_point.y, 0.6);
  bool success = moveArm(object_pose);

  // determine the orientation of the object
  double obj_theta = task1_getOrient(shape_type);
  ROS_INFO("====== The object orient as (%f) degree ======", obj_theta* (180/pi_));

  // get the width of the object
  if (activate_octomap_)
  {
    // in task3, calculate the width
    calculateWidth(shape_type, obj_theta);
  }
  else
  {
    // in task1, get fixed width
    obj_width_ = 0.04;
  } 

  // determine the grasp position and place position
  geometry_msgs::Point pick_obj_point = object_point;
  geometry_msgs::Point plac_tar_point = target_point;

  // add offset according the orientation
  if (shape_type == "nought") 
  {
    pick_obj_point.x += 2 * obj_width_ * sin(obj_theta);
    pick_obj_point.y += 2 * obj_width_ * cos(obj_theta);
    plac_tar_point.y += 2 * obj_width_;
    ROS_INFO("Get nought object");
  } 
  else if (shape_type == "cross") 
  {
    pick_obj_point.x += 1.2 * obj_width_ * cos(obj_theta);
    pick_obj_point.y -= 1.2 * obj_width_ * sin(obj_theta);
    plac_tar_point.x += 1.2 * obj_width_;
    ROS_INFO("Get cross object");
  } 
  else 
  {
    ROS_INFO("Invalid Type for object type, check your input!");
  }

  // Arm pick obj then place to basket
  ROS_INFO("====== object_point: [%f, %f, %f] ======", pick_obj_point.x, pick_obj_point.y, pick_obj_point.z);
  success = success * pickAndPlace(pick_obj_point, plac_tar_point, obj_theta);
  return true;
}

///////////////////////////////////////////////////////////////////////////////

int64_t
cw3::solve_task2(std::vector<geometry_msgs::PointStamped> ref_points, 
  geometry_msgs::PointStamped mystery_point)
{
  /* core function to solve task 2 */

  ROS_INFO("====================");
  ROS_INFO("--- task 2 start ---");
  ROS_INFO("====================");

  // move the gripper to the starting pose
  geometry_msgs::Pose init_pose = setPose(check_height_, 0, check_height_);
  bool success = moveArm(init_pose);

  // check the shape of the object on the first reference point
  std::string res_ref = checkShape(ref_points[0].point);
  ROS_INFO("====== The shape on the first ref point is: %s ======", res_ref.c_str());

  // check the shape of the target object
  std::string res_mystery = checkShape(mystery_point.point);
  ROS_INFO("====== The shape of the target object is: %s ======", res_mystery.c_str());

  // task finished, move the gripper to the original starting pose
  success = success & moveArm(init_pose);

  // judge if the target object is the same as ref 1 or ref 2
  int64_t res;
  if (res_ref == res_mystery) 
  {
    ROS_INFO("====== The shape of mystery is the same as ref 1, which is %s ======", res_mystery.c_str());
    res = 1;
  } 
  else 
  {
    ROS_INFO("====== The shape of mystery is the same as ref 2, which is %s ======", res_mystery.c_str());
    res = 2;
  }

  ROS_INFO("====================");
  ROS_INFO("---- task 2 end ----");
  ROS_INFO("====================");
  return res;
}

///////////////////////////////////////////////////////////////////////////////

void
cw3::solve_task3()
{
  /* core function to solve task 3 */

  // clear pointers  
  pcl_cloud_filtered_octo_->clear();
  octomap_->clear();
  octomap_filtered_->clear();
  centroids_vector_.clear();

  // clear point clouds in the octomap server
  if (client.call(srv)) 
  {
    ROS_INFO("======= Octomap service cleared successfully ======");
  } 
  else 
  {
    ROS_INFO("======= Octomap service cleared failed ======");
  }

  // add collision object to avoid the gripper moving below 0.22m
  // determine the point, vector3 and quaternion and pass the parameters to addCollisionObject function
  geometry_msgs::Point centre; centre.x = 0.3; centre.y = 0.0; centre.z = 0.11;
  geometry_msgs::Vector3 dimensions; dimensions.x = 0.3; dimensions.y = 1.2; dimensions.z = 0.22;
  geometry_msgs::Quaternion ori; ori.x = 0.0; ori.y = 0.0; ori.z = 0.0; ori.w = 1.0;
  addCollisionObject(std::string("ground1"), centre, dimensions, ori);

  centre.x = -0.3; centre.y = 0.0; centre.z = 0.11; dimensions.x = 0.3; dimensions.y = 1.2; dimensions.z = 0.22;
  addCollisionObject(std::string("ground2"), centre, dimensions, ori);
  centre.x = 0.0; centre.y = 0.3; centre.z = 0.11; dimensions.x = 1.2; dimensions.y = 0.3; dimensions.z = 0.22;
  addCollisionObject(std::string("ground3"), centre, dimensions, ori);
  centre.x = 0.0; centre.y = -0.3; centre.z = 0.11; dimensions.x = 1.2; dimensions.y = 0.3; dimensions.z = 0.22;
  addCollisionObject(std::string("ground4"), centre, dimensions, ori);
  ROS_INFO("====== collision object added ======");

  arm_group_.setMaxVelocityScalingFactor(0.3); // adjust the velocity when scanning

  // scan the environment
  ROS_INFO("------------------------------------------------------");
  ROS_INFO("Start scanning the plane and construct octomap point cloud");
  ROS_INFO("------------------------------------------------------");
  scanGround();
  ROS_INFO("==================== Scanning Finished ===================");

  // create a pointer to store the octomap point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_octomap(new pcl::PointCloud<pcl::PointXYZ>);
  current_octomap->clear();
  *current_octomap = *octomap_filtered_;

  // create a vector to store the centroids of each cluster
  // apply clustering
  ROS_INFO("==================== Clustering Result ===================");
  clustering(current_octomap);

  arm_group_.setMaxVelocityScalingFactor(0.1); // set back the velocity

  // sort the candidate points in the vector
  std::sort(centroids_vector_.begin(), centroids_vector_.end(),
    [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) -> bool {
        // If both points have y > 0, the one with smaller x comes first
        if (a.y > 0 && b.y > 0) return a.x < b.x;
        // If both points have y < 0, the one with larger x comes first
        if (a.y < 0 && b.y < 0) return a.x > b.x;
        // Points with y > 0 come before points with y < 0
        return a.y > b.y;
    });

  for (int i = 0; i < centroids_vector_.size(); ++ i)
  {
    ROS_INFO("point %d: %lf, %lf, %lf", i, centroids_vector_[i].x, centroids_vector_[i].y, centroids_vector_[i].z);
  }

  // create a map whose key is the centroid coordinate and values is the shape
  std::map<std::tuple<float, float, float>, std::string> point_shape_map;

  ROS_INFO("=============== Checking Shape for Each Clusters ==============");

  for(const auto& point_pcl : centroids_vector_) 
  { 
    ROS_INFO("This is centroid: (%f, %f, %f)", point_pcl.x, point_pcl.y, point_pcl.z);
    geometry_msgs::Point point_geo;
    point_geo.x = point_pcl.x;
    point_geo.y = point_pcl.y;
    point_geo.z = point_pcl.z;
    std::string shape = checkShape(point_geo);
    std::tuple<float, float, float> point_tuple = std::make_tuple(point_pcl.x, point_pcl.y, point_pcl.z);
    if (shape != "obstacle")
    {
      // add the point and shape info to the map if not obstacles
      point_shape_map[point_tuple] = shape;
      ROS_INFO("Shape: %s", shape.c_str());
    }
    else
    {
      ROS_INFO("Obstacle detected, shape not added to map.");
    }
  }

  for (const auto& item : point_shape_map) 
  {
    const auto& key = item.first; // The key is a tuple representing the point
    const std::string& value = item.second; // The value is a string representing the shape

    float x, y, z;
    std::tie(x, y, z) = key;

    // Print out the coordiate and the shape
    ROS_INFO("coordinate: (%f, %f, %f) -> shape: %s", x, y, z, value.c_str());
  }
  
  // remove collosion object
  removeCollisionObject(std::string("ground1"));
  removeCollisionObject(std::string("ground2"));
  removeCollisionObject(std::string("ground3"));
  removeCollisionObject(std::string("ground4"));
  ROS_INFO("====== collision object removed =======");

  // find the shape that occurs the most and store its position
  geometry_msgs::Point most_common_point = findMostCommonPoint(point_shape_map);
  
  // convert Point to Tuple
  std::tuple<float, float, float> most_common_tuple = std::make_tuple(most_common_point.x, most_common_point.y, most_common_point.z);
  tuple_key_ = most_common_tuple;

  // extract the most common shape from the map
  std::string most_common_shape = point_shape_map[most_common_tuple];

  ROS_INFO("========== most common shape: %s, most common point: %f, %f, %f ============", most_common_shape.c_str(), std::get<0>(most_common_tuple), std::get<1>(most_common_tuple), std::get<2>(most_common_tuple));

  // create a point for grasp
  geometry_msgs::Point most_common_point_grasp;
  most_common_point_grasp = most_common_point;
  most_common_point_grasp.z = most_common_point.z - obj_height_ - 0.01; // apppy offset in z

  ROS_INFO("=============== Start picking and placing the Most Common Shape ==============");
  solve_task1(most_common_point_grasp, basket_point_, most_common_shape);

  // reset
  activate_octomap_ = false;
}

///////////////////////////////////////////////////////////////////////////////

double
cw3::task1_getOrient(std::string shape_type) 
{
  /* calculate the orientation of a cross or nought */
  
  // initial the points cloud pointer
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_points(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_points(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // Get the current camera points cloud
  std::lock_guard<std::mutex> guard(cloud_mutex_); // lock the mutex for pcl_cloud_
  *current_points = *pcl_cloud_;

  // filter the points cloud according the type
  if (shape_type == "nought") 
  {
    crop_filter_.setInputCloud(current_points);
    crop_filter_.setMin(Eigen::Vector4f(-0.13, 0.05, 0.46, 1.0));
    crop_filter_.setMax(Eigen::Vector4f(0.13, 0.4, 0.51, 1.0));
    crop_filter_.filter(*pcl_crop_filtered_);
    *obj_points = *pcl_crop_filtered_;
    ROS_INFO("Get nought object, apply crop filter");
  } 
  else if (shape_type == "cross") 
  {
    pass_filter_.setInputCloud(current_points);
    pass_filter_.setFilterFieldName("z");
    pass_filter_.setFilterLimits(0.46, 0.51);
    pass_filter_.filter(*pcl_pass_filtered_);
    *obj_points = *pcl_pass_filtered_;
    ROS_INFO("Get cross object, apply pass filter");
  } 
  else 
  {
    ROS_INFO("Invalid Type for object type, check your input!");
  } 

  // apply PCA to get orientation
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(obj_points);

  // Get the orientation of the first principal component
  Eigen::Vector3f eigenVectors = pca.getEigenVectors().col(0);
  // get y-component of eigen
  double dot = eigenVectors(1); 
  // get x-component of eigen
  double det = eigenVectors(0); 

  // Compute angle between the major axis and the global x-axis
  double obj_orient = atan2(dot, det) ; 
  ROS_INFO("== PCA result == The object orient as (%f) degree", obj_orient* (180/pi_));
  
  // Normalize the orientation to be within -PI/4 to PI/4
  if (obj_orient > pi_/2) 
  {
    obj_orient += - pi_;
  } 
  else if (obj_orient < - pi_/2) 
  {
    obj_orient += pi_;
  }

  if (obj_orient > pi_/4) 
  {
    obj_orient += - pi_/2;
  } 
  else if (obj_orient < - pi_/4)
  {
    obj_orient += pi_/2;
  }

  // return the degree result
  ROS_INFO("==normal result== The object orient as (%f) degree", obj_orient* (180/pi_));  
  return obj_orient;
}

////////////////////////////////////////////////////////////////////

void
cw3::calculateWidth(std::string shape_type, double obj_orient)
{
  /* calculate the width of a cross or nought */

  // initial the object points cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr obj_points(new pcl::PointCloud<pcl::PointXYZ>);
  obj_points = centroid_cluster_map_[tuple_key_];

  // define the coordinate max value and min value
  double minX = std::numeric_limits<double>::max();
  double maxX = -std::numeric_limits<double>::max();
  double minY = std::numeric_limits<double>::max();
  double maxY = -std::numeric_limits<double>::max();

  // travel all the points to find the min and max
  for (const auto& point : *obj_points) 
  {
    if (point.x < minX) minX = point.x;
    if (point.x > maxX) maxX = point.x;
    if (point.y < minY) minY = point.y;
    if (point.y > maxY) maxY = point.y;
  }
  ROS_INFO("== edge == Min X: %f, Max X: %f, Min Y: %f, Max Y: %f", minX, maxX, minY, maxY);

  // calculate the length in the x-axis and y-axis
  double length_X = maxX - minX;
  double length_Y = maxY - minY;
  ROS_INFO("== length == L_X: %f, L_Y: %f", length_X, length_Y);
  
  // use geometric method to find the width
  if (shape_type == "cross") 
  { 
    obj_width_ = 0.5*(length_X+length_Y) / (5*cos(obj_orient)+abs(sin(obj_orient))) ;
    ROS_INFO("== width == The cross width is: %f", obj_width_);
  }
  else if (shape_type == "nought") 
  { 
    obj_width_ = 0.5*(length_X+length_Y) / (5*cos(obj_orient)+5*abs(sin(obj_orient))) ;
    ROS_INFO("== width == The nought width is: %f", obj_width_);
  }
}

////////////////////////////////////////////////////////////////////

std::string
cw3::checkShape(geometry_msgs::Point point) 
{
  /* check the type of the object: cross, nought, basket or obstacle*/

  geometry_msgs::Pose pose = setPose(point.x - camera_offset_, point.y, check_height_);
  bool success = moveArm(pose);

  // extract RGB value of the central data
  // camera size: 480x640
  std::lock_guard<std::mutex> guard(cloud_mutex_); // lock the mutex for pcl_cloud_
  uint32_t height = pcl_cloud_->height;
  uint32_t width = pcl_cloud_->width;
  pcl::PointXYZRGB center_point = pcl_cloud_->points[height * 0.5 * width + width * 0.5];

  uint32_t rgb = *reinterpret_cast<int*>(&center_point.rgb);
  const uint8_t red = (rgb >> 16) & 0x0000ff;
  const uint8_t green = (rgb >> 8) & 0x0000ff;
  const uint8_t blue = (rgb) & 0x0000ff;
  ROS_INFO("====== check shape result: red: %u, green: %u, blue: %u ======", red, green, blue);

  // judge shape
  std::string res;
  if (red < 35 && green < 35 && blue < 35)
  {
    res = "obstacle";
  } 
  else if (red > 100 && red < 180 && green > 40 && green < 100 && blue < 80 && red > green)
  {
    res = "basket";
    basket_point_ = point;
  }
  else if (green > red && green > blue)
  {
    res = "nought";
  }
  else
  {
    res = "cross";
  }
  return res;
}

///////////////////////////////////////////////////////////////////////////////

void
cw3::scanGround()
{
  /* scanning the ground to get the full map point cloud */

  // move to the initial pose
  geometry_msgs::Pose pose;
  pose = setPose(0.5, 0.0, 0.6);
  bool success = moveArm(pose);

  // store the position of 4 corners of the ground in a array
  double positions[5][3] = {{-0.5, 0.4, 0.6}, {-0.5, -0.4, 0.6}, {0.5, -0.4, 0.6}, {0.5, 0.4, 0.6}, {-0.5, 0.4, 0.6}};
  double x, y, z;

  // add some middle points to plan the trajectory
  pose = setPose(positions[2][0], positions[2][1], positions[2][2]);
  success = success & moveArm(pose);
  pose = setPose(positions[1][0], positions[1][1], positions[1][2]);
  success = success & moveArm(pose);

  // starting to scan the entire ground
  for (int i = 0; i < 4; ++ i)
  {
    // move to the next corner
    pose = setPose(positions[i][0], positions[i][1], positions[i][2]);
    success = success & moveArm(pose);

    // activate to scan the ground
    if (i == 0)
    {
      activate_octomap_ = true;
    }
    
    // scan 3 intermediate points between 2 corner points
    for (int j = 1; j <= 3; ++ j)
    {
      x = positions[i][0] + (0.25 * j * (positions[i+1][0] - positions[i][0]));
      y = positions[i][1] + (0.25 * j  * (positions[i+1][1] - positions[i][1]));
      z = positions[i][2];

      pose = setPose(x, y, z);
      success = success & moveArm(pose);
    }
  }
}

////////////////////////////////////////////////////////////////////////

void 
cw3::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input)
{
  /* cluster the full map point cloud to get the location of every object */

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

    if ((std::abs(centroid[0]) > 0.15 || std::abs(centroid[1]) > 0.15) && (std::abs(centroid[0]) < 0.8 && std::abs(centroid[1]) < 0.8)) // get rid of the cluster at the robot base
    {
      // Add the centroid to the centroids vector
      pcl::PointXYZ simple_centroid(centroid[0], centroid[1], centroid[2]);
      centroids_vector_.push_back(simple_centroid);

      // Convert centroid to tuple
      std::tuple<float, float, float> centroid_tuple(centroid[0], centroid[1], centroid[2]);

      // Store centroid and corresponding cluster in the map
      centroid_cluster_map_[centroid_tuple] = cluster_cloud;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

geometry_msgs::Point 
cw3::findMostCommonPoint(const std::map<std::tuple<float, float, float>, std::string>& point_shape_map) 
{
  /* find one object whose type occurs most */

  // Map to store the count of each shape
  std::map<std::string, int> shape_count;
  int total_shape_count = 0; 

  // Iterate over the map and count occurrences of each shape
  for(const auto& pair : point_shape_map) 
  {
    const std::string& shape = pair.second;
    if(shape == "basket") // discard basket
      continue;
    shape_count[shape]++;
    total_shape_count++;
  }

  total_num_shapes_ = total_shape_count;

  // Find the most common shape
  std::string most_common_shape;
  int max_count = 0;

  for(const auto& pair : shape_count) 
  {
    if(pair.second > max_count) 
    {
      max_count = pair.second;
      most_common_shape = pair.first;
    }
  }

  // extract the number of most common shape
  num_most_common_shape_ = max_count;

  // Find the key corresponding to the most common shape
  std::tuple<float, float, float> most_common_tuple;
  geometry_msgs::Point most_common_point;

  for(const auto& pair : point_shape_map) 
  {
    if(pair.second == most_common_shape) 
    {
      most_common_tuple = pair.first;
      // convert tuple to point
      most_common_point.x = std::get<0>(most_common_tuple);
      most_common_point.y = std::get<1>(most_common_tuple);
      most_common_point.z = std::get<2>(most_common_tuple);
      break;
    }
  }
  return most_common_point;
}

///////////////////////////////////////////////////////////////////////////////

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

bool
cw3::pickAndPlace(geometry_msgs::Point pointCube, geometry_msgs::Point pointBask, double rotate_angle)
{
  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose = setPose(pointCube.x, pointCube.y, pointCube.z + gripper_offset_);
  geometry_msgs::Pose place_pose = setPose(pointBask.x, pointBask.y, pointBask.z + gripper_offset_);

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_ + rotate_angle);
  // determine the grasping orientation
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // set the desired orient
  grasp_pose.orientation = grasp_orientation;
  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose = grasp_pose;
  approach_pose = grasp_pose;
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

  // grasp the object
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

  approach_pose.position.z += 0.2;
  success *= moveArm(approach_pose);

  // move over the basket
  approach_pose.position.x = pointBask.x; 
  approach_pose.position.y = pointBask.y;
  success *= moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Moving over the basket failed");
  }

  // place the cube
  place_pose.position.z = approach_pose.position.z - 0.2;
  success *= moveArm(place_pose);
  success *= moveGripper(gripper_open_);
  if (not success)
  {
    ROS_ERROR("Placing the cube failed");
  }
  ROS_INFO("Placing successful");

  // return back to the initial pose
  place_pose = setPose(0.5, 0.0, 0.5);
  success *= moveArm(place_pose);
  ROS_INFO("============ Returning to the Inital Position ============");
  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
cw3::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  // hint: what about collision_object.REMOVE?
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

///////////////////////////////////////////////////////////////////////////////

void
cw3::removeCollisionObject(std::string object_name)
{
  /* remove a collision object from the planning scene */

  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input the name and specify we want it removed
  collision_object.id = object_name;
  collision_object.operation = collision_object.REMOVE;

  // apply this collision object removal to the scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

///////////////////////////////////////////////////////////////////////////////