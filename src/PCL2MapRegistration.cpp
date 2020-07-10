#include "PCL2MapRegistration.h"

namespace mrs_pcl_tools
{

/* onInit() //{ */
void PCL2MapRegistration::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Get parameters from config file
  mrs_lib::ParamLoader param_loader(nh, "PCL2MapRegistration");

  std::string path_map;
  std::string path_pcl;
  std::string path_save_as;
  param_loader.loadParam("map", path_map);
  param_loader.loadParam("pcl", path_pcl);
  param_loader.loadParam("save_as", path_save_as);
  param_loader.loadParam<std::string>("map_frame", _frame_map, "origin");

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCL2MapRegistration]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  _pc_map  = load_pc(path_map);
  _pc_slam = load_pc(path_pcl);

  _pc_map_normals  = boost::make_shared<PC_NORM>();
  _map_has_normals = load_pc_normals(path_map, _pc_map_normals);

  pcl::toROSMsg(*_pc_map, *_pc_map_msg);
  pcl::toROSMsg(*_pc_slam, *_pc_slam_msg);

  _pc_map_msg->header.frame_id  = _frame_map;
  _pc_slam_msg->header.frame_id = _frame_map;

  _pub_cloud_source  = nh.advertise<sensor_msgs::PointCloud2>("cloud_source_out", 1);
  _pub_cloud_target  = nh.advertise<sensor_msgs::PointCloud2>("cloud_target_out", 1);
  _pub_cloud_aligned = nh.advertise<sensor_msgs::PointCloud2>("cloud_aligned_out", 1);

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh));
  ReconfigureServer::CallbackType f = boost::bind(&PCL2MapRegistration::callbackReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  NODELET_INFO_ONCE("[PCL2MapRegistration] Nodelet initialized");

  _timer_registration = nh.createTimer(ros::Duration(_registration_period), &PCL2MapRegistration::callbackRegistration, this, false, true);

  is_initialized = true;
}
//}

/* callbackRegistration() //{*/
void PCL2MapRegistration::callbackRegistration([[maybe_unused]] const ros::TimerEvent &event) {
  if (!is_initialized) {
    return;
  }

  PC::Ptr pc_aligned = boost::make_shared<PC>();

  {
    std::scoped_lock lock(_mutex_registration);

    // Set initial alignment estimate found using robot odometry.
    auto [converged, score, T, aligned] = pcl2map_ndt(_pc_slam, _pc_map, _initial_guess);

    if (converged) {

      pc_aligned = aligned;

      // Transforming unfiltered, input cloud using found transform.
      /* pcl::transformPointCloud(*_pc_slam, *pc_registered, T); */

      // Setup clouds header
      ros::Time now              = ros::Time::now();
      _pc_map_msg->header.stamp  = now;
      _pc_slam_msg->header.stamp = now;
      pcl_conversions::toPCL(now, pc_aligned->header.stamp);
      pc_aligned->header.frame_id = _frame_map;

      ROS_INFO("[PCL2MapRegistration] NDT converged with score: %0.2f", score);
      std::cout << "T: " << T << std::endl;

    } else {
      ROS_ERROR("[PCL2MapRegistration] NDT did not converge with current parameters.");
    }
  }

  publishCloudMsg(_pub_cloud_source, _pc_slam_msg);
  publishCloudMsg(_pub_cloud_target, _pc_map_msg);
  publishCloud(_pub_cloud_aligned, pc_aligned);
}
/*//}*/

/* callbackReconfigure() //{ */
void PCL2MapRegistration::callbackReconfigure(Config &config, [[maybe_unused]] uint32_t level) {
  if (!is_initialized) {
    return;
  }
  NODELET_INFO("[PCL2MapRegistration] Reconfigure callback.");

  std::scoped_lock lock(_mutex_registration);

  _timer_registration.setPeriod(ros::Duration(config.reg_period));

  _ndt_transformation_epsilon = config.ndt_transformation_epsilon;
  _ndt_step_size              = config.ndt_step_size;
  _ndt_resolution             = config.ndt_resolution;
  _ndt_maximum_iterations     = config.ndt_maximum_iterations;

  Eigen::AngleAxisf    init_rotation(config.init_guess_yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0, 0, 0);
  _initial_guess = (init_translation * init_rotation).matrix();
}
//}

/* pcl2map_ndt() //{*/
std::tuple<bool, float, Eigen::Matrix4f, PC::Ptr> PCL2MapRegistration::pcl2map_ndt(PC::Ptr pc, PC::Ptr pc_map, Eigen::Matrix4f init_guess) {

  // Filtering input scan to roughly to increase speed of registration.
  PC::Ptr pc_filtered = boost::make_shared<PC>();
  PC::Ptr pc_aligned  = boost::make_shared<PC>();

  pcl::ApproximateVoxelGrid<pt_XYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
  approximate_voxel_filter.setInputCloud(pc);
  approximate_voxel_filter.filter(*pc_filtered);

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pt_XYZ, pt_XYZ> ndt;
  ndt.setInputSource(pc_filtered);
  ndt.setInputTarget(pc_map);

  // Setting minimum transformation difference for termination condition.
  // Setting maximum step size for More-Thuente line search.
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  // Setting max number of registration iterations.
  ndt.setTransformationEpsilon(_ndt_transformation_epsilon);
  ndt.setStepSize(_ndt_step_size);
  ndt.setResolution(_ndt_resolution);
  ndt.setMaximumIterations(_ndt_maximum_iterations);

  // Calculating required rigid transform to align the input cloud to the target cloud.
  ndt.align(*pc_aligned);
  /* ndt.align(*pc_aligned, init_guess); */

  return std::make_tuple(ndt.hasConverged(), ndt.getFitnessScore(), ndt.getFinalTransformation(), pc_aligned);
}
/*//}*/

/*//{ load_pc() */
PC::Ptr PCL2MapRegistration::load_pc(const std::string &path) {

  PC::Ptr pc = boost::make_shared<PC>();

  ROS_INFO("[PCL2MapRegistration] Reading pointcloud from path %s", path.c_str());
  if (pcl::io::loadPCDFile<pt_XYZ>(path, *pc) < 0) {
    ROS_ERROR("[PCL2MapRegistration] Couldn't read PCD file from path: %s.", path.c_str());
    ros::shutdown();
  } else {
    ROS_INFO("[PCL2MapRegistration] Loaded XYZ pcl with %ld points.", pc->points.size());
  }

  return pc;
}
/*//}*/

/*//{ load_pc_normals() */
bool PCL2MapRegistration::load_pc_normals(const std::string &path, PC_NORM::Ptr &cloud) {
  ROS_INFO("[PCL2MapRegistration] Reading normals of pointcloud from path %s", path.c_str());

  // Read header of PCD file
  pcl::PCDReader      reader_pcd;
  pcl::PCLPointCloud2 pc_fields;
  if (reader_pcd.readHeader(path, pc_fields) < 0) {
    ROS_ERROR("[PCL2MapRegistration] Couldn't read header of PCD file: %s.", path.c_str());
    return false;
  }

  // Check header for normals (normal_x, normal_y, normal_z, curvature)
  unsigned int normal_fields   = 0;
  bool         curvature_field = false;
  for (auto field : pc_fields.fields) {
    if (field.name.rfind("normal", 0) == 0) {
      normal_fields++;
    } else if (field.name == "curvature") {
      curvature_field = true;
    }
  }
  if (normal_fields != 3 || !curvature_field) {
    ROS_WARN("[PCL2MapRegistration] No normals in PCD file at path: %s.", path.c_str());
    return false;
  }

  // Load PCD file
  if (reader_pcd.read(path, *cloud) < 0) {
    ROS_ERROR("[PCL2MapRegistration] Couldn't read normals of PCD file: %s.", path.c_str());
    return false;
  }

  ROS_INFO("[PCL2MapRegistration] Loaded PCL normals with %ld points.", cloud->points.size());
  return true;
}
/*//}*/

/*//{ publishCloud() */
void PCL2MapRegistration::publishCloud(const ros::Publisher pub, PC::Ptr cloud) {
  if (pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud, *cloud_msg);
    publishCloudMsg(pub, cloud_msg);
  }
}
/*//}*/

/*//{ publishCloudMsg() */
void PCL2MapRegistration::publishCloudMsg(const ros::Publisher pub, sensor_msgs::PointCloud2::Ptr cloud_msg) {
  try {
    pub.publish(cloud_msg);
  }
  catch (...) {
    NODELET_ERROR("[PCL2MapRegistration]: Exception caught during publishing on topic: %s", pub.getTopic().c_str());
  }
}
/*//}*/


//
}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCL2MapRegistration, nodelet::Nodelet);
