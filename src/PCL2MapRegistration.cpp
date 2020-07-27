#include "PCL2MapRegistration.h"

namespace mrs_pcl_tools
{

/* onInit() //{ */
void PCL2MapRegistration::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Get parameters from config file
  mrs_lib::ParamLoader param_loader(nh, "PCL2MapRegistration");

  std::string path_save_as;
  param_loader.loadParam("map", _path_map);
  param_loader.loadParam("pcl", _path_pcl);
  param_loader.loadParam("save_as", path_save_as);
  param_loader.loadParam<std::string>("map_frame", _frame_map, "origin");

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCL2MapRegistration]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  _pc_map  = load_pc_norm(_path_map);
  _pc_slam = load_pc_norm(_path_pcl);

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

  PC_NORM::Ptr pc_aligned = boost::make_shared<PC_NORM>();

  {
    std::scoped_lock lock(_mutex_registration);

    bool            converged;
    float           score;
    Eigen::Matrix4f T;
    PC_NORM::Ptr    aligned;

    switch (_registration_method) {
      case 0:
        std::tie(converged, score, T, aligned) = pcl2map_fpfh(_pc_slam, _pc_map);
        break;
      case 1:
        std::tie(converged, score, T, aligned) = pcl2map_ndt(_pc_slam, _pc_map);
        break;
      default:
        ROS_ERROR("[PCL2MapRegistration] Unknown registration method of type: %d. Allowed: {0=FPFH, 1=NDT}", _registration_method);
        return;
    }

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

      ROS_INFO("[PCL2MapRegistration] Registration converged with score: %0.2f", score);
      printEigenMatrix(T, "Transformation matrix:");

    } else {
      ROS_ERROR("[PCL2MapRegistration] Registration did not converge -- try to change registration parameters.");
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

  _registration_method = config.reg_method;

  _timer_registration.setPeriod(ros::Duration(config.reg_period));

  // Re-estimate normals
  if (std::fabs(_normal_estimation_radius - config.norm_estim_rad) > 1e-5) {
    _normal_estimation_radius = config.norm_estim_rad;
    if (!pcd_file_has_normals(_path_map)) {
      _pc_map = load_pc_norm(_path_map);
    }
    if (!pcd_file_has_normals(_path_pcl)) {
      _pc_slam = load_pc_norm(_path_pcl);
    }
  }

  // FPFH registration parameters
  _fpfh_voxel_leaf           = config.fpfh_voxel_leaf;
  _fpfh_search_rad           = config.fpfh_search_rad;
  _fpfh_similarity_threshold = config.fpfh_similarity_threshold;
  _fpfh_inlier_fraction      = config.fpfh_inlier_fraction;
  _fpfh_ransac_max_iter      = config.fpfh_ransac_max_iter;
  _fpfh_number_of_samples    = config.fpfh_number_of_samples;
  _fpfh_corr_randomness      = config.fpfh_corr_randomness;

  // NDT registration parameters
  _ndt_transformation_epsilon = config.ndt_transformation_epsilon;
  _ndt_step_size              = config.ndt_step_size;
  _ndt_resolution             = config.ndt_resolution;
  _ndt_maximum_iterations     = config.ndt_maximum_iterations;

  // Initial guess for registration
  _use_init_guess = config.use_init_guess;
  Eigen::AngleAxisf    init_rotation(config.init_guess_yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0, 0, 0);
  _initial_guess = (init_translation * init_rotation).matrix();
}
//}

/* pcl2map_ndt() //{*/
std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> PCL2MapRegistration::pcl2map_ndt(const PC_NORM::Ptr pc, const PC_NORM::Ptr pc_map) {
  TicToc t;

  // Filtering input scan to roughly to increase speed of registration.
  PC_NORM::Ptr pc_filtered = boost::make_shared<PC_NORM>();
  PC_NORM::Ptr pc_aligned  = boost::make_shared<PC_NORM>();

  pcl::ApproximateVoxelGrid<pt_NORM> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
  approximate_voxel_filter.setInputCloud(pc);
  approximate_voxel_filter.filter(*pc_filtered);

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pt_NORM, pt_NORM> ndt;
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
  if (_use_init_guess) {
    ndt.align(*pc_aligned, _initial_guess);
  } else {
    ndt.align(*pc_aligned);
  }

  t.toc_print("[PCL2MapRegistration] NDT registration run time");

  return std::make_tuple(ndt.hasConverged(), ndt.getFitnessScore(), ndt.getFinalTransformation(), pc_aligned);
}
/*//}*/

/* pcl2map_fpfh() //{*/
std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> PCL2MapRegistration::pcl2map_fpfh(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ) {

  TicToc t;

  // Create filtered objects
  PC_NORM::Ptr pc_aligned   = boost::make_shared<PC_NORM>();
  PC_NORM::Ptr pc_src_filt  = boost::make_shared<PC_NORM>();
  PC_NORM::Ptr pc_targ_filt = boost::make_shared<PC_NORM>();

  // Downsample to leaf size
  ROS_INFO("[PCL2MapRegistration] FPFH -- applying voxel grid");
  pcl::VoxelGrid<pt_NORM> grid;
  grid.setLeafSize(_fpfh_voxel_leaf, _fpfh_voxel_leaf, _fpfh_voxel_leaf);
  grid.setInputCloud(pc_src);
  grid.filter(*pc_src_filt);
  grid.setInputCloud(pc_targ);
  grid.filter(*pc_targ_filt);

  // Estimate FPFH features
  ROS_INFO("[PCL2MapRegistration] FPFH -- estimating FPFH features");
  PC_FPFH::Ptr                                        pc_fpfh_src  = boost::make_shared<PC_FPFH>();
  PC_FPFH::Ptr                                        pc_fpfh_targ = boost::make_shared<PC_FPFH>();
  pcl::FPFHEstimationOMP<pt_NORM, pt_NORM, feat_FPFH> fest;
  fest.setRadiusSearch(_fpfh_search_rad);
  fest.setInputCloud(pc_src_filt);
  fest.setInputNormals(pc_src_filt);
  fest.compute(*pc_fpfh_src);
  fest.setInputCloud(pc_targ_filt);
  fest.setInputNormals(pc_targ_filt);
  fest.compute(*pc_fpfh_targ);

  // Perform alignment
  ROS_INFO("[PCL2MapRegistration] FPFH -- aligning");
  pcl::SampleConsensusPrerejective<pt_NORM, pt_NORM, feat_FPFH> align;
  align.setInputSource(pc_src_filt);
  align.setSourceFeatures(pc_fpfh_src);
  align.setInputTarget(pc_targ_filt);
  align.setTargetFeatures(pc_fpfh_targ);
  align.setMaximumIterations(_fpfh_ransac_max_iter);            // Number of RANSAC iterations
  align.setNumberOfSamples(_fpfh_number_of_samples);            // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(_fpfh_corr_randomness);     // Number of nearest features to use
  align.setSimilarityThreshold(_fpfh_similarity_threshold);     // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(2.5f * _fpfh_voxel_leaf);  // Inlier threshold
  align.setInlierFraction(_fpfh_inlier_fraction);               // Required inlier fraction for accepting a pose hypothesis

  if (_use_init_guess) {
    align.align(*pc_aligned, _initial_guess);
  } else {
    align.align(*pc_aligned);
  }

  t.toc_print("[PCL2MapRegistration] FPFH registration run time");

  return std::make_tuple(align.hasConverged(), align.getFitnessScore(), align.getFinalTransformation(), pc_aligned);
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

/*//{ load_pc_norm() */
PC_NORM::Ptr PCL2MapRegistration::load_pc_norm(const std::string &path) {
  ROS_INFO("[PCL2MapRegistration] Loading PCD file: %s.", path.c_str());

  PC_NORM::Ptr pc_norm = boost::make_shared<PC_NORM>();

  if (pcd_file_has_normals(path)) {

    // Load points with normals
    if (pcl::io::loadPCDFile<pt_NORM>(path, *pc_norm) < 0) {
      ROS_ERROR("[PCL2MapRegistration] Couldn't read PCD (PointNormal) file: %s.", path.c_str());
      ros::shutdown();
    }
    ROS_INFO("[PCL2MapRegistration] Loaded PointNormal PC with %ld points.", pc_norm->points.size());

  } else {

    // Load points
    PC::Ptr pc_xyz = boost::make_shared<PC>();
    if (pcl::io::loadPCDFile<pt_XYZ>(path, *pc_xyz) < 0) {
      ROS_ERROR("[PCL2MapRegistration] Couldn't read PCD (PointXYZ) file: %s.", path.c_str());
      ros::shutdown();
    }
    ROS_INFO("[PCL2MapRegistration] Loaded XYZ PC with %ld points. Estimating the PC normals.", pc_xyz->points.size());

    // Estimate normals
    PC_NORM::Ptr normals = estimate_normals(pc_xyz, _normal_estimation_radius);

    // Merge points and normals
    pcl::concatenateFields(*pc_xyz, *normals, *pc_norm);
  }

  return pc_norm;
}
/*//}*/

/*//{ load_pc_normals() */
bool PCL2MapRegistration::load_pc_normals(const std::string &path, PC_NORM::Ptr &cloud) {

  // Check if normals are present in the PCD file by looking at the header
  if (!pcd_file_has_normals(path)) {
    return false;
  }

  // Load PCD file
  ROS_INFO("[PCL2MapRegistration] Loading normals from PCD file: %s.", path.c_str());
  if (pcl::io::loadPCDFile(path, *cloud) < 0) {
    ROS_ERROR("[PCL2MapRegistration] Couldn't read normals of PCD file: %s.", path.c_str());
    return false;
  }

  ROS_INFO("[PCL2MapRegistration] Loaded PCL normals with %ld points.", cloud->points.size());
  return true;
}
/*//}*/

/*//{ pcd_file_has_normals() */
bool PCL2MapRegistration::pcd_file_has_normals(const std::string path) {

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
    ROS_INFO("[PCL2MapRegistration] No normals in PCD file: %s.", path.c_str());
    return false;
  }

  return true;
}
/*//}*/

/*//{ estimate_normals() */
PC_NORM::Ptr PCL2MapRegistration::estimate_normals(const PC::Ptr cloud, const float nest_radius) {

  // XYZ type to XYZNormal
  PC_NORM::Ptr cloud_norm = boost::make_shared<PC_NORM>();
  pcl::copyPointCloud(*cloud, *cloud_norm);

  // Estimate normals
  pcl::NormalEstimationOMP<pt_NORM, pt_NORM> nest;
  nest.setRadiusSearch(nest_radius);
  nest.setInputCloud(cloud_norm);
  nest.compute(*cloud_norm);

  return cloud_norm;
}
/*//}*/

/*//{ publishCloud() */
void PCL2MapRegistration::publishCloud(const ros::Publisher pub, const PC_NORM::Ptr cloud) {
  if (pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud, *cloud_msg);
    publishCloudMsg(pub, cloud_msg);
  }
}
/*//}*/

/*//{ publishCloudMsg() */
void PCL2MapRegistration::publishCloudMsg(const ros::Publisher pub, const sensor_msgs::PointCloud2::Ptr cloud_msg) {
  try {
    pub.publish(cloud_msg);
  }
  catch (...) {
    NODELET_ERROR("[PCL2MapRegistration]: Exception caught during publishing on topic: %s", pub.getTopic().c_str());
  }
}
/*//}*/

/*//{ printEigenMatrix() */
void PCL2MapRegistration::printEigenMatrix(const Eigen::Matrix4f mat, const std::string prefix) {
  const std::string st = (prefix.size() > 0) ? prefix : "Eigen matrix:";
  ROS_INFO("[PCL2MapRegistration] %s", st.c_str());
  ROS_INFO("    | %2.3f %2.3f %2.3f |", mat(0, 0), mat(0, 1), mat(0, 2));
  ROS_INFO("R = | %2.3f %2.3f %2.3f |", mat(1, 0), mat(1, 1), mat(1, 2));
  ROS_INFO("    | %2.3f %2.3f %2.3f |", mat(2, 0), mat(2, 1), mat(2, 2));
  ROS_INFO("t = < %2.3f, %2.3f, %2.3f >", mat(0, 3), mat(1, 3), mat(2, 3));
}
/*//}*/

//
}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCL2MapRegistration, nodelet::Nodelet);
