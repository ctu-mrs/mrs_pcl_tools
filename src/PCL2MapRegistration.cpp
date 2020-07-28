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

  {
    std::scoped_lock lock(_mutex_registration);
    _pc_map  = loadPcWithNormals(_path_map);
    _pc_slam = loadPcWithNormals(_path_pcl);

    pcl::toROSMsg(*_pc_map, *_pc_map_msg);
    pcl::toROSMsg(*_pc_slam, *_pc_slam_msg);
  }

  _pc_map_msg->header.frame_id  = _frame_map;
  _pc_slam_msg->header.frame_id = _frame_map;

  _pub_cloud_source  = nh.advertise<sensor_msgs::PointCloud2>("cloud_source_out", 1);
  _pub_cloud_target  = nh.advertise<sensor_msgs::PointCloud2>("cloud_target_out", 1);
  _pub_cloud_aligned = nh.advertise<sensor_msgs::PointCloud2>("cloud_aligned_out", 1);

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh));
  ReconfigureServer::CallbackType f = boost::bind(&PCL2MapRegistration::callbackReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  NODELET_INFO_ONCE("[PCL2MapRegistration] Nodelet initialized");

  /* _timer_registration = nh.createTimer(ros::Duration(_registration_period), &PCL2MapRegistration::callbackRegistration, this, true, false); */
  _srv_server_registration = nh.advertiseService("srv_register", &PCL2MapRegistration::callbackSrvRegister, this);

  is_initialized = true;
}
//}

/* callbackRegistration() //{*/
void PCL2MapRegistration::callbackRegistration([[maybe_unused]] const ros::TimerEvent &event) {
  if (!is_initialized) {
    return;
  }

  // Preprocess data
  PC_NORM::Ptr pc_map_filt  = boost::make_shared<PC_NORM>();
  PC_NORM::Ptr pc_slam_filt = boost::make_shared<PC_NORM>();
  {
    std::scoped_lock lock(_mutex_registration);
    applyVoxelGridFilter(pc_map_filt, _pc_map, _clouds_voxel_leaf);
    applyVoxelGridFilter(pc_slam_filt, _pc_slam, _clouds_voxel_leaf);
  }
  matchPcCenters(pc_slam_filt, pc_map_filt);

  // Publish data
  std::uint64_t stamp;
  pcl_conversions::toPCL(ros::Time::now(), stamp);
  pc_slam_filt->header.stamp    = stamp;
  pc_slam_filt->header.frame_id = _frame_map;
  pc_map_filt->header.stamp     = stamp;
  pc_map_filt->header.frame_id  = _frame_map;
  publishCloud(_pub_cloud_source, pc_slam_filt);
  publishCloud(_pub_cloud_target, pc_map_filt);

  // Register given pc to map cloud
  registerCloudToCloud(pc_slam_filt, pc_map_filt);
}
/*//}*/

/* callbackSrvRegister() //{*/
bool PCL2MapRegistration::callbackSrvRegister([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  if (!is_initialized) {
    res.success = false;
    res.message = "Registration unitialized.";
    return false;
  }

  // Preprocess data
  PC_NORM::Ptr pc_map_filt  = boost::make_shared<PC_NORM>();
  PC_NORM::Ptr pc_slam_filt = boost::make_shared<PC_NORM>();
  {
    std::scoped_lock lock(_mutex_registration);
    applyVoxelGridFilter(pc_map_filt, _pc_map, _clouds_voxel_leaf);
    applyVoxelGridFilter(pc_slam_filt, _pc_slam, _clouds_voxel_leaf);
  }
  matchPcCenters(pc_slam_filt, pc_map_filt);

  // Publish data
  std::uint64_t stamp;
  pcl_conversions::toPCL(ros::Time::now(), stamp);
  pc_slam_filt->header.stamp    = stamp;
  pc_slam_filt->header.frame_id = _frame_map;
  pc_map_filt->header.stamp     = stamp;
  pc_map_filt->header.frame_id  = _frame_map;
  publishCloud(_pub_cloud_source, pc_slam_filt);
  publishCloud(_pub_cloud_target, pc_map_filt);

  // Register given pc to map cloud
  std::pair<bool, std::string> ret = registerCloudToCloud(pc_slam_filt, pc_map_filt);
  res.success                      = ret.first;
  res.message                      = ret.second;

  return res.success;
}
/*//}*/

/*//{ registerCloudToCloud() */
std::pair<bool, std::string> PCL2MapRegistration::registerCloudToCloud(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ) {
  bool            converged;
  float           score;
  Eigen::Matrix4f T_init;
  PC_NORM::Ptr    pc_aligned;

  std::scoped_lock lock(_mutex_registration);

  /*//{ Perform initial registration */
  switch (_registration_method_initial) {
    case 0:
      ROS_INFO("[PCL2MapRegistration] Registration (initial) with: FPFH");
      std::tie(converged, score, T_init, pc_aligned) = pcl2map_fpfh(pc_src, pc_targ);
      break;
    case 1:
      ROS_INFO("[PCL2MapRegistration] Registration (initial) with: NDT");
      std::tie(converged, score, T_init, pc_aligned) = pcl2map_ndt(pc_src, pc_targ);
      break;
    case 2:
      ROS_INFO("[PCL2MapRegistration] Registration (initial) with: GICP");
      std::tie(converged, score, T_init, pc_aligned) = pcl2map_gicp(pc_src, pc_targ);
      break;
    case 3:
      ROS_INFO("[PCL2MapRegistration] Registration (initial) with: ICPN");
      std::tie(converged, score, T_init, pc_aligned) = pcl2map_icpn(pc_src, pc_targ);
      break;
    default:
      ROS_ERROR("[PCL2MapRegistration] Unknown registration (initial) method of type: %d. Allowed: {0=FPFH, 1=NDT, 2=GICP, 3=ICPN}",
                _registration_method_initial);
      return std::make_pair<bool, std::string>(false, "Unknown registration (initial) method.");
  }
  /*//}*/

  std::pair<bool, std::string> ret;
  ret.second = "Registration successfull.";

  if (converged) {

    // Print transformation matrix
    ROS_INFO("[PCL2MapRegistration] Registration (initial) converged with score: %0.2f", score);
    printEigenMatrix(T_init, "Transformation matrix (initial):");

    // Publish initially aligned cloud
    pc_aligned->header.stamp    = pc_src->header.stamp;
    pc_aligned->header.frame_id = _frame_map;
    publishCloud(_pub_cloud_aligned, pc_aligned);

    /*//{ Perform fine tuning registration */
    const bool      perform_fine_tuning = _registration_method_fine_tune >= 1 && _registration_method_fine_tune <= 4;
    Eigen::Matrix4f T_fine              = Eigen::Matrix4f::Identity();
    switch (_registration_method_fine_tune) {
      case 0:
        ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: None");
        break;
      case 1:
        ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: FPFH");
        std::tie(converged, score, T_fine, pc_aligned) = pcl2map_fpfh(pc_aligned, pc_targ, false);
        break;
      case 2:
        ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: NDT");
        std::tie(converged, score, T_fine, pc_aligned) = pcl2map_ndt(pc_aligned, pc_targ, false);
        break;
      case 3:
        ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: GICP");
        std::tie(converged, score, T_fine, pc_aligned) = pcl2map_gicp(pc_aligned, pc_targ, false);
        break;
      case 4:
        ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: ICPN");
        std::tie(converged, score, T_fine, pc_aligned) = pcl2map_icpn(pc_aligned, pc_targ, false);
        break;
      default:
        ROS_ERROR("[PCL2MapRegistration] Unknown registration (fine tuning) method of type: %d. Allowed: {0=None, 1=FPFH, 2=NDT, 3=GICP, 4=ICPN}",
                  _registration_method_fine_tune);
        return std::make_pair<bool, std::string>(false, "Unknown registration (fine tuning) method.");
    }
    /*//}*/

    if (perform_fine_tuning) {

      if (converged) {
        // Print transformation matrix
        ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) converged with score: %0.2f", score);
        printEigenMatrix(T_fine, "Transformation matrix (fine tuning):");

        // Publish aligned cloud
        pc_aligned->header.stamp    = pc_src->header.stamp;
        pc_aligned->header.frame_id = _frame_map;
        publishCloud(_pub_cloud_aligned, pc_aligned);

      } else {
        ROS_ERROR("[PCL2MapRegistration] Registration (fine tuning) did not converge -- try to change registration (fine tuning) parameters.");
        ret.second = "Registered (fine tuning) dit not converge.";
      }
    }

    // Print final transformation matrix
    const Eigen::Matrix4f T = T_fine * T_init;
    printEigenMatrix(T, "Transformation matrix (final):");

  } else {
    ROS_ERROR("[PCL2MapRegistration] Registration (initial) did not converge -- try to change registration (initial) parameters.");
    return std::make_pair<bool, std::string>(false, "Registered (initial) dit not converge.");
  }

  ret.first = converged;
  return ret;
}
/*//}*/

/* callbackReconfigure() //{ */
void PCL2MapRegistration::callbackReconfigure(Config &config, [[maybe_unused]] uint32_t level) {
  if (!is_initialized) {
    return;
  }
  NODELET_INFO("[PCL2MapRegistration] Reconfigure callback.");

  std::scoped_lock lock(_mutex_registration);

  _registration_method_initial   = config.init_reg_method;
  _registration_method_fine_tune = config.fine_tune_reg_method;
  _clouds_voxel_leaf             = config.clouds_voxel_leaf;

  _timer_registration.setPeriod(ros::Duration(config.reg_period));

  // Re-estimate normals
  if (std::fabs(_normal_estimation_radius - config.norm_estim_rad) > 1e-5) {
    _normal_estimation_radius = config.norm_estim_rad;
    if (!pcdFileHasNormals(_path_map)) {
      _pc_map = loadPcWithNormals(_path_map);
    }
    if (!pcdFileHasNormals(_path_pcl)) {
      _pc_slam = loadPcWithNormals(_path_pcl);
    }
  }

  // FPFH registration parameters
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

  // GICP registration parameters
  _gicp_max_corr_dist        = config.gicp_max_corr_dist;
  _gicp_ransac_outl_rej_thrd = config.gicp_ransac_outl_rej_thrd;
  _gicp_trans_eps            = config.gicp_trans_eps;
  _gicp_max_iter             = config.gicp_max_iter;
  _gicp_max_opt_iter         = config.gicp_max_opt_iter;
  _gicp_ransac_iter          = config.gicp_ransac_iter;
  _gicp_use_recip_corr       = config.gicp_use_recip_corr;

  // ICPN registration parameters
  _icpn_max_corr_dist        = config.icpn_max_corr_dist;
  _icpn_ransac_outl_rej_thrd = config.icpn_ransac_outl_rej_thrd;
  _icpn_trans_eps            = config.icpn_trans_eps;
  _icpn_max_iter             = config.icpn_max_iter;
  _icpn_eucld_fitn_eps       = config.icpn_eucld_fitn_eps;
  _icpn_ransac_iter          = config.icpn_ransac_iter;
  _icpn_use_recip_corr       = config.icpn_use_recip_corr;

  // Initial guess for registration
  _use_init_guess = config.use_init_guess;
  Eigen::AngleAxisf    init_rotation(config.init_guess_yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0, 0, 0);
  _initial_guess = (init_translation * init_rotation).matrix();

  /* if ((bool)config.reg_enabled) { */
  /*   _timer_registration.start(); */
  /* } */
}
//}

/* pcl2map_ndt() //{*/
std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> PCL2MapRegistration::pcl2map_ndt(const PC_NORM::Ptr pc, const PC_NORM::Ptr pc_map,
                                                                                        const bool enable_init_guess) {
  TicToc t;

  // Filtering input scan to roughly to increase speed of registration.
  PC_NORM::Ptr pc_aligned = boost::make_shared<PC_NORM>();

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pt_NORM, pt_NORM> ndt;
  ndt.setInputSource(pc);
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
  if (enable_init_guess && _use_init_guess) {
    ROS_INFO("[PCL2MapRegistration] NDT -- using initial guess.");
    ndt.align(*pc_aligned, _initial_guess);
  } else {
    ROS_INFO("[PCL2MapRegistration] NDT -- no initial guess given.");
    ndt.align(*pc_aligned);
  }

  t.toc_print("[PCL2MapRegistration] NDT registration run time");

  return std::make_tuple(ndt.hasConverged(), ndt.getFitnessScore(), ndt.getFinalTransformation(), pc_aligned);
}
/*//}*/

/* pcl2map_fpfh() //{*/
std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> PCL2MapRegistration::pcl2map_fpfh(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ,
                                                                                         const bool enable_init_guess) {

  TicToc t;

  // Create filtered objects
  PC_NORM::Ptr pc_aligned = boost::make_shared<PC_NORM>();

  // Estimate FPFH features
  ROS_INFO("[PCL2MapRegistration] FPFH -- estimating FPFH features");
  PC_FPFH::Ptr                                        pc_fpfh_src  = boost::make_shared<PC_FPFH>();
  PC_FPFH::Ptr                                        pc_fpfh_targ = boost::make_shared<PC_FPFH>();
  pcl::FPFHEstimationOMP<pt_NORM, pt_NORM, feat_FPFH> fest;
  fest.setRadiusSearch(_fpfh_search_rad);
  fest.setInputCloud(pc_src);
  fest.setInputNormals(pc_src);
  fest.compute(*pc_fpfh_src);
  fest.setInputCloud(pc_targ);
  fest.setInputNormals(pc_targ);
  fest.compute(*pc_fpfh_targ);

  // Perform alignment
  ROS_INFO("[PCL2MapRegistration] FPFH -- aligning");
  pcl::SampleConsensusPrerejective<pt_NORM, pt_NORM, feat_FPFH> align;
  align.setInputSource(pc_src);
  align.setSourceFeatures(pc_fpfh_src);
  align.setInputTarget(pc_targ);
  align.setTargetFeatures(pc_fpfh_targ);
  align.setMaximumIterations(_fpfh_ransac_max_iter);              // Number of RANSAC iterations
  align.setNumberOfSamples(_fpfh_number_of_samples);              // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(_fpfh_corr_randomness);       // Number of nearest features to use
  align.setSimilarityThreshold(_fpfh_similarity_threshold);       // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(1.5f * _clouds_voxel_leaf);  // Inlier threshold
  align.setInlierFraction(_fpfh_inlier_fraction);                 // Required inlier fraction for accepting a pose hypothesis

  if (enable_init_guess && _use_init_guess) {
    ROS_INFO("[PCL2MapRegistration] FPFH -- using initial guess.");
    align.align(*pc_aligned, _initial_guess);
  } else {
    ROS_INFO("[PCL2MapRegistration] FPFH -- no initial guess given.");
    align.align(*pc_aligned);
  }

  t.toc_print("[PCL2MapRegistration] FPFH registration run time");

  return std::make_tuple(align.hasConverged(), align.getFitnessScore(), align.getFinalTransformation(), pc_aligned);
}
/*//}*/

/* pcl2map_gicp() //{*/
std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> PCL2MapRegistration::pcl2map_gicp(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ,
                                                                                         const bool enable_init_guess) {

  TicToc t;

  // Create filtered objects
  PC_NORM::Ptr pc_aligned = boost::make_shared<PC_NORM>();

  ROS_INFO("[PCL2MapRegistration] GICP -- aligning");
  pcl::GeneralizedIterativeClosestPoint<pt_NORM, pt_NORM> gicp;
  gicp.setInputSource(pc_src);
  gicp.setInputTarget(pc_targ);

  gicp.setMaxCorrespondenceDistance(_gicp_max_corr_dist);
  gicp.setMaximumIterations(_gicp_max_iter);
  gicp.setMaximumOptimizerIterations(_gicp_max_opt_iter);
  gicp.setRANSACIterations(_gicp_ransac_iter);
  gicp.setRANSACOutlierRejectionThreshold(_gicp_ransac_outl_rej_thrd);
  gicp.setTransformationEpsilon(_gicp_trans_eps);
  gicp.setUseReciprocalCorrespondences(_gicp_use_recip_corr);

  if (enable_init_guess && _use_init_guess) {
    ROS_INFO("[PCL2MapRegistration] GICP -- using initial guess.");
    gicp.align(*pc_aligned, _initial_guess);
  } else {
    ROS_INFO("[PCL2MapRegistration] GICP -- no initial guess given.");
    gicp.align(*pc_aligned);
  }

  t.toc_print("[PCL2MapRegistration] GICP registration run time");

  return std::make_tuple(gicp.hasConverged(), gicp.getFitnessScore(), gicp.getFinalTransformation(), pc_aligned);
}
/*//}*/

/* pcl2map_icpn() //{*/
std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> PCL2MapRegistration::pcl2map_icpn(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ,
                                                                                         const bool enable_init_guess) {

  TicToc t;

  // Create filtered objects
  PC_NORM::Ptr pc_aligned = boost::make_shared<PC_NORM>();

  ROS_INFO("[PCL2MapRegistration] ICPN -- aligning");
  pcl::IterativeClosestPointWithNormals<pt_NORM, pt_NORM> icpn;
  icpn.setInputSource(pc_src);
  icpn.setInputTarget(pc_targ);

  icpn.setTransformationEpsilon(1e-8);

  icpn.setMaxCorrespondenceDistance(_icpn_max_corr_dist);
  icpn.setMaximumIterations(_icpn_max_iter);
  icpn.setTransformationEpsilon(_icpn_trans_eps);
  icpn.setEuclideanFitnessEpsilon(_icpn_eucld_fitn_eps);
  icpn.setRANSACIterations(_icpn_ransac_iter);
  icpn.setRANSACOutlierRejectionThreshold(_icpn_ransac_outl_rej_thrd);
  icpn.setUseReciprocalCorrespondences(_icpn_use_recip_corr);

  if (enable_init_guess && _use_init_guess) {
    ROS_INFO("[PCL2MapRegistration] ICPN -- using initial guess.");
    icpn.align(*pc_aligned, _initial_guess);
  } else {
    ROS_INFO("[PCL2MapRegistration] ICPN -- no initial guess given.");
    icpn.align(*pc_aligned);
  }

  t.toc_print("[PCL2MapRegistration] ICPN registration run time");

  return std::make_tuple(icpn.hasConverged(), icpn.getFitnessScore(), icpn.getFinalTransformation(), pc_aligned);
}
/*//}*/

/*//{ matchPcCenters() */
void PCL2MapRegistration::matchPcCenters(PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ) {

  // Compute centroids of both clouds
  Eigen::Vector4f centroid_src;
  Eigen::Vector4f centroid_targ;
  pcl::compute3DCentroid(*pc_src, centroid_src);
  pcl::compute3DCentroid(*pc_targ, centroid_targ);
  Eigen::Vector4f centroid_diff = centroid_targ - centroid_src;

  // Compute min/max Z axis points
  pt_NORM pt_min_src;
  pt_NORM pt_min_targ;
  pt_NORM pt_max;
  pcl::getMinMax3D(*pc_src, pt_min_src, pt_max);
  pcl::getMinMax3D(*pc_targ, pt_min_targ, pt_max);

  // Build transformation matrix
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3)           = centroid_diff.x();
  T(1, 3)           = centroid_diff.y();
  T(2, 3)           = pt_min_targ.z - pt_min_src.z;

  // Transform pc_src
  pcl::transformPointCloud(*pc_src, *pc_src, T);
}
/*//}*/

/*//{ loadPcXYZ() */
PC::Ptr PCL2MapRegistration::loadPcXYZ(const std::string &path) {

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

/*//{ loadPcWithNormals() */
PC_NORM::Ptr PCL2MapRegistration::loadPcWithNormals(const std::string &path) {
  ROS_INFO("[PCL2MapRegistration] Loading PCD file: %s.", path.c_str());

  PC_NORM::Ptr pc_norm = boost::make_shared<PC_NORM>();

  if (pcdFileHasNormals(path)) {

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
    PC_NORM::Ptr normals = estimateNormals(pc_xyz, _normal_estimation_radius);

    // Merge points and normals
    pcl::concatenateFields(*pc_xyz, *normals, *pc_norm);
  }

  return pc_norm;
}
/*//}*/

/*//{ loadPcNormals() */
bool PCL2MapRegistration::loadPcNormals(const std::string &path, PC_NORM::Ptr &cloud) {

  // Check if normals are present in the PCD file by looking at the header
  if (!pcdFileHasNormals(path)) {
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

/*//{ pcdFileHasNormals() */
bool PCL2MapRegistration::pcdFileHasNormals(const std::string path) {

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

/*//{ estimateNormals() */
PC_NORM::Ptr PCL2MapRegistration::estimateNormals(const PC::Ptr cloud, const float nest_radius) {

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

/*//{ applyVoxelGridFilter() */
void PCL2MapRegistration::applyVoxelGridFilter(PC_NORM::Ptr cloud_out, const PC_NORM::Ptr cloud_in, const float leaf_size) {
  pcl::VoxelGrid<pt_NORM> grid;
  grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  grid.setInputCloud(cloud_in);
  grid.filter(*cloud_out);
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
