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

  _srv_server_registration_offline     = nh.advertiseService("srv_register_offline", &PCL2MapRegistration::callbackSrvRegisterOffline, this);
  _srv_server_registration_pointcloud2 = nh.advertiseService("srv_register_online", &PCL2MapRegistration::callbackSrvRegisterPointCloud2, this);

  is_initialized = true;
}
//}

/* callbackSrvRegisterOffline() //{*/
bool PCL2MapRegistration::callbackSrvRegisterOffline([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
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

    // for debugging: apply random translation on the slam pc
    applyRandomTransformation(pc_slam_filt);
  }
  correlateCloudToCloud(pc_slam_filt, pc_map_filt);

  // Publish input data
  std::uint64_t stamp;
  pcl_conversions::toPCL(ros::Time::now(), stamp);
  pc_slam_filt->header.stamp    = stamp;
  pc_slam_filt->header.frame_id = _frame_map;
  pc_map_filt->header.stamp     = stamp;
  pc_map_filt->header.frame_id  = _frame_map;
  publishCloud(_pub_cloud_source, pc_slam_filt);
  publishCloud(_pub_cloud_target, pc_map_filt);

  // Register given pc to map cloud
  std::tie(res.success, res.message, std::ignore) = registerCloudToCloud(pc_slam_filt, pc_map_filt);

  return res.success;
}
/*//}*/

/* callbackSrvRegisterPointCloud2() //{*/
bool PCL2MapRegistration::callbackSrvRegisterPointCloud2(mrs_pcl_tools::SrvRegisterPointCloud2::Request & req,
                                                         mrs_pcl_tools::SrvRegisterPointCloud2::Response &res) {
  if (!is_initialized) {
    res.success = false;
    res.message = "Registration unitialized.";
    return false;
  }

  // Preprocess data
  PC_NORM::Ptr pc_map_filt  = boost::make_shared<PC_NORM>();
  PC_NORM::Ptr pc_slam_filt = boost::make_shared<PC_NORM>();
  PC_NORM::Ptr pc_slam      = boost::make_shared<PC_NORM>();
  pcl::fromROSMsg(req.cloud, *pc_slam);
  {
    std::scoped_lock lock(_mutex_registration);
    applyVoxelGridFilter(pc_map_filt, _pc_map, _clouds_voxel_leaf);
  }
  applyVoxelGridFilter(pc_slam_filt, pc_slam, _clouds_voxel_leaf);
  correlateCloudToCloud(pc_slam_filt, pc_map_filt);

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
  Eigen::Matrix4f T;
  std::tie(res.success, res.message, T) = registerCloudToCloud(pc_slam_filt, pc_map_filt);

  // Convert transformation T to geometry_msgs/Pose
  if (res.success) {
    geometry_msgs::Pose              pose;
    const Eigen::Matrix3d            R = T.block<3, 3>(0, 0).cast<double>();
    const mrs_lib::AttitudeConverter atti(R);
    pose.orientation   = atti;
    pose.position.x    = T(0, 3);
    pose.position.y    = T(1, 3);
    pose.position.z    = T(2, 3);
    res.transformation = pose;
  }

  return res.success;
}
/*//}*/

/*//{ registerCloudToCloud() */
std::tuple<bool, std::string, Eigen::Matrix4f> PCL2MapRegistration::registerCloudToCloud(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ) {
  bool            converged;
  float           score;
  Eigen::Matrix4f T_init;
  PC_NORM::Ptr    pc_aligned;

  /*//{ Perform initial registration */
  {
    std::scoped_lock lock(_mutex_registration);
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
      case 4:
        ROS_INFO("[PCL2MapRegistration] Registration (initial) with: SICPN");
        std::tie(converged, score, T_init, pc_aligned) = pcl2map_sicpn(pc_src, pc_targ);
        break;
      default:
        ROS_ERROR("[PCL2MapRegistration] Unknown registration (initial) method of type: %d. Allowed: {0=FPFH, 1=NDT, 2=GICP, 3=ICPN, 4=SICPN}",
                  _registration_method_initial);
        return std::make_tuple<bool, std::string, Eigen::Matrix4f>(false, "Unknown registration (initial) method.", Eigen::Matrix4f::Identity());
    }
  }
  /*//}*/

  std::tuple<bool, std::string, Eigen::Matrix4f> ret;
  std::get<1>(ret) = "Registration successfull.";

  if (converged) {

    // Print transformation matrix
    ROS_INFO("[PCL2MapRegistration] Registration (initial) converged with score: %0.2f", score);
    printEigenMatrix(T_init, "Transformation matrix (initial):");
    std::get<2>(ret) = T_init;

    // Publish initially aligned cloud
    pc_aligned->header.stamp    = pc_src->header.stamp;
    pc_aligned->header.frame_id = _frame_map;
    publishCloud(_pub_cloud_aligned, pc_aligned);

    /*//{ Perform fine tuning registration */
    Eigen::Matrix4f T_fine              = Eigen::Matrix4f::Identity();
    const bool      perform_fine_tuning = _registration_method_fine_tune >= 0 && _registration_method_fine_tune <= 4;

    if (_registration_method_fine_tune == _registration_method_initial) {
      ROS_INFO("[PCL2MapRegistration] No registration (fine tuning) as method matches registration (initial).");
    } else {

      std::scoped_lock lock(_mutex_registration);
      switch (_registration_method_fine_tune) {
        case -1:
          ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: None");
          break;
        case 0:
          ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: FPFH");
          std::tie(converged, score, T_fine, pc_aligned) = pcl2map_fpfh(pc_aligned, pc_targ, false);
          break;
        case 1:
          ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: NDT");
          std::tie(converged, score, T_fine, pc_aligned) = pcl2map_ndt(pc_aligned, pc_targ, false);
          break;
        case 2:
          ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: GICP");
          std::tie(converged, score, T_fine, pc_aligned) = pcl2map_gicp(pc_aligned, pc_targ, false);
          break;
        case 3:
          ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: ICPN");
          std::tie(converged, score, T_fine, pc_aligned) = pcl2map_icpn(pc_aligned, pc_targ, false);
          break;
        case 4:
          ROS_INFO("[PCL2MapRegistration] Registration (fine tuning) with: SICPN");
          std::tie(converged, score, T_fine, pc_aligned) = pcl2map_sicpn(pc_aligned, pc_targ);
          break;
        default:
          ROS_ERROR("[PCL2MapRegistration] Unknown registration (fine tuning) method of type: %d. Allowed: {-1=None, 0=FPFH, 1=NDT, 2=GICP, 3=ICPN, 4=SICPN}",
                    _registration_method_fine_tune);
          return std::make_tuple<bool, std::string, Eigen::Matrix4f>(false, "Unknown registration (fine tuning) method.", Eigen::Matrix4f::Identity());
      }
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
        std::get<1>(ret) = "Registered (fine tuning) dit not converge.";
      }

    }

    // Print final transformation matrix
    const Eigen::Matrix4f T = T_fine * T_init;
    std::get<2>(ret)        = T;
    printEigenMatrix(T, "Transformation matrix (final):");

  } else {
    ROS_ERROR("[PCL2MapRegistration] Registration (initial) did not converge -- try to change registration (initial) parameters.");
    return std::make_tuple<bool, std::string, Eigen::Matrix4f>(false, "Registered (initial) dit not converge.", Eigen::Matrix4f::Identity());
  }

  if (converged && score > _min_convergence_score) {
    converged = false;
    ROS_ERROR("[PCL2MapRegistration] Registration (final) converged with low score (score: %0.2f, min_score: %0.2f)", score, _min_convergence_score);
  }

  std::get<0>(ret) = converged;
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

  _registration_method_initial     = config.init_reg_method;
  _registration_method_fine_tune   = config.fine_tune_reg_method;
  _clouds_voxel_leaf               = config.clouds_voxel_leaf;
  _cloud_correlation_z_crop_offset = config.cloud_correlation_z_crop_offset;
  _min_convergence_score           = config.min_convergence_score;

  // Re-estimate normals
  if (std::fabs(_normal_estimation_radius - config.norm_estim_rad) > 1e-5) {
    _normal_estimation_radius = config.norm_estim_rad;
    if (!hasNormals(_path_map)) {
      _pc_map = loadPcWithNormals(_path_map);
    }
    if (!hasNormals(_path_pcl)) {
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

  // SICPN registration parameters
  _sicpn_number_of_samples    = config.sicpn_number_of_samples;
  _sicpn_max_corr_dist        = config.sicpn_max_corr_dist;
  _sicpn_ransac_outl_rej_thrd = config.sicpn_ransac_outl_rej_thrd;
  _sicpn_trans_eps            = config.sicpn_trans_eps;
  _sicpn_max_iter             = config.sicpn_max_iter;
  _sicpn_eucld_fitn_eps       = config.sicpn_eucld_fitn_eps;
  _sicpn_ransac_iter          = config.sicpn_ransac_iter;
  _sicpn_use_recip_corr       = config.sicpn_use_recip_corr;

  // Initial guess for registration
  _use_init_guess = config.use_init_guess;
  Eigen::AngleAxisf    init_rotation(config.init_guess_yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0, 0, 0);
  _initial_guess = (init_translation * init_rotation).matrix();
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

  pcl::IterativeClosestPointWithNormals<pt_NORM, pt_NORM> icpn;
  icpn.setInputSource(pc_src);
  icpn.setInputTarget(pc_targ);

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

/* pcl2map_sicpn() //{*/
std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> PCL2MapRegistration::pcl2map_sicpn(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ) {

  TicToc t;

  // Prepare best-score variables
  float           score_best = std::numeric_limits<float>::infinity();
  Eigen::Matrix4f T_best;
  PC_NORM::Ptr    pc_aligned_best = boost::make_shared<PC_NORM>();

  // Compute src cloud centroid
  Eigen::Vector4f centroid_pc_src;
  pcl::compute3DCentroid(*pc_src, centroid_pc_src);

  // Prepare ICP object
  pcl::IterativeClosestPointWithNormals<pt_NORM, pt_NORM> sicpn;
  sicpn.setInputSource(pc_src);
  sicpn.setInputTarget(pc_targ);
  sicpn.setMaxCorrespondenceDistance(_icpn_max_corr_dist);
  sicpn.setMaximumIterations(_icpn_max_iter);
  sicpn.setTransformationEpsilon(_icpn_trans_eps);
  sicpn.setEuclideanFitnessEpsilon(_icpn_eucld_fitn_eps);
  sicpn.setRANSACIterations(_icpn_ransac_iter);
  sicpn.setRANSACOutlierRejectionThreshold(_icpn_ransac_outl_rej_thrd);
  sicpn.setUseReciprocalCorrespondences(_icpn_use_recip_corr);

  for (unsigned int i = 0; i < _sicpn_number_of_samples; i++) {

    // Create initial guess as a rotation by `heading` around pc_src centroid
    const float             heading = (float)i * 2.0f * M_PI / (float)_sicpn_number_of_samples;
    const Eigen::AngleAxisf heading_ax(heading, Eigen::Vector3f::UnitZ());
    const Eigen::Matrix4f   T_rot = getRotationMatrixAroundPoint(heading_ax.matrix(), centroid_pc_src);

    // Perform fine tuning registration
    PC_NORM::Ptr pc_aligned = boost::make_shared<PC_NORM>();
    sicpn.align(*pc_aligned, T_rot);

    // Store best score
    if (sicpn.hasConverged()) {
      const float score = sicpn.getFitnessScore();
      ROS_INFO("[PCL2MapRegistration] Registration (heading: %0.2f) converged with score: %0.2f", heading, score);
      if (score < score_best) {
        score_best      = score;
        T_best          = sicpn.getFinalTransformation();
        pc_aligned_best = pc_aligned;
      }
    }
  }

  // Return tuple
  std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> ret;
  std::get<0>(ret) = std::isfinite(score_best);
  std::get<1>(ret) = score_best;
  std::get<2>(ret) = T_best;
  std::get<3>(ret) = pc_aligned_best;

  if (std::get<0>(ret)) {

    ROS_INFO("[PCL2MapRegistration] Registration (SICPN) converged with score: %0.2f", score_best);

    // Publish aligned cloud
    pc_aligned_best->header.stamp    = pc_src->header.stamp;
    pc_aligned_best->header.frame_id = _frame_map;
    publishCloud(_pub_cloud_aligned, pc_aligned_best);

  } else {
    ROS_ERROR("[PCL2MapRegistration] Registration (SICPN) did not converge -- try to change registration (SICPN) parameters.");
  }

  t.toc_print("[PCL2MapRegistration] SICPN registration run time");

  return ret;
}
/*//}*/

/*//{ correlateCloudToCloud() */
void PCL2MapRegistration::correlateCloudToCloud(PC_NORM::Ptr pc_src, PC_NORM::Ptr pc_targ) {

  // Compute centroids of both clouds
  Eigen::Vector4f centroid_src;
  Eigen::Vector4f centroid_targ;
  pcl::compute3DCentroid(*pc_src, centroid_src);
  pcl::compute3DCentroid(*pc_targ, centroid_targ);
  Eigen::Vector4f centroid_diff = centroid_targ - centroid_src;

  // Compute min/max Z axis points
  pt_NORM pt_min_src;
  pt_NORM pt_min_targ;
  pt_NORM pt_max_src;
  pt_NORM pt_max_targ;
  pcl::getMinMax3D(*pc_src, pt_min_src, pt_max_src);
  pcl::getMinMax3D(*pc_targ, pt_min_targ, pt_max_targ);

  // Build transformation matrix
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3)           = centroid_diff.x();
  T(1, 3)           = centroid_diff.y();
  T(2, 3)           = pt_min_targ.z - pt_min_src.z;

  // Transform pc_src to pc_targ
  pcl::transformPointCloud(*pc_src, *pc_src, T);
  pcl::getMinMax3D(*pc_src, pt_min_src, pt_max_src);

  // Crop pc_targ in z-axis w.r.t. pc_src (assumption that we takeoff from ground and clouds roll/pitch angles can be neglected)
  pcl::CropBox<pt_NORM> box;
  box.setMin(Eigen::Vector4f(pt_min_targ.x, pt_min_targ.y, pt_min_targ.z, 1.0f));
  box.setMax(Eigen::Vector4f(pt_max_targ.x, pt_max_targ.y, pt_max_src.z + _cloud_correlation_z_crop_offset, 1.0f));
  box.setInputCloud(pc_targ);
  box.filter(*pc_targ);
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

  if (hasNormals(path)) {

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
  if (!hasNormals(path)) {
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

/*//{ hasNormals(std::string) */
bool PCL2MapRegistration::hasNormals(const std::string path) {

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

/*//{ hasNormals(sensor_msgs::PointCloud2)*/
bool PCL2MapRegistration::hasNormals(const sensor_msgs::PointCloud2::ConstPtr &cloud) {

  // Check header for normals (normal_x, normal_y, normal_z, curvature)
  unsigned int normal_fields   = 0;
  bool         curvature_field = false;
  for (auto field : cloud->fields) {
    if (field.name.rfind("normal", 0) == 0) {
      normal_fields++;
    } else if (field.name == "curvature") {
      curvature_field = true;
    }
  }
  if (normal_fields != 3 || !curvature_field) {
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

/*//{ applyRandomTransformation() */
void PCL2MapRegistration::applyRandomTransformation(PC_NORM::Ptr cloud) {

  Eigen::Matrix4f T;

  // Random rotation in some random interval (for debugging)
  const Eigen::AngleAxisf roll(-0.03f * M_PI + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.06f * M_PI))), Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf yaw(-0.03f * M_PI + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.06f * M_PI))), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf pitch(-M_PI + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2.0f * M_PI))), Eigen::Vector3f::UnitZ());
  T.block<3, 3>(0, 0) = (yaw * pitch * roll).matrix();

  // Random translation in some random interval (for debugging)
  T(0, 3) = -25.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 50.0f));
  T(1, 3) = -25.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 50.0f));
  T(2, 3) = -15.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 30.0f));

  // Transform cloud
  pcl::transformPointCloud(*cloud, *cloud, T);
}
/*//}*/

/*//{ getRotationMatrixAroundPoint() */
Eigen::Matrix4f PCL2MapRegistration::getRotationMatrixAroundPoint(const Eigen::Matrix3f rotation, const Eigen::Vector4f point) {
  Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T2 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T3 = Eigen::Matrix4f::Identity();

  // To the point
  T1(0, 3) = -point.x();
  T1(1, 3) = -point.y();
  T1(2, 3) = -point.z();

  // Rotate
  T2.block<3, 3>(0, 0) = rotation;

  // Back to the origin
  T3(0, 3) = point.x();
  T3(1, 3) = point.y();
  T3(2, 3) = point.z();

  return T3 * T2 * T1;
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
  const std::string          st = (prefix.size() > 0) ? prefix : "Eigen matrix:";
  mrs_lib::AttitudeConverter atti(mat.block<3, 3>(0, 0).cast<double>());
  ROS_INFO("[PCL2MapRegistration] %s", st.c_str());
  ROS_INFO("    | %2.3f %2.3f %2.3f |", mat(0, 0), mat(0, 1), mat(0, 2));
  ROS_INFO("R = | %2.3f %2.3f %2.3f |", mat(1, 0), mat(1, 1), mat(1, 2));
  ROS_INFO("    | %2.3f %2.3f %2.3f |", mat(2, 0), mat(2, 1), mat(2, 2));
  ROS_INFO("E = | %2.3f %2.3f %2.3f |", atti.getRoll(), atti.getPitch(), atti.getYaw());
  ROS_INFO("t = < %2.3f, %2.3f, %2.3f >", mat(0, 3), mat(1, 3), mat(2, 3));
}
/*//}*/

//
}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCL2MapRegistration, nodelet::Nodelet);
