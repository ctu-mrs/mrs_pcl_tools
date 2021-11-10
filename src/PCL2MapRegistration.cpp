#include "mrs_pcl_tools/PCL2MapRegistration.h"

namespace mrs_pcl_tools
{

/* onInit() //{ */
void PCL2MapRegistration::onInit() {

  _nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Get parameters from config file
  mrs_lib::ParamLoader param_loader(_nh, "PCL2MapRegistration");

  param_loader.loadParam("target_pcd", _path_map);
  param_loader.loadParam<std::string>("source_pcd", _path_pcl, "");
  param_loader.loadParam<std::string>("map_frame", _frame_map, "global_origin");
  param_loader.loadParam<std::string>("topic_pointcloud2", _topic_pc2, "");

  // General parameters
  param_loader.loadParam("method/initial", _registration_method_initial, 4);
  param_loader.loadParam("method/fine_tune", _registration_method_fine_tune, 3);
  param_loader.loadParam("use_init_guess", _use_init_guess, false);
  param_loader.loadParam("clouds_voxel_leaf", _clouds_voxel_leaf, 0.3f);
  param_loader.loadParam("normal_estimation_radius", _normal_estimation_radius, 0.25f);
  param_loader.loadParam("cloud_correlation/method", _cloud_correlation_method, std::string("centroid"));
  param_loader.loadParam("cloud_correlation/z_crop_offset", _cloud_correlation_z_crop_offset, 2.0f);
  param_loader.loadParam("min_convergence_score", _min_convergence_score, 0.5f);

  // Parameters: ICPN
  param_loader.loadParam("icpn/max_corr_dist", _icpn_max_corr_dist);
  param_loader.loadParam("icpn/ransac_outl_rej_thrd", _icpn_ransac_outl_rej_thrd);
  param_loader.loadParam("icpn/trans_ep", _icpn_trans_eps);
  param_loader.loadParam("icpn/eucld_fitn_eps", _icpn_eucld_fitn_eps);
  param_loader.loadParam("icpn/max_iter", _icpn_max_iter);
  param_loader.loadParam("icpn/ransac_iter", _icpn_ransac_iter);
  param_loader.loadParam("icpn/use_recip_corr", _icpn_use_recip_corr);

  // Parameters: SICPN
  param_loader.loadParam("sicpn/number_of_samples", _sicpn_number_of_samples);
  param_loader.loadParam("sicpn/max_corr_dist", _sicpn_max_corr_dist);
  param_loader.loadParam("sicpn/ransac_outl_rej_thrd", _sicpn_ransac_outl_rej_thrd);
  param_loader.loadParam("sicpn/trans_ep", _sicpn_trans_eps);
  param_loader.loadParam("sicpn/eucld_fitn_eps", _sicpn_eucld_fitn_eps);
  param_loader.loadParam("sicpn/max_iter", _sicpn_max_iter);
  param_loader.loadParam("sicpn/ransac_iter", _sicpn_ransac_iter);
  param_loader.loadParam("sicpn/use_recip_corr", _sicpn_use_recip_corr);

  const std::vector<std::string> supported_correlation_methods = {"polyline_barycenter", "centroid"};
  if (std::find(std::begin(supported_correlation_methods), std::end(supported_correlation_methods), _cloud_correlation_method) ==
      std::end(supported_correlation_methods)) {
    NODELET_ERROR("[PCL2MapRegistration]: Cloud correlation method (%s) is not supported.", _cloud_correlation_method.c_str());
    ros::shutdown();
    return;
  } else if (_cloud_correlation_method == "polyline_barycenter") {
    param_loader.loadParam("cloud_correlation/polyline_barycenter/alpha", _cloud_correlation_poly_bary_alpha);
  }

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCL2MapRegistration]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
    return;
  }

  {
    std::scoped_lock lock(_mutex_registration);
    _pc_map        = loadPcWithNormals(_path_map);
    _map_available = _pc_map->points.size() > 0;

    if (_path_pcl.size() > 0) {
      _pc_offline           = loadPcWithNormals(_path_pcl);
      _pc_offline_available = _pc_offline->points.size() > 0;
    }
  }

  _pub_cloud_source  = _nh.advertise<sensor_msgs::PointCloud2>("cloud_source_out", 1);
  _pub_cloud_target  = _nh.advertise<sensor_msgs::PointCloud2>("cloud_target_out", 1);
  _pub_cloud_aligned = _nh.advertise<sensor_msgs::PointCloud2>("cloud_aligned_out", 1);

  _pub_dbg_hull_src    = _nh.advertise<visualization_msgs::MarkerArray>("dbg_hull_source_out", 1);
  _pub_dbg_hull_target = _nh.advertise<visualization_msgs::MarkerArray>("dbg_hull_target_out", 1);

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, _nh));
  ReconfigureServer::CallbackType f = boost::bind(&PCL2MapRegistration::callbackReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  NODELET_INFO_ONCE("[PCL2MapRegistration] Nodelet initialized");

  _srv_server_registration_offline     = _nh.advertiseService("srv_register_offline", &PCL2MapRegistration::callbackSrvRegisterOffline, this);
  _srv_server_registration_pointcloud2 = _nh.advertiseService("srv_register_online", &PCL2MapRegistration::callbackSrvRegisterPointCloud, this);

  _is_initialized = true;
}
//}

/* callbackSrvRegisterOffline() //{*/
bool PCL2MapRegistration::callbackSrvRegisterOffline(mrs_pcl_tools::SrvRegisterPointCloudOffline::Request & req,
                                                     mrs_pcl_tools::SrvRegisterPointCloudOffline::Response &res) {
  if (!_is_initialized) {
    res.success = false;
    res.message = "Registration unitialized.";
    return false;
  } else if (!_map_available) {
    res.success = false;
    res.message = "Reference map has 0 points.";
    return false;
  } else if (!_pc_offline_available) {
    res.success = false;
    res.message = "No offline PC was pre-loaded to the memory or the pre-loaded PC has 0 points.";
    return false;
  }

  // Preprocess data
  PC_NORM::Ptr pc_targ_filt;
  PC_NORM::Ptr pc_src_filt;
  {
    std::scoped_lock lock(_mutex_registration);
    pc_targ_filt = filters::applyVoxelGridFilter(_pc_map, _clouds_voxel_leaf);
    pc_src_filt  = filters::applyVoxelGridFilter(_pc_offline, _clouds_voxel_leaf);

    // for debugging: apply random translation on the slam pc
    if (req.apply_random_transform) {
      applyRandomTransformation(pc_src_filt);
    }
  }

  // Correlate two clouds
  Eigen::Matrix4f T_corr;
  if (req.init_guess_use) {

    const Eigen::Vector3f &init_guess_translation = Eigen::Vector3f(req.init_guess_translation.x, req.init_guess_translation.y, req.init_guess_translation.z);
    T_corr                                        = translationYawToMatrix(init_guess_translation, req.init_guess_yaw);

    // Correlate: transform source to target
    pcl::transformPointCloud(*pc_src_filt, *pc_src_filt, T_corr);

  } else {

    // Match clouds
    T_corr = correlateCloudToCloud(pc_src_filt, pc_targ_filt);
  }

  // Publish input data
  std::uint64_t stamp;
  pcl_conversions::toPCL(ros::Time::now(), stamp);
  pc_src_filt->header.stamp     = stamp;
  pc_src_filt->header.frame_id  = _frame_map;
  pc_targ_filt->header.stamp    = stamp;
  pc_targ_filt->header.frame_id = _frame_map;
  publishCloud(_pub_cloud_source, pc_src_filt);
  publishCloud(_pub_cloud_target, pc_targ_filt);

  // Register given pc to map cloud
  Eigen::Matrix4f T;
  std::tie(res.success, res.message, T) = registerCloudToCloud(pc_src_filt, pc_targ_filt);

  // Include initial correlation matrix
  T = T * T_corr;

  // Save result as static TF pc_src.frame -> pc_targ.frame
  if (res.success) {
    geometry_msgs::TransformStamped tf_msg;

    // Prepare header
    pcl_conversions::fromPCL(stamp, tf_msg.header.stamp);
    tf_msg.header.frame_id = "frame_of_offline_pcd";
    tf_msg.child_frame_id  = _frame_map;

    // Fill transform
    tf_msg.transform = matrixToTfTransform(T.inverse());

    res.transform = tf_msg;
  }

  return res.success;
}
/*//}*/

/* callbackSrvRegisterPointCloud() //{*/
bool PCL2MapRegistration::callbackSrvRegisterPointCloud(mrs_pcl_tools::SrvRegisterPointCloudByName::Request & req,
                                                        mrs_pcl_tools::SrvRegisterPointCloudByName::Response &res) {
  if (!_is_initialized) {
    res.success = false;
    res.message = "Registration unitialized.";
    return false;
  } else if (!_map_available) {
    res.success = false;
    res.message = "Reference map has 0 points.";
    return false;
  } else if (_topic_pc2.size() == 0) {
    res.success = false;
    res.message = "No topic with msg type 'sensor_msgs/PointCloud2' specified.";
    return false;
  }

  // Preprocess data
  PC_NORM::Ptr pc_src_filt  = boost::make_shared<PC_NORM>();
  PC_NORM::Ptr pc_targ_filt = boost::make_shared<PC_NORM>();

  // Catch the latest msg and store it as source cloud
  PC_NORM::Ptr      pc_src;
  const std::string topic   = std::string("/") + req.uav_name + std::string("/") + _topic_pc2;
  auto              msg_ret = subscribeSinglePointCloudMsg(topic);

  if (msg_ret) {
    pc_src = msg_ret.value();
    if (pc_src->points.size() == 0) {
      res.success = false;
      res.message = "Received point cloud with 0 points.";
      NODELET_ERROR("[PCL2MapRegistration] Requested cloud registration to map, but the received data on topic (%s) contain 0 points.", topic.c_str());
      return false;
    }
  } else {
    res.success = false;
    res.message = "No point cloud data received.";
    NODELET_ERROR("[PCL2MapRegistration] Requested cloud registration to map, but did not receive data (for %0.2f sec) on topic: %s.",
                  _SUBSCRIBE_MSG_TIMEOUT.toSec(), topic.c_str());
    return false;
  }

  // Voxelize both clouds
  {
    std::scoped_lock lock(_mutex_registration);
    pc_targ_filt = filters::applyVoxelGridFilter(_pc_map, _clouds_voxel_leaf);
  }
  pc_src_filt = filters::applyVoxelGridFilter(pc_src, _clouds_voxel_leaf);

  // TODO: SOR and ROR filter

  // Correlate two clouds
  Eigen::Matrix4f T_corr;
  if (req.init_guess_use) {

    const Eigen::Vector3f &init_guess_translation = Eigen::Vector3f(req.init_guess_translation.x, req.init_guess_translation.y, req.init_guess_translation.z);
    T_corr                                        = translationYawToMatrix(init_guess_translation, req.init_guess_yaw);

    // Correlate: transform source to target
    pcl::transformPointCloud(*pc_src_filt, *pc_src_filt, T_corr);

  } else {

    // Match clouds
    T_corr = correlateCloudToCloud(pc_src_filt, pc_targ_filt);
  }

  // Publish input data
  pc_src_filt->header.stamp     = pc_src->header.stamp;
  pc_src_filt->header.frame_id  = _frame_map;
  pc_targ_filt->header.stamp    = pc_src->header.stamp;
  pc_targ_filt->header.frame_id = _frame_map;
  publishCloud(_pub_cloud_source, pc_src_filt);
  publishCloud(_pub_cloud_target, pc_targ_filt);

  // Register given pc to map cloud
  Eigen::Matrix4f T;
  std::tie(res.success, res.message, T) = registerCloudToCloud(pc_src_filt, pc_targ_filt);

  // Include initial correlation matrix
  T = T * T_corr;
  printEigenMatrix(T, "Transformation matrix (after correlation):");

  // Save result as static TF pc_src.frame -> pc_targ.frame
  if (res.success) {
    geometry_msgs::TransformStamped tf_msg;

    // Prepare header
    pcl_conversions::fromPCL(pc_src->header.stamp, tf_msg.header.stamp);
    tf_msg.header.frame_id = pc_src->header.frame_id;
    tf_msg.child_frame_id  = _frame_map;

    // Fill transform
    tf_msg.transform = matrixToTfTransform(T.inverse());

    res.transform = tf_msg;
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
        NODELET_INFO("[PCL2MapRegistration] Registration (initial) with: FPFH");
        std::tie(converged, score, T_init, pc_aligned) = pcl2map_fpfh(pc_src, pc_targ);
        break;
      case 1:
        NODELET_INFO("[PCL2MapRegistration] Registration (initial) with: NDT");
        std::tie(converged, score, T_init, pc_aligned) = pcl2map_ndt(pc_src, pc_targ);
        break;
      case 2:
        NODELET_INFO("[PCL2MapRegistration] Registration (initial) with: GICP");
        std::tie(converged, score, T_init, pc_aligned) = pcl2map_gicp(pc_src, pc_targ);
        break;
      case 3:
        NODELET_INFO("[PCL2MapRegistration] Registration (initial) with: ICPN");
        std::tie(converged, score, T_init, pc_aligned) = pcl2map_icpn(pc_src, pc_targ);
        break;
      case 4:
        NODELET_INFO("[PCL2MapRegistration] Registration (initial) with: SICPN");
        std::tie(converged, score, T_init, pc_aligned) = pcl2map_sicpn(pc_src, pc_targ);
        break;
      default:
        NODELET_ERROR("[PCL2MapRegistration] Unknown registration (initial) method of type: %d. Allowed: {0=FPFH, 1=NDT, 2=GICP, 3=ICPN, 4=SICPN}",
                      _registration_method_initial);
        return std::make_tuple<bool, std::string, Eigen::Matrix4f>(false, "Unknown registration (initial) method.", Eigen::Matrix4f::Identity());
    }
  }
  /*//}*/

  std::tuple<bool, std::string, Eigen::Matrix4f> ret;
  std::get<1>(ret) = "Registration successfull.";

  if (converged) {

    // Print transformation matrix
    NODELET_INFO("[PCL2MapRegistration] Registration (initial) converged with score: %0.2f", score);
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
      NODELET_INFO("[PCL2MapRegistration] No registration (fine tuning) as method matches registration (initial).");
    } else {

      std::scoped_lock lock(_mutex_registration);
      switch (_registration_method_fine_tune) {
        case -1:
          NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) with: None");
          break;
        case 0:
          NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) with: FPFH");
          std::tie(converged, score, T_fine, pc_aligned) = pcl2map_fpfh(pc_aligned, pc_targ, false);
          break;
        case 1:
          NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) with: NDT");
          std::tie(converged, score, T_fine, pc_aligned) = pcl2map_ndt(pc_aligned, pc_targ, false);
          break;
        case 2:
          NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) with: GICP");
          std::tie(converged, score, T_fine, pc_aligned) = pcl2map_gicp(pc_aligned, pc_targ, false);
          break;
        case 3:
          NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) with: ICPN");
          std::tie(converged, score, T_fine, pc_aligned) = pcl2map_icpn(pc_aligned, pc_targ, false);
          break;
        case 4:
          NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) with: SICPN");
          std::tie(converged, score, T_fine, pc_aligned) = pcl2map_sicpn(pc_aligned, pc_targ);
          break;
        default:
          NODELET_ERROR(
              "[PCL2MapRegistration] Unknown registration (fine tuning) method of type: %d. Allowed: {-1=None, 0=FPFH, 1=NDT, 2=GICP, 3=ICPN, 4=SICPN}",
              _registration_method_fine_tune);
          return std::make_tuple<bool, std::string, Eigen::Matrix4f>(false, "Unknown registration (fine tuning) method.", Eigen::Matrix4f::Identity());
      }
    }
    /*//}*/

    if (perform_fine_tuning) {

      if (converged) {
        // Print transformation matrix
        NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) converged with score: %0.2f", score);
        printEigenMatrix(T_fine, "Transformation matrix (fine tuning):");

        // Publish aligned cloud
        pc_aligned->header.stamp    = pc_src->header.stamp;
        pc_aligned->header.frame_id = _frame_map;
        publishCloud(_pub_cloud_aligned, pc_aligned);

      } else {
        NODELET_ERROR("[PCL2MapRegistration] Registration (fine tuning) did not converge -- try to change registration (fine tuning) parameters.");
        std::get<1>(ret) = "Registered (fine tuning) dit not converge.";
      }
    }

    // Print final transformation matrix
    const Eigen::Matrix4f T = T_fine * T_init;
    std::get<2>(ret)        = T;
    printEigenMatrix(T, "Transformation matrix (final):");

  } else {
    NODELET_ERROR("[PCL2MapRegistration] Registration (initial) did not converge -- try to change registration (initial) parameters.");
    return std::make_tuple<bool, std::string, Eigen::Matrix4f>(false, "Registered (initial) dit not converge.", Eigen::Matrix4f::Identity());
  }

  if (converged && score > _min_convergence_score) {
    converged = false;
    NODELET_ERROR("[PCL2MapRegistration] Registration (final) converged with low score (score: %0.2f, min_score: %0.2f)", score, _min_convergence_score);
  }

  std::get<0>(ret) = converged;
  return ret;
}
/*//}*/

/* callbackReconfigure() //{ */
void PCL2MapRegistration::callbackReconfigure(Config &config, [[maybe_unused]] uint32_t level) {
  if (!_is_initialized) {
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
      _pc_map        = loadPcWithNormals(_path_map);
      _map_available = _pc_map->points.size() > 0;
    }

    if (_path_pcl.size() > 0 && !hasNormals(_path_pcl)) {
      _pc_offline           = loadPcWithNormals(_path_pcl);
      _pc_offline_available = _pc_offline->points.size() > 0;
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
    NODELET_INFO("[PCL2MapRegistration] NDT -- using initial guess.");
    ndt.align(*pc_aligned, _initial_guess);
  } else {
    NODELET_INFO("[PCL2MapRegistration] NDT -- no initial guess given.");
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
  NODELET_INFO("[PCL2MapRegistration] FPFH -- estimating FPFH features");
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
  NODELET_INFO("[PCL2MapRegistration] FPFH -- aligning");
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
    NODELET_INFO("[PCL2MapRegistration] FPFH -- using initial guess.");
    align.align(*pc_aligned, _initial_guess);
  } else {
    NODELET_INFO("[PCL2MapRegistration] FPFH -- no initial guess given.");
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

  NODELET_INFO("[PCL2MapRegistration] GICP -- aligning");
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
    NODELET_INFO("[PCL2MapRegistration] GICP -- using initial guess.");
    gicp.align(*pc_aligned, _initial_guess);
  } else {
    NODELET_INFO("[PCL2MapRegistration] GICP -- no initial guess given.");
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
    NODELET_INFO("[PCL2MapRegistration] ICPN -- using initial guess.");
    icpn.align(*pc_aligned, _initial_guess);
  } else {
    NODELET_INFO("[PCL2MapRegistration] ICPN -- no initial guess given.");
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
    const float             heading = float(double(i) * 2.0 * M_PI / double(_sicpn_number_of_samples));
    const Eigen::AngleAxisf heading_ax(heading, Eigen::Vector3f::UnitZ());
    const Eigen::Matrix4f   T_rot = getRotationMatrixAroundPoint(heading_ax.matrix(), centroid_pc_src);

    // Perform fine tuning registration
    PC_NORM::Ptr pc_aligned = boost::make_shared<PC_NORM>();
    sicpn.align(*pc_aligned, T_rot);

    // Store best score
    if (sicpn.hasConverged()) {
      const double score = sicpn.getFitnessScore();
      NODELET_INFO("[PCL2MapRegistration] Registration (heading: %0.2f) converged with score: %0.2f", heading, score);
      if (score < score_best) {
        score_best      = float(score);
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

    NODELET_INFO("[PCL2MapRegistration] Registration (SICPN) converged with score: %0.2f", score_best);

    // Publish aligned cloud
    pc_aligned_best->header.stamp    = pc_src->header.stamp;
    pc_aligned_best->header.frame_id = _frame_map;
    publishCloud(_pub_cloud_aligned, pc_aligned_best);

  } else {
    NODELET_ERROR("[PCL2MapRegistration] Registration (SICPN) did not converge -- try to change registration (SICPN) parameters.");
  }

  t.toc_print("[PCL2MapRegistration] SICPN registration run time");

  return ret;
}
/*//}*/

/*//{ correlateCloudToCloud() */
Eigen::Matrix4f PCL2MapRegistration::correlateCloudToCloud(PC_NORM::Ptr &pc_src, PC_NORM::Ptr &pc_targ) {

  std::pair<Eigen::Vector3f, Eigen::Vector3f> origins;

  if (_cloud_correlation_method == "polyline_barycenter") {
    NODELET_INFO("[PCL2MapRegistration] Correlating clouds by their polyline barycenter.");
    origins = getPolylineBarycenters(pc_src, pc_targ);
  } else {
    NODELET_INFO("[PCL2MapRegistration] Correlating clouds by their centroids.");
    origins = getCentroids(pc_src, pc_targ);
  }

  const Eigen::Vector3f origin_src  = origins.first;
  const Eigen::Vector3f origin_targ = origins.second;
  const Eigen::Vector3f origin_diff = origin_targ - origin_src;

  NODELET_INFO("[PCL2MapRegistration] Cloud correlation origins:");
  NODELET_INFO("[PCL2MapRegistration]  source: (%.1f, %.1f, %.1f)", origin_src.x(), origin_src.y(), origin_src.z());
  NODELET_INFO("[PCL2MapRegistration]  target: (%.1f, %.1f, %.1f)", origin_targ.x(), origin_targ.y(), origin_targ.z());
  NODELET_INFO("[PCL2MapRegistration]  s->t:   (%.1f, %.1f, %.1f)", origin_diff.x(), origin_diff.y(), origin_diff.z());

  // Compute min/max Z axis points
  pt_NORM pt_min_src;
  pt_NORM pt_min_targ;
  pt_NORM pt_max_src;
  pt_NORM pt_max_targ;
  pcl::getMinMax3D(*pc_src, pt_min_src, pt_max_src);
  pcl::getMinMax3D(*pc_targ, pt_min_targ, pt_max_targ);

  // Build transformation matrix
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3)           = origin_diff.x();
  T(1, 3)           = origin_diff.y();
  T(2, 3)           = pt_min_targ.z - pt_min_src.z;

  // Transform pc_src to pc_targ
  pcl::transformPointCloud(*pc_src, *pc_src, T);

  // Crop pc_targ in z-axis w.r.t. pc_src (assumption that we takeoff from ground and clouds roll/pitch angles can be neglected)
  pcl::CropBox<pt_NORM> box;
  pcl::getMinMax3D(*pc_src, pt_min_src, pt_max_src);
  box.setMin(Eigen::Vector4f(pt_min_targ.x, pt_min_targ.y, pt_min_targ.z, 1.0f));
  box.setMax(Eigen::Vector4f(pt_max_targ.x, pt_max_targ.y, pt_max_src.z + _cloud_correlation_z_crop_offset, 1.0f));
  box.setInputCloud(pc_targ);
  box.filter(*pc_targ);

  return T;
}
/*//}*/

/*//{ getCentroids() */
std::pair<Eigen::Vector3f, Eigen::Vector3f> PCL2MapRegistration::getCentroids(const PC_NORM::Ptr &pc_src, const PC_NORM::Ptr &pc_targ) {
  return {getCentroid(pc_src), getCentroid(pc_targ)};
}
/*//}*/

/*//{ getPolylineBarycenters() */
std::pair<Eigen::Vector3f, Eigen::Vector3f> PCL2MapRegistration::getPolylineBarycenters(const PC_NORM::Ptr &pc_src, const PC_NORM::Ptr &pc_targ) {

  HULL hull_src                        = getConcaveHull(pc_src, _cloud_correlation_poly_bary_alpha);
  hull_src.polyline_barycenter         = getPolylineBarycenter(hull_src.edges);
  hull_src.cloud_hull->header.frame_id = _frame_map;

  HULL hull_trg;
  {
    std::scoped_lock lock(_mutex_hull_map);

    if (_hull_concave_map.has_data) {

      hull_trg = _hull_concave_map;

    } else {

      hull_trg                             = getConcaveHull(pc_targ, _cloud_correlation_poly_bary_alpha);
      hull_trg.polyline_barycenter         = getPolylineBarycenter(hull_trg.edges);
      hull_trg.cloud_hull->header.frame_id = _frame_map;
      hull_trg.has_data                    = true;

      _hull_concave_map = hull_trg;
    }
  }

  publishHull(_pub_dbg_hull_src, hull_src, Eigen::Vector3f(1, 0, 0));
  publishHull(_pub_dbg_hull_target, hull_trg, Eigen::Vector3f(0, 0, 1));

  return {hull_src.polyline_barycenter, hull_trg.polyline_barycenter};
}
/*//}*/

/*//{ getConcaveHull() */
HULL PCL2MapRegistration::getConcaveHull(const PC_NORM::Ptr &pc, const double alpha) {

  HULL hull;
  hull.concave    = true;
  hull.has_data   = true;
  hull.cloud_hull = boost::make_shared<PC_NORM>();

  // Construct concave hull in 3D
  std::vector<pcl::Vertices> polygons;
  pcl::ConcaveHull<pt_NORM>  concave_hull;
  concave_hull.setInputCloud(pc);
  concave_hull.setAlpha(alpha);
  concave_hull.setKeepInformation(true);
  concave_hull.setDimension(3);
  concave_hull.reconstruct(*hull.cloud_hull, polygons);

  /*//{ Get set of unique pair indices (edges) in the hull */
  typedef std::pair<uint32_t, uint32_t> EDGE;
  const auto                            edge_cmp = [](const EDGE &l, const EDGE &r) {
    if ((l.first == r.first && l.second == r.second) || (l.first == r.second && l.second == r.first)) {
      return false;
    }
    if (l.first == r.first) {
      return l.second < r.second;
    }
    return l.first < r.first;
  };

  std::set<EDGE, decltype(edge_cmp)> edges_idxs(edge_cmp);

  for (const auto &polygon : polygons) {

    if (polygon.vertices.size() != 3) {
      NODELET_ERROR("[PCL2MapRegistration] Number of vertices does not match 3! Should not happen, something is wrong.");
      continue;
    }

    const auto &vertices = polygon.vertices;

    /* NODELET_ERROR("inserting:"); */
    /* NODELET_ERROR(" 1) %d -> %d)", vertices.at(0), vertices.at(1)); */
    /* NODELET_ERROR(" 2) %d -> %d)", vertices.at(0), vertices.at(2)); */
    /* NODELET_ERROR(" 3) %d -> %d)", vertices.at(1), vertices.at(2)); */

    // Add all edges in the triangle-polygon
    edges_idxs.insert({vertices.at(0), vertices.at(1)});
    edges_idxs.insert({vertices.at(0), vertices.at(2)});
    edges_idxs.insert({vertices.at(1), vertices.at(2)});
  }
  /*//}*/

  // Convert pair indices to 3D point format
  unsigned int it = 0;
  hull.edges.resize(edges_idxs.size());
  for (const auto &edge_idx : edges_idxs) {
    const auto &point_A = hull.cloud_hull->points.at(edge_idx.first);
    const auto &point_B = hull.cloud_hull->points.at(edge_idx.second);
    hull.edges.at(it++) = {point_A, point_B};

    /* NODELET_ERROR("%d: (%.2f, %.2f, %.2f) -> %d: (%.2f, %.2f, %.2f)", edge_idx.first, point_A.x, point_A.y, point_A.z, edge_idx.second, point_B.x, point_B.y,
     */
    /* point_B.z); */
  }

  /* NODELET_ERROR("cloud hull size: %ld", hull.cloud_hull->size()); */

  return hull;
}
/*//}*/

/*//{ getCentroid() */
Eigen::Vector3f PCL2MapRegistration::getCentroid(const PC_NORM::Ptr &pc) {

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*pc, centroid);

  return Eigen::Vector3f(centroid.x(), centroid.y(), centroid.z());
}
/*//}*/

/*//{ getPolylineBarycenter() */
Eigen::Vector3f PCL2MapRegistration::getPolylineBarycenter(const std::vector<std::pair<pt_NORM, pt_NORM>> &edges) {

  Eigen::Vector3f midpoint         = Eigen::Vector3f::Zero();
  float           edges_sum_length = 0.0f;

  for (const auto &edge : edges) {

    const Eigen::Vector3f edge_from     = Eigen::Vector3f(edge.first.x, edge.first.y, edge.first.z);
    const Eigen::Vector3f edge_to       = Eigen::Vector3f(edge.second.x, edge.second.y, edge.second.z);
    const Eigen::Vector3f edge_vec      = edge_to - edge_from;
    const Eigen::Vector3f edge_midpoint = edge_from + 0.5f * edge_vec;
    const float           edge_norm     = edge_vec.norm();

    // Weighted sum with weigth being the length of the edges
    midpoint += edge_norm * edge_midpoint;
    edges_sum_length += edge_norm;
  }

  return midpoint / edges_sum_length;
}
/*//}*/

/*//{ loadPcWithNormals() */
PC_NORM::Ptr PCL2MapRegistration::loadPcWithNormals(const std::string &pcd_file) {
  NODELET_INFO("[PCL2MapRegistration] Loading PCD file: %s.", pcd_file.c_str());

  PC_NORM::Ptr pc_norm = boost::make_shared<PC_NORM>();

  if (hasNormals(pcd_file)) {

    // Load points with normals
    if (pcl::io::loadPCDFile<pt_NORM>(pcd_file, *pc_norm) < 0) {
      NODELET_ERROR("[PCL2MapRegistration] Couldn't read PCD (PointNormal) file: %s.", pcd_file.c_str());
      ros::shutdown();
    }
    NODELET_INFO("[PCL2MapRegistration] Loaded PointNormal PC with %ld points.", pc_norm->points.size());

  } else {

    // Load XYZ points
    PC::Ptr pc_xyz;
    auto    ret = loadPcXYZ(pcd_file);

    if (ret) {
      pc_xyz = ret.value();
    } else {
      NODELET_ERROR("[PCL2MapRegistration] Couldn't read PCD (PointXYZ) file: %s.", pcd_file.c_str());
      ros::shutdown();
    }

    NODELET_INFO("[PCL2MapRegistration] Loaded XYZ PC with %ld points. Estimating the PC normals.", pc_xyz->points.size());

    // Estimate normals
    PC_NORM::Ptr normals = estimateNormals(pc_xyz, _normal_estimation_radius);

    // Merge points and normals
    pcl::concatenateFields(*pc_xyz, *normals, *pc_norm);
  }

  return pc_norm;
}
/*//}*/

/*//{ subscribeSinglePointCloudMsg() */
std::optional<PC_NORM::Ptr> PCL2MapRegistration::subscribeSinglePointCloudMsg(const std::string &topic) {
  sensor_msgs::PointCloud2::ConstPtr cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, _nh, _SUBSCRIBE_MSG_TIMEOUT);

  if (cloud_msg) {

    PC_NORM::Ptr cloud;

    if (hasNormals(cloud_msg)) {

      cloud = boost::make_shared<PC_NORM>();
      pcl::fromROSMsg(*cloud_msg, *cloud);

    } else {

      PC::Ptr cloud_xyz = boost::make_shared<PC>();
      pcl::fromROSMsg(*cloud_msg, *cloud_xyz);
      cloud = estimateNormals(cloud_xyz, _normal_estimation_radius);
    }

    return cloud;
  }

  return std::nullopt;
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

/*//{ translationYawToMatrix() */
const Eigen::Matrix4f PCL2MapRegistration::translationYawToMatrix(const Eigen::Vector3f &translation, const float yaw) {

  // Initialize
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

  // Fill in translation
  T(0, 3) = translation.x();
  T(1, 3) = translation.y();
  T(2, 3) = translation.z();

  // Fill in rotation
  const Eigen::AngleAxisf rot_ax  = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  const Eigen::Matrix3d   rot_mat = mrs_lib::AttitudeConverter(rot_ax);
  T.block<3, 3>(0, 0)             = rot_mat.cast<float>();

  return T;
}
/*//}*/

/*//{ matrixToTfTransform() */
const geometry_msgs::Transform PCL2MapRegistration::matrixToTfTransform(const Eigen::Matrix4f &mat) {

  geometry_msgs::Transform tf_msg;

  tf_msg.translation.x = mat(0, 3);
  tf_msg.translation.y = mat(1, 3);
  tf_msg.translation.z = mat(2, 3);
  tf_msg.rotation      = mrs_lib::AttitudeConverter(mat.block<3, 3>(0, 0).cast<double>());

  return tf_msg;
}
/*//}*/

/*//{ publishCloud() */
void PCL2MapRegistration::publishCloud(const ros::Publisher &pub, const PC_NORM::Ptr &cloud) {
  if (pub.getNumSubscribers() > 0) {
    try {
      pub.publish(cloud);
    }
    catch (...) {
      NODELET_ERROR("[PCL2MapRegistration::publishCloud]: Exception caught during publishing on topic: %s", pub.getTopic().c_str());
    }
  }
}
/*//}*/

/*//{ publishHull() */
void PCL2MapRegistration::publishHull(const ros::Publisher &pub, const HULL &hull, const Eigen::Vector3f &color_rgb) {

  if (pub.getNumSubscribers() == 0) {
    return;
  }

  const ros::Time now = ros::Time::now();

  /*//{ Vertices */
  visualization_msgs::Marker m_vertices;
  m_vertices.ns              = "vertices";
  m_vertices.action          = visualization_msgs::Marker::ADD;
  m_vertices.type            = visualization_msgs::Marker::SPHERE_LIST;
  m_vertices.header.frame_id = hull.cloud_hull->header.frame_id;
  m_vertices.header.stamp    = now;
  m_vertices.scale.x         = 0.3;
  m_vertices.scale.y         = 0.3;
  m_vertices.scale.z         = 0.3;
  m_vertices.color.a         = 1.0;
  m_vertices.color.r         = color_rgb.x();
  m_vertices.color.g         = color_rgb.y();
  m_vertices.color.b         = color_rgb.z();
  m_vertices.points.resize(hull.cloud_hull->size());
  m_vertices.pose.orientation.w = 1.0;
  for (unsigned int i = 0; i < hull.cloud_hull->size(); i++) {
    m_vertices.points.at(i).x = hull.cloud_hull->points.at(i).x;
    m_vertices.points.at(i).y = hull.cloud_hull->points.at(i).y;
    m_vertices.points.at(i).z = hull.cloud_hull->points.at(i).z;
  }
  /*//}*/

  /*//{ Edges */
  visualization_msgs::Marker m_edges;
  m_edges.ns                 = "edges";
  m_edges.action             = visualization_msgs::Marker::ADD;
  m_edges.type               = visualization_msgs::Marker::LINE_LIST;
  m_edges.header             = m_vertices.header;
  m_edges.scale.x            = 0.07;
  m_edges.color              = m_vertices.color;
  m_edges.pose.orientation.w = 1.0;
  m_edges.points.resize(2 * hull.edges.size());

  unsigned int idx = 0;
  for (unsigned int i = 0; i < hull.edges.size(); i++) {

    m_edges.points.at(idx).x   = hull.edges.at(i).first.x;
    m_edges.points.at(idx).y   = hull.edges.at(i).first.y;
    m_edges.points.at(idx++).z = hull.edges.at(i).first.z;

    m_edges.points.at(idx).x   = hull.edges.at(i).second.x;
    m_edges.points.at(idx).y   = hull.edges.at(i).second.y;
    m_edges.points.at(idx++).z = hull.edges.at(i).second.z;
  }
  /*//}*/

  /*//{ Vertices */
  visualization_msgs::Marker m_barycenter;
  m_barycenter.ns                 = "barycenter";
  m_barycenter.action             = visualization_msgs::Marker::ADD;
  m_barycenter.type               = visualization_msgs::Marker::SPHERE;
  m_barycenter.header.frame_id    = hull.cloud_hull->header.frame_id;
  m_barycenter.header.stamp       = now;
  m_barycenter.scale.x            = 1.0;
  m_barycenter.scale.y            = 1.0;
  m_barycenter.scale.z            = 1.0;
  m_barycenter.color.a            = 1.0;
  m_barycenter.color.r            = color_rgb.x();
  m_barycenter.color.g            = color_rgb.y();
  m_barycenter.color.b            = color_rgb.z();
  m_barycenter.pose.position.x    = hull.polyline_barycenter.x();
  m_barycenter.pose.position.y    = hull.polyline_barycenter.y();
  m_barycenter.pose.position.z    = hull.polyline_barycenter.z();
  m_barycenter.pose.orientation.w = 1.0;
  /*//}*/

  visualization_msgs::MarkerArray ma;
  ma.markers = {m_vertices, m_edges, m_barycenter};

  try {
    pub.publish(ma);
  }
  catch (...) {
    NODELET_ERROR("[PCL2MapRegistration::publishHull]: Exception caught during publishing on topic: %s", pub.getTopic().c_str());
  }
}
/*//}*/

}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCL2MapRegistration, nodelet::Nodelet);
