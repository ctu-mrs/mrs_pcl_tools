#include "mrs_pcl_tools/PCL2MapRegistration.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>

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
  param_loader.loadParam("preprocess/voxel_leaf", _preprocess_voxel_leaf, 0.3f);
  param_loader.loadParam("preprocess/ror/radius", _preprocess_ror_radius, 0.0f);
  param_loader.loadParam("preprocess/ror/neighbors", _preprocess_ror_neighbors, 0);
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
    const std::string hull_type               = param_loader.loadParam2("cloud_correlation/polyline_barycenter/hull", std::string("concave"));
    _cloud_correlation_poly_bary_hull_concave = hull_type == "concave";

    if (_cloud_correlation_poly_bary_hull_concave) {
      param_loader.loadParam("cloud_correlation/polyline_barycenter/concave/alpha", _cloud_correlation_poly_bary_alpha);
    }
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
  _pub_dbg_pca         = _nh.advertise<visualization_msgs::MarkerArray>("dbg_pca_out", 1);

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
    pc_targ_filt = filters::applyVoxelGridFilter(_pc_map, _preprocess_voxel_leaf);

    pc_src_filt = filters::applyVoxelGridFilter(_pc_offline, _preprocess_voxel_leaf);
    if (_preprocess_ror_radius > 0.0f && _preprocess_ror_neighbors > 0) {
      pc_src_filt = filters::applyRadiusOutlierFilter(pc_src_filt, _preprocess_ror_radius, _preprocess_ror_neighbors, false);
    }

    // for debugging: apply random translation on the slam pc
    if (req.apply_random_transform) {
      applyRandomTransformation(pc_src_filt);
    }
  }

  RegistrationInput reg_input;
  reg_input.cloud_source      = pc_src_filt;
  reg_input.cloud_target      = pc_targ_filt;
  reg_input.enable_init_guess = req.init_guess_use;

  // Correlate two clouds
  if (req.init_guess_use) {

    const Eigen::Vector3f &init_guess_translation = Eigen::Vector3f(req.init_guess_translation.x, req.init_guess_translation.y, req.init_guess_translation.z);
    reg_input.T_guess                             = translationYawToMatrix(init_guess_translation, req.init_guess_yaw);

  } else {

    // Match clouds
    correlateCloudToCloud(reg_input);
    printEigenMatrix(reg_input.T_guess, "Transformation matrix (initial guess by cloud correlation):");
  }

  // Publish input data
  std::uint64_t stamp;
  pcl_conversions::toPCL(ros::Time::now(), stamp);
  pc_src_filt->header.stamp     = stamp;
  pc_src_filt->header.frame_id  = _frame_map;
  pc_targ_filt->header.stamp    = stamp;
  pc_targ_filt->header.frame_id = _frame_map;
  publishCloud(_pub_cloud_source, pc_src_filt, reg_input.T_guess);
  publishCloud(_pub_cloud_target, pc_targ_filt);

  // Register given pc to map cloud
  const RegistrationOutput reg_output = registerCloudToCloud(reg_input);
  res.success                         = reg_output.converged;
  res.message                         = reg_output.status_msg;

  // Save result as static TF pc_src.frame -> pc_targ.frame
  if (res.success) {
    geometry_msgs::TransformStamped tf_msg;

    // Prepare header
    pcl_conversions::fromPCL(stamp, tf_msg.header.stamp);
    tf_msg.header.frame_id = "frame_of_offline_pcd";
    tf_msg.child_frame_id  = _frame_map;

    // Fill transform
    tf_msg.transform = matrixToTfTransform(reg_output.transformation.inverse());

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
  PC_NORM::Ptr pc_src_filt;
  PC_NORM::Ptr pc_targ_filt;

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
    pc_targ_filt = filters::applyVoxelGridFilter(_pc_map, _preprocess_voxel_leaf);
  }
  pc_src_filt = filters::applyVoxelGridFilter(pc_src, _preprocess_voxel_leaf);
  if (_preprocess_ror_radius > 0.0f && _preprocess_ror_neighbors > 0) {
    pc_src_filt = filters::applyRadiusOutlierFilter(pc_src_filt, _preprocess_ror_radius, _preprocess_ror_neighbors, false);
  }

  RegistrationInput reg_input;
  reg_input.cloud_source      = pc_src_filt;
  reg_input.cloud_target      = pc_targ_filt;
  reg_input.enable_init_guess = req.init_guess_use;

  // Correlate two clouds
  if (req.init_guess_use) {

    const Eigen::Vector3f &init_guess_translation = Eigen::Vector3f(req.init_guess_translation.x, req.init_guess_translation.y, req.init_guess_translation.z);
    reg_input.T_guess                             = translationYawToMatrix(init_guess_translation, req.init_guess_yaw);

  } else {

    // Match clouds
    correlateCloudToCloud(reg_input);
    printEigenMatrix(reg_input.T_guess, "Transformation matrix (initial guess by cloud correlation):");
  }

  // Publish input data
  pc_src_filt->header.stamp     = pc_src->header.stamp;
  pc_src_filt->header.frame_id  = _frame_map;
  pc_targ_filt->header.stamp    = pc_src->header.stamp;
  pc_targ_filt->header.frame_id = _frame_map;
  publishCloud(_pub_cloud_source, pc_src_filt, reg_input.T_guess);
  publishCloud(_pub_cloud_target, pc_targ_filt);

  // Register given pc to map cloud
  const RegistrationOutput reg_output = registerCloudToCloud(reg_input);
  res.success                         = reg_output.converged;
  res.message                         = reg_output.status_msg;

  // Save result as static TF pc_src.frame -> pc_targ.frame
  if (res.success) {
    geometry_msgs::TransformStamped tf_msg;

    // Prepare header
    pcl_conversions::fromPCL(pc_src->header.stamp, tf_msg.header.stamp);
    tf_msg.header.frame_id = pc_src->header.frame_id;
    tf_msg.child_frame_id  = _frame_map;

    // Fill transform
    tf_msg.transform = matrixToTfTransform(reg_output.transformation.inverse());

    res.transform = tf_msg;
  }

  return res.success;
}
/*//}*/

/*//{ registerCloudToCloud() */
RegistrationOutput PCL2MapRegistration::registerCloudToCloud(RegistrationInput &input) {

  RegistrationOutput reg_output;

  /*//{ Perform initial registration */
  {
    std::scoped_lock lock(_mutex_registration);
    switch (_registration_method_initial) {
      case 0:
        NODELET_INFO("[PCL2MapRegistration] Registration (initial) with: FPFH");
        reg_output = pcl2map_fpfh(input);
        break;
      case 1:
        NODELET_INFO("[PCL2MapRegistration] Registration (initial) with: NDT");
        reg_output = pcl2map_ndt(input);
        break;
      case 2:
        NODELET_INFO("[PCL2MapRegistration] Registration (initial) with: GICP");
        reg_output = pcl2map_gicp(input);
        break;
      case 3:
        NODELET_INFO("[PCL2MapRegistration] Registration (initial) with: ICPN");
        reg_output = pcl2map_icpn(input);
        break;
      case 4:
        NODELET_INFO("[PCL2MapRegistration] Registration (initial) with: SICPN");
        reg_output = pcl2map_sicpn(input);
        break;
      default:
        NODELET_ERROR("[PCL2MapRegistration] Unknown registration (initial) method of type: %d. Allowed: {0=FPFH, 1=NDT, 2=GICP, 3=ICPN, 4=SICPN}",
                      _registration_method_initial);

        reg_output.status_msg = "Unknown registration (initial) method.";
        return reg_output;
    }
  }
  /*//}*/

  reg_output.status_msg = "Registration successfull.";

  if (reg_output.converged) {

    // Print transformation matrix
    NODELET_INFO("[PCL2MapRegistration] Registration (initial) converged with score: %0.2f", reg_output.fitness_score);
    printEigenMatrix(reg_output.transformation, "Transformation matrix (after initial registration):");

    // Store for fine registration and disable user's initial guess for fine registration
    input.T_guess           = reg_output.transformation;
    input.enable_init_guess = false;

    // Publish initially aligned cloud
    reg_output.cloud_aligned->header.stamp    = input.cloud_source->header.stamp;
    reg_output.cloud_aligned->header.frame_id = _frame_map;
    publishCloud(_pub_cloud_aligned, reg_output.cloud_aligned);

    /*//{ Perform fine tuning registration */
    const bool perform_fine_tuning =
        _registration_method_fine_tune >= 0 && _registration_method_fine_tune <= 4 && _registration_method_fine_tune != _registration_method_initial;

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
          reg_output = pcl2map_fpfh(input);
          break;
        case 1:
          NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) with: NDT");
          reg_output = pcl2map_ndt(input);
          break;
        case 2:
          NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) with: GICP");
          reg_output = pcl2map_gicp(input);
          break;
        case 3:
          NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) with: ICPN");
          reg_output = pcl2map_icpn(input);
          break;
        case 4:
          NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) with: SICPN");
          reg_output = pcl2map_sicpn(input);
          break;
        default:
          NODELET_ERROR(
              "[PCL2MapRegistration] Unknown registration (fine tuning) method of type: %d. Allowed: {-1=None, 0=FPFH, 1=NDT, 2=GICP, 3=ICPN, 4=SICPN}",
              _registration_method_fine_tune);

          reg_output.status_msg = "Unknown registration (fine tuning) method.";
          return reg_output;
      }
    }
    /*//}*/

    if (perform_fine_tuning) {

      if (reg_output.converged) {

        // Print transformation matrix
        NODELET_INFO("[PCL2MapRegistration] Registration (fine tuning) converged with score: %0.2f", reg_output.fitness_score);
        printEigenMatrix(reg_output.transformation, "Transformation matrix (after fine tuning):");

        // Store for future registrations
        input.T_guess = reg_output.transformation;

        // Publish aligned cloud
        reg_output.cloud_aligned->header.stamp    = input.cloud_source->header.stamp;
        reg_output.cloud_aligned->header.frame_id = _frame_map;
        publishCloud(_pub_cloud_aligned, reg_output.cloud_aligned);

      } else {

        NODELET_ERROR("[PCL2MapRegistration] Registration (fine tuning) did not converge -- try to change registration (fine tuning) parameters.");
        reg_output.status_msg = "Registered (fine tuning) dit not converge.";
        return reg_output;
      }
    }

  } else {

    NODELET_ERROR("[PCL2MapRegistration] Registration (initial) did not converge -- try to change registration (initial) parameters.");
    reg_output.status_msg = "Registered (initial) dit not converge.";
    return reg_output;
  }

  if (reg_output.converged && reg_output.fitness_score > _min_convergence_score) {
    NODELET_ERROR("[PCL2MapRegistration] Registration (final) converged with low score (score: %0.2f, min_score: %0.2f)", reg_output.fitness_score,
                  _min_convergence_score);
    reg_output.converged  = false;
    reg_output.status_msg = "Registration converged with low score.";
  }

  return reg_output;
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
  _preprocess_voxel_leaf           = config.clouds_voxel_leaf;
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
RegistrationOutput PCL2MapRegistration::pcl2map_ndt(RegistrationInput &input) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("PCL2MapRegistration::pcl2map_ndt", nullptr, false);

  RegistrationOutput output;
  output.cloud_aligned = boost::make_shared<PC_NORM>();

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pt_NORM, pt_NORM> ndt;
  ndt.setInputSource(input.cloud_source);
  ndt.setInputTarget(input.cloud_target);

  // Setting minimum transformation difference for termination condition.
  // Setting maximum step size for More-Thuente line search.
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  // Setting max number of registration iterations.
  ndt.setTransformationEpsilon(_ndt_transformation_epsilon);
  ndt.setStepSize(_ndt_step_size);
  ndt.setResolution(_ndt_resolution);
  ndt.setMaximumIterations(_ndt_maximum_iterations);

  // Calculating required rigid transform to align the input cloud to the target cloud.
  if (input.enable_init_guess && _use_init_guess) {
    NODELET_INFO("[PCL2MapRegistration] NDT -- using initial guess.");
    ndt.align(*output.cloud_aligned, _initial_guess);
  } else {
    NODELET_INFO("[PCL2MapRegistration] NDT -- no initial guess given.");
    ndt.align(*output.cloud_aligned, input.T_guess);
  }

  ROS_INFO("[PCL2MapRegistration] NDT registration run time: %ld ms", timer.getLifetime());

  output.converged      = ndt.hasConverged();
  output.fitness_score  = ndt.getFitnessScore();
  output.transformation = ndt.getFinalTransformation();

  return output;
}
/*//}*/

/* pcl2map_fpfh() //{*/
RegistrationOutput PCL2MapRegistration::pcl2map_fpfh(RegistrationInput &input) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("PCL2MapRegistration::pcl2map_fpfh", nullptr, false);

  RegistrationOutput output;
  output.cloud_aligned = boost::make_shared<PC_NORM>();

  // Estimate FPFH features
  NODELET_INFO("[PCL2MapRegistration] FPFH -- estimating FPFH features");
  PC_FPFH::Ptr                                        pc_fpfh_src  = boost::make_shared<PC_FPFH>();
  PC_FPFH::Ptr                                        pc_fpfh_targ = boost::make_shared<PC_FPFH>();
  pcl::FPFHEstimationOMP<pt_NORM, pt_NORM, feat_FPFH> fest;
  fest.setRadiusSearch(_fpfh_search_rad);
  fest.setInputCloud(input.cloud_source);
  fest.setInputNormals(input.cloud_source);
  fest.compute(*pc_fpfh_src);
  fest.setInputCloud(input.cloud_target);
  fest.setInputNormals(input.cloud_target);
  fest.compute(*pc_fpfh_targ);

  // Perform alignment
  NODELET_INFO("[PCL2MapRegistration] FPFH -- aligning");
  pcl::SampleConsensusPrerejective<pt_NORM, pt_NORM, feat_FPFH> align;
  align.setInputSource(input.cloud_source);
  align.setSourceFeatures(pc_fpfh_src);
  align.setInputTarget(input.cloud_target);
  align.setTargetFeatures(pc_fpfh_targ);
  align.setMaximumIterations(_fpfh_ransac_max_iter);                  // Number of RANSAC iterations
  align.setNumberOfSamples(_fpfh_number_of_samples);                  // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(_fpfh_corr_randomness);           // Number of nearest features to use
  align.setSimilarityThreshold(_fpfh_similarity_threshold);           // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(1.5f * _preprocess_voxel_leaf);  // Inlier threshold
  align.setInlierFraction(_fpfh_inlier_fraction);                     // Required inlier fraction for accepting a pose hypothesis

  if (input.enable_init_guess && _use_init_guess) {
    NODELET_INFO("[PCL2MapRegistration] FPFH -- using initial guess.");
    align.align(*output.cloud_aligned, _initial_guess);
  } else {
    NODELET_INFO("[PCL2MapRegistration] FPFH -- no initial guess given.");
    align.align(*output.cloud_aligned, input.T_guess);
  }

  ROS_INFO("[PCL2MapRegistration] FPFH registration run time: %ld ms", timer.getLifetime());

  output.converged      = align.hasConverged();
  output.fitness_score  = align.getFitnessScore();
  output.transformation = align.getFinalTransformation();

  return output;
}
/*//}*/

/* pcl2map_gicp() //{*/
RegistrationOutput PCL2MapRegistration::pcl2map_gicp(RegistrationInput &input) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("PCL2MapRegistration::pcl2map_gicp", nullptr, false);

  RegistrationOutput output;
  output.cloud_aligned = boost::make_shared<PC_NORM>();

  NODELET_INFO("[PCL2MapRegistration] GICP -- aligning");
  pcl::GeneralizedIterativeClosestPoint<pt_NORM, pt_NORM> gicp;
  gicp.setInputSource(input.cloud_source);
  gicp.setInputTarget(input.cloud_target);

  gicp.setMaxCorrespondenceDistance(_gicp_max_corr_dist);
  gicp.setMaximumIterations(_gicp_max_iter);
  gicp.setMaximumOptimizerIterations(_gicp_max_opt_iter);
  gicp.setRANSACIterations(_gicp_ransac_iter);
  gicp.setRANSACOutlierRejectionThreshold(_gicp_ransac_outl_rej_thrd);
  gicp.setTransformationEpsilon(_gicp_trans_eps);
  gicp.setUseReciprocalCorrespondences(_gicp_use_recip_corr);

  if (input.enable_init_guess && _use_init_guess) {
    NODELET_INFO("[PCL2MapRegistration] GICP -- using initial guess.");
    gicp.align(*output.cloud_aligned, _initial_guess);
  } else {
    NODELET_INFO("[PCL2MapRegistration] GICP -- no initial guess given.");
    gicp.align(*output.cloud_aligned, input.T_guess);
  }

  ROS_INFO("[PCL2MapRegistration] GICP registration run time: %ld ms", timer.getLifetime());

  output.converged      = gicp.hasConverged();
  output.fitness_score  = gicp.getFitnessScore();
  output.transformation = gicp.getFinalTransformation();

  return output;
}
/*//}*/

/* pcl2map_icpn() //{*/
RegistrationOutput PCL2MapRegistration::pcl2map_icpn(RegistrationInput &input) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("PCL2MapRegistration::pcl2map_icpn", nullptr, false);

  RegistrationOutput output;
  output.cloud_aligned = boost::make_shared<PC_NORM>();

  pcl::IterativeClosestPointWithNormals<pt_NORM, pt_NORM> icpn;
  icpn.setInputSource(input.cloud_source);
  icpn.setInputTarget(input.cloud_target);

  icpn.setMaxCorrespondenceDistance(_icpn_max_corr_dist);
  icpn.setMaximumIterations(_icpn_max_iter);
  icpn.setTransformationEpsilon(_icpn_trans_eps);
  icpn.setEuclideanFitnessEpsilon(_icpn_eucld_fitn_eps);
  icpn.setRANSACIterations(_icpn_ransac_iter);
  icpn.setRANSACOutlierRejectionThreshold(_icpn_ransac_outl_rej_thrd);
  icpn.setUseReciprocalCorrespondences(_icpn_use_recip_corr);

  if (input.enable_init_guess && _use_init_guess) {
    NODELET_INFO("[PCL2MapRegistration] ICPN -- using initial guess.");
    icpn.align(*output.cloud_aligned, _initial_guess);
  } else {
    NODELET_INFO("[PCL2MapRegistration] ICPN -- no initial guess given.");
    icpn.align(*output.cloud_aligned, input.T_guess);
  }

  ROS_INFO("[PCL2MapRegistration] ICPN registration run time: %ld ms", timer.getLifetime());

  output.converged      = icpn.hasConverged();
  output.fitness_score  = icpn.getFitnessScore();
  output.transformation = icpn.getFinalTransformation();

  return output;
}
/*//}*/

/* pcl2map_sicpn() //{*/
RegistrationOutput PCL2MapRegistration::pcl2map_sicpn(RegistrationInput &input) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("PCL2MapRegistration::pcl2map_sicpn", nullptr, false);

  RegistrationOutput output;

  // Prepare best-score variables
  float           score_best = std::numeric_limits<float>::infinity();
  Eigen::Matrix4f T_best;
  PC_NORM::Ptr    cloud_aligned_best = boost::make_shared<PC_NORM>();

  // Compute src cloud origin
  Eigen::Vector4f cloud_source_origin;
  if (input.has_origins) {
    cloud_source_origin = Eigen::Vector4f(input.origin_source.x(), input.origin_source.y(), input.origin_source.z(), 1.0f);
  } else {
    pcl::compute3DCentroid(*input.cloud_source, cloud_source_origin);
  }

  // Prepare ICP object
  pcl::IterativeClosestPointWithNormals<pt_NORM, pt_NORM> sicpn;
  sicpn.setInputSource(input.cloud_source);
  sicpn.setInputTarget(input.cloud_target);
  sicpn.setMaxCorrespondenceDistance(_icpn_max_corr_dist);
  sicpn.setMaximumIterations(_icpn_max_iter);
  sicpn.setTransformationEpsilon(_icpn_trans_eps);
  sicpn.setEuclideanFitnessEpsilon(_icpn_eucld_fitn_eps);
  sicpn.setRANSACIterations(_icpn_ransac_iter);
  sicpn.setRANSACOutlierRejectionThreshold(_icpn_ransac_outl_rej_thrd);
  sicpn.setUseReciprocalCorrespondences(_icpn_use_recip_corr);

  for (unsigned int i = 0; i < _sicpn_number_of_samples; i++) {

    // Create initial guess as a rotation by `heading` around input.cloud_source origin
    const float             heading = float(double(i) * 2.0 * M_PI / double(_sicpn_number_of_samples));
    const Eigen::AngleAxisf heading_ax(heading, Eigen::Vector3f::UnitZ());
    const Eigen::Matrix4f   T_rot       = getRotationMatrixAroundPoint(heading_ax.matrix(), cloud_source_origin);
    const Eigen::Matrix4f   T_guess_rot = input.T_guess * T_rot;

    // Perform fine tuning registration
    const PC_NORM::Ptr pc_aligned = boost::make_shared<PC_NORM>();
    sicpn.align(*pc_aligned, T_guess_rot);

    // Store best score
    if (sicpn.hasConverged()) {

      publishCloud(_pub_cloud_source, input.cloud_source, T_guess_rot);
      publishCloud(_pub_cloud_aligned, pc_aligned);

      const double score = sicpn.getFitnessScore();
      NODELET_INFO("[PCL2MapRegistration] Registration (heading: %0.2f) converged with score: %0.2f", heading, score);
      if (score < score_best) {
        score_best         = float(score);
        T_best             = sicpn.getFinalTransformation();
        cloud_aligned_best = pc_aligned;
      }
    }
  }

  output.converged      = std::isfinite(score_best);
  output.fitness_score  = score_best;
  output.transformation = T_best;
  output.cloud_aligned  = cloud_aligned_best;

  if (output.converged) {

    NODELET_INFO("[PCL2MapRegistration] Registration (SICPN) converged with score: %0.2f", score_best);

    // Publish aligned cloud
    cloud_aligned_best->header.stamp    = input.cloud_source->header.stamp;
    cloud_aligned_best->header.frame_id = _frame_map;
    publishCloud(_pub_cloud_aligned, cloud_aligned_best);

  } else {
    NODELET_ERROR("[PCL2MapRegistration] Registration (SICPN) did not converge -- try to change registration (SICPN) parameters.");
  }

  ROS_INFO("[PCL2MapRegistration] SICPN registration run time: %ld ms", timer.getLifetime());

  return output;
}
/*//}*/

/*//{ correlateCloudToCloud() */
void PCL2MapRegistration::correlateCloudToCloud(RegistrationInput &input) {

  std::pair<Eigen::Vector3f, Eigen::Vector3f> origins;

  bool                  has_hulls = false;
  std::pair<HULL, HULL> hulls;

  /*//{ Find correlation *translation* source->target */
  if (_cloud_correlation_method == "polyline_barycenter") {

    NODELET_INFO("[PCL2MapRegistration] Correlating clouds by their polyline barycenter.");

    hulls     = getHulls(input.cloud_source, input.cloud_target);
    has_hulls = hulls.first.has_data && hulls.second.has_data;

    origins = {hulls.first.polyline_barycenter, hulls.second.polyline_barycenter};

  } else {

    NODELET_INFO("[PCL2MapRegistration] Correlating clouds by their centroids.");
    origins = getCentroids(input.cloud_source, input.cloud_target);
  }

  input.has_origins   = true;
  input.origin_source = origins.first;
  input.origin_target = origins.second;

  const Eigen::Vector3f origin_src  = origins.first;
  const Eigen::Vector3f origin_targ = origins.second;
  const Eigen::Vector3f origin_diff = origin_targ - origin_src;
  /*//}*/

  NODELET_INFO("[PCL2MapRegistration] Cloud correlation:");

  /*//{ Find correlation *orientation* source->target */
  float                                 azimuth          = 0.0f;
  std::pair<EigenVectors, EigenVectors> eigenvectors     = getEigenVectors(input.cloud_source, input.cloud_target);
  const bool                            has_eigenvectors = eigenvectors.first.valid && eigenvectors.second.valid;

  if (has_eigenvectors) {

    Eigen::Vector3f vec_maxnorm_src_xyproj  = Eigen::Vector3f(eigenvectors.first.x.x(), eigenvectors.first.x.y(), 0.0f);
    Eigen::Vector3f vec_maxnorm_targ_xyproj = Eigen::Vector3f(eigenvectors.second.x.x(), eigenvectors.second.x.y(), 0.0f);

    const Eigen::Vector3f vec_y_src_xyproj  = Eigen::Vector3f(eigenvectors.first.y.x(), eigenvectors.first.y.y(), 0.0f);
    const Eigen::Vector3f vec_y_targ_xyproj = Eigen::Vector3f(eigenvectors.second.y.x(), eigenvectors.second.y.y(), 0.0f);

    if (vec_y_src_xyproj.norm() > vec_maxnorm_src_xyproj.norm()) {
      vec_maxnorm_src_xyproj = vec_y_src_xyproj;
    }

    if (vec_y_targ_xyproj.norm() > vec_maxnorm_targ_xyproj.norm()) {
      vec_maxnorm_targ_xyproj = vec_y_targ_xyproj;
    }

    const float azimuth_src  = std::atan2(vec_maxnorm_src_xyproj.y(), vec_maxnorm_src_xyproj.x());
    const float azimuth_targ = std::atan2(vec_maxnorm_targ_xyproj.y(), vec_maxnorm_targ_xyproj.x());

    azimuth = float(std::fmod(azimuth_targ - azimuth_src, 2.0 * M_PI));
    if (azimuth < -M_PI) {
      azimuth += 2.0 * M_PI;
    } else if (azimuth > M_PI) {
      azimuth -= 2.0 * M_PI;
    }

    NODELET_INFO("[PCL2MapRegistration]  source: xyz: (%.1f, %.1f, %.1f), largest-eigenvector azimuth: %.3f", origin_src.x(), origin_src.y(), origin_src.z(),
                 azimuth_src);
    NODELET_INFO("[PCL2MapRegistration]  target: xyz: (%.1f, %.1f, %.1f), largest-eigenvector azimuth: %.3f", origin_targ.x(), origin_targ.y(), origin_targ.z(),
                 azimuth_targ);
    NODELET_INFO("[PCL2MapRegistration]  s->t:   xyz: (%.1f, %.1f, %.1f), largest-eigenvector azimuth: %.3f", origin_diff.x(), origin_diff.y(), origin_diff.z(),
                 azimuth);

  } else {

    NODELET_INFO("[PCL2MapRegistration]  source: xyz: (%.1f, %.1f, %.1f), largest-eigenvector azimuth: N/A", origin_src.x(), origin_src.y(), origin_src.z());
    NODELET_INFO("[PCL2MapRegistration]  target: xyz: (%.1f, %.1f, %.1f), largest-eigenvector azimuth: N/A", origin_targ.x(), origin_targ.y(), origin_targ.z());
    NODELET_INFO("[PCL2MapRegistration]  s->t:   xyz: (%.1f, %.1f, %.1f), largest-eigenvector azimuth: N/A", origin_diff.x(), origin_diff.y(), origin_diff.z());
  }
  /*//}*/

  // Compute min/max Z axis points
  pt_NORM pt_min_src;
  pt_NORM pt_min_targ;
  pt_NORM pt_max_src;
  pt_NORM pt_max_targ;
  pcl::getMinMax3D(*input.cloud_source, pt_min_src, pt_max_src);
  pcl::getMinMax3D(*input.cloud_target, pt_min_targ, pt_max_targ);

  /*//{ Build transformation matrix */
  // Get rotation around the target centroid
  const Eigen::Vector4f   origin_src_4f = Eigen::Vector4f(origin_src.x(), origin_src.y(), origin_src.z(), 1.0f);
  const Eigen::AngleAxisf azimuth_ax    = Eigen::AngleAxisf(azimuth, Eigen::Vector3f::UnitZ());
  const Eigen::Matrix4f   T_rot         = getRotationMatrixAroundPoint(azimuth_ax.matrix(), origin_src_4f);

  // Get translation
  const Eigen::Matrix4f T_translation = translationToMatrix(Eigen::Vector3f(origin_diff.x(), origin_diff.y(), pt_min_targ.z - pt_min_src.z));

  // Build transformation matrix: rotate around the target centroid and translate to it
  input.T_guess = T_translation * T_rot;

  /*//}*/

  // Crop input.cloud_target in z-axis w.r.t. input.cloud_source (assumption that we takeoff from ground and clouds roll/pitch angles can be neglected)
  pcl::CropBox<pt_NORM> box;
  box.setMin(Eigen::Vector4f(pt_min_targ.x, pt_min_targ.y, pt_min_targ.z, 1.0f));
  box.setMax(Eigen::Vector4f(pt_max_targ.x, pt_max_targ.y, pt_min_targ.z + (pt_max_src.z - pt_min_src.z) + _cloud_correlation_z_crop_offset, 1.0f));
  box.setInputCloud(input.cloud_target);
  box.filter(*input.cloud_target);

  /*//{ Publish debugs */
  if (_pub_dbg_hull_src.getNumSubscribers() != 0 || _pub_dbg_hull_target.getNumSubscribers() != 0 || _pub_dbg_pca != 0) {

    // Transform source cloud origin (polyline barycenter or centroid)
    transformEigenVector(origins.first, Eigen::Affine3f(input.T_guess));

    if (has_hulls) {

      // Transform source hull only
      transformHull(hulls.first, input.T_guess);

      // Publish both hulls
      publishHull(_pub_dbg_hull_src, hulls.first, Eigen::Vector3f(1, 0, 0));
      publishHull(_pub_dbg_hull_target, hulls.second, Eigen::Vector3f(0, 0, 1));
    }

    if (has_eigenvectors) {

      // Transform source eigenvectors (rotate only)
      transformEigenVectors(eigenvectors.first, T_rot);

      // Publish both eigenvectors
      publishPCA(_pub_dbg_pca, _frame_map, eigenvectors, origins);
    }
  }
  /*//}*/
}
/*//}*/

/*//{ getCentroids() */
std::pair<Eigen::Vector3f, Eigen::Vector3f> PCL2MapRegistration::getCentroids(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ) {
  return {getCentroid(pc_src), getCentroid(pc_targ)};
}
/*//}*/

/*//{ getHulls() */
std::pair<HULL, HULL> PCL2MapRegistration::getHulls(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ) {

  HULL hull_src                        = getHull(pc_src, _cloud_correlation_poly_bary_hull_concave, _cloud_correlation_poly_bary_alpha);
  hull_src.polyline_barycenter         = getPolylineBarycenter(hull_src.edges);
  hull_src.cloud_hull->header.frame_id = _frame_map;
  hull_src.has_data                    = true;

  HULL hull_trg;
  {
    std::scoped_lock lock(_mutex_hull_map);

    if (_hull_concave_map.has_data) {

      hull_trg = _hull_concave_map;

    } else {

      hull_trg                             = getHull(pc_targ, _cloud_correlation_poly_bary_hull_concave, _cloud_correlation_poly_bary_alpha);
      hull_trg.polyline_barycenter         = getPolylineBarycenter(hull_trg.edges);
      hull_trg.cloud_hull->header.frame_id = _frame_map;
      hull_trg.has_data                    = true;

      _hull_concave_map = hull_trg;
    }
  }

  return {hull_src, hull_trg};
}
/*//}*/

/*//{ getEigenVectors() */
std::pair<EigenVectors, EigenVectors> PCL2MapRegistration::getEigenVectors(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ) {

  const EigenVectors eigenvectors_src  = getEigenVectors(pc_src);
  const EigenVectors eigenvectors_targ = getEigenVectors(pc_targ);

  print(eigenvectors_src, "source");
  print(eigenvectors_targ, "targ");

  return {eigenvectors_src, eigenvectors_targ};
}
/*//}*/

/*//{ getHull() */
HULL PCL2MapRegistration::getHull(const PC_NORM::Ptr pc, const bool concave, const double alpha) {

  HULL hull;
  hull.cloud_hull = boost::make_shared<PC_NORM>();
  hull.concave    = concave;

  // Construct hull in 3D
  std::vector<pcl::Vertices> polygons;
  computeHull(pc, hull.cloud_hull, polygons, concave, alpha);

  if (hull.cloud_hull->empty() || polygons.empty()) {
    NODELET_ERROR("[PCL2MapRegistration] Hull was not found!");
    return hull;
  }
  NODELET_INFO("[PCL2MapRegistration] %s hull was found (vertices: %ld).", hull.concave ? "Concave" : "Convex", hull.cloud_hull->size());

  hull.has_data = true;

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

    /* NODELET_ERROR("%d: (%.2f, %.2f, %.2f) -> %d: (%.2f, %.2f, %.2f)", edge_idx.first, point_A.x, point_A.y, point_A.z, edge_idx.second, point_B.x,
     * point_B.y,
     */
    /* point_B.z); */
  }

  return hull;
}
/*//}*/

/*//{ computeHull() */
void PCL2MapRegistration::computeHull(const PC_NORM::Ptr cloud_pc_in, const PC_NORM::Ptr cloud_hull_out, std::vector<pcl::Vertices> &polygons,
                                      const bool concave, const double alpha) {

  if (concave) {

    pcl::ConcaveHull<pt_NORM> concave_hull;
    concave_hull.setAlpha(alpha);
    concave_hull.setKeepInformation(true);
    concave_hull.setDimension(3);
    concave_hull.setInputCloud(cloud_pc_in);
    concave_hull.reconstruct(*cloud_hull_out, polygons);

  } else {

    pcl::ConvexHull<pt_NORM> convex_hull;
    convex_hull.setInputCloud(cloud_pc_in);
    convex_hull.setDimension(3);
    convex_hull.reconstruct(*cloud_hull_out, polygons);
  }
}
/*//}*/

/*//{ getCentroid() */
Eigen::Vector3f PCL2MapRegistration::getCentroid(const PC_NORM::Ptr pc) {

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

/*//{ getEigenVectors() */
EigenVectors PCL2MapRegistration::getEigenVectors(const PC_NORM::Ptr cloud) {

  EigenVectors ret;

  if (cloud->empty()) {
    NODELET_ERROR("[PCL2MapRegistration] Input cloud is empty. Cannot compute eigenvectors.");
    return ret;
  }

  float num_valid_points = 0.0f;

  // Compute mean
  float mean[3] = {0.0f, 0.0f, 0.0f};
  for (const auto &point : cloud->points) {

    mean[0] += point.x;
    mean[1] += point.y;
    mean[2] += point.z;

    num_valid_points += 1.0f;
  }

  mean[0] /= num_valid_points;
  mean[1] /= num_valid_points;
  mean[2] /= num_valid_points;

  // Compute covariance matrix
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = i; j < 3; j++) {

      for (const auto &point : cloud->points) {

        const auto point_vec = point.getArray3fMap();
        cov(i, j) += (point_vec[i] - mean[i]) * (point_vec[j] - mean[j]);
      }

      cov(i, j) /= num_valid_points;
      cov(j, i) = cov(i, j);
    }
  }
  /* std::cout << "number of points: " << num_valid_points << std::endl; */
  /* std::cout << "mean:" << std::endl; */
  /* std::cout << "  x: " << mean[0] << std::endl; */
  /* std::cout << "  y: " << mean[1] << std::endl; */
  /* std::cout << "  z: " << mean[2] << std::endl; */
  /* std::cout << "covariance matrix:" << std::endl; */
  /* std::cout << cov << std::endl; */

  // Compute eigenvectors
  Eigen::EigenSolver<Eigen::Matrix3f> es           = Eigen::EigenSolver<Eigen::Matrix3f>(cov);
  const auto                          eigenvectors = es.eigenvectors();
  const auto                          eigenvals    = es.eigenvalues();

  ret.x = Eigen::Vector3f(eigenvectors(0, 0).real(), eigenvectors(1, 0).real(), eigenvectors(2, 0).real()).normalized() * eigenvals(0).real();
  ret.y = Eigen::Vector3f(eigenvectors(0, 1).real(), eigenvectors(1, 1).real(), eigenvectors(2, 1).real()).normalized() * eigenvals(1).real();
  ret.z = Eigen::Vector3f(eigenvectors(0, 2).real(), eigenvectors(1, 2).real(), eigenvectors(2, 2).real()).normalized() * eigenvals(2).real();

  ret.valid = true;
  return ret;
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
    const PC_NORM::Ptr normals = estimateNormals(pc_xyz, _normal_estimation_radius);

    // Merge points and normals
    pcl::concatenateFields(*pc_xyz, *normals, *pc_norm);
  }

  removeNans(pc_norm);
  NODELET_INFO("[PCL2MapRegistration] Cloud size after NaN removal: %ld.", pc_norm->points.size());

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
void PCL2MapRegistration::applyRandomTransformation(const PC_NORM::Ptr cloud) {

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

  // Random rotation in some random interval (for debugging)
  const Eigen::AngleAxisf roll(-0.03f * M_PI + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.06f * M_PI))), Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf yaw(-0.03f * M_PI + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.06f * M_PI))), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf pitch(-M_PI + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2.0f * M_PI))), Eigen::Vector3f::UnitZ());
  T.block<3, 3>(0, 0) = (yaw * pitch * roll).matrix();

  // Random translation in some random interval (for debugging)
  T(0, 3) = -25.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 50.0f));
  T(1, 3) = -25.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 50.0f));
  T(2, 3) = -15.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 30.0f));

  printEigenMatrix(T, "Applying random transformation:");

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

/*//{ translationToMatrix() */
const Eigen::Matrix4f PCL2MapRegistration::translationToMatrix(const Eigen::Vector3f &translation) {

  // Initialize
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

  // Fill in translation
  T(0, 3) = translation.x();
  T(1, 3) = translation.y();
  T(2, 3) = translation.z();

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
void PCL2MapRegistration::publishCloud(const ros::Publisher &pub, const PC_NORM::Ptr pc, const Eigen::Matrix4f &transform) {
  if (pub.getNumSubscribers() > 0) {

    PC_NORM::Ptr cloud;

    if (!transform.isIdentity()) {
      cloud = boost::make_shared<PC_NORM>();
      pcl::transformPointCloud(*pc, *cloud, transform);
      cloud->header = pc->header;
    } else {
      cloud = pc;
    }

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

  if (pub.getNumSubscribers() == 0 || !hull.has_data) {
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

  /*//{ Barycenters */
  visualization_msgs::Marker m_barycenter;
  m_barycenter.ns                 = "barycenter";
  m_barycenter.action             = visualization_msgs::Marker::ADD;
  m_barycenter.type               = visualization_msgs::Marker::SPHERE;
  m_barycenter.header.frame_id    = hull.cloud_hull->header.frame_id;
  m_barycenter.header.stamp       = now;
  m_barycenter.scale.x            = 1.0;
  m_barycenter.scale.y            = 1.0;
  m_barycenter.scale.z            = 1.0;
  m_barycenter.color.a            = 0.5;
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

/*//{ publishPCA() */
void PCL2MapRegistration::publishPCA(const ros::Publisher &pub, const std::string &frame_id, const std::pair<EigenVectors, EigenVectors> &eigenvectors,
                                     const std::pair<Eigen::Vector3f, Eigen::Vector3f> &origins) {

  if (pub.getNumSubscribers() == 0 || !eigenvectors.first.valid || !eigenvectors.second.valid) {
    return;
  }

  const ros::Time       now           = ros::Time::now();
  const Eigen::Vector3f color_red     = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  const Eigen::Vector3f color_green   = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  const Eigen::Vector3f color_blue    = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  const Eigen::Vector3f color_grey    = Eigen::Vector3f(0.84f, 0.84f, 1.84f);
  const double          vec_vis_scale = 0.07;

  /*//{ First */

  visualization_msgs::Marker m_first;

  {
    m_first.ns              = "eigenvectors_source";
    m_first.action          = visualization_msgs::Marker::ADD;
    m_first.type            = visualization_msgs::Marker::LINE_LIST;
    m_first.header.frame_id = frame_id;
    m_first.header.stamp    = now;
    m_first.scale.x         = vec_vis_scale;
    m_first.colors.resize(8);
    m_first.points.resize(8);

    const auto &origin = origins.first;
    const auto &vecs   = eigenvectors.first;

    // Scale and 3D-place vectors
    const float scale  = std::max(vecs.x.norm(), std::max(vecs.y.norm(), vecs.z.norm()));
    const auto  vec_x  = origin + vecs.x / scale;
    const auto  vec_y  = origin + vecs.y / scale;
    const auto  vec_z  = origin + vecs.z / scale;
    const auto  vec_xy = origin + (vecs.x + vecs.y) / scale;

    // Visualize x
    m_first.points.at(0) = toGeometryMsg(origin);
    m_first.points.at(1) = toGeometryMsg(vec_x);
    m_first.colors.at(0) = toColorMsg(color_red);
    m_first.colors.at(1) = toColorMsg(color_red);

    // Visualize y
    m_first.points.at(2) = toGeometryMsg(origin);
    m_first.points.at(3) = toGeometryMsg(vec_y);
    m_first.colors.at(2) = toColorMsg(color_green);
    m_first.colors.at(3) = toColorMsg(color_green);

    // Visualize x
    m_first.points.at(4) = toGeometryMsg(origin);
    m_first.points.at(5) = toGeometryMsg(vec_z);
    m_first.colors.at(4) = toColorMsg(color_blue);
    m_first.colors.at(5) = toColorMsg(color_blue);

    // Visualize x
    m_first.points.at(6) = toGeometryMsg(origin);
    m_first.points.at(7) = toGeometryMsg(vec_xy);
    m_first.colors.at(6) = toColorMsg(color_grey);
    m_first.colors.at(7) = toColorMsg(color_grey);
  }
  /*//}*/

  /*//{ Second */

  visualization_msgs::Marker m_second;

  {
    m_second.ns              = "eigenvectors_target";
    m_second.action          = visualization_msgs::Marker::ADD;
    m_second.type            = visualization_msgs::Marker::LINE_LIST;
    m_second.header.frame_id = frame_id;
    m_second.header.stamp    = now;
    m_second.scale.x         = vec_vis_scale;
    m_second.colors.resize(8);
    m_second.points.resize(8);

    const auto &origin = origins.second;
    const auto &vecs   = eigenvectors.second;

    // Scale and 3D-place vectors
    const float scale  = std::max(vecs.x.norm(), std::max(vecs.y.norm(), vecs.z.norm()));
    const auto  vec_x  = origin + vecs.x / scale;
    const auto  vec_y  = origin + vecs.y / scale;
    const auto  vec_z  = origin + vecs.z / scale;
    const auto  vec_xy = origin + (vecs.x + vecs.y) / scale;

    // Visualize x
    m_second.points.at(0) = toGeometryMsg(origin);
    m_second.points.at(1) = toGeometryMsg(vec_x);
    m_second.colors.at(0) = toColorMsg(color_red);
    m_second.colors.at(1) = toColorMsg(color_red);

    // Visualize y
    m_second.points.at(2) = toGeometryMsg(origin);
    m_second.points.at(3) = toGeometryMsg(vec_y);
    m_second.colors.at(2) = toColorMsg(color_green);
    m_second.colors.at(3) = toColorMsg(color_green);

    // Visualize x
    m_second.points.at(4) = toGeometryMsg(origin);
    m_second.points.at(5) = toGeometryMsg(vec_z);
    m_second.colors.at(4) = toColorMsg(color_blue);
    m_second.colors.at(5) = toColorMsg(color_blue);

    // Visualize x
    m_second.points.at(6) = toGeometryMsg(origin);
    m_second.points.at(7) = toGeometryMsg(vec_xy);
    m_second.colors.at(6) = toColorMsg(color_grey);
    m_second.colors.at(7) = toColorMsg(color_grey);
  }
  /*//}*/

  visualization_msgs::MarkerArray ma;
  ma.markers = {m_first, m_second};

  try {
    pub.publish(ma);
  }
  catch (...) {
    NODELET_ERROR("[PCL2MapRegistration::publishPCA]: Exception caught during publishing on topic: %s", pub.getTopic().c_str());
  }
}
/*//}*/

/*//{ removeNans() */
void PCL2MapRegistration::removeNans(PC_NORM::Ptr &cloud) {

  size_t             k         = 0;
  const PC_NORM::Ptr cloud_out = boost::make_shared<PC_NORM>();
  cloud_out->points.resize(cloud->size());

  for (const auto &point : cloud->points) {
    const bool xyz_valid  = std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    const bool norm_valid = std::isfinite(point.normal_x) && std::isfinite(point.normal_y) && std::isfinite(point.normal_z);

    if (xyz_valid && norm_valid) {
      cloud_out->points.at(k++) = point;
    }
  }

  cloud_out->points.resize(k);
  cloud_out->header   = cloud->header;
  cloud_out->is_dense = true;
  cloud_out->width    = k;
  cloud_out->height   = 1;

  cloud = cloud_out;
}
/*//}*/

/*//{ transformHull() */
void PCL2MapRegistration::transformHull(HULL &hull, const Eigen::Matrix4f &mat) {

  // Transform cloud
  pcl::transformPointCloud(*hull.cloud_hull, *hull.cloud_hull, mat);

  // Transform edges
  const Eigen::Affine3f mat_affine = Eigen::Affine3f(mat);
  for (auto &edge_pair : hull.edges) {
    edge_pair.first  = pcl::transformPointWithNormal(edge_pair.first, mat_affine);
    edge_pair.second = pcl::transformPointWithNormal(edge_pair.second, mat_affine);
  }

  // Transform polyline barycenter
  transformEigenVector(hull.polyline_barycenter, mat_affine);
}
/*//}*/

/*//{ transformEigenVector() */
void PCL2MapRegistration::transformEigenVector(Eigen::Vector3f &vec, const Eigen::Affine3f &mat) {
  vec = mat * vec;
}
/*//}*/

/*//{ transformEigenVectors() */
void PCL2MapRegistration::transformEigenVectors(EigenVectors &eigenvectors, const Eigen::Matrix4f &mat) {

  const Eigen::Affine3f T_affine = Eigen::Affine3f(mat);

  transformEigenVector(eigenvectors.x, T_affine);
  transformEigenVector(eigenvectors.y, T_affine);
  transformEigenVector(eigenvectors.z, T_affine);
}
/*//}*/

/*//{ translateEigenVector() */
void PCL2MapRegistration::translateEigenVector(Eigen::Vector3f &vec, const Eigen::Matrix4f &mat) {
  vec = vec + Eigen::Vector3f(mat(0, 3), mat(1, 3), mat(2, 3));
}
/*//}*/

/*//{ checkNans */
bool PCL2MapRegistration::checkNans(const PC_NORM::Ptr cloud, const std::string &ns) {
  size_t xyz_invalid_count  = 0;
  size_t norm_invalid_count = 0;
  size_t both_invalid_count = 0;

  for (const auto &point : cloud->points) {

    const bool xyz_invalid  = !(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z));
    const bool norm_invalid = !(std::isfinite(point.normal_x) && std::isfinite(point.normal_y) && std::isfinite(point.normal_z));

    if (xyz_invalid && norm_invalid) {
      both_invalid_count++;
    } else if (xyz_invalid && !norm_invalid) {
      xyz_invalid_count++;
    } else if (norm_invalid && !xyz_invalid) {
      norm_invalid_count++;
    }
  }

  if (xyz_invalid_count > 0 || norm_invalid_count > 0 || both_invalid_count > 0) {
    NODELET_ERROR("[PCL2MapRegistration] Cloud %shas NaNs (xyz+normals: %ld, xyz only: %ld, normals only: %ld)", ns.empty() ? " " : (ns + " ").c_str(),
                  both_invalid_count, xyz_invalid_count, norm_invalid_count);
    return true;
  }

  NODELET_INFO("[PCL2MapRegistration] Cloud has no NaNs.");
  return false;
}
/*//}*/

/*//{ print */
void PCL2MapRegistration::print(const EigenVectors &eigenvectors, const std::string &ns) {

  if (!ns.empty()) {
    NODELET_INFO("[PCL2MapRegistration] Eigenvectors of %s", ns.c_str());
  } else {
    NODELET_INFO("[PCL2MapRegistration] Eigenvectors:");
  }
  NODELET_INFO("[PCL2MapRegistration]   x: (%.1f, %.1f, %.1f)", eigenvectors.x.x(), eigenvectors.x.y(), eigenvectors.x.z());
  NODELET_INFO("[PCL2MapRegistration]   y: (%.1f, %.1f, %.1f)", eigenvectors.y.x(), eigenvectors.y.y(), eigenvectors.y.z());
  NODELET_INFO("[PCL2MapRegistration]   z: (%.1f, %.1f, %.1f)", eigenvectors.z.x(), eigenvectors.z.y(), eigenvectors.z.z());
}
/*//}*/

/*//{ toGeometryMsg */
geometry_msgs::Point PCL2MapRegistration::toGeometryMsg(const Eigen::Vector3f &point) {
  geometry_msgs::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  geom_point.z = point.z();
  return geom_point;
}
/*//}*/

/*//{ toColorMsg */
std_msgs::ColorRGBA PCL2MapRegistration::toColorMsg(const Eigen::Vector3f &rgb, const float alpha) {
  std_msgs::ColorRGBA color_msg;
  color_msg.r = rgb.x();
  color_msg.g = rgb.y();
  color_msg.b = rgb.z();
  color_msg.a = alpha;
  return color_msg;
}
/*//}*/

}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCL2MapRegistration, nodelet::Nodelet);
