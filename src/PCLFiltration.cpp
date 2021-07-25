#include "mrs_pcl_tools/PCLFiltration.h"
#include <algorithm>
#include <cmath>
#include <pcl/pcl_base.h>

namespace mrs_pcl_tools
{

/* onInit() //{ */
void PCLFiltration::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Set PCL verbosity level to errors and higher
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  // Get parameters from config file
  mrs_lib::ParamLoader param_loader(nh, "PCLFiltration");

  /* 3D LIDAR */
  param_loader.loadParam("lidar3d/vfov", _lidar3d_vfov, M_PI_2);
  param_loader.loadParam("lidar3d/row_samples", _lidar3d_row_samples, 1024);
  param_loader.loadParam("lidar3d/pcl2_over_max_range", _lidar3d_pcl2_over_max_range, false);
  param_loader.loadParam("lidar3d/min_range", _lidar3d_min_range_sq, 0.4f);
  param_loader.loadParam("lidar3d/max_range", _lidar3d_max_range_sq, 100.0f);
  _lidar3d_min_range_mm = _lidar3d_min_range_sq * 1000;
  _lidar3d_max_range_mm = _lidar3d_max_range_sq * 1000;
  _lidar3d_min_range_sq *= _lidar3d_min_range_sq;
  _lidar3d_max_range_sq *= _lidar3d_max_range_sq;

  param_loader.loadParam("lidar3d/filter/intensity/enable", _lidar3d_filter_intensity_en, false);
  param_loader.loadParam("lidar3d/filter/intensity/threshold", _lidar3d_filter_intensity_thrd, std::numeric_limits<int>::max());
  param_loader.loadParam("lidar3d/filter/intensity/range", _lidar3d_filter_intensity_range_sq, std::numeric_limits<float>::max());
  _lidar3d_filter_intensity_range_mm = _lidar3d_filter_intensity_range_sq * 1000;
  _lidar3d_filter_intensity_range_sq *= _lidar3d_filter_intensity_range_sq;

  param_loader.loadParam("lidar3d/filter/sor/global/enable", _lidar3d_filter_sor_global_en, false);
  param_loader.loadParam("lidar3d/filter/sor/global/neighbors", _lidar3d_filter_sor_global_neighbors, std::numeric_limits<int>::max());
  param_loader.loadParam("lidar3d/filter/sor/global/stddev", _lidar3d_filter_sor_global_stddev, 1.0);

  param_loader.loadParam("lidar3d/filter/sor/local/enable", _lidar3d_filter_sor_local_en, false);
  param_loader.loadParam("lidar3d/filter/sor/local/range", _lidar3d_filter_sor_local_range, std::numeric_limits<double>::max());
  param_loader.loadParam("lidar3d/filter/sor/local/close/neighbors", _lidar3d_filter_sor_local_close_neighbors, std::numeric_limits<int>::max());
  param_loader.loadParam("lidar3d/filter/sor/local/close/stddev", _lidar3d_filter_sor_local_close_stddev, 1.0);
  param_loader.loadParam("lidar3d/filter/sor/local/distant/neighbors", _lidar3d_filter_sor_local_distant_neighbors, std::numeric_limits<int>::max());
  param_loader.loadParam("lidar3d/filter/sor/local/distant/stddev", _lidar3d_filter_sor_local_distant_stddev, 1.0);

  param_loader.loadParam("lidar3d/filter/ror/enable", _lidar3d_filter_ror_en, false);
  param_loader.loadParam("lidar3d/filter/ror/neighbors", _lidar3d_filter_ror_neighbors, 0);
  param_loader.loadParam("lidar3d/filter/ror/radius", _lidar3d_filter_ror_radius, 1.0);

  param_loader.loadParam("lidar3d/over_max_range/filter/sor/enable", _lidar3d_over_max_range_filter_sor_global_en, false);
  param_loader.loadParam("lidar3d/over_max_range/filter/sor/neighbors", _lidar3d_over_max_range_filter_sor_global_neighbors, 1);
  param_loader.loadParam("lidar3d/over_max_range/filter/sor/stddev", _lidar3d_over_max_range_filter_sor_global_stddev, 1.0);

  param_loader.loadParam("lidar3d/filter/fog/enable", _fog_detector_en, false);

  param_loader.loadParam("lidar3d/filter/fog/volumetric_test/segment_count", _fog_detector_volumetric_test_segment_count, 4);
  param_loader.loadParam("lidar3d/filter/fog/volumetric_test/points_ratio_thrd", _fog_detector_volumetric_test_points_ratio_threshold, 0.0);
  param_loader.loadParam("lidar3d/filter/fog/volumetric_test/voxel_resolution", _fog_detector_volumetric_test_voxel_resolution, 0.0);

  param_loader.loadParam("lidar3d/filter/fog/statistical_test/nn_k", _fog_detector_mean_k, 5);
  param_loader.loadParam("lidar3d/filter/fog/statistical_test/subt_mean", _fog_detector_mean_exp, 0.0);
  param_loader.loadParam("lidar3d/filter/fog/statistical_test/subt_stddev", _fog_detector_stddev_exp, 0.0);
  param_loader.loadParam("lidar3d/filter/fog/statistical_test/z_test_prob_thrd", _fog_detector_z_test_prob_thrd, 0.7);
  /* param_loader.loadParam("lidar3d/filter/fog/mean/thrd", _fog_detector_mean_thrd, std::numeric_limits<double>::max()); */
  /* param_loader.loadParam("lidar3d/filter/fog/stddev/thrd", _fog_detector_stddev_thrd, std::numeric_limits<double>::max()); */

  _sub_lidar3d                = nh.subscribe("lidar3d_in", 10, &PCLFiltration::lidar3dCallback, this, ros::TransportHints().tcpNoDelay());
  _pub_lidar3d                = nh.advertise<sensor_msgs::PointCloud2>("lidar3d_out", 10);
  _pub_lidar3d_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("lidar3d_over_max_range_out", 10);

  NODELET_INFO("[PCLFiltration]: Subscribed to topic: %s", _sub_lidar3d.getTopic().c_str());
  NODELET_INFO("[PCLFiltration]: Publishing at topic: %s", _pub_lidar3d.getTopic().c_str());
  NODELET_INFO("[PCLFiltration]: Publishing at topic: %s", _pub_lidar3d_over_max_range.getTopic().c_str());

  /* MAV type */
  std::string mav_type_name;
  param_loader.loadParam("mav_type", mav_type_name);

  _mav_type       = std::make_shared<MAVType>();
  _mav_type->name = mav_type_name;

  if (mav_type_name == "MARBLE_QAV500") {
    _mav_type->lidar_col_step                  = 1;
    _mav_type->lidar_row_step                  = 4;
    _mav_type->process_cameras                 = true;
    _mav_type->filter_out_projected_self_frame = true;
    _mav_type->skip_nth_lidar_frame            = 4;               // 20 Hz -> 15 Hz
    _mav_type->keep_nth_vert_camera_frame      = 3;               // 30 Hz -> 10 Hz
    _mav_type->keep_nth_front_camera_frame     = 3;               // 30 Hz -> 10 Hz
    _mav_type->hfov_vert_camera                = 1.08210414f;     // 62deg hfov, 224 px width
    _mav_type->hfov_front_camera               = 1.51843645f;     // 87deg hfov, 640 px width
    _mav_type->focal_length_vert_camera        = 186.4f;          // 62deg hfov, 224 px width
    _mav_type->focal_length_front_camera       = 303.668661581f;  // 87deg hfov, 640 px width
    _mav_type->vert_camera_max_range           = 4.0f;
    _mav_type->front_camera_max_range          = 10.0f;
  } else if (mav_type_name == "CTU_CRAS_NORLAB_X500") {
    _mav_type->lidar_col_step                  = 1;
    _mav_type->lidar_row_step                  = 1;
    _mav_type->process_cameras                 = true;
    _mav_type->filter_out_projected_self_frame = true;
    _mav_type->skip_nth_lidar_frame            = 0;
    _mav_type->keep_nth_vert_camera_frame      = 1;
    _mav_type->keep_nth_front_camera_frame     = 1;
    _mav_type->hfov_vert_camera                = 1.50098f;
    _mav_type->hfov_front_camera               = 1.50098f;
    _mav_type->focal_length_vert_camera        = 343.158f;  // hfov -> 1.50098
    _mav_type->focal_length_front_camera       = 343.158f;  //
    _mav_type->vert_camera_max_range           = 10.0f;
    _mav_type->front_camera_max_range          = 10.0f;
  } else if (mav_type_name == "EXPLORER_DS1") {
    _mav_type->lidar_col_step                  = 1;
    _mav_type->lidar_row_step                  = 1;
    _mav_type->process_cameras                 = true;
    _mav_type->filter_out_projected_self_frame = false;
    _mav_type->skip_nth_lidar_frame            = 0;  // 10 Hz
    _mav_type->keep_nth_vert_camera_frame      = 2;  // 20 Hz -> 10 Hz
    _mav_type->keep_nth_front_camera_frame     = 2;  // 20 Hz -> 10 Hz
    _mav_type->hfov_vert_camera                = 1.0472f;
    _mav_type->hfov_front_camera               = 1.0472f;
    _mav_type->focal_length_vert_camera        = 554.25469;  //
    _mav_type->focal_length_front_camera       = 554.25469;  //
    _mav_type->vert_camera_max_range           = 10.0f;
    _mav_type->front_camera_max_range          = 10.0f;
  } else if (mav_type_name == "SSCI_X4") {
    _mav_type->lidar_col_step                  = 1;
    _mav_type->lidar_row_step                  = 1;
    _mav_type->process_cameras                 = false;
    _mav_type->filter_out_projected_self_frame = false;
    _mav_type->skip_nth_lidar_frame            = 0;     // 10 Hz
    _mav_type->keep_nth_vert_camera_frame      = 0;     // no cam
    _mav_type->keep_nth_front_camera_frame     = 0;     // no cam
    _mav_type->hfov_vert_camera                = 0.0f;  // no cam
    _mav_type->hfov_front_camera               = 0.0f;  // no cam
    _mav_type->focal_length_vert_camera        = 0.0f;  // no cam
    _mav_type->focal_length_front_camera       = 0.0f;  // no cam
    _mav_type->vert_camera_max_range           = 0.0f;  // no cam
    _mav_type->front_camera_max_range          = 0.0f;  // no cam
  } else {
    NODELET_ERROR("[PCLFiltration] Unsupported MAV type: %s.", mav_type_name.c_str());
    ros::shutdown();
  }
  NODELET_INFO("[PCLFiltration]: Loaded MAV type: %s", _mav_type->name.c_str());

  /* RGBD / ToF cameras */
  if (_mav_type->process_cameras) {

    /* Upper camera */
    _camera_up                      = std::make_shared<Camera>();
    _camera_up->name                = std::string("camera-top");
    _camera_up->hfov                = _mav_type->hfov_vert_camera;
    _camera_up->focal_length        = _mav_type->focal_length_vert_camera;
    _camera_up->max_range           = _mav_type->vert_camera_max_range;
    _camera_up->detect_landing_area = false;
    _camera_up->keep_nth_frame      = _mav_type->keep_nth_vert_camera_frame;
    _camera_up->data_frame          = 0;
    param_loader.loadParam("camera/up/pcl2_over_max_range", _camera_up->publish_pcl2_over_max_range, false);
    param_loader.loadParam("camera/up/downsample_scale", _camera_up->downsample_scale, 0);
    param_loader.loadParam("camera/up/min_range", _camera_up->min_range, 0.1f);
    param_loader.loadParam("camera/up/voxel_grid_resolution", _camera_up->voxel_grid_resolution, 0.0f);

    _camera_up->sub_image = nh.subscribe<sensor_msgs::Image>("camera_up_in", 1, boost::bind(&PCLFiltration::callbackCameraImage, this, _1, _camera_up));
    _camera_up->pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("camera_up_out", 1);
    _camera_up->pub_cloud_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("camera_up_over_max_range_out", 1);

    /* Bottom camera */
    _camera_down                      = std::make_shared<Camera>();
    _camera_down->name                = std::string("camera-bottom");
    _camera_down->hfov                = _mav_type->hfov_vert_camera;
    _camera_down->focal_length        = _mav_type->focal_length_vert_camera;
    _camera_down->max_range           = _mav_type->vert_camera_max_range;
    _camera_down->detect_landing_area = false;  // TODO: detect landing are should be set here to true
    _camera_down->keep_nth_frame      = _mav_type->keep_nth_vert_camera_frame;
    _camera_down->data_frame          = 0;
    param_loader.loadParam("camera/down/pcl2_over_max_range", _camera_down->publish_pcl2_over_max_range, false);
    param_loader.loadParam("camera/down/downsample_scale", _camera_down->downsample_scale, 0);
    param_loader.loadParam("camera/down/min_range", _camera_down->min_range, 0.1f);
    param_loader.loadParam("camera/down/voxel_grid_resolution", _camera_down->voxel_grid_resolution, 0.0f);

    param_loader.loadParam("camera/down/ground_detection/square_size", _ground_detection_square_size, 0.8f);
    param_loader.loadParam("camera/down/ground_detection/ransac_dist_thrd", _ground_detection_ransac_distance_thrd, 0.05f);
    param_loader.loadParam("camera/down/ground_detection/n_z_max_diff", _ground_detection_n_z_max_diff, 0.1f);

    _camera_down->sub_image = nh.subscribe<sensor_msgs::Image>("camera_down_in", 1, boost::bind(&PCLFiltration::callbackCameraImage, this, _1, _camera_down));
    _camera_down->pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("camera_down_out", 1);
    _camera_down->pub_cloud_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("camera_down_over_max_range_out", 1);
    if (_camera_down->detect_landing_area) {
      _camera_down->pub_landing_spot = nh.advertise<darpa_mrs_msgs::LandingSpot>("landing_feasible_out", 1);
    }

    /* Front camera */
    _camera_front                      = std::make_shared<Camera>();
    _camera_front->name                = std::string("camera-front");
    _camera_front->hfov                = _mav_type->hfov_front_camera;
    _camera_front->focal_length        = _mav_type->focal_length_front_camera;
    _camera_front->max_range           = _mav_type->front_camera_max_range;
    _camera_front->detect_landing_area = false;
    _camera_front->keep_nth_frame      = _mav_type->keep_nth_front_camera_frame;
    _camera_front->data_frame          = 0;
    param_loader.loadParam("camera/front/pcl2_over_max_range", _camera_front->publish_pcl2_over_max_range, false);
    param_loader.loadParam("camera/front/downsample_scale", _camera_front->downsample_scale, 0);
    param_loader.loadParam("camera/front/min_range", _camera_front->min_range, 0.1f);
    param_loader.loadParam("camera/front/voxel_grid_resolution", _camera_front->voxel_grid_resolution, 0.0f);

    _camera_front->sub_image =
        nh.subscribe<sensor_msgs::Image>("camera_front_in", 1, boost::bind(&PCLFiltration::callbackCameraImage, this, _1, _camera_front));
    _camera_front->pub_cloud                = nh.advertise<sensor_msgs::PointCloud2>("camera_front_out", 1);
    _camera_front->pub_cloud_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("camera_front_over_max_range_out", 1);

    NODELET_INFO("[PCLFiltration] Processing cameras at topics:");
    NODELET_INFO("[PCLFiltration]     top: %s", _camera_up->sub_image.getTopic().c_str());
    NODELET_INFO("[PCLFiltration]  bottom: %s", _camera_down->sub_image.getTopic().c_str());
    NODELET_INFO("[PCLFiltration]   front: %s", _camera_front->sub_image.getTopic().c_str());
  }

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCLFiltration]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  _pub_fog_detection   = nh.advertise<darpa_mrs_msgs::FogDetection>("fog_detection_out", 1);
  _timer_fog_detection = nh.createTimer(ros::Rate(5.0), &PCLFiltration::fogDetectionTimer, this);

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh));
  /* reconfigure_server_->updateConfig(last_drs_config); */
  ReconfigureServer::CallbackType f = boost::bind(&PCLFiltration::callbackReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  NODELET_INFO_ONCE("[PCLFiltration] Nodelet initialized");

  is_initialized = true;
}
//}

/* callbackReconfigure() //{ */
void PCLFiltration::callbackReconfigure(Config &config, [[maybe_unused]] uint32_t level) {
  if (!is_initialized) {
    return;
  }
  NODELET_INFO("[PCLFiltration] Reconfigure callback.");

  _lidar3d_filter_intensity_en       = config.lidar3d_filter_intensity_en;
  _lidar3d_filter_intensity_thrd     = config.lidar3d_filter_intensity_thrd;
  _lidar3d_filter_intensity_range_sq = std::pow(config.lidar3d_filter_intensity_rng, 2);

  _lidar3d_filter_sor_global_en        = config.lidar3d_filter_sor_global_en;
  _lidar3d_filter_sor_global_neighbors = config.lidar3d_filter_sor_global_neighbors;
  _lidar3d_filter_sor_global_stddev    = config.lidar3d_filter_sor_global_stddev;

  _lidar3d_filter_sor_local_en                = config.lidar3d_filter_sor_local_en;
  _lidar3d_filter_sor_local_range             = config.lidar3d_filter_sor_local_range;
  _lidar3d_filter_sor_local_close_neighbors   = config.lidar3d_filter_sor_local_close_neighbors;
  _lidar3d_filter_sor_local_close_stddev      = config.lidar3d_filter_sor_local_close_stddev;
  _lidar3d_filter_sor_local_distant_neighbors = config.lidar3d_filter_sor_local_distant_neighbors;
  _lidar3d_filter_sor_local_distant_stddev    = config.lidar3d_filter_sor_local_distant_stddev;

  _lidar3d_filter_ror_en        = config.lidar3d_filter_ror_en;
  _lidar3d_filter_ror_neighbors = config.lidar3d_filter_ror_neighbors;
  _lidar3d_filter_ror_radius    = config.lidar3d_filter_ror_radius;

  _fog_detector_en = config.fog_detector_en;

  _fog_detector_volumetric_test_segment_count          = config.fog_detector_volumetric_test_segment_count;
  _fog_detector_volumetric_test_points_ratio_threshold = config.fog_detector_volumetric_test_points_ratio_threshold;
  _fog_detector_volumetric_test_voxel_resolution       = config.fog_detector_volumetric_test_voxel_resolution;

  _fog_detector_mean_k           = config.fog_detector_statistical_test_mean_k;
  _fog_detector_mean_exp         = config.fog_detector_statistical_test_mean_exp;
  _fog_detector_stddev_exp       = config.fog_detector_statistical_test_stddev_exp;
  _fog_detector_z_test_prob_thrd = config.fog_detector_statistical_test_z_test_prob_thrd;
  /* _fog_detector_mean_thrd   = config.fog_detector_mean_thrd; */
  /* _fog_detector_stddev_thrd = config.fog_detector_stddev_thrd; */
}
//}

/* lidar3dCallback() //{ */
void PCLFiltration::lidar3dCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  if (!is_initialized) {
    return;
  }

  _lidar3d_frame++;

  if (_mav_type->skip_nth_lidar_frame > 1 && _lidar3d_frame % _mav_type->skip_nth_lidar_frame == 0) {
    return;
  }

  if (msg->width % _mav_type->lidar_col_step != 0 || msg->height % _mav_type->lidar_row_step != 0) {
    NODELET_WARN(
        "[PCLFiltration] Step-based downsampling of 3D lidar data would create nondeterministic results. Data (w: %d, h: %d) with downsampling step (w: %d, "
        "h: %d) would leave some samples untouched. Skipping lidar frame.",
        msg->width, msg->height, _mav_type->lidar_col_step, _mav_type->lidar_row_step);
    return;
  }

  TicToc t;

  unsigned int points_before = msg->height * msg->width;
  unsigned int points_after  = 0;

  PC_OS::Ptr cloud;
  PC_OS::Ptr cloud_over_max_range;

  if (hasField("range", msg) && hasField("ring", msg) && hasField("t", msg)) {
    NODELET_INFO_ONCE("[PCLFiltration] Subscribing 3D LIDAR messages. Point type: ouster_ros::OS1::PointOS1. MAV type: %s.", _mav_type->name.c_str());
    removeCloseAndFarPointCloudOS1(cloud, cloud_over_max_range, msg, _lidar3d_pcl2_over_max_range, _lidar3d_min_range_mm, _lidar3d_max_range_mm,
                                   _lidar3d_filter_intensity_en, _lidar3d_filter_intensity_range_mm, _lidar3d_filter_intensity_thrd);
  } else {
    NODELET_INFO_ONCE("[PCLFiltration] Subscribing 3D LIDAR messages. Point type: pcl::PointXYZI. MAV type: %s.", _mav_type->name.c_str());
    removeCloseAndFarPointCloud(cloud, cloud_over_max_range, points_after, msg, _lidar3d_pcl2_over_max_range, _lidar3d_min_range_sq, _lidar3d_max_range_sq);
  }


  // Convert ouster_ros cloud to XYZ only
  PC::Ptr cloud_xyz = boost::make_shared<PC>();
  PC::Ptr cloud_xyz_over_max_range;
  copyCloudOS2XYZ(cloud, cloud_xyz);

  boost::shared_ptr<std::vector<int>> indices_close;
  boost::shared_ptr<std::vector<int>> indices_distant;
  if (_lidar3d_filter_sor_local_en || _fog_detector_en) {
    indices_close   = boost::make_shared<std::vector<int>>();
    indices_distant = boost::make_shared<std::vector<int>>();
    splitCloudByRange(cloud_xyz, indices_close, indices_distant, _lidar3d_filter_sor_local_range);
  }
  if (_lidar3d_pcl2_over_max_range && _lidar3d_over_max_range_filter_sor_global_en) {
    cloud_xyz_over_max_range = boost::make_shared<PC>();
    copyCloudOS2XYZ(cloud_over_max_range, cloud_xyz_over_max_range);
  }

  // Run filters in parallel
  auto thr_ror_global = std::async(getInvalidIndicesRorFilter, _lidar3d_filter_ror_en, cloud_xyz, _lidar3d_filter_ror_radius, _lidar3d_filter_ror_neighbors);

  auto thr_sor_global =
      std::async(getInvalidIndicesSorFilter, _lidar3d_filter_sor_global_en, cloud_xyz, _lidar3d_filter_sor_global_neighbors, _lidar3d_filter_sor_global_stddev);

  auto thr_sor_local_close = std::async(getInvalidIndicesSorFilterIndices, _lidar3d_filter_sor_local_en, cloud_xyz, indices_close,
                                        _lidar3d_filter_sor_local_close_neighbors, _lidar3d_filter_sor_local_close_stddev);

  auto thr_sor_local_distant = std::async(getInvalidIndicesSorFilterIndices, _lidar3d_filter_sor_local_en, cloud_xyz, indices_distant,
                                          _lidar3d_filter_sor_local_distant_neighbors, _lidar3d_filter_sor_local_distant_stddev);

  auto thr_sor_global_over_max_range = std::async(getInvalidIndicesSorFilter, _lidar3d_over_max_range_filter_sor_global_en, cloud_xyz_over_max_range,
                                                  _lidar3d_over_max_range_filter_sor_global_neighbors, _lidar3d_over_max_range_filter_sor_global_stddev);

  /* std::thread thr_fog_detection(&PCLFiltration::detectFogInLidarDataStatisticsTest, this, cloud_xyz, indices_close, _lidar3d_filter_sor_local_range, */
  /*                               msg->header.stamp); */
  std::thread thr_fog_detection(&PCLFiltration::detectFogInLidarDataVolumetricTest, this, cloud_xyz, indices_close, _lidar3d_filter_sor_local_range,
                                msg->header.stamp, _fog_detector_volumetric_test_segment_count, _fog_detector_volumetric_test_points_ratio_threshold,
                                _fog_detector_volumetric_test_voxel_resolution);

  // Apply all filters
  invalidatePointsAtIndices(thr_sor_local_close.get(), cloud);
  invalidatePointsAtIndices(thr_sor_local_distant.get(), cloud);
  invalidatePointsAtIndices(thr_sor_global.get(), cloud);
  invalidatePointsAtIndices(thr_ror_global.get(), cloud);
  invalidatePointsAtIndices(thr_sor_global_over_max_range.get(), cloud_over_max_range);

  publishCloud(_pub_lidar3d, *cloud);

  if (_lidar3d_pcl2_over_max_range) {
    publishCloud(_pub_lidar3d_over_max_range, *cloud_over_max_range);
  }

  // Wait for fog detection finish
  thr_fog_detection.join();

  NODELET_INFO_THROTTLE(
      5.0, "[PCLFiltration] Processed 3D LIDAR data (run time: %0.1f ms; points before: %d, after: %d; dim before: (w: %d, h: %d), after: (w: %d, h: %d)).",
      t.toc(), points_before, points_after, msg->width, msg->height, cloud->width, cloud->height);
}
//}

/* callbackCameraImage() //{ */
void PCLFiltration::callbackCameraImage(const sensor_msgs::Image::ConstPtr &depth_msg, const std::shared_ptr<Camera> &camera) {

  if (!is_initialized || !_mav_type->process_cameras) {
    return;
  }

  camera->data_frame++;
  if (camera->keep_nth_frame > 1 && camera->data_frame % camera->keep_nth_frame != 0) {
    return;
  }

  NODELET_INFO_ONCE("[PCLFiltration] Subscribing %s messages. Camera data will %s used to detect landing area.", camera->name.c_str(),
                    camera->detect_landing_area ? "be" : "not be");

  TicToc t;

  // DONE: Check if data contain NaNs or 0s and then replace these data with an over-the-max-range data to allow space-freeing
  const std::pair<PC::Ptr, PC::Ptr> clouds_both = imageToPcFiltered(depth_msg, camera, 50.0f);

  // Voxel grid sampling
  if (camera->voxel_grid_resolution > 0.0f) {
    /* NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] \t - Applied voxel sampling filter (res: %0.2f m).", camera->voxel_grid_resolution); */
    pcl::VoxelGrid<pt_XYZ> vg;
    vg.setInputCloud(clouds_both.first);
    vg.setLeafSize(camera->voxel_grid_resolution, camera->voxel_grid_resolution, camera->voxel_grid_resolution);
    vg.filter(*clouds_both.first);
  }

  const bool in_fog = detectFogInDepthData(clouds_both.first, camera, 0.15f, depth_msg->width, depth_msg->height, camera->voxel_grid_resolution);
  if (in_fog) {
    NODELET_INFO_THROTTLE(0.5, "[PCLFiltration] Camera detects fog. Processed data will not be published out.");
    return;
  }

  NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] Processed camera data (run time: %0.1f ms; points before: %d, after: %ld, over_max_range: %ld).", t.toc(),
                        depth_msg->height * depth_msg->width, clouds_both.first->points.size(), clouds_both.second->points.size());

  // Detect landing area
  if (camera->detect_landing_area && camera->pub_landing_spot.getNumSubscribers() > 0) {

    const int8_t ground = detectGround(clouds_both.first);

    // TODO: probably make this as service only
    // Publish landing spot msg
    try {
      darpa_mrs_msgs::LandingSpot::Ptr landing_spot_msg = boost::make_shared<darpa_mrs_msgs::LandingSpot>();
      landing_spot_msg->stamp                           = depth_msg->header.stamp;
      landing_spot_msg->landing_spot                    = ground;

      camera->pub_landing_spot.publish(landing_spot_msg);
    }
    catch (...) {
      NODELET_ERROR("[PCLFiltration]: Exception caught during publishing on topic: %s", camera->pub_landing_spot.getTopic().c_str());
    }
  }

  // Publish data
  publishCloud(camera->pub_cloud, *clouds_both.first);

  // Publish data over max range
  if (camera->publish_pcl2_over_max_range) {

    publishCloud(camera->pub_cloud_over_max_range, *clouds_both.second);
  }
}
//}

/*//{ removeCloseAndFarPointCloud() */
void PCLFiltration::removeCloseAndFarPointCloud(PC_OS::Ptr &cloud_out, PC_OS::Ptr &cloud_over_max_range, unsigned int &valid_points,
                                                const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range, const float &min_range_sq,
                                                const float &max_range_sq) {

  const unsigned int w          = msg->width / _mav_type->lidar_col_step;
  const unsigned int h          = msg->height / _mav_type->lidar_row_step;
  const unsigned int cloud_size = w * h;

  // Convert to pcl object
  PC_OS::Ptr cloud_in = boost::make_shared<PC_OS>();
  pcl::fromROSMsg(*msg, *cloud_in);

  cloud_out           = boost::make_shared<PC_OS>(w, h);
  cloud_out->width    = w;
  cloud_out->height   = h;
  cloud_out->is_dense = false;
  cloud_out->header   = cloud_in->header;

  size_t count_over = 0;

  if (ret_cloud_over_max_range) {
    cloud_over_max_range         = boost::make_shared<PC_OS>();
    cloud_over_max_range->header = cloud_in->header;
    cloud_over_max_range->points.resize(cloud_size);
  }

  for (unsigned int j = _mav_type->lidar_col_step - 1; j < msg->width; j += _mav_type->lidar_col_step) {
    const unsigned int c               = j / _mav_type->lidar_col_step;
    const float        point_intensity = float(c) / float(msg->width);

    for (unsigned int i = _mav_type->lidar_row_step - 1; i < msg->height; i += _mav_type->lidar_row_step) {
      const unsigned int r = i / _mav_type->lidar_row_step;

      pt_OS       point    = cloud_in->at(j, i);
      const float range_sq = point.x * point.x + point.y * point.y + point.z * point.z;

      point.range     = std::sqrt(range_sq) * 1000.0f;  // in mm
      point.intensity = point_intensity;
      point.ring      = r;

      if (!point.getArray4fMap().allFinite()) {
        invalidatePoint(point);
      } else if (range_sq < min_range_sq) {
        invalidatePoint(point);
      } else if (range_sq <= max_range_sq) {

        bool self_projection = false;
        if (_mav_type->filter_out_projected_self_frame) {
          // SUBT HOTFIX
          const float x = std::fabs(point.x);
          const float y = std::fabs(point.y);
          const float z = std::fabs(point.z);
          if ((z > subt_frame_z_lower && z < subt_frame_z_upper) &&
              ((x > subt_frame_x_lower && x < subt_frame_x_upper) || (y > subt_frame_y_lower && y < subt_frame_y_upper))) {
            self_projection = true;
            invalidatePoint(point);
          }
        }

        if (!self_projection) {
          valid_points++;
        }

      } else {

        if (ret_cloud_over_max_range) {
          cloud_over_max_range->points.at(count_over++) = point;
        }

        invalidatePoint(point);
      }

      cloud_out->at(c, r) = point;
    }
  }

  // Resize
  if (ret_cloud_over_max_range) {
    if (count_over != cloud_size) {
      cloud_over_max_range->points.resize(count_over);
    }
    cloud_over_max_range->height   = 1;
    cloud_over_max_range->width    = static_cast<uint32_t>(count_over);
    cloud_over_max_range->is_dense = true;
  }
}
/*//}*/

/*//{ removeCloseAndFarPointCloudOS1() */
void PCLFiltration::removeCloseAndFarPointCloudOS1(PC_OS::Ptr &cloud, PC_OS::Ptr &cloud_over_max_range, const sensor_msgs::PointCloud2::ConstPtr &msg,
                                                   const bool &ret_cloud_over_max_range, const uint32_t &min_range_mm, const uint32_t &max_range_mm,
                                                   const bool &filter_intensity, const uint32_t &filter_intensity_range_mm, const int &filter_intensity_thrd) {

  // Convert to pcl object
  pcl::fromROSMsg(*msg, *cloud);

  size_t j          = 0;
  size_t k          = 0;
  size_t cloud_size = cloud->points.size();

  if (ret_cloud_over_max_range) {
    cloud_over_max_range         = boost::make_shared<PC_OS>();
    cloud_over_max_range->header = cloud->header;
    cloud_over_max_range->points.resize(cloud_size);
  }

  for (size_t i = 0; i < cloud_size; i++) {

    uint32_t range_mm = cloud->points.at(i).range;

    if (range_mm < min_range_mm) {
      continue;

    } else if (range_mm <= max_range_mm) {

      // Filter out low intensities in close radius
      if (filter_intensity && cloud->points.at(i).intensity < filter_intensity_thrd && range_mm < filter_intensity_range_mm) {
        continue;
      }

      cloud->points.at(j++) = cloud->points.at(i);

    } else if (ret_cloud_over_max_range) {
      cloud_over_max_range->points.at(k++) = cloud->points.at(i);
    }
  }

  // Resize both clouds
  if (j != cloud_size) {
    cloud->points.resize(j);
  }
  cloud->height   = 1;
  cloud->width    = static_cast<uint32_t>(j);
  cloud->is_dense = true;

  if (ret_cloud_over_max_range) {
    if (k != cloud_size) {
      cloud_over_max_range->points.resize(k);
    }
    cloud_over_max_range->height   = 1;
    cloud_over_max_range->width    = static_cast<uint32_t>(k);
    cloud_over_max_range->is_dense = true;
  }
}
/*//}*/

/*//{ removeCloseAndFarPointCloudXYZ() */
std::pair<PC::Ptr, PC::Ptr> PCLFiltration::removeCloseAndFarPointCloudXYZ(const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range,
                                                                          const float &min_range_sq, const float &max_range_sq) {

  // Convert to pcl object
  PC::Ptr cloud                = boost::make_shared<PC>();
  PC::Ptr cloud_over_max_range = boost::make_shared<PC>();
  pcl::fromROSMsg(*msg, *cloud);

  size_t       j          = 0;
  size_t       k          = 0;
  const size_t cloud_size = cloud->points.size();

  if (ret_cloud_over_max_range) {
    cloud_over_max_range->header = cloud->header;
    cloud_over_max_range->points.resize(cloud_size);
  }

  for (size_t i = 0; i < cloud_size; i++) {

    const float range_sq =
        cloud->points.at(i).x * cloud->points.at(i).x + cloud->points.at(i).y * cloud->points.at(i).y + cloud->points.at(i).z * cloud->points.at(i).z;

    if (range_sq < min_range_sq) {
      continue;

    } else if (range_sq <= max_range_sq) {

      cloud->points.at(j++) = cloud->points.at(i);

    } else if (ret_cloud_over_max_range && cloud->points.at(i).getArray3fMap().allFinite()) {

      cloud_over_max_range->points.at(k++) = cloud->points.at(i);
    }
  }

  // Resize both clouds
  if (j != cloud_size) {
    cloud->points.resize(j);
  }
  cloud->height   = 1;
  cloud->width    = static_cast<uint32_t>(j);
  cloud->is_dense = false;


  if (ret_cloud_over_max_range) {
    if (k != cloud_size) {
      cloud_over_max_range->points.resize(k);
    }
    cloud_over_max_range->header   = cloud->header;
    cloud_over_max_range->height   = 1;
    cloud_over_max_range->width    = static_cast<uint32_t>(k);
    cloud_over_max_range->is_dense = false;
  }

  return std::make_pair(cloud, cloud_over_max_range);
}
/*//}*/

/*//{ downsampleCloud() */
PC::Ptr PCLFiltration::downsampleCloud(const PC::Ptr &cloud, const unsigned int &keep_every_nth_point) {

  NODELET_INFO_ONCE("[PCLFiltration] Downsampling cloud with scale factor: %d", keep_every_nth_point);

  if (keep_every_nth_point <= 1) {
    return cloud;
  }

  const unsigned int height = std::max(1, int(cloud->height / keep_every_nth_point));
  const unsigned int width  = std::max(1, int(cloud->width / keep_every_nth_point));

  /* NODELET_ERROR("[PCLFiltration] cloud in: (h: %d, w: %d), cloud out: (h: %d, w: %d)", cloud->height, cloud->width, height, width); */

  PC::Ptr cloud_downsampled = boost::make_shared<PC>(height, width);
  cloud_downsampled->header = cloud->header;

  int c = 0;
  for (int i = 0; i < cloud->height - 1; i += keep_every_nth_point) {
    int r = 0;
    for (int j = 0; j < cloud->width - 1; j += keep_every_nth_point) {
      cloud_downsampled->at(r++, c) = cloud->at(j, i);
    }
    c++;
  }

  return cloud_downsampled;
}
/*//}*/

/*//{ detectGround() */
int8_t PCLFiltration::detectGround(const PC::Ptr &cloud) {

  // Crop square of landing spot size in data
  PC::Ptr     square = boost::make_shared<PC>();
  const float d      = _ground_detection_square_size / 2.0f;

  // TODO: TEST Filter data by xy distance (keep middle-area square of edge size 2*d)
  pcl::CropBox<pt_XYZ> filter;
  filter.setMin(Eigen::Vector4f(-d, -d, 0.0f, 1.0f));
  filter.setMax(Eigen::Vector4f(d, d, 100.0f, 1.0f));
  filter.setInputCloud(cloud);
  filter.filter(*square);

  if (square->points.size() == 0) {
    return darpa_mrs_msgs::LandingSpot::LANDING_NO_DATA;
  }

  // TODO: Transform data to world

  // TODO: TEST Match planar surface on cropped data
  pcl::ModelCoefficients       coefficients;
  pcl::PointIndices            inliers;
  pcl::SACSegmentation<pt_XYZ> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(_ground_detection_ransac_distance_thrd);
  seg.setInputCloud(square);
  seg.segment(inliers, coefficients);

  if (inliers.indices.size() == 0) {
    return darpa_mrs_msgs::LandingSpot::LANDING_NO_DATA;
  }

  // TODO: Decide if planar surface is ground or not
  const float n_z = std::fabs(coefficients.values[2]);
  if (std::fabs(n_z - 1.0f) < _ground_detection_n_z_max_diff) {
    return darpa_mrs_msgs::LandingSpot::LANDING_SAFE;
  }

  return darpa_mrs_msgs::LandingSpot::LANDING_UNSAFE;
}
/*//}*/

/*//{ imageToPcFiltered() */
std::pair<PC::Ptr, PC::Ptr> PCLFiltration::imageToPcFiltered(const sensor_msgs::Image::ConstPtr &depth_msg, const std::shared_ptr<Camera> &camera,
                                                             const float &nan_depth) {

  PC::Ptr cloud_out                = boost::make_shared<PC>();
  PC::Ptr cloud_over_max_range_out = boost::make_shared<PC>();

  pcl::uint64_t stamp;
  pcl_conversions::toPCL(depth_msg->header.stamp, stamp);

  const int no_of_points = depth_msg->width * depth_msg->height;
  cloud_out->points.resize(no_of_points);
  cloud_over_max_range_out->points.resize(no_of_points);

  cloud_out->header.frame_id                = depth_msg->header.frame_id;
  cloud_out->header.seq                     = depth_msg->header.seq;
  cloud_out->header.stamp                   = stamp;
  cloud_over_max_range_out->header.frame_id = depth_msg->header.frame_id;
  cloud_over_max_range_out->header.seq      = depth_msg->header.seq;
  cloud_over_max_range_out->header.stamp    = stamp;

  // Method inspired by: https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h#L48

  // Use correct principal point from calibration
  const float center_x = depth_msg->width / 2.0f;
  const float center_y = depth_msg->height / 2.0f;

  // Combine unit conversion (not needed) with scaling by focal length for computing (X,Y)
  const float constant_x = 1.0f / camera->focal_length;
  const float constant_y = 1.0f / camera->focal_length;

  const float *depth_row = reinterpret_cast<const float *>(&depth_msg->data[0]);
  const int    row_step  = depth_msg->step / sizeof(float);

  int points_cloud                = 0;
  int points_cloud_over_max_range = 0;

  const float        min_range            = camera->min_range;
  const float        max_range            = camera->max_range;
  const unsigned int keep_every_nth_point = camera->downsample_scale;

  unsigned int row = 0;
  for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step) {

    if (row++ % keep_every_nth_point != 0) {
      continue;
    }

    for (int u = 0; u < (int)depth_msg->width; ++u) {

      if (u % keep_every_nth_point != 0) {
        continue;
      }

      const float depth  = depth_row[u];
      const bool  finite = std::isfinite(depth);

      if (finite && depth > min_range && depth < max_range) {

        cloud_out->points.at(points_cloud++) = imagePointToCloudPoint(u, v, center_x, center_y, depth, constant_x, constant_y);

      } else if (!finite || depth <= 0.0f || depth > max_range) {

        cloud_over_max_range_out->points.at(points_cloud_over_max_range++) =
            imagePointToCloudPoint(u, v, center_x, center_y, nan_depth, constant_x, constant_y);
      }
    }
  }

  cloud_out->width  = points_cloud;
  cloud_out->height = points_cloud > 0 ? 1 : 0;
  if (points_cloud != no_of_points) {
    cloud_out->points.resize(points_cloud);
  }

  cloud_over_max_range_out->width  = points_cloud_over_max_range;
  cloud_over_max_range_out->height = points_cloud_over_max_range > 0 ? 1 : 0;
  if (points_cloud_over_max_range != no_of_points) {
    cloud_over_max_range_out->points.resize(points_cloud_over_max_range);
  }

  return std::make_pair(cloud_out, cloud_over_max_range_out);
}
/*//}*/

/*//{ imagePointToCloudPoint() */
pt_XYZ PCLFiltration::imagePointToCloudPoint(const int &x, const int &y, const float &cx, const float &cy, const float &depth, const float &ifx,
                                             const float &ify) {
  // Rotate from depth-image coordination frame (z forward)
  pt_XYZ pt;
  pt.x = depth;
  pt.y = -(x - cx) * depth * ifx;
  pt.z = -(y - cy) * depth * ify;
  return pt;
}
/*//}*/

/*//{ publishCloud() */
template <typename T>
void PCLFiltration::publishCloud(const ros::Publisher &pub, const pcl::PointCloud<T> cloud) {
  if (pub.getNumSubscribers() > 0 && cloud.points.size() > 0) {
    try {
      sensor_msgs::PointCloud2::Ptr pcl_msg = boost::make_shared<sensor_msgs::PointCloud2>();
      pcl::toROSMsg(cloud, *pcl_msg);
      pub.publish(pcl_msg);
      /* NODELET_DEBUG("[PCLFiltration]: Publishing at time (%0.2f) cloud on topic: %s", pcl_msg->header.stamp.toSec(), pub.getTopic().c_str()); */
    }
    catch (...) {
      NODELET_ERROR("[PCLFiltration]: Exception caught during publishing on topic: %s", pub.getTopic().c_str());
    }
  }
}
/*//}*/

/*//{ invalidatePoint() */
void PCLFiltration::invalidatePoint(pt_OS &point) {
  point.x         = nan;
  point.y         = nan;
  point.z         = nan;
  point.range     = 0;
  point.intensity = 0;
}
/*//}*/

/*//{ detectFogInLidarDataStatisticsTest() */
void PCLFiltration::detectFogInLidarDataStatisticsTest(const PC::Ptr &cloud, const boost::shared_ptr<std::vector<int>> &indices, const float range,
                                                       const ros::Time stamp) {

  if (!_fog_detector_en || indices->empty()) {
    return;
  }

  // Estimate mean and stddev of local data
  float data_mean, data_stddev;
  int   sample_size_data;
  generateNNStatistics(cloud, indices, data_mean, data_stddev, sample_size_data, _fog_detector_mean_k);

  /* const bool in_fog = */
  /*     std::fabs(_fog_detector_mean_exp - mean_data) < _fog_detector_mean_thrd && std::fabs(_fog_detector_stddev_exp - stddev_data) <
   * _fog_detector_stddev_thrd; */

  const double z = zTest(_fog_detector_mean_exp, _fog_detector_stddev_exp, 1, data_mean, data_stddev, sample_size_data);
  {
    std::scoped_lock lock(_mutex_fog_detector_data);

    const double cndf   = zTableLookup(z);
    const bool   in_fog = cndf < _fog_detector_z_test_prob_thrd;

    _fog_detector_lidar_data_mean   = data_mean;
    _fog_detector_lidar_data_stddev = data_stddev;
    _fog_detector_lidar_cndf        = cndf;

    if (in_fog) {
      _fog_detector_lidar_time_last = stamp;
    }
  }
}
/*//}*/

/*//{ detectFogInLidarDataVolumetricTest() */
void PCLFiltration::detectFogInLidarDataVolumetricTest(const PC::Ptr &cloud, const boost::shared_ptr<std::vector<int>> &indices, const float range,
                                                       const ros::Time stamp, const int segment_count, const float points_ratio_thrd,
                                                       const float cloud_resolution) {

  if (!_fog_detector_en || indices->empty() || cloud_resolution <= 0.0f || points_ratio_thrd < 0.0f)
    return;

  // Count max number of points in one segment
  // V: full sphere volume, X: spherical sector volume -> lidar vfov volume = V - 2 * X = V - 2 * (2/3 * pi * r^3 * (1 - cos((pi - vfov) / 2))
  const double segment_volume         = (4.0 / 3.0) * M_PI * std::pow(range, 3) * std::cos((M_PI - _lidar3d_vfov) / 2.0) / double(segment_count);
  const int    max_points_per_segment = segment_volume / std::pow(cloud_resolution, 3);
  if (max_points_per_segment == 0)
    return;

  const double         seg_width = 2.0 * M_PI / double(segment_count);
  std::vector<PC::Ptr> segments(segment_count, boost::make_shared<PC>());

  // Split given data to segments
  for (const auto &i : *indices) {
    const auto &point   = cloud->at(i);
    double      azimuth = std::atan2(point.y, point.x);
    if (azimuth < 0.0)
      azimuth += 2 * M_PI;
    const int segment = std::floor(azimuth / seg_width);

    segments.at(segment)->push_back(point);
  }

  // Voxelize to given resolution and store the maximum volumetric ratio
  float max_ratio             = 0.0f;
  int   max_points_in_segment = 0;

  pcl::VoxelGrid<PC::PointType> vg;
  vg.setLeafSize(cloud_resolution, cloud_resolution, cloud_resolution);

  for (auto &cloud : segments) {

    if (cloud->size() > 1) {

      vg.setInputCloud(cloud);
      vg.filter(*cloud);

      const int   cloud_size    = cloud->size();
      const float segment_ratio = float(cloud_size) / float(max_points_per_segment);
      if (segment_ratio > max_ratio) {
        max_ratio             = segment_ratio;
        max_points_in_segment = cloud_size;
      }
    }
  }

  bool in_fog;
  {
    std::scoped_lock lock(_mutex_fog_detector_data);

    // Apply simple thresholding (a lot of points within the lidar vfov segment means a fog probability is high)
    in_fog                          = max_ratio > points_ratio_thrd;
    _fog_detector_lidar_point_ratio = max_ratio;

    if (in_fog)
      _fog_detector_lidar_time_last = stamp;
  }

  NODELET_INFO_THROTTLE(
      1.0, "[PCLFiltration::detectFogInLidarDataVolumetricTest] in_fog: %s, segment_volume: %0.2f, max_ratio: %0.2f (point count -> true: %d, full: %d)",
      in_fog ? "true" : "false", segment_volume, max_ratio, max_points_in_segment, max_points_per_segment);
}
/*//}*/

/*//{ detectFogInDepthData() */
bool PCLFiltration::detectFogInDepthData(const PC::Ptr &cloud, const std::shared_ptr<Camera> &camera, const float points_ratio_thrd, const int image_width,
                                         const int image_height, const float cloud_resolution) {

  if (!_fog_detector_en || cloud->empty() || cloud_resolution <= 0.0f || points_ratio_thrd < 0.0f)
    return false;

  // Precompute ranges of all points (x-axis is original distance in the depth image)
  std::vector<float> depths(cloud->size());
  float              depth_max;
  for (int i = 0; i < cloud->size(); i++) {
    const auto &depth = cloud->at(i).x;
    depths.at(i)      = depth;
    if (depth > depth_max) {
      depth_max = depth;
    }
  }

  // Compute number of points up to range R
  const float R                = depth_max / 1.5f;
  const int   true_point_count = std::count_if(depths.begin(), depths.end(), [&R](const float d) { return d < R; });

  if (true_point_count == 0) {
    /* NODELET_ERROR("[PCLFiltration::detectFogInDepthData] true point count is 0"); */
    return false;
  }

  // Compute camera vertical field of view
  const float tan_hfov_half = std::tan(camera->hfov / 2.0f);
  const float vfov_half     = std::atan(tan_hfov_half * float(image_height) / float(image_width));

  // Compute pyramid parameters (depth R, base dimensions a/b)
  const float a      = 2.0f * R * tan_hfov_half;
  const float b      = 2.0f * R * std::sin(vfov_half);
  const float volume = a * b * R / 3.0f;

  // Compute the expected number of points in full pyramid volume at the resolution of input cloud
  const int full_point_count = volume / std::pow(cloud_resolution, 3);

  if (full_point_count == 0) {
    /* NODELET_ERROR("[PCLFiltration::detectFogInDepthData] full point count is 0"); */
    return false;
  }

  // Compute the ratio of points in the pyramid
  const float true_points_ratio = float(true_point_count) / float(full_point_count);

  bool in_fog;

  {
    std::scoped_lock lock(_mutex_fog_detector_data);

    // Apply simple thresholding (a lot of points within the pyramid means a fog probability is high)
    in_fog = true_points_ratio > points_ratio_thrd;

    if (in_fog) {
      _fog_detector_depth_time_last   = ros::Time::now();
      _fog_detector_depth_point_ratio = true_points_ratio;
    }
  }

  NODELET_INFO_THROTTLE(1.0, "[PCLFiltration::detectFogInDepthData] in_fog: %s, R: %0.1f, volume: %0.1f, ratio: %0.1f (point count -> true: %d, full: %d)",
                        in_fog ? "true" : "false", R, volume, true_points_ratio, true_point_count, full_point_count);

  return in_fog;
}
/*//}*/

/*//{ generateNNStatistics() */
void PCLFiltration::generateNNStatistics(const PC::Ptr &cloud, const boost::shared_ptr<std::vector<int>> &indices, float &mean, float &stddev, int &sample_size,
                                         const int mean_k) {

  pcl::search::KdTree<pt_XYZ>::Ptr tree = boost::make_shared<pcl::search::KdTree<pt_XYZ>>(false);
  tree->setInputCloud(cloud);

  // Allocate enough space to hold the results
  std::vector<int>   nn_indices(mean_k);
  std::vector<float> nn_dists(mean_k);

  std::vector<float> distances(indices->size(), 0.0f);

  size_t k    = 0;
  sample_size = 0;

  // Go over all the points and calculate the mean or smallest distance
  for (const auto &i : *indices) {

    if (!std::isfinite(cloud->at(i).x) || !std::isfinite(cloud->at(i).y) || !std::isfinite(cloud->at(i).z)) {
      k++;
      continue;
    }

    if (tree->nearestKSearch(i, mean_k, nn_indices, nn_dists) == 0) {
      k++;
      NODELET_WARN("[PCLFiltration::generateNNStatistics] Searching for the closest %d neighbors failed.\n", mean_k);
      continue;
    }

    // Minimum distance (if mean_k == 2) or mean distance
    float dist_sum = 0;
    for (int j = 1; j < mean_k; ++j) {
      dist_sum += sqrt(nn_dists[j]);
    }
    distances[k++] = static_cast<float>(dist_sum / static_cast<float>(mean_k - 1));
    sample_size++;
  }

  // Estimate the mean and the standard deviation of the distance vector
  float sum = 0, sq_sum = 0;
  for (const float &d : distances) {
    sum += d;
    sq_sum += d * d;
  }

  mean                 = sum / static_cast<float>(sample_size);
  const float variance = (sq_sum - sum * sum / static_cast<float>(sample_size)) / (static_cast<float>(sample_size) - 1);
  stddev               = sqrt(variance);
}
/*//}*/

/*//{ copyCloudOS2XYZ() */
void PCLFiltration::copyCloudOS2XYZ(const PC_OS::Ptr &cloud_OS, PC::Ptr &cloud_xyz) {

  cloud_xyz->header   = cloud_OS->header;
  cloud_xyz->height   = 1;
  cloud_xyz->width    = cloud_OS->width * cloud_OS->height;
  cloud_xyz->is_dense = cloud_OS->is_dense;
  cloud_xyz->resize(cloud_OS->size());

  for (int i = 0; i < cloud_OS->size(); i++) {
    cloud_xyz->at(i).x = cloud_OS->at(i).x;
    cloud_xyz->at(i).y = cloud_OS->at(i).y;
    cloud_xyz->at(i).z = cloud_OS->at(i).z;
  }
}
/*//}*/

/*//{ splitCloudByRange() */
void PCLFiltration::splitCloudByRange(const PC::Ptr &cloud, boost::shared_ptr<std::vector<int>> &indices_close,
                                      boost::shared_ptr<std::vector<int>> &indices_distant, const double range) {

  for (int i = 0; i < cloud->size(); i++) {

    const Eigen::Vector3f pt       = cloud->at(i).getArray3fMap();
    const float           pt_range = pt.norm();

    if (pt_range < range) {
      indices_close->push_back(i);
    } else {
      indices_distant->push_back(i);
    }
  }
}
/*//}*/

/*//{ invalidatePointsAtIndices() */
void PCLFiltration::invalidatePointsAtIndices(const pcl::IndicesConstPtr &indices, PC_OS::Ptr &cloud) {
  for (auto it = indices->begin(); it != indices->end(); it++) {
    invalidatePoint(cloud->at(*it));
  }
}
/*//}*/

/*//{ zTest() */
double PCLFiltration::zTest(const double mean_1, const double stddev_1, const int sample_size_1, const double mean_2, const double stddev_2,
                            const int sample_size_2) {
  // Too noisy with sample size included in the formula
  /* return (mean_1 - mean_2) / std::sqrt((stddev_1 * stddev_1 / double(sample_size_1)) + (stddev_2 * stddev_2 / double(sample_size_2))); */
  return (mean_1 - mean_2) / std::sqrt(stddev_1 * stddev_1 + stddev_2 * stddev_2);
}
/*//}*/

/*//{ zTableLookup() */
double PCLFiltration::zTableLookup(const double z) {
  // Return cummulative normal distribution function
  // URL: https://stackoverflow.com/questions/58371163/how-to-change-the-z-value-to-the-one-from-the-table-z-table-from-normal-distrib
  return 0.5 * std::erfc(-z * M_SQRT1_2);
}
/*//}*/

/*//{ fogDetectionTimer() */
void PCLFiltration::fogDetectionTimer(const ros::TimerEvent &event) {

  if (_pub_fog_detection.getNumSubscribers() > 0) {

    const ros::Time now        = ros::Time::now();
    const double    secs_lidar = 0.5;
    const double    secs_depth = 1.5;

    darpa_mrs_msgs::FogDetection::Ptr msg = boost::make_shared<darpa_mrs_msgs::FogDetection>();
    msg->stamp                            = now;

    {
      std::scoped_lock lock(_mutex_fog_detector_data);

      const bool lidar_in_fog = (now - _fog_detector_lidar_time_last).toSec() < secs_lidar;
      const bool depth_in_fog = (now - _fog_detector_depth_time_last).toSec() < secs_depth;

      msg->in_fog                = lidar_in_fog || depth_in_fog;
      msg->lidar_in_fog          = lidar_in_fog;
      msg->depth_in_fog          = depth_in_fog;
      msg->lidar_detection_range = _lidar3d_filter_sor_local_range;
      msg->lidar_point_ratio     = _fog_detector_lidar_point_ratio;
      msg->depth_point_ratio     = _fog_detector_depth_point_ratio;
    }

    try {
      _pub_fog_detection.publish(msg);
    }
    catch (...) {
      NODELET_ERROR("[PCLFiltration]: Exception caught during publishing on topic: %s", _pub_fog_detection.getTopic().c_str());
    }
  }
}
/*//}*/

}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCLFiltration, nodelet::Nodelet);
