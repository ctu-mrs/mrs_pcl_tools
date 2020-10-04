#include "mrs_pcl_tools/PCLFiltration.h"

namespace mrs_pcl_tools
{

/* onInit() //{ */
void PCLFiltration::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Get parameters from config file
  mrs_lib::ParamLoader param_loader(nh, "PCLFiltration");

  /* 3D LIDAR */
  param_loader.loadParam("lidar3d/republish", _lidar3d_republish, false);
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

  /* ToF cameras */
  param_loader.loadParam("top/republish", _tof_republish, true);
  param_loader.loadParam("tof/pcl2_over_max_range", _tof_pcl2_over_max_range, false);
  param_loader.loadParam("tof/min_range", _tof_min_range_sq, 0.1f);
  param_loader.loadParam("tof/max_range", _tof_max_range_sq, 4.0f);
  param_loader.loadParam("tof/downsample_scale", _tof_downsample_scale, 0);
  _tof_min_range_sq *= _tof_min_range_sq;
  _tof_max_range_sq *= _tof_max_range_sq;

  /* Landing spot detection */
  param_loader.loadParam("tof/ground_detection/square_size", _ground_detection_square_size, 0.8f);
  param_loader.loadParam("tof/ground_detection/ransac_dist_thrd", _ground_detection_ransac_distance_thrd, 0.05f);
  param_loader.loadParam("tof/ground_detection/n_z_max_diff", _ground_detection_n_z_max_diff, 0.1f);

  /* Depth camera */
  param_loader.loadParam("depth/republish", _depth_republish, true);
  param_loader.loadParam("depth/pcl2_over_max_range", _depth_pcl2_over_max_range, false);
  param_loader.loadParam("depth/min_range", _depth_min_range_sq, 0.0f);
  param_loader.loadParam("depth/max_range", _depth_max_range_sq, 8.0f);
  param_loader.loadParam("depth/voxel_resolution", _depth_voxel_resolution, 0.0f);
  param_loader.loadParam("depth/minimum_grid_resolution", _depth_minimum_grid_resolution, 0.03f);
  param_loader.loadParam("depth/use_bilateral", _depth_use_bilateral, false);
  param_loader.loadParam("depth/bilateral_sigma_S", _depth_bilateral_sigma_S, 5.0f);
  param_loader.loadParam("depth/bilateral_sigma_R", _depth_bilateral_sigma_R, 5e-3f);
  param_loader.loadParam("depth/radius_outlier_filter/radius", _depth_radius_outlier_filter_radius, 0.0f);
  param_loader.loadParam("depth/radius_outlier_filter/neighbors", _depth_radius_outlier_filter_neighbors, 0);
  _depth_min_range_sq *= _depth_min_range_sq;
  _depth_max_range_sq *= _depth_max_range_sq;
  _depth_use_radius_outlier_filter = _depth_radius_outlier_filter_radius > 0.0f && _depth_radius_outlier_filter_neighbors > 0;

  /* RPLidar */
  /* param_loader.loadParam("rplidar/republish", _rplidar_republish, false); */
  /* param_loader.loadParam("rplidar/voxel_resolution", _rplidar_voxel_resolution, 0.0f); */

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCLFiltration]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  if (_lidar3d_republish) {
    _sub_lidar3d                = nh.subscribe("lidar3d_in", 10, &PCLFiltration::lidar3dCallback, this, ros::TransportHints().tcpNoDelay());
    _pub_lidar3d                = nh.advertise<sensor_msgs::PointCloud2>("lidar3d_out", 10);
    _pub_lidar3d_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("lidar3d_over_max_range_out", 10);
  }

  if (_tof_republish) {

    _pub_tof_top                = nh.advertise<sensor_msgs::PointCloud2>("tof_top_out", 1);
    _pub_tof_top_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("tof_top_over_max_range_out", 1);
    _sub_tof_top =
        nh.subscribe<sensor_msgs::PointCloud2>("tof_top_in", 1, boost::bind(&PCLFiltration::callbackTof, this, _1, false, _pub_tof_top_over_max_range));

    _pub_tof_bottom                = nh.advertise<sensor_msgs::PointCloud2>("tof_bottom_out", 1);
    _pub_tof_bottom_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("tof_bottom_over_max_range_out", 1);
    _pub_landing_spot              = nh.advertise<darpa_mrs_msgs::LandingSpot>("landing_spot_out", 1);
    _sub_tof_bottom =
        nh.subscribe<sensor_msgs::PointCloud2>("tof_bottom_in", 1, boost::bind(&PCLFiltration::callbackTof, this, _1, true, _pub_tof_bottom_over_max_range));
  }

  if (_depth_republish) {
    _sub_depth                = nh.subscribe("depth_in", 1, &PCLFiltration::depthCallback, this, ros::TransportHints().tcpNoDelay());
    _pub_depth                = nh.advertise<sensor_msgs::PointCloud2>("depth_out", 10);
    _pub_depth_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("depth_over_max_range_out", 10);
  }

  /* if (_rplidar_republish) { */
  /*   _sub_rplidar = nh.subscribe("rplidar_in", 1, &PCLFiltration::rplidarCallback, this, ros::TransportHints().tcpNoDelay()); */
  /*   _pub_rplidar = nh.advertise<sensor_msgs::LaserScan>("rplidar_out", 10); */
  /* } */

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
}
//}

/* lidar3dCallback() //{ */
void PCLFiltration::lidar3dCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  if (is_initialized && _lidar3d_republish) {

    TicToc t;

    sensor_msgs::PointCloud2::ConstPtr new_msg;

    // HOTFIX for subt virtual
    if (msg->height == 64) {
      PC_I::Ptr cloud_64_1024 = boost::make_shared<PC_I>();
      pcl::fromROSMsg(*msg, *cloud_64_1024);

      PC_I::Ptr cloud_16_1024 = boost::make_shared<PC_I>(1024, 16);

      for (int i = 0; i < 64; i++) {
        for (int j = 0; j < 1024; j++) {
          if (i % 4 == 0) {
            cloud_16_1024->at(j, floor(i / 4.0)) = cloud_64_1024->at(j, i);
          }
        }
      }
      sensor_msgs::PointCloud2 msg_cloud_16_1024;
      pcl::toROSMsg(*cloud_16_1024, msg_cloud_16_1024);
      msg_cloud_16_1024.header = msg->header;

      new_msg = boost::make_shared<sensor_msgs::PointCloud2>(msg_cloud_16_1024);
    } else {
      new_msg = msg;
    }

    unsigned int                         points_before = new_msg->height * new_msg->width;
    unsigned int                         points_after;
    std::variant<PC_OS1::Ptr, PC_I::Ptr> pcl_variant;
    std::variant<PC_OS1::Ptr, PC_I::Ptr> pcl_over_max_range_variant;

    if (hasField("range", new_msg)) {
      NODELET_INFO_ONCE("[PCLFiltration] Subscribing 3D LIDAR messages. Point type: ouster_ros::OS1::PointOS1.");
      removeCloseAndFarPointCloudOS1(pcl_variant, pcl_over_max_range_variant, new_msg, _lidar3d_pcl2_over_max_range, _lidar3d_min_range_mm,
                                     _lidar3d_max_range_mm, _lidar3d_filter_intensity_en, _lidar3d_filter_intensity_range_mm, _lidar3d_filter_intensity_thrd);
    } else {
      NODELET_INFO_ONCE("[PCLFiltration] Subscribing 3D LIDAR messages. Point type: pcl::PointXYZI.");
      removeCloseAndFarPointCloud(pcl_variant, pcl_over_max_range_variant, new_msg, _lidar3d_pcl2_over_max_range, _lidar3d_min_range_sq, _lidar3d_max_range_sq);
    }

    auto cloud_visitor = [&](const auto &pc) {
      points_after = pc->points.size();
      publishCloud(_pub_lidar3d, *pc);
    };
    std::visit(cloud_visitor, pcl_variant);

    if (_lidar3d_pcl2_over_max_range) {
      std::visit([&](const auto &pc) { publishCloud(_pub_lidar3d_over_max_range, *pc); }, pcl_over_max_range_variant);
    }

    NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] Processed 3D LIDAR data (run time: %0.1f ms; points before: %d, after: %d).", t.toc(), points_before,
                          points_after);
  }
}
//}

/* callbackTof() //{ */
void PCLFiltration::callbackTof(const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &detect_landing_area, const ros::Publisher &pub_over_max_range) {

  if (!is_initialized) {
    return;
  }

  NODELET_INFO_ONCE("[PCLFiltration] Subscribing ToF messages. ToF data will %s used to detect landing area.", detect_landing_area ? "be" : "not be");

  TicToc t;

  const unsigned int points_before = msg->height * msg->width;

  const PC::Ptr cloud = downsampleCloud(msg, _tof_downsample_scale);

  // TODO: Check if data contain NaNs or 0s and then replace these data with an over-the-max-range data to allow space-freeing
  // TODO: Subt sim is ideal case, no need for range-based data filtration in this case, but fill `over_max_range` with data for free-raying
  const PC::Ptr cloud_over_max_range = getCloudOverMaxRange(cloud);

  // Detect landing area
  if (detect_landing_area && _pub_landing_spot.getNumSubscribers() > 0) {

    const int8_t ground = detectGround(cloud);

    // TODO: probably make this as service only
    // Publish landing spot msg
    try {
      darpa_mrs_msgs::LandingSpot::Ptr landing_spot_msg = boost::make_shared<darpa_mrs_msgs::LandingSpot>();
      landing_spot_msg->stamp                           = msg->header.stamp;
      landing_spot_msg->landing_spot                    = ground;

      _pub_landing_spot.publish(landing_spot_msg);
    }
    catch (...) {
      NODELET_ERROR("[PCLFiltration]: Exception caught during publishing on topic: %s", _pub_landing_spot.getTopic().c_str());
    }
  }

  // Publish data
  publishCloud(_pub_tof_top, *cloud);

  // Publish data over max range
  if (_tof_pcl2_over_max_range) {
    publishCloud(pub_over_max_range, *cloud_over_max_range);
  }

  NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] Processed ToF camera data (run time: %0.1f ms; points before: %d, after: %ld).", t.toc(), points_before,
                        cloud->points.size());
}
//}

/* depthCallback() //{ */
void PCLFiltration::depthCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  if (is_initialized && _depth_republish) {
    NODELET_INFO_ONCE("[PCLFiltration] Subscribing depth camera messages.");
    TicToc t;

    unsigned int points_before = msg->height * msg->width;

    std::pair<PC::Ptr, PC::Ptr> clouds = removeCloseAndFarPointCloudXYZ(msg, _depth_pcl2_over_max_range, _depth_min_range_sq, _depth_max_range_sq);

    PC::Ptr pcl = clouds.first;

    // Voxel grid sampling
    if (_depth_voxel_resolution > 0.0f) {
      /* NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] \t - Applied voxel sampling filter (res: %0.2f m).", _depth_voxel_resolution); */
      pcl::VoxelGrid<pt_XYZ> vg;
      vg.setInputCloud(pcl);
      vg.setLeafSize(_depth_voxel_resolution, _depth_voxel_resolution, _depth_voxel_resolution);
      vg.filter(*pcl);
    }

    // Radius outlier filter
    if (_depth_use_radius_outlier_filter) {
      pcl::RadiusOutlierRemoval<pt_XYZ> outrem;
      outrem.setInputCloud(pcl);
      outrem.setRadiusSearch(_depth_radius_outlier_filter_radius);
      outrem.setMinNeighborsInRadius(_depth_radius_outlier_filter_neighbors);
      outrem.setKeepOrganized(true);
      outrem.filter(*pcl);
    }

    // Bilateral filter
    if (_depth_use_bilateral) {
      /* NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] \t - Applied fast bilateral OMP filter (sigma S: %0.2f, sigma R: %0.2f).", _depth_bilateral_sigma_S, */
      /*                       _depth_bilateral_sigma_R); */
      pcl::FastBilateralFilterOMP<pt_XYZ> fbf;
      fbf.setInputCloud(pcl);
      fbf.setSigmaS(_depth_bilateral_sigma_S);
      fbf.setSigmaR(_depth_bilateral_sigma_R);
      fbf.applyFilter(*pcl);
    }

    // Grid minimum
    if (_depth_minimum_grid_resolution > 0.0f) {
      /* NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] \t - Applied minimum grid filter (res: %0.2f m).", _depth_minimum_grid_resolution); */
      pcl::GridMinimum<pt_XYZ> gmf(_depth_minimum_grid_resolution);
      gmf.setInputCloud(pcl);
      gmf.filter(*pcl);
    }

    // Publish data
    publishCloud(_pub_depth, *pcl);

    // Publish data over max range
    if (_depth_pcl2_over_max_range) {
      publishCloud(_pub_depth_over_max_range, *clouds.second);
    }

    NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] Processed depth camera data (run time: %0.1f ms; points before: %d, after: %ld).", t.toc(), points_before,
                          pcl->points.size());
  }
}
//}

/* /1* rplidarCallback() //{ *1/ */
/* void PCLFiltration::rplidarCallback([[maybe_unused]] const sensor_msgs::LaserScan::ConstPtr msg) { */
/*   if (is_initialized && _rplidar_republish) { */
/*     NODELET_INFO_ONCE("[PCLFiltration] Subscribing RPLidar messages."); */
/*     NODELET_WARN_THROTTLE(2.0, "[PCLFiltration] rplidarCallback() not implemented!"); */
/*   } */
/* } */
/* //} */

/*//{ removeCloseAndFarPointCloud() */
void PCLFiltration::removeCloseAndFarPointCloud(std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_var, std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_over_max_range_var,
                                                const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range, const float &min_range_sq,
                                                const float &max_range_sq) {
  // SUBT HOTFIX
  const float subt_frame_det_dist_thrd = 0.15;
  const float subt_frame_x_lower       = 0.800 - subt_frame_det_dist_thrd;
  const float subt_frame_x_upper       = 0.800 + subt_frame_det_dist_thrd;
  const float subt_frame_y_lower       = 0.800 - subt_frame_det_dist_thrd;
  const float subt_frame_y_upper       = 0.800 + subt_frame_det_dist_thrd;
  const float subt_frame_z_lower       = 0.265 - 3.0 * subt_frame_det_dist_thrd;
  const float subt_frame_z_upper       = 0.265 + subt_frame_det_dist_thrd;

  // Convert to pcl object
  PC_I::Ptr cloud = boost::make_shared<PC_I>();
  PC_I::Ptr cloud_over_max_range;
  pcl::fromROSMsg(*msg, *cloud);

  size_t       j          = 0;
  size_t       k          = 0;
  const size_t cloud_size = cloud->points.size();

  if (ret_cloud_over_max_range) {
    cloud_over_max_range         = boost::make_shared<PC_I>();
    cloud_over_max_range->header = cloud->header;
    cloud_over_max_range->points.resize(cloud_size);
  }

  for (size_t i = 0; i < cloud_size; i++) {

    const float range_sq =
        cloud->points.at(i).x * cloud->points.at(i).x + cloud->points.at(i).y * cloud->points.at(i).y + cloud->points.at(i).z * cloud->points.at(i).z;

    if (range_sq < min_range_sq) {
      continue;
    }

    // SUBT HOTFIX
    const float x = std::fabs(cloud->points.at(i).x);
    const float y = std::fabs(cloud->points.at(i).y);
    const float z = std::fabs(cloud->points.at(i).z);
    if ((z > subt_frame_z_lower && z < subt_frame_z_upper) &&
        ((x > subt_frame_x_lower && x < subt_frame_x_upper) || (y > subt_frame_y_lower && y < subt_frame_y_upper))) {
      continue;

    } else if (range_sq <= max_range_sq) {

      cloud->points.at(j++) = cloud->points.at(i);

    } else if (ret_cloud_over_max_range && cloud->points.at(i).getArray4fMap().allFinite()) {

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

  cloud_var.emplace<1>(cloud);

  if (ret_cloud_over_max_range) {
    if (k != cloud_size) {
      cloud_over_max_range->points.resize(k);
    }
    cloud_over_max_range->height   = 1;
    cloud_over_max_range->width    = static_cast<uint32_t>(k);
    cloud_over_max_range->is_dense = false;

    cloud_over_max_range_var.emplace<1>(cloud_over_max_range);
  }
}
/*//}*/

/*//{ removeCloseAndFarPointCloudOS1() */
void PCLFiltration::removeCloseAndFarPointCloudOS1(std::variant<PC_OS1::Ptr, PC_I::Ptr> &    cloud_var,
                                                   std::variant<PC_OS1::Ptr, PC_I::Ptr> &    cloud_over_max_range_var,
                                                   const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range,
                                                   const uint32_t &min_range_mm, const uint32_t &max_range_mm, const bool &filter_intensity,
                                                   const uint32_t &filter_intensity_range_mm, const int &filter_intensity_thrd) {

  // Convert to pcl object
  PC_OS1::Ptr cloud = boost::make_shared<PC_OS1>();
  PC_OS1::Ptr cloud_over_max_range;
  pcl::fromROSMsg(*msg, *cloud);

  size_t j          = 0;
  size_t k          = 0;
  size_t cloud_size = cloud->points.size();

  if (ret_cloud_over_max_range) {
    cloud_over_max_range         = boost::make_shared<PC_OS1>();
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
  cloud->is_dense = false;
  cloud_var.emplace<0>(cloud);

  if (ret_cloud_over_max_range) {
    if (k != cloud_size) {
      cloud_over_max_range->points.resize(k);
    }
    cloud_over_max_range->height   = 1;
    cloud_over_max_range->width    = static_cast<uint32_t>(k);
    cloud_over_max_range->is_dense = false;

    cloud_over_max_range_var.emplace<0>(cloud_over_max_range);
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
PC::Ptr PCLFiltration::downsampleCloud(const sensor_msgs::PointCloud2::ConstPtr &msg, const unsigned int &keep_every_nth_point) {

  if (msg->width <= 1 || msg->height <= 1) {
    return boost::make_shared<PC>();
  }

  PC::Ptr cloud = boost::make_shared<PC>();
  pcl::fromROSMsg(*msg, *cloud);

  if (keep_every_nth_point <= 1) {
    return cloud;
  }

  const unsigned int height = cloud->height / keep_every_nth_point;
  const unsigned int width  = cloud->width / keep_every_nth_point;

  PC::Ptr cloud_downsampled = boost::make_shared<PC>(height, width);

  int r = 0;
  for (int i = 0; i < cloud->height; i += keep_every_nth_point) {
    int c = 0;
    for (int j = 0; j < cloud->width; j += keep_every_nth_point) {
      cloud_downsampled->at(r, c++) = cloud->at(i, j);
    }
    r++;
  }

  return cloud_downsampled;
}
/*//}*/

/*//{ getCloudOverMaxRange() */
PC::Ptr PCLFiltration::getCloudOverMaxRange(const PC::Ptr &cloud, const float &placeholder_points_distance) {

  // Convert to pcl object
  PC::Ptr cloud_over_max_range = boost::make_shared<PC>(cloud->width, cloud->height);

  if (!cloud->isOrganized()) {
    NODELET_ERROR("[PCLFiltration]: `getCloudOverMaxRange()` input cloud is not organized.");
    return cloud_over_max_range;
  }

  // TODO: Implement (for each invalid point in data, insert a "placeholder" point in large distance to which a free ray will be inserted in mapping)

  return cloud_over_max_range;
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

  if (!square->points.size() == 0) {
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

/*//{ publishCloud() */
template <typename T>
void PCLFiltration::publishCloud(const ros::Publisher &pub, const pcl::PointCloud<T> cloud) {
  if (pub.getNumSubscribers() > 0) {
    try {
      sensor_msgs::PointCloud2::Ptr pcl_msg = boost::make_shared<sensor_msgs::PointCloud2>();
      pcl::toROSMsg(cloud, *pcl_msg);
      pub.publish(pcl_msg);
    }
    catch (...) {
      NODELET_ERROR("[PCLFiltration]: Exception caught during publishing on topic: %s", pub.getTopic().c_str());
    }
  }
}
/*//}*/

}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCLFiltration, nodelet::Nodelet);
