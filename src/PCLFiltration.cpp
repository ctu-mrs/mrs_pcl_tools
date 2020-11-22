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
  param_loader.loadParam("lidar3d/row_step", _lidar3d_row_step, 1);
  param_loader.loadParam("lidar3d/col_step", _lidar3d_col_step, 1);
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
  param_loader.loadParam("rplidar/republish", _rplidar_republish, false);
  param_loader.loadParam("rplidar/voxel_resolution", _rplidar_voxel_resolution, 0.0f);

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCLFiltration]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }


  if (_lidar3d_republish) {

    if (_lidar3d_row_step <= 0 || _lidar3d_col_step <= 0) {
      NODELET_ERROR("[PCLFiltration]: Downsampling row/col steps for 3D lidar must be >=1, ending nodelet.");
      ros::shutdown();
    }

    _sub_lidar3d                = nh.subscribe("lidar3d_in", 10, &PCLFiltration::lidar3dCallback, this, ros::TransportHints().tcpNoDelay());
    _pub_lidar3d                = nh.advertise<sensor_msgs::PointCloud2>("lidar3d_out", 10);
    _pub_lidar3d_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("lidar3d_over_max_range_out", 10);
  }

  if (_depth_republish) {
    _sub_depth                = nh.subscribe("depth_in", 1, &PCLFiltration::depthCallback, this, ros::TransportHints().tcpNoDelay());
    _pub_depth                = nh.advertise<sensor_msgs::PointCloud2>("depth_out", 10);
    _pub_depth_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("depth_over_max_range_out", 10);
  }

  if (_rplidar_republish) {
    _sub_rplidar = nh.subscribe("rplidar_in", 1, &PCLFiltration::rplidarCallback, this, ros::TransportHints().tcpNoDelay());
    _pub_rplidar = nh.advertise<sensor_msgs::LaserScan>("rplidar_out", 10);
  }

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

    if (msg->width % _lidar3d_col_step != 0 || msg->height % _lidar3d_row_step != 0) {
      NODELET_WARN(
          "[PCLFiltration] Step-based downsampling of 3D lidar data would create nondeterministic results. Data (w: %d, h: %d) with downsampling step (w: %d, "
          "h: %d) would leave some samples untouched. Skipping lidar frame.",
          msg->width, msg->height, _lidar3d_col_step, _lidar3d_row_step);
      return;
    }

    TicToc t;

    unsigned int points_before = msg->height * msg->width;
    unsigned int points_after;

    std::variant<PC_OS1::Ptr, PC_I::Ptr> pcl_variant;
    std::variant<PC_OS1::Ptr, PC_I::Ptr> pcl_over_max_range_variant;

    if (hasField("range", msg) && hasField("noise", msg)) {
      NODELET_INFO_ONCE("[PCLFiltration] Subscribing 3D LIDAR messages. Point type: ouster_ros::OS1::PointOS1.");
      removeCloseAndFarPointCloudOS1(pcl_variant, pcl_over_max_range_variant, points_after, msg, _lidar3d_pcl2_over_max_range, _lidar3d_min_range_mm,
                                     _lidar3d_max_range_mm, _lidar3d_filter_intensity_en, _lidar3d_filter_intensity_range_mm, _lidar3d_filter_intensity_thrd);
    } else {
      NODELET_INFO_ONCE("[PCLFiltration] Subscribing 3D LIDAR messages. Point type: pcl::PointXYZI.");
      removeCloseAndFarPointCloud(pcl_variant, pcl_over_max_range_variant, msg, _lidar3d_pcl2_over_max_range, _lidar3d_min_range_sq, _lidar3d_max_range_sq);
    }

    unsigned int width_after;
    unsigned int height_after;
    auto         cloud_visitor = [&](const auto &pc) {
      publishCloud(_pub_lidar3d, *pc);
      width_after  = pc->width;
      height_after = pc->height;
    };
    std::visit(cloud_visitor, pcl_variant);

    if (_lidar3d_pcl2_over_max_range) {
      std::visit([&](const auto &pc) { publishCloud(_pub_lidar3d_over_max_range, *pc); }, pcl_over_max_range_variant);
    }

    NODELET_INFO_THROTTLE(
        5.0, "[PCLFiltration] Processed 3D LIDAR data (run time: %0.1f ms; points before: %d, after: %d; dim before: (w: %d, h: %d), after: (w: %d, h: %d)).",
        t.toc(), points_before, points_after, msg->width, msg->height, width_after, height_after);
  }
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

/* rplidarCallback() //{ */
void PCLFiltration::rplidarCallback([[maybe_unused]] const sensor_msgs::LaserScan::ConstPtr msg) {
  if (is_initialized && _rplidar_republish) {
    NODELET_INFO_ONCE("[PCLFiltration] Subscribing RPLidar messages.");
    NODELET_WARN_THROTTLE(2.0, "[PCLFiltration] rplidarCallback() not implemented!");
  }
}
//}

/*//{ removeCloseAndFarPointCloud() */
void PCLFiltration::removeCloseAndFarPointCloud(std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_var, std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_over_max_range_var,
                                                const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range, const float &min_range_sq,
                                                const float &max_range_sq) {

  // Convert to pcl object
  PC_I::Ptr cloud = boost::make_shared<PC_I>();
  PC_I::Ptr cloud_over_max_range;
  pcl::fromROSMsg(*msg, *cloud);

  unsigned int j          = 0;
  unsigned int k          = 0;
  unsigned int cloud_size = cloud->points.size();

  if (ret_cloud_over_max_range) {
    cloud_over_max_range         = boost::make_shared<PC_I>();
    cloud_over_max_range->header = cloud->header;
    cloud_over_max_range->points.resize(cloud_size);
  }

  for (unsigned int i = 0; i < cloud_size; i++) {

    float range_sq =
        cloud->points.at(i).x * cloud->points.at(i).x + cloud->points.at(i).y * cloud->points.at(i).y + cloud->points.at(i).z * cloud->points.at(i).z;

    if (range_sq < min_range_sq) {
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
void PCLFiltration::removeCloseAndFarPointCloudOS1(std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_var,
                                                   std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_over_max_range_var, unsigned int &valid_points,
                                                   const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range,
                                                   const uint32_t &min_range_mm, const uint32_t &max_range_mm, const bool &filter_intensity,
                                                   const uint32_t &filter_intensity_range_mm, const int &filter_intensity_thrd) {

  const unsigned int w   = msg->width / _lidar3d_col_step;
  const unsigned int h   = msg->height / _lidar3d_row_step;
  const float        nan = std::numeric_limits<float>::quiet_NaN();
  valid_points           = 0;

  // Convert to pcl object
  PC_OS1::Ptr cloud_out = boost::make_shared<PC_OS1>(w, h);
  PC_OS1::Ptr cloud_over_max_range;

  PC_OS1::Ptr cloud = boost::make_shared<PC_OS1>();
  pcl::fromROSMsg(*msg, *cloud);

  cloud_out->width    = w;
  cloud_out->height   = h;
  cloud_out->is_dense = false;
  cloud_out->header   = cloud->header;

  if (ret_cloud_over_max_range) {
    cloud_over_max_range         = boost::make_shared<PC_OS1>();
    cloud_over_max_range->header = cloud->header;
    cloud_over_max_range->resize(w * h);
  }
  unsigned int k = 0;

  for (unsigned int j = 0; j < msg->width; j += _lidar3d_col_step) {
    const unsigned int c = j / _lidar3d_col_step;

    for (unsigned int i = 0; i < msg->height; i += _lidar3d_row_step) {
      const unsigned int r = i / _lidar3d_row_step;

      /* ROS_DEBUG("j|i|idx|c|r %d|%d|%d|%d|%d", j, i, idx, c, r); */
      /* ROS_DEBUG("... xyz|ring (%0.1f %0.1f %0.1f)|%d", point.x, point.y, point.z, point.ring); */

      const unsigned int idx   = j * msg->height + i;
      pt_OS1             point = cloud->at(idx);

      point.ring = r;

      if (point.range < min_range_mm) {

        invalidatePoint(point, nan);

      } else if (point.range <= max_range_mm) {

        // Filter out low intensities in close radius
        if (filter_intensity && point.intensity < filter_intensity_thrd && point.range < filter_intensity_range_mm) {

          invalidatePoint(point, nan);

        } else {
          valid_points++;
        }

      } else {

        if (ret_cloud_over_max_range) {
          cloud_over_max_range->at(k++) = point;
        }

        invalidatePoint(point, nan);
      }

      cloud_out->at(c, r) = point;
    }
  }

  // Resize both clouds
  cloud_var.emplace<0>(cloud_out);

  if (ret_cloud_over_max_range) {
    cloud_over_max_range->resize(k);
    cloud_over_max_range->is_dense = true;
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

  unsigned int j          = 0;
  unsigned int k          = 0;
  unsigned int cloud_size = cloud->points.size();

  if (ret_cloud_over_max_range) {
    cloud_over_max_range->header = cloud->header;
    cloud_over_max_range->points.resize(cloud_size);
  }

  for (unsigned int i = 0; i < cloud_size; i++) {

    float range_sq =
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

/*//{ invalidatePoint() */
void PCLFiltration::invalidatePoint(pt_OS1 &point, const float inv_value) {
  point.x = inv_value;
  point.y = inv_value;
  point.z = inv_value;
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
