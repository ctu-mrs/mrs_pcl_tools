#include "PCLFiltration.h"

namespace mrs_pcl_tools
{

/* onInit() //{ */
void PCLFiltration::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Get parameters from config file
  mrs_lib::ParamLoader param_loader(nh, "PCLFiltration");

  /* Ouster */
  param_loader.loadParam("ouster/republish", _ouster_republish, false);
  param_loader.loadParam("ouster/pcl2_over_max_range", _ouster_pcl2_over_max_range, false);
  param_loader.loadParam("ouster/min_range", _ouster_min_range_sq, 0.4f);
  param_loader.loadParam("ouster/max_range", _ouster_max_range_sq, 100.0f);
  _ouster_min_range_mm = _ouster_min_range_sq * 1000;
  _ouster_max_range_mm = _ouster_max_range_sq * 1000;
  _ouster_min_range_sq *= _ouster_min_range_sq;
  _ouster_max_range_sq *= _ouster_max_range_sq;

  param_loader.loadParam("ouster/filter/intensity/enable", _ouster_filter_intensity_en, false);
  param_loader.loadParam("ouster/filter/intensity/threshold", _ouster_filter_intensity_thrd, std::numeric_limits<int>::max());
  param_loader.loadParam("ouster/filter/intensity/range", _ouster_filter_intensity_range_sq, std::numeric_limits<float>::max());
  _ouster_filter_intensity_range_mm = _ouster_filter_intensity_range_sq * 1000;
  _ouster_filter_intensity_range_sq *= _ouster_filter_intensity_range_sq;

  /* Realsense */
  param_loader.loadParam("realsense/republish", _realsense_republish, true);
  param_loader.loadParam("realsense/min_range", _realsense_min_range_sq, 0.0f);
  param_loader.loadParam("realsense/max_range", _realsense_max_range_sq, 8.0f);
  param_loader.loadParam("realsense/voxel_resolution", _realsense_voxel_resolution, 0.0f);
  param_loader.loadParam("realsense/minimum_grid_resolution", _realsense_minimum_grid_resolution, 0.03f);
  param_loader.loadParam("realsense/use_bilateral", _realsense_use_bilateral, false);
  param_loader.loadParam("realsense/bilateral_sigma_S", _realsense_bilateral_sigma_S, 5.0f);
  param_loader.loadParam("realsense/bilateral_sigma_R", _realsense_bilateral_sigma_R, 5e-3f);
  param_loader.loadParam("realsense/frame", _realsense_frame, std::string(""));
  _realsense_min_range_sq *= _realsense_min_range_sq;
  _realsense_max_range_sq *= _realsense_max_range_sq;

  /* RPLidar */
  param_loader.loadParam("rplidar/republish", _rplidar_republish, false);
  param_loader.loadParam("rplidar/voxel_resolution", _rplidar_voxel_resolution, 0.0f);

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCLFiltration]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  if (_ouster_republish) {
    _sub_ouster                = nh.subscribe("ouster_in", 10, &PCLFiltration::ousterCallback, this, ros::TransportHints().tcpNoDelay());
    _pub_ouster                = nh.advertise<sensor_msgs::PointCloud2>("ouster_out", 10);
    _pub_ouster_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("ouster_over_max_range_out", 10);
  }

  if (_realsense_republish) {
    _sub_realsense = nh.subscribe("realsense_in", 1, &PCLFiltration::realsenseCallback, this, ros::TransportHints().tcpNoDelay());
    _pub_realsense = nh.advertise<sensor_msgs::PointCloud2>("realsense_out", 10);
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

  _ouster_filter_intensity_en       = config.ouster_filter_intensity_en;
  _ouster_filter_intensity_thrd     = config.ouster_filter_intensity_thrd;
  _ouster_filter_intensity_range_sq = std::pow(config.ouster_filter_intensity_rng, 2);
}
//}

/* ousterCallback() //{ */
void PCLFiltration::ousterCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  if (is_initialized && _ouster_republish) {

    std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();

    unsigned int points_before = msg->height * msg->width;
    unsigned int points_after;

    std::variant<PC_OS1::Ptr, PC_I::Ptr> pcl_variant;
    std::variant<PC_OS1::Ptr, PC_I::Ptr> pcl_over_max_range_variant;

    if (hasField("range", msg)) {
      NODELET_INFO_ONCE("[PCLFiltration] Subscribing Ouster messages. Point type: ouster_ros::OS1::PointOS1.");
      removeCloseAndFarPointCloudOS1(pcl_variant, pcl_over_max_range_variant, msg, _ouster_pcl2_over_max_range, _ouster_min_range_mm, _ouster_max_range_mm,
                                     _ouster_filter_intensity_en, _ouster_filter_intensity_range_mm, _ouster_filter_intensity_thrd);
    } else {
      NODELET_INFO_ONCE("[PCLFiltration] Subscribing Ouster messages. Point type: pcl::PointXYZI.");
      removeCloseAndFarPointCloud(pcl_variant, pcl_over_max_range_variant, msg, _ouster_pcl2_over_max_range, _ouster_min_range_sq, _ouster_max_range_sq);
    }

    auto cloud_visitor = [&](const auto &pc) {
      points_after = pc->points.size();
      publishCloud(_pub_ouster, *pc);
    };
    std::visit(cloud_visitor, pcl_variant);

    if (_ouster_pcl2_over_max_range) {
      std::visit([&](const auto &pc) { publishCloud(_pub_ouster_over_max_range, *pc); }, pcl_over_max_range_variant);
    }

    std::chrono::duration<float> elapsed_ms = std::chrono::system_clock::now() - start_time;
    NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] Processed OS1 data (run time: %0.1f ms; points before: %d, after: %d).", elapsed_ms.count() * 1000,
                          points_before, points_after);
  }
}
//}

/* realsenseCallback() //{ */
void PCLFiltration::realsenseCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  if (is_initialized && _realsense_republish) {
    NODELET_INFO_ONCE("[PCLFiltration] Subscribing Realsense messages.");
    std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();

    unsigned int points_before = msg->height * msg->width;

    PC::Ptr pcl = removeCloseAndFarPointCloudXYZ(msg, false, _realsense_min_range_sq, _realsense_max_range_sq).first;

    // Bilateral filter
    if (_realsense_use_bilateral) {
      NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] \t - Applying fast bilateral OMP filter.");
      pcl::FastBilateralFilterOMP<pt_XYZ> fbf;
      fbf.setInputCloud(pcl);
      fbf.setSigmaS(_realsense_bilateral_sigma_S);
      fbf.setSigmaR(_realsense_bilateral_sigma_R);
      fbf.applyFilter(*pcl);
    }

    // Grid minimum
    if (_realsense_minimum_grid_resolution > 0.0f) {
      NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] \t - Applying minimum grid filter.");
      pcl::GridMinimum<pt_XYZ> gmf(_realsense_minimum_grid_resolution);
      gmf.setInputCloud(pcl);
      gmf.filter(*pcl);
    }

    // Voxel grid sampling
    if (_realsense_voxel_resolution > 0.0f) {
      NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] \t - Applying voxel sampling filter.");
      pcl::VoxelGrid<pt_XYZ> vg;
      vg.setInputCloud(pcl);
      vg.setLeafSize(_realsense_voxel_resolution, _realsense_voxel_resolution, _realsense_voxel_resolution);
      vg.filter(*pcl);
    }

    // Publish data
    publishCloud(_pub_realsense, *pcl);

    std::chrono::duration<float> elapsed_ms = std::chrono::system_clock::now() - start_time;
    NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] Processed RealSense data (run time: %0.1f ms; points before: %d, after: %ld).", elapsed_ms.count() * 1000,
                          points_before, pcl->points.size());
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
                                                const sensor_msgs::PointCloud2::ConstPtr &msg, const bool ret_cloud_over_max_range, const float min_range_sq,
                                                const float max_range_sq) {

  // Convert to pcl object
  PC_I::Ptr cloud                = boost::make_shared<PC_I>();
  PC_I::Ptr cloud_over_max_range = boost::make_shared<PC_I>();
  pcl::fromROSMsg(*msg, *cloud);

  size_t j          = 0;
  size_t k          = 0;
  size_t cloud_size = cloud->points.size();

  if (ret_cloud_over_max_range) {
    cloud_over_max_range->header = cloud->header;
    cloud_over_max_range->points.resize(cloud_size);
  }

  for (size_t i = 0; i < cloud_size; i++) {

    float range_sq =
        cloud->points.at(i).x * cloud->points.at(i).x + cloud->points.at(i).y * cloud->points.at(i).y + cloud->points.at(i).z * cloud->points.at(i).z;

    if (range_sq < min_range_sq) {
      continue;

    } else if (range_sq <= max_range_sq) {

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

  cloud_var.emplace<1>(cloud);

  if (cloud_over_max_range) {
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
                                                   const sensor_msgs::PointCloud2::ConstPtr &msg, const bool ret_cloud_over_max_range,
                                                   const uint32_t min_range_mm, const uint32_t max_range_mm, const bool filter_intensity,
                                                   const uint32_t filter_intensity_range_mm, const int filter_intensity_thrd) {

  // Convert to pcl object
  PC_OS1::Ptr cloud                = boost::make_shared<PC_OS1>();
  PC_OS1::Ptr cloud_over_max_range = boost::make_shared<PC_OS1>();
  pcl::fromROSMsg(*msg, *cloud);

  size_t j          = 0;
  size_t k          = 0;
  size_t cloud_size = cloud->points.size();

  if (ret_cloud_over_max_range) {
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

  if (cloud_over_max_range) {
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
std::pair<PC::Ptr, PC::Ptr> PCLFiltration::removeCloseAndFarPointCloudXYZ(const sensor_msgs::PointCloud2::ConstPtr msg, const bool ret_cloud_over_max_range,
                                                                          const float min_range_sq, const float max_range_sq) {

  // Convert to pcl object
  PC::Ptr cloud(new PC());
  PC::Ptr cloud_over_max_range(new PC());
  pcl::fromROSMsg(*msg, *cloud);

  size_t j          = 0;
  size_t k          = 0;
  size_t cloud_size = cloud->points.size();

  if (ret_cloud_over_max_range) {
    cloud_over_max_range->header = cloud->header;
    cloud_over_max_range->points.resize(cloud_size);
  }

  for (size_t i = 0; i < cloud_size; i++) {

    float range_sq =
        cloud->points.at(i).x * cloud->points.at(i).x + cloud->points.at(i).y * cloud->points.at(i).y + cloud->points.at(i).z * cloud->points.at(i).z;

    if (range_sq < min_range_sq) {
      continue;

    } else if (range_sq <= max_range_sq) {

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

  if (cloud_over_max_range) {
    if (k != cloud_size) {
      cloud_over_max_range->points.resize(k);
    }
    cloud_over_max_range->height   = 1;
    cloud_over_max_range->width    = static_cast<uint32_t>(k);
    cloud_over_max_range->is_dense = false;
  }

  return std::make_pair(cloud, cloud_over_max_range);
}
/*//}*/

/*//{ publishCloud() */
template <typename T>
void PCLFiltration::publishCloud(const ros::Publisher pub, const pcl::PointCloud<T> cloud) {
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

/*//{ hasField() */
bool PCLFiltration::hasField(const std::string field, const sensor_msgs::PointCloud2::ConstPtr &msg) {
  for (auto f : msg->fields) {
    if (f.name == field) {
      return true;
    }
  }
  return false;
}
/*//}*/

}  // namespace mrs_pcl_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCLFiltration, nodelet::Nodelet);
