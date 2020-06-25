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
  _ouster_min_range_sq *= _ouster_min_range_sq;
  _ouster_max_range_sq *= _ouster_max_range_sq;

  param_loader.loadParam("ouster/filter/intensity", _ouster_filter_intensity_en, false);
  param_loader.loadParam("ouster/filter/intensity_thrd", _ouster_filter_intensity_thrd, std::numeric_limits<double>::max());
  param_loader.loadParam("ouster/filter/intensity_range", _ouster_filter_intensity_range_sq, std::numeric_limits<float>::max());
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

  NODELET_INFO_ONCE("[PCLFiltration] Cartographer tools initialized");

  is_initialized = true;
}
//}

/* callbackReconfigure() //{ */
void PCLFiltration::callbackReconfigure(Config& config, [[maybe_unused]] uint32_t level) {
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
void PCLFiltration::ousterCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
  if (is_initialized && _ouster_republish) {
    NODELET_INFO_ONCE("[PCLFiltration] Subscribing Ouster messages.");
    NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] Processing Ouster data.");

    // Convert to PCL format
    PCI pcl;
    PCI pclOverMaxRange;
    pcl::fromROSMsg(*msg, pcl);

    // NaN filter
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(pcl, pcl, indices);

    // Min/Max range filter
    removeCloseAndFarPointCloud(pcl, pcl, pclOverMaxRange, _ouster_min_range_sq, _ouster_max_range_sq);

    // Publish
    if (_pub_ouster.getNumSubscribers() > 0) {
      try {
        sensor_msgs::PointCloud2::Ptr msg_out = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(pcl, *msg_out);
        _pub_ouster.publish(msg_out);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publish of ouster data processed. ");
      }
    }

    if (_ouster_pcl2_over_max_range && _pub_ouster_over_max_range.getNumSubscribers() > 0) {
      try {
        sensor_msgs::PointCloud2::Ptr msg_over_max_range_out = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(pclOverMaxRange, *msg_over_max_range_out);
        _pub_ouster_over_max_range.publish(msg_over_max_range_out);
      }
      catch (...) {
        ROS_ERROR("Exception caught during over-range ouster data. ");
      }
    }
  }
}
//}

/* realsenseCallback() //{ */
void PCLFiltration::realsenseCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
  if (is_initialized && _realsense_republish) {
    NODELET_INFO_ONCE("[PCLFiltration] Subscribing Realsense messages.");
    NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] Processing Realsense data.");

    // Convert to PCL format
    PC::Ptr pcPtr(new PC);
    pcl::fromROSMsg(*msg, *pcPtr);

    // NaN filter
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcPtr, *pcPtr, indices);

    // Min/Max range filter
    removeCloseAndFarPointCloud(*pcPtr, *pcPtr, _realsense_min_range_sq, _realsense_max_range_sq);

    // Bilateral filter
    if (_realsense_use_bilateral) {
      NODELET_INFO_THROTTLE(1.0, "[PCLFiltration] \t - Applying fast bilateral OMP filter.");
      pcl::FastBilateralFilterOMP<pcl::PointXYZ> fbf;
      fbf.setInputCloud(pcPtr);
      fbf.setSigmaS(_realsense_bilateral_sigma_S);
      fbf.setSigmaR(_realsense_bilateral_sigma_R);
      fbf.applyFilter(*pcPtr);
    }

    // Grid minimum
    if (_realsense_minimum_grid_resolution > 0.0f) {
      NODELET_INFO_THROTTLE(1.0, "[PCLFiltration] \t - Applying minimum grid filter.");
      pcl::GridMinimum<pcl::PointXYZ> gmf(_realsense_minimum_grid_resolution);
      gmf.setInputCloud(pcPtr);
      gmf.filter(*pcPtr);
    }

    // Voxel grid sampling
    if (_realsense_voxel_resolution > 0.0f) {
      NODELET_INFO_THROTTLE(1.0, "[PCLFiltration] \t - Applying voxel sampling filter.");
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(pcPtr);
      vg.setLeafSize(_realsense_voxel_resolution, _realsense_voxel_resolution, _realsense_voxel_resolution);
      vg.filter(*pcPtr);
    }

    // Convert to ROS msg format
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*pcPtr, msg_out);
    msg_out.header = msg->header;
    if (_realsense_frame.size() > 0) {
      msg_out.header.frame_id = _realsense_frame;
    }
    _pub_realsense.publish(msg_out);
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

/* removeCloseAndFarPointCloud //{*/
void PCLFiltration::removeCloseAndFarPointCloud(const PC& cloud_in, PC& cloud_out, const float min_range_sq, const float max_range_sq) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }

  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    double dist_sq = cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z;
    if (dist_sq > min_range_sq && dist_sq <= max_range_sq) {
      cloud_out.points[j++] = cloud_in.points[i];
    }
  }
  if (j != cloud_in.points.size()) {
    cloud_out.points.resize(j);
  }

  cloud_out.height   = 1;
  cloud_out.width    = static_cast<uint32_t>(j);
  cloud_out.is_dense = false;
}

void PCLFiltration::removeCloseAndFarPointCloud(const PCI& cloud_in, PCI& cloud_out, const float min_range_sq, const float max_range_sq) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }

  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    double dist_sq = cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z;
    // Filter out low intensities in close radius
    if (_ouster_filter_intensity_en && cloud_in.points[i].intensity < _ouster_filter_intensity_thrd && dist_sq < _ouster_filter_intensity_range_sq) {
      continue;
    }
    if (dist_sq > min_range_sq && dist_sq <= max_range_sq) {
      cloud_out.points[j++] = cloud_in.points[i];
    }
  }
  if (j != cloud_in.points.size()) {
    cloud_out.points.resize(j);
  }

  cloud_out.height   = 1;
  cloud_out.width    = static_cast<uint32_t>(j);
  cloud_out.is_dense = false;
}

void PCLFiltration::removeCloseAndFarPointCloud(const PCI& cloud_in, PCI& cloud_out, PCI& cloud_out_over_max_range, const float min_range_sq,
                                                const float max_range_sq) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }
  if (&cloud_in != &cloud_out_over_max_range) {
    cloud_out_over_max_range.header = cloud_in.header;
    cloud_out_over_max_range.points.resize(cloud_in.points.size());
  }

  size_t j = 0;
  size_t k = 0;

  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    double dist_sq = cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z;
    if (dist_sq < min_range_sq) {
      continue;
    } else if (dist_sq <= max_range_sq) {
      // Filter out low intensities in close radius
      if (_ouster_filter_intensity_en && cloud_in.points[i].intensity < _ouster_filter_intensity_thrd && dist_sq < _ouster_filter_intensity_range_sq) {
        continue;
      }
      cloud_out.points[j++] = cloud_in.points[i];
    } else {
      cloud_out_over_max_range.points[k++] = cloud_in.points[i];
    }
  }
  if (j != cloud_in.points.size()) {
    cloud_out.points.resize(j);
  }
  if (k != cloud_in.points.size()) {
    cloud_out_over_max_range.points.resize(k);
  }

  cloud_out.height   = 1;
  cloud_out.width    = static_cast<uint32_t>(j);
  cloud_out.is_dense = false;

  cloud_out_over_max_range.height   = 1;
  cloud_out_over_max_range.width    = static_cast<uint32_t>(k);
  cloud_out_over_max_range.is_dense = false;
}
/*//}*/

}  // namespace mrs_pcl_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCLFiltration, nodelet::Nodelet);
