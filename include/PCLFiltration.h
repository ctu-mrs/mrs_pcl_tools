#pragma once

/* includes //{ */
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <chrono>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/grid_minimum.h>

#include <sensor_msgs/LaserScan.h>

#include <mrs_lib/param_loader.h>

#include <dynamic_reconfigure/server.h>

#include "mrs_pcl_tools/pcl_filtration_dynparamConfig.h"

//}

/*//{ typedefs */
typedef pcl::PointCloud<pcl::PointXYZ>  PC;
typedef pcl::PointCloud<pcl::PointXYZI> PCI;
//}

namespace mrs_pcl_tools
{

/* class PCLFiltration //{ */
class PCLFiltration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized = false;

  ros::Subscriber _sub_ouster;
  ros::Subscriber _sub_realsense;
  ros::Subscriber _sub_rplidar;

  ros::Publisher _pub_ouster;
  ros::Publisher _pub_ouster_over_max_range;
  ros::Publisher _pub_realsense;
  ros::Publisher _pub_rplidar;

  ros::Timer _timer_check_subscribers;

  boost::recursive_mutex                                        config_mutex_;
  typedef mrs_pcl_tools::pcl_filtration_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>                   ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                          reconfigure_server_;
  /* mrs_pcl_tools::mrs_pcl_tools_dynparamConfig         last_drs_config; */

  void callbackReconfigure(mrs_pcl_tools::pcl_filtration_dynparamConfig &config, uint32_t level);

  /* Ouster */
  void   ousterCallback(const sensor_msgs::PointCloud2::ConstPtr msg);
  bool   _ouster_republish;
  bool   _ouster_pcl2_over_max_range;
  bool   _ouster_filter_intensity_en;
  float  _ouster_min_range_sq;
  float  _ouster_max_range_sq;
  float  _ouster_filter_intensity_range_sq;
  double _ouster_filter_intensity_thrd;

  /* Realsense */
  void        realsenseCallback(const sensor_msgs::PointCloud2::ConstPtr msg);
  bool        _realsense_republish;
  bool        _realsense_use_bilateral;
  float       _realsense_min_range_sq;
  float       _realsense_max_range_sq;
  float       _realsense_minimum_grid_resolution;
  float       _realsense_bilateral_sigma_S;
  float       _realsense_bilateral_sigma_R;
  float       _realsense_voxel_resolution;
  std::string _realsense_frame;

  /* RPLidar */
  void  rplidarCallback(const sensor_msgs::LaserScan::ConstPtr msg);
  bool  _rplidar_republish;
  float _rplidar_voxel_resolution;

  /* Functions */
  void removeCloseAndFarPointCloud(const PC &cloud_in, PC &cloud_out, const float min_range_sq, const float max_range_sq);
  void removeCloseAndFarPointCloud(const PCI &cloud_in, PCI &cloud_out, const float min_range_sq, const float max_range_sq);
  void removeCloseAndFarPointCloud(const PCI &cloud_in, PCI &cloud_out, PCI &cloud_out_over_max_range, const float min_range_sq, const float max_range_sq);

  /* void checkSubscribers(const ros::TimerEvent& te); */
};
//}

}  // namespace mrs_pcl_tools
