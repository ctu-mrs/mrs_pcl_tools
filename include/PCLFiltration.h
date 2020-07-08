#pragma once

/* includes //{ */
#include "common_includes_and_typedefs.h"

#include <variant>

#include <sensor_msgs/LaserScan.h>

#include "mrs_pcl_tools/pcl_filtration_dynparamConfig.h"

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

  boost::recursive_mutex                               config_mutex_;
  typedef mrs_pcl_tools::pcl_filtration_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>          ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                 reconfigure_server_;
  /* mrs_pcl_tools::mrs_pcl_tools_dynparamConfig         last_drs_config; */

  void callbackReconfigure(mrs_pcl_tools::pcl_filtration_dynparamConfig &config, uint32_t level);

  /* Ouster */
  void     ousterCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  bool     _ouster_republish;
  bool     _ouster_pcl2_over_max_range;
  bool     _ouster_filter_intensity_en;
  float    _ouster_min_range_sq;
  float    _ouster_max_range_sq;
  float    _ouster_filter_intensity_range_sq;
  int      _ouster_filter_intensity_thrd;
  uint32_t _ouster_min_range_mm;
  uint32_t _ouster_max_range_mm;
  uint32_t _ouster_filter_intensity_range_mm;

  /* Realsense */
  void        realsenseCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
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
  void removeCloseAndFarPointCloudOS1(std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_var, std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_over_max_range_var,
                                      const sensor_msgs::PointCloud2::ConstPtr &msg, const bool ret_cloud_over_max_range, const uint32_t min_range_mm,
                                      const uint32_t max_range_mm, const bool filter_intensity, const uint32_t filter_intensity_range_mm,
                                      const int filter_intensity_thrd);

  void removeCloseAndFarPointCloud(std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_var, std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_over_max_range_var,
                                   const sensor_msgs::PointCloud2::ConstPtr &msg, const bool ret_cloud_over_max_range, const float min_range_sq,
                                   const float max_range_sq);

  std::pair<PC::Ptr, PC::Ptr> removeCloseAndFarPointCloudXYZ(const sensor_msgs::PointCloud2::ConstPtr msg, const bool ret_cloud_over_max_range,
                                                             const float min_range_sq, const float max_range_sq);

  template <typename T>
  void publishCloud(const ros::Publisher pub, const pcl::PointCloud<T> cloud);

  bool hasField(const std::string field, const sensor_msgs::PointCloud2::ConstPtr& msg);
};
//}

}  // namespace mrs_pcl_tools
