#pragma once

/* includes //{ */
#include "support.h"

#include <variant>

#include <sensor_msgs/LaserScan.h>

#include "mrs_pcl_tools/pcl_filtration_dynparamConfig.h"
#include "darpa_mrs_msgs/LandingSpot.h"

//}

namespace mrs_pcl_tools
{

/* class PCLFiltration //{ */
class PCLFiltration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized = false;

  ros::Subscriber _sub_lidar3d;
  ros::Subscriber _sub_depth;
  ros::Subscriber _sub_tof_top;
  ros::Subscriber _sub_tof_bottom;
  /* ros::Subscriber _sub_rplidar; */

  ros::Publisher _pub_lidar3d;
  ros::Publisher _pub_lidar3d_over_max_range;
  ros::Publisher _pub_tof_top;
  ros::Publisher _pub_tof_top_over_max_range;
  ros::Publisher _pub_tof_bottom;
  ros::Publisher _pub_tof_bottom_over_max_range;
  ros::Publisher _pub_depth;
  ros::Publisher _pub_depth_over_max_range;
  ros::Publisher _pub_landing_spot;
  /* ros::Publisher _pub_rplidar; */

  ros::Timer _timer_check_subscribers;

  boost::recursive_mutex                               config_mutex_;
  typedef mrs_pcl_tools::pcl_filtration_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>          ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                 reconfigure_server_;
  /* mrs_pcl_tools::mrs_pcl_tools_dynparamConfig         last_drs_config; */

  void callbackReconfigure(mrs_pcl_tools::pcl_filtration_dynparamConfig &config, uint32_t level);

  /* 3D LIDAR */
  void     lidar3dCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  bool     _lidar3d_republish;
  bool     _lidar3d_pcl2_over_max_range;
  bool     _lidar3d_filter_intensity_en;
  float    _lidar3d_min_range_sq;
  float    _lidar3d_max_range_sq;
  float    _lidar3d_filter_intensity_range_sq;
  int      _lidar3d_filter_intensity_thrd;
  uint32_t _lidar3d_min_range_mm;
  uint32_t _lidar3d_max_range_mm;
  uint32_t _lidar3d_filter_intensity_range_mm;

  /* Time of flight cameras */
  void  callbackTof(const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &detect_landing_area, const ros::Publisher &pub_over_max_range);
  bool  _tof_republish;
  bool  _tof_pcl2_over_max_range;
  float _tof_min_range_sq;
  float _tof_max_range_sq;
  float _tof_voxel_resolution;

  /* Depth camera */
  void  depthCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  bool  _depth_republish;
  bool  _depth_pcl2_over_max_range;
  bool  _depth_use_bilateral;
  bool  _depth_use_radius_outlier_filter;
  float _depth_min_range_sq;
  float _depth_max_range_sq;
  float _depth_minimum_grid_resolution;
  float _depth_bilateral_sigma_S;
  float _depth_bilateral_sigma_R;
  float _depth_voxel_resolution;
  float _depth_radius_outlier_filter_radius;
  int   _depth_radius_outlier_filter_neighbors;

  /* RPLidar */
  /* void  rplidarCallback(const sensor_msgs::LaserScan::ConstPtr msg); */
  /* bool  _rplidar_republish; */
  /* float _rplidar_voxel_resolution; */

  /* Functions */
  void removeCloseAndFarPointCloudOS1(std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_var, std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_over_max_range_var,
                                      const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range, const uint32_t &min_range_mm,
                                      const uint32_t &max_range_mm, const bool &filter_intensity, const uint32_t &filter_intensity_range_mm,
                                      const int &filter_intensity_thrd);

  void removeCloseAndFarPointCloud(std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_var, std::variant<PC_OS1::Ptr, PC_I::Ptr> &cloud_over_max_range_var,
                                   const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range, const float &min_range_sq,
                                   const float &max_range_sq);

  std::pair<PC::Ptr, PC::Ptr> removeCloseAndFarPointCloudXYZ(const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range,
                                                             const float &min_range_sq, const float &max_range_sq);

  std::optional<bool> containsPlanarSurface(const PC::Ptr &cloud);

  template <typename T>
  void publishCloud(const ros::Publisher &pub, const pcl::PointCloud<T> cloud);
};
//}

}  // namespace mrs_pcl_tools
