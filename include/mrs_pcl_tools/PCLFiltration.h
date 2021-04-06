#pragma once

/* includes //{ */
#include "support.h"

#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <variant>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Transform.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/smart_ptr/make_shared_array.hpp>
#include <limits>


#include "mrs_pcl_tools/pcl_filtration_dynparamConfig.h"

//}

namespace mrs_pcl_tools
{

/* class PCLFiltration //{ */
class PCLFiltration : public nodelet::Nodelet {

  using vec3_t = Eigen::Vector3f;
  using vec4_t = Eigen::Vector4f;
  using quat_t = Eigen::Quaternionf;

public:
  virtual void onInit();

private:
  bool is_initialized = false;

  ros::Subscriber _sub_lidar3d;
  ros::Subscriber _sub_depth;
  ros::Subscriber _sub_rplidar;
  mrs_lib::SubscribeHandler<sensor_msgs::Range> _sh_range;

  ros::Publisher _pub_lidar3d;
  ros::Publisher _pub_lidar3d_over_max_range;
  ros::Publisher _pub_lidar3d_below_ground;
  ros::Publisher _pub_fitted_plane;
  ros::Publisher _pub_ground_point;
  ros::Publisher _pub_depth;
  ros::Publisher _pub_depth_over_max_range;
  ros::Publisher _pub_rplidar;

  mrs_lib::Transformer _transformer;

  boost::recursive_mutex                               config_mutex_;
  typedef mrs_pcl_tools::pcl_filtration_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>          ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                 reconfigure_server_;
  /* mrs_pcl_tools::mrs_pcl_tools_dynparamConfig         last_drs_config; */

  void callbackReconfigure(mrs_pcl_tools::pcl_filtration_dynparamConfig &config, uint32_t level);

  /* 3D LIDAR */
  void         lidar3dCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  bool         _lidar3d_republish;
  bool         _lidar3d_dynamic_row_selection_enabled;

  bool         _lidar3d_rangeclip_use;
  float        _lidar3d_rangeclip_min_sq;
  float        _lidar3d_rangeclip_max_sq;
  uint32_t     _lidar3d_rangeclip_min_mm;
  uint32_t     _lidar3d_rangeclip_max_mm;

  bool         _lidar3d_filter_intensity_en;
  float        _lidar3d_filter_intensity_range_sq;
  uint32_t     _lidar3d_filter_intensity_range_mm;
  int          _lidar3d_filter_intensity_thrd;
  int          _lidar3d_row_step;
  int          _lidar3d_col_step;

  bool         _lidar3d_groundremoval_use;
  bool         _lidar3d_groundremoval_range_use;
  std::string  _lidar3d_groundremoval_frame_id;
  float        _lidar3d_groundremoval_max_height;
  float        _lidar3d_groundremoval_ransac_max_inlier_dist;
  float        _lidar3d_groundremoval_ransac_max_angle_diff;
  float        _lidar3d_groundremoval_offset;

  bool         _lidar3d_cropbox_use;
  std::string  _lidar3d_cropbox_frame_id;
  vec4_t       _lidar3d_cropbox_min;
  vec4_t       _lidar3d_cropbox_max;
  unsigned int _lidar3d_dynamic_row_selection_offset = 0;


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
  void  rplidarCallback(const sensor_msgs::LaserScan::ConstPtr msg);
  bool  _rplidar_republish;
  float _rplidar_voxel_resolution;

  /* Functions */
  template <typename PC>
  void process_msg(typename boost::shared_ptr<PC> pc_ptr);

  template <typename PC>
  typename boost::shared_ptr<PC> removeBelowGround(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed = false);
  template <typename PC>
  void cropBoxPointCloud(boost::shared_ptr<PC>& inout_pc_ptr);
  void removeCloseAndFarPointCloudOS(std::variant<PC_OS::Ptr, PC_I::Ptr> &cloud_var, std::variant<PC_OS::Ptr, PC_I::Ptr> &cloud_over_max_range_var,
                                      unsigned int &valid_points, const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range,
                                      const uint32_t &min_range_mm, const uint32_t &max_range_mm, const bool &filter_intensity,
                                      const uint32_t &filter_intensity_range_mm, const int &filter_intensity_thrd);

  template <typename PC>
  typename boost::shared_ptr<PC> removeCloseAndFar(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed = false);

  template <typename PC>
  typename boost::shared_ptr<PC> removeLowIntensity(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed = false);

  template <typename PC>
  typename boost::shared_ptr<PC> removeCloseAndFarAndLowIntensity(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed = false);

  std::pair<PC::Ptr, PC::Ptr> removeCloseAndFarPointCloudXYZ(const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range,
                                                             const float &min_range_sq, const float &max_range_sq);

  template <typename pt_t>
  void invalidatePoint(pt_t &point, const float inv_value = std::numeric_limits<float>::quiet_NaN());

  template <typename T>
  void publishCloud(const ros::Publisher &pub, const pcl::PointCloud<T> cloud);

  visualization_msgs::MarkerArray plane_visualization(const vec3_t& plane_normal, float plane_d, const std_msgs::Header& header);
};
//}

}  // namespace mrs_pcl_tools
