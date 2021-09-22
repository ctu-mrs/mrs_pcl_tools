#pragma once

/* includes //{ */
#include "support.h"


#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <pcl_conversions/pcl_conversions.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>

#include <darpa_mrs_msgs/LandingSpot.h>

#include <mrs_msgs/BoolStamped.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Transform.h>

#include <visualization_msgs/MarkerArray.h>

#include <boost/smart_ptr/make_shared_array.hpp>
#include <limits>

#include "mrs_pcl_tools/pcl_filtration_dynparamConfig.h"

//}

namespace mrs_pcl_tools
{
using vec3_t = Eigen::Vector3f;
using vec4_t = Eigen::Vector4f;
using quat_t = Eigen::Quaternionf;

/* class RemoveBelowGroundFilter //{ */

class RemoveBelowGroundFilter {
public:
  void initialize(ros::NodeHandle& nh, mrs_lib::ParamLoader& pl, const std::shared_ptr<mrs_lib::Transformer> transformer = nullptr,
                  const bool publish_plane_marker = false) {
    keep_organized = pl.loadParamReusable<bool>("keep_organized", false);
    pl.loadParam("ground_removal/range/use", range_use, false);
    pl.loadParam("ground_removal/range/max_difference", range_max_diff, 1.0);
    pl.loadParam("ground_removal/range/max_difference_without_rangefinder", range_max_diff_without_rangefinder, 1.5);
    pl.loadParam("ground_removal/static_frame_id", static_frame_id);
    pl.loadParam("ground_removal/max_precrop_height", max_precrop_height, std::numeric_limits<double>::infinity());
    pl.loadParam("ground_removal/ransac/max_inlier_distance", max_inlier_dist, 3.0);
    pl.loadParam("ground_removal/ransac/max_angle_difference", max_angle_diff, 15.0 / 180.0 * M_PI);
    pl.loadParam("ground_removal/plane_offset", plane_offset, 1.0);

    if (transformer == nullptr) {
      const auto pfx = pl.getPrefix();
      pl.setPrefix("");
      const std::string uav_name = pl.loadParamReusable<std::string>("uav_name");
      pl.setPrefix(pfx);
      this->transformer = std::make_shared<mrs_lib::Transformer>("RemoveBelowGroundFilter", uav_name);
    } else
      this->transformer = transformer;

    if (range_use) {
      mrs_lib::SubscribeHandlerOptions shopts(nh);
      shopts.node_name          = "RemoveBelowGroundFilter";
      shopts.no_message_timeout = ros::Duration(5.0);
      mrs_lib::construct_object(sh_range, shopts, "rangefinder_in");
    }

    if (publish_plane_marker)
      pub_fitted_plane = nh.advertise<visualization_msgs::MarkerArray>("lidar3d_fitted_plane", 10);

    initialized = true;
  }

  bool used() const {
    return initialized;
  }

  template <typename PC>
  typename boost::shared_ptr<PC> applyInPlace(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed = false);

private:
  bool initialized = false;

  std::shared_ptr<mrs_lib::Transformer>         transformer      = nullptr;
  std::optional<ros::Publisher>                 pub_fitted_plane = std::nullopt;
  mrs_lib::SubscribeHandler<sensor_msgs::Range> sh_range;

  bool        keep_organized                     = false;
  bool        range_use                          = false;
  std::string static_frame_id                    = "";
  double      max_precrop_height                 = 1.0;                  // metres
  double      max_angle_diff                     = 15.0 / 180.0 * M_PI;  // 15 degrees
  double      max_inlier_dist                    = 3.0;                  // metres
  double      plane_offset                       = 1.0;                  // metres
  double      range_max_diff                     = 1.0;                  // metres
  double      range_max_diff_without_rangefinder = 1.5;                  // metres
};

#include <impl/remove_below_ground_filter.hpp>

//}

/* class SensorDepthCamera //{ */

/*//{ DepthTraits */

// Encapsulate differences between processing float and uint16_t depths in RGBD data
template <typename T>
struct DepthTraits
{};

template <>
struct DepthTraits<uint16_t>
{
  static inline bool valid(uint16_t depth) {
    return depth != 0;
  }
  static inline float toMeters(uint16_t depth) {
    return depth * 0.001f;
  }  // originally mm
  static inline uint16_t fromMeters(float depth) {
    return (depth * 1000.0f) + 0.5f;
  }
};

template <>
struct DepthTraits<float>
{
  static inline bool valid(float depth) {
    return std::isfinite(depth);
  }
  static inline float toMeters(float depth) {
    return depth;
  }
  static inline float fromMeters(float depth) {
    return depth;
  }
};

/*//}*/

class SensorDepthCamera {
public:
  void initialize(const ros::NodeHandle& nh, mrs_lib::ParamLoader& pl, const std::shared_ptr<mrs_lib::Transformer>& transformer, const std::string& prefix,
                  const std::string& name);

private:
  template <typename T>
  void convertDepthToCloud(const sensor_msgs::Image::ConstPtr& depth_msg, PC::Ptr& cloud_out, PC::Ptr& cloud_over_max_range_out,
                           const bool return_removed_close = false, const bool return_removed_far = false, const bool replace_nans = false);

  void imagePointToCloudPoint(const int x, const int y, const float depth, pt_XYZ& point);

  void process_depth_msg(mrs_lib::SubscribeHandler<sensor_msgs::Image>& sh);
  void process_camera_info_msg(mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo>& sh);

  pt_XYZRGB colorPointXYZ(const pt_XYZ& point, const uint8_t& ch_r, const uint8_t& ch_g, const uint8_t& ch_b);
  pt_XYZRGB colorPointXYZ(const Eigen::Vector4f& point, const uint8_t& ch_r, const uint8_t& ch_g, const uint8_t& ch_b);
  pt_XYZRGB colorPointXYZ(const float& x, const float& y, const float& z, const uint8_t& ch_r, const uint8_t& ch_g, const uint8_t& ch_b);

  std::tuple<bool, float, geometry_msgs::Point, PC_RGB::Ptr> detectLandingPosition(const PC::Ptr& cloud, const bool ret_dbg_pcl);

private:
  bool            initialized = false;
  ros::NodeHandle _nh;
  ros::Publisher  pub_points;
  ros::Publisher  pub_points_over_max_range;
  ros::Publisher  pub_landing_spot_detection;
  ros::Publisher  pub_landing_spot_dbg_pcl;

  mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> sh_camera_info;
  mrs_lib::SubscribeHandler<sensor_msgs::Image>      sh_depth;

  std::shared_ptr<mrs_lib::Transformer> _transformer;

private:
  std::string depth_in, depth_camera_info_in, points_out, points_over_max_range_out;
  std::string landing_spot_detection_out, landing_spot_detection_dbg_out;
  std::string sensor_name;

  bool has_camera_info = false;
  bool publish_over_max_range;

  unsigned int image_width;
  unsigned int image_height;

  float replace_nan_depth;
  float image_center_x;
  float image_center_y;
  float focal_length;
  float focal_length_inverse;

  // Filters parameters
private:
  int downsample_step_col;
  int downsample_step_row;

  bool  range_clip_use;
  float range_clip_min;
  float range_clip_max;

  bool  voxel_grid_use;
  float voxel_grid_resolution;

  bool  radius_outlier_use;
  float radius_outlier_radius;
  int   radius_outlier_neighbors;

  bool  minimum_grid_use;
  float minimum_grid_resolution;

  bool  bilateral_use;
  float bilateral_sigma_S;
  float bilateral_sigma_R;

private:
  std::string                                   frame_world;
  std::unique_ptr<pcl::SACSegmentation<pt_XYZ>> _seg_plane_ptr;

  // Landing spot detection parameters
  bool  landing_spot_detection_use;
  float landing_spot_detection_square_size;
  float landing_spot_detection_square_size_min_ratio;
  float landing_spot_detection_square_max_ratio;
  float landing_spot_detection_z_normal_threshold;
  float landing_spot_detection_ransac_distance_threshold;
  float landing_spot_detection_min_inliers_ratio;
  int   landing_spot_detection_frame_step;
  int   landing_spot_detection_frame = 0;
};

#include "impl/sensors.hpp"

//}

/* class PCLFiltration //{ */
class PCLFiltration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized = false;

  ros::Subscriber _sub_lidar3d;
  ros::Subscriber _sub_depth;
  ros::Subscriber _sub_rplidar;

  ros::Publisher _pub_lidar3d;
  ros::Publisher _pub_lidar3d_over_max_range;
  ros::Publisher _pub_lidar3d_below_ground;
  ros::Publisher _pub_lidar3d_dust_detection;
  ros::Publisher _pub_fitted_plane;
  ros::Publisher _pub_ground_point;
  ros::Publisher _pub_depth;
  ros::Publisher _pub_depth_over_max_range;
  ros::Publisher _pub_rplidar;

  std::shared_ptr<mrs_lib::Transformer> _transformer;

  boost::recursive_mutex                               config_mutex_;
  typedef mrs_pcl_tools::pcl_filtration_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>          ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                 reconfigure_server_;
  /* mrs_pcl_tools::mrs_pcl_tools_dynparamConfig         last_drs_config; */

  RemoveBelowGroundFilter _filter_removeBelowGround;

  void callbackReconfigure(mrs_pcl_tools::pcl_filtration_dynparamConfig& config, uint32_t level);

  /* 3D LIDAR */
  void  lidar3dCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  bool  _lidar3d_keep_organized;
  bool  _lidar3d_republish;
  float _lidar3d_invalid_value;
  bool  _lidar3d_dynamic_row_selection_enabled;
  bool  _lidar3d_downsample_use;

  bool     _lidar3d_rangeclip_use;
  float    _lidar3d_rangeclip_min_sq;
  float    _lidar3d_rangeclip_max_sq;
  uint32_t _lidar3d_rangeclip_min_mm;
  uint32_t _lidar3d_rangeclip_max_mm;

  bool     _lidar3d_filter_intensity_use;
  float    _lidar3d_filter_intensity_range_sq;
  uint32_t _lidar3d_filter_intensity_range_mm;
  int      _lidar3d_filter_intensity_threshold;
  int      _lidar3d_row_step;
  int      _lidar3d_col_step;

  bool     _lidar3d_dust_detection_use;
  uint32_t _lidar3d_filter_intensity_dust_detection_range_mm;
  float    _lidar3d_filter_intensity_dust_detection_range_sq;
  int      _lidar3d_filter_intensity_dust_detection_max_point_count;

  bool         _lidar3d_cropbox_use;
  std::string  _lidar3d_cropbox_frame_id;
  vec4_t       _lidar3d_cropbox_min;
  vec4_t       _lidar3d_cropbox_max;
  unsigned int _lidar3d_dynamic_row_selection_offset = 0;


  /* Depth camera */
  std::vector<std::shared_ptr<SensorDepthCamera>> _sensors_depth_cameras;

  /* RPLidar */
  void  rplidarCallback(const sensor_msgs::LaserScan::ConstPtr msg);
  bool  _rplidar_republish;
  float _rplidar_voxel_resolution;

  /* Functions */
  template <typename PC>
  void process_msg(typename boost::shared_ptr<PC> pc_ptr);

  template <typename PC>
  void cropBoxPointCloud(boost::shared_ptr<PC>& inout_pc_ptr);

  template <typename PC>
  typename boost::shared_ptr<PC> removeCloseAndFar(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed_close = false,
                                                   const bool return_removed_far = false);

  template <typename PC>
  typename boost::shared_ptr<PC> removeLowIntensity(typename boost::shared_ptr<PC>& inout_pc, int& removed_point_count_intensity_close,
                                                    const bool return_removed = false);

  template <typename PC>
  typename boost::shared_ptr<PC> removeCloseAndFarAndLowIntensity(typename boost::shared_ptr<PC>& inout_pc, int& removed_point_count_intensity_close,
                                                                  const bool clip_return_removed_close = false, const bool clip_return_removed_far = false,
                                                                  const bool intensity_return_removed = false);

  template <typename PC>
  void downsample(boost::shared_ptr<PC>& inout_pc_ptr, const size_t scale_row, const size_t scale_col, const size_t row_offset = 0);

  std::pair<PC::Ptr, PC::Ptr> removeCloseAndFarPointCloudXYZ(const sensor_msgs::PointCloud2::ConstPtr& msg, const bool& ret_cloud_over_max_range,
                                                             const float& min_range_sq, const float& max_range_sq);

  template <typename pt_t>
  void invalidatePoint(pt_t& point);

  template <typename PC>
  void invalidatePointsAtIndices(const pcl::IndicesConstPtr& indices, typename boost::shared_ptr<PC>& cloud);

  visualization_msgs::MarkerArray plane_visualization(const vec3_t& plane_normal, float plane_d, const std_msgs::Header& header);
};
//}

}  // namespace mrs_pcl_tools
