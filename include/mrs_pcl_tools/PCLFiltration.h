#pragma once

/* includes //{ */
#include "support.h"

#include <variant>

#include <sensor_msgs/LaserScan.h>

#include "mrs_pcl_tools/pcl_filtration_dynparamConfig.h"
#include "darpa_mrs_msgs/LandingSpot.h"

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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

  ros::Publisher _pub_lidar3d;
  ros::Publisher _pub_lidar3d_over_max_range;
  ros::Publisher _pub_depth;
  ros::Publisher _pub_depth_over_max_range;

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

  /* Landing spot detection */
  float _ground_detection_square_size;
  float _ground_detection_ransac_distance_thrd;
  float _ground_detection_n_z_max_diff;

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

  template <typename T>
  void publishCloud(const ros::Publisher &pub, const pcl::PointCloud<T> cloud);

  /* Time of flight cameras */

  struct Camera
  {
  public:
    std::string     name;
    bool            publish_pcl2_over_max_range;
    bool            has_camera_model;
    bool            detect_landing_area;
    unsigned int    downsample_scale;
    float           voxel_grid_resolution;
    float           min_range;
    float           max_range;
    ros::Subscriber sub_image;
    ros::Subscriber sub_camera_info;
    ros::Publisher  pub_cloud;
    ros::Publisher  pub_cloud_over_max_range;
    ros::Publisher  pub_landing_spot;

    image_geometry::PinholeCameraModel camera_model;
  };

  std::shared_ptr<Camera> _tof_top;
  std::shared_ptr<Camera> _tof_bottom;

  void callbackTofImage(const sensor_msgs::Image::ConstPtr &depth_msg, const std::shared_ptr<Camera> &tof);
  void callbackTofCameraInfo(const sensor_msgs::CameraInfo::ConstPtr &info_msg, const std::shared_ptr<Camera> &tof);

  std::pair<PC::Ptr, PC::Ptr> imageToPcFiltered(const sensor_msgs::Image::ConstPtr &msg, const image_geometry::PinholeCameraModel &model,
                                                const float &min_range, const float &max_range, const float &nan_depth = 100.0f,
                                                const unsigned int &keep_every_nth_point = 1);
  PC::Ptr                     downsampleCloud(const PC::Ptr &cloud, const unsigned int &keep_every_nth_point = 1);
  int8_t                      detectGround(const PC::Ptr &cloud);
  pt_XYZ imagePointToCloudPoint(const int &x, const int &y, const float &cx, const float &cy, const float &depth, const float &ifx, const float &ify);
};
//}

}  // namespace mrs_pcl_tools
