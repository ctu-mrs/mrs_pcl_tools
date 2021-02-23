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

  ros::Publisher _pub_lidar3d;
  ros::Publisher _pub_lidar3d_over_max_range;

  ros::Timer _timer_check_subscribers;

  boost::recursive_mutex                               config_mutex_;
  typedef mrs_pcl_tools::pcl_filtration_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>          ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                 reconfigure_server_;
  /* mrs_pcl_tools::mrs_pcl_tools_dynparamConfig         last_drs_config; */

  void callbackReconfigure(mrs_pcl_tools::pcl_filtration_dynparamConfig &config, uint32_t level);

  /* 3D LIDAR */
  void              lidar3dCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  bool              _lidar3d_pcl2_over_max_range;
  bool              _lidar3d_filter_intensity_en;
  float             _lidar3d_min_range_sq;
  float             _lidar3d_max_range_sq;
  float             _lidar3d_filter_intensity_range_sq;
  int               _lidar3d_filter_intensity_thrd;
  uint32_t          _lidar3d_min_range_mm;
  uint32_t          _lidar3d_max_range_mm;
  uint32_t          _lidar3d_filter_intensity_range_mm;
  long unsigned int _lidar3d_frame = 0;

  /* Landing spot detection */
  float _ground_detection_square_size;
  float _ground_detection_ransac_distance_thrd;
  float _ground_detection_n_z_max_diff;

  /* Functions */
  void removeCloseAndFarPointCloudOS1(PC_OS::Ptr &cloud_var, PC_OS::Ptr &cloud_over_max_range_var, const sensor_msgs::PointCloud2::ConstPtr &msg,
                                      const bool &ret_cloud_over_max_range, const uint32_t &min_range_mm, const uint32_t &max_range_mm,
                                      const bool &filter_intensity, const uint32_t &filter_intensity_range_mm, const int &filter_intensity_thrd);

  void removeCloseAndFarPointCloud(PC_OS::Ptr &cloud_var, PC_OS::Ptr &cloud_over_max_range_var, unsigned int &valid_points,
                                   const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range, const float &min_range_sq,
                                   const float &max_range_sq);

  std::pair<PC::Ptr, PC::Ptr> removeCloseAndFarPointCloudXYZ(const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range,
                                                             const float &min_range_sq, const float &max_range_sq);

  template <typename T>
  void publishCloud(const ros::Publisher &pub, const pcl::PointCloud<T> cloud);

  /* CAMERA processing */
  struct Camera
  {
  public:
    std::string       name;
    bool              detect_landing_area;
    bool              publish_pcl2_over_max_range;
    int               downsample_scale;
    float             voxel_grid_resolution;
    float             min_range;
    float             max_range;
    float             focal_length;
    long unsigned int keep_nth_frame;
    long unsigned int data_frame;
    ros::Subscriber   sub_image;
    ros::Publisher    pub_cloud;
    ros::Publisher    pub_cloud_over_max_range;
    ros::Publisher    pub_landing_spot;
  };

  std::shared_ptr<Camera> _camera_up;
  std::shared_ptr<Camera> _camera_down;
  std::shared_ptr<Camera> _camera_front;

  void callbackCameraImage(const sensor_msgs::Image::ConstPtr &depth_msg, const std::shared_ptr<Camera> &tof);

  std::pair<PC::Ptr, PC::Ptr> imageToPcFiltered(const sensor_msgs::Image::ConstPtr &msg, const std::shared_ptr<Camera> &tof, const float &nan_depth = 100.0f);
  PC::Ptr                     downsampleCloud(const PC::Ptr &cloud, const unsigned int &keep_every_nth_point = 1);
  int8_t                      detectGround(const PC::Ptr &cloud);
  pt_XYZ imagePointToCloudPoint(const int &x, const int &y, const float &cx, const float &cy, const float &depth, const float &ifx, const float &ify);

  void invalidatePoint(pt_OS &point);

  // SUBT HOTFIX
  const float subt_frame_det_dist_thrd = 0.15;
  const float subt_frame_x_lower       = 0.800 - subt_frame_det_dist_thrd;
  const float subt_frame_x_upper       = 0.800 + subt_frame_det_dist_thrd;
  const float subt_frame_y_lower       = 0.800 - subt_frame_det_dist_thrd;
  const float subt_frame_y_upper       = 0.800 + subt_frame_det_dist_thrd;
  const float subt_frame_z_lower       = 0.265 - 3.0 * subt_frame_det_dist_thrd;
  const float subt_frame_z_upper       = 0.265 + subt_frame_det_dist_thrd;

  const float nan = std::numeric_limits<float>::quiet_NaN();

  // MAV type-specific variables
  struct MAVType
  {
  public:
    std::string       name;
    int               lidar_col_step;
    int               lidar_row_step;
    bool              process_cameras;
    bool              filter_out_projected_self_frame;
    unsigned long int skip_nth_lidar_frame;
    unsigned long int keep_nth_vert_camera_frame;
    unsigned long int keep_nth_front_camera_frame;
    float             focal_length_vert_camera;
    float             focal_length_front_camera;
    float             vert_camera_max_range;
    float             front_camera_max_range;
  };
  std::shared_ptr<MAVType> _mav_type;
};
//}

}  // namespace mrs_pcl_tools
