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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
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
  
  class RemoveBelowGroundFilter
  {
    public:
      void initialize(ros::NodeHandle& nh, mrs_lib::ParamLoader& pl, const std::shared_ptr<mrs_lib::Transformer> transformer = nullptr, const bool publish_plane_marker = false)
      {
        keep_organized = pl.loadParamReusable<bool>("keep_organized", false);
        pl.loadParam("ground_removal/range/use", range_use, false);
        pl.loadParam("ground_removal/static_frame_id", static_frame_id);
        pl.loadParam("ground_removal/max_precrop_height", max_precrop_height, std::numeric_limits<double>::infinity());
        pl.loadParam("ground_removal/ransac/max_inlier_distance", max_inlier_dist, 3.0);
        pl.loadParam("ground_removal/ransac/max_angle_difference", max_angle_diff, 15.0/180.0*M_PI);
        pl.loadParam("ground_removal/plane_offset", plane_offset, 1.0);
  
        if (transformer == nullptr)
        {
          const auto pfx = pl.getPrefix();
          pl.setPrefix("");
          const std::string uav_name = pl.loadParamReusable<std::string>("uav_name");
          pl.setPrefix(pfx);
          this->transformer = std::make_shared<mrs_lib::Transformer>("RemoveBelowGroundFilter", uav_name);
        }
        else
          this->transformer = transformer;
  
        if (range_use)
        {
          mrs_lib::SubscribeHandlerOptions shopts(nh);
          shopts.node_name = "RemoveBelowGroundFilter";
          shopts.no_message_timeout = ros::Duration(5.0);
          mrs_lib::construct_object(sh_range, shopts, "rangefinder_in");
        }
  
        if (publish_plane_marker)
          pub_fitted_plane = nh.advertise<visualization_msgs::MarkerArray>("lidar3d_fitted_plane", 10);
  
        initialized = true;
      }
  
      bool used() const
      {
        return initialized;
      }
  
      template <typename PC>
      typename boost::shared_ptr<PC> applyInPlace(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed = false);
  
    private:
      bool initialized = false;
  
      std::shared_ptr<mrs_lib::Transformer> transformer = nullptr;
      std::optional<ros::Publisher> pub_fitted_plane    = std::nullopt;
      mrs_lib::SubscribeHandler<sensor_msgs::Range> sh_range;
  
      bool keep_organized         = false;
      bool range_use              = false;
      std::string static_frame_id = "";
      double max_precrop_height   = 1.0;              // metres
      double max_angle_diff       = 15.0/180.0*M_PI;  // 15 degrees
      double max_inlier_dist      = 3.0;              // metres
      double plane_offset         = 1.0;              // metres
  };

#include <impl/remove_below_ground_filter.hpp>
  
  //}
  
  /* class SensorDepthCamera //{ */

  /*//{ DepthTraits */
  // Encapsulate differences between processing float and uint16_t depths
  template<typename T> struct DepthTraits {};

  template<>
    struct DepthTraits<uint16_t>
    {
      static inline bool valid(uint16_t depth) { return depth != 0; }
      static inline float toMeters(uint16_t depth) { return depth * 0.001f; } // originally mm
      static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
    };

  template<>
    struct DepthTraits<float>
    {
      static inline bool valid(float depth) { return std::isfinite(depth); }
      static inline float toMeters(float depth) { return depth; }
      static inline float fromMeters(float depth) { return depth; }
    };
  /*//}*/
  
  class SensorDepthCamera
  {
    public:
      void initialize(ros::NodeHandle& nh, mrs_lib::ParamLoader& pl, const std::string &prefix, const std::string &name)
      {
        _nh         = nh;
        sensor_name = name;
 
        pl.loadParam("depth/" + sensor_name + "/filter/downsample/step/col", downsample_step_col, 1);
        pl.loadParam("depth/" + sensor_name + "/filter/downsample/step/row", downsample_step_row, 1);
        if (downsample_step_col < 1)
          downsample_step_col = 1;
        if (downsample_step_row < 1)
          downsample_step_row = 1;

        pl.loadParam("depth/" + sensor_name + "/filter/range_clip/min", range_clip_min, 0.0f);
        pl.loadParam("depth/" + sensor_name + "/filter/range_clip/max", range_clip_max, std::numeric_limits<float>::max());
        range_clip_use                     = range_clip_max > 0.0f && range_clip_max > range_clip_min;
        artificial_depth_of_removed_points = range_clip_use ? 2.0f * range_clip_max : 1000.0f;
        
        pl.loadParam("depth/" + sensor_name + "/filter/voxel_grid/resolution", voxel_grid_resolution, 0.0f);
        voxel_grid_use = voxel_grid_resolution > 0.0f;
        
        pl.loadParam("depth/" + sensor_name + "/filter/radius_outlier/radius", radius_outlier_radius, 0.0f);
        pl.loadParam("depth/" + sensor_name + "/filter/radius_outlier/neighbors", radius_outlier_neighbors, 0);
        radius_outlier_use = radius_outlier_radius > 0.0f && radius_outlier_neighbors > 0;
        
        pl.loadParam("depth/" + sensor_name + "/filter/minimum_grid/resolution", minimum_grid_resolution, 0.0f);
        minimum_grid_use = minimum_grid_use > 0.0f;
        
        pl.loadParam("depth/" + sensor_name + "/filter/bilateral/sigma_S", bilateral_sigma_S, 0.0f);
        pl.loadParam("depth/" + sensor_name + "/filter/bilateral/sigma_R", bilateral_sigma_R, 0.0f);
        bilateral_use = bilateral_sigma_S > 0.0f && bilateral_sigma_R > 0.0f;

        pl.loadParam("depth/" + sensor_name + "/topic/depth_in", depth_in);
        pl.loadParam("depth/" + sensor_name + "/topic/depth_camera_info_in", depth_camera_info_in);
        pl.loadParam("depth/" + sensor_name + "/topic/points_out", points_out);
        pl.loadParam("depth/" + sensor_name + "/topic/points_over_max_range_out", points_over_max_range_out, std::string(""));

        publish_over_max_range = points_over_max_range_out.size() > 0 && range_clip_use;

        if (prefix.size() > 0) {

          depth_in             = "/" + prefix + "/" + depth_in;
          depth_camera_info_in = "/" + prefix + "/" + depth_camera_info_in;
          points_out           = "/" + prefix + "/" + points_out;
          
          if (publish_over_max_range)
            points_over_max_range_out = "/" + prefix + "/" + points_over_max_range_out;
        }
  
        // Start subscribe handler for depth camera info
        mrs_lib::SubscribeHandlerOptions shopts(_nh);
        shopts.node_name  = "SensorDepthCamera::CameraInfo::" + sensor_name;
        shopts.threadsafe = true;
        sh_camera_info    = mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> (shopts, depth_camera_info_in);
        mrs_lib::construct_object(sh_camera_info, shopts, depth_camera_info_in, ros::Duration(1.0), &SensorDepthCamera::process_camera_info_msg, this);
  
        initialized = true;
      }

    private:

      /*//{ convertDepthToCloud() */
      // Depth conversion inspired by: https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h#L48
      template<typename T>
        void convertDepthToCloud(const sensor_msgs::Image::ConstPtr& depth_msg, PC::Ptr &cloud_out, PC::Ptr &cloud_over_max_range_out, const bool return_removed) {

          const unsigned int max_points_count = (image_height / downsample_step_row) * (image_width / downsample_step_col);

          cloud_out                = boost::make_shared<PC>();
          cloud_out->resize(max_points_count);
          if (return_removed)
            cloud_over_max_range_out = boost::make_shared<PC>();
            cloud_over_max_range_out->resize(max_points_count);

          const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
          const int row_step = downsample_step_row * depth_msg->step / sizeof(T);

          int points_cloud                = 0;
          int points_cloud_over_max_range = 0;

          for (int v = 0; v < (int)depth_msg->height; v+=downsample_step_row, depth_row += row_step) 
          {
            for (int u = 0; u < (int)depth_msg->width; u+=downsample_step_col)
            {
              const auto  depth_raw = depth_row[u];
              const bool  valid     = DepthTraits<T>::valid(depth_raw);
              const float depth     = DepthTraits<T>::toMeters(depth_raw);

              // Convert to point cloud points and optionally clip range
              if (valid && (!range_clip_use || depth > range_clip_min && depth <= range_clip_max)) {

                imagePointToCloudPoint(u, v, depth, cloud_out->points.at(points_cloud++));

              } else if (return_removed && (!valid || (range_clip_use && depth <= range_clip_min || depth > range_clip_max))) {

                imagePointToCloudPoint(u, v, artificial_depth_of_removed_points, cloud_over_max_range_out->points.at(points_cloud_over_max_range++));
              }
            }
          }

          // Fill headers
          cloud_out->width    = points_cloud;
          cloud_out->height   = points_cloud > 0 ? 1 : 0;
          cloud_out->is_dense = true;
          pcl_conversions::toPCL(depth_msg->header, cloud_out->header);
          cloud_out->points.resize(points_cloud);

          // Publish depth msg data over range_clip_max
          if (return_removed) {
            cloud_over_max_range_out->width    = points_cloud_over_max_range;
            cloud_over_max_range_out->height   = points_cloud_over_max_range > 0 ? 1 : 0;
            cloud_over_max_range_out->is_dense = true;
            cloud_over_max_range_out->header   = cloud_out->header;
            cloud_over_max_range_out->points.resize(points_cloud_over_max_range);
          }
        }
      /*//}*/

      /*//{ imagePointToCloudPoint() */
      void imagePointToCloudPoint(const int x, const int y, const float depth, pt_XYZ &point) {

        const float dc = depth * focal_length_inverse;

        point.x = (x - image_center_x) * dc;
        point.y = (y - image_center_y) * dc;
        point.z = depth;
      }
      /*//}*/

      /*//{ process_depth_msg() */
      void process_depth_msg(mrs_lib::SubscribeHandler<sensor_msgs::Image>& sh) {

        // TODO: keep ordered

        if (!initialized)
          return;

        PC::Ptr cloud, cloud_over_max_range;
        const auto depth_msg = sh.getMsg();

        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || depth_msg->encoding == sensor_msgs::image_encodings::MONO16)
        {
          convertDepthToCloud<uint16_t>(depth_msg, cloud, cloud_over_max_range, publish_over_max_range);
        }
        else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        {
          convertDepthToCloud<float>(depth_msg, cloud, cloud_over_max_range, publish_over_max_range);
        }
        else
        {
          ROS_ERROR_THROTTLE(5.0, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
          return;
        }
        
        // TODO: apply filters

        pub_points.publish(cloud);
        if (publish_over_max_range)
          pub_points_over_max_range.publish(cloud_over_max_range);

      }
      /*//}*/

      /*//{ process_camera_info_msg() */
      void process_camera_info_msg(mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo>& sh) {

        if (!initialized || has_camera_info)
          return;

        const auto msg = sh.getMsg();

        // Store data required for 2D -> 3D projection
        image_width          = msg->width; 
        image_height         = msg->height; 
        focal_length         = msg->K.at(0);
        image_center_x       = msg->K.at(2);
        image_center_y       = msg->K.at(5);

        const bool valid = image_width > 0 && image_height > 0 && focal_length > 0.0 && image_center_x >= 0.0 && image_center_y >= 0.0;

        if (valid) {

          // Shutdown this subscribe handler
          has_camera_info = true;
          sh_camera_info.stop();

          ROS_INFO("[SensorDepthCamera::process_camera_info_msg::%s] Received valid depth camera info (width: %d, height: %d, focal length: %0.2f, cx: %0.1f, cy: %0.1f)", sensor_name.c_str(), image_width, image_height, focal_length, image_center_x, image_center_y);

          focal_length_inverse = 1.0 / focal_length;

          // Advertise publishers
          pub_points = _nh.advertise<sensor_msgs::PointCloud2>(points_out, 10);
          if (publish_over_max_range) {
            pub_points_over_max_range = _nh.advertise<sensor_msgs::PointCloud2>(points_over_max_range_out, 10);
          }

          // Start subscribe handler for depth data
          mrs_lib::SubscribeHandlerOptions shopts(_nh);
          shopts.node_name  = "SensorDepthCamera::Image::" + sensor_name;
          shopts.threadsafe = true;
          sh_depth          = mrs_lib::SubscribeHandler<sensor_msgs::Image> (shopts, depth_in);
          mrs_lib::construct_object(sh_depth, shopts, depth_in, ros::Duration(1.0), &SensorDepthCamera::process_depth_msg, this);
        }
      }
      /*//}*/

 
    private:
      bool initialized = false;
      ros::NodeHandle _nh;
      ros::Publisher pub_points;
      ros::Publisher pub_points_over_max_range;

      mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> sh_camera_info;
      mrs_lib::SubscribeHandler<sensor_msgs::Image>      sh_depth;

    private:
        
      std::string depth_in, depth_camera_info_in, points_out, points_over_max_range_out;
      std::string sensor_name;
      
      bool has_camera_info = false;
      bool publish_over_max_range;

      unsigned int image_width;
      unsigned int image_height;

      float artificial_depth_of_removed_points;
      float image_center_x;
      float image_center_y;
      float focal_length;
      float focal_length_inverse;

    // Filters parameters
    private:

      int  downsample_step_col;
      int  downsample_step_row;

      bool range_clip_use;
      float range_clip_min;
      float range_clip_max;

      bool voxel_grid_use;
      float voxel_grid_resolution;

      bool radius_outlier_use;
      float radius_outlier_radius;
      int radius_outlier_neighbors;

      bool minimum_grid_use;
      float minimum_grid_resolution;

      bool bilateral_use;
      float bilateral_sigma_S;
      float bilateral_sigma_R;
  
  };

  
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

  void callbackReconfigure(mrs_pcl_tools::pcl_filtration_dynparamConfig &config, uint32_t level);

  /* 3D LIDAR */
  void         lidar3dCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  bool         _lidar3d_keep_organized;
  bool         _lidar3d_republish;
  float        _lidar3d_invalid_value;
  bool         _lidar3d_dynamic_row_selection_enabled;

  bool         _lidar3d_rangeclip_use;
  float        _lidar3d_rangeclip_min_sq;
  float        _lidar3d_rangeclip_max_sq;
  uint32_t     _lidar3d_rangeclip_min_mm;
  uint32_t     _lidar3d_rangeclip_max_mm;

  bool         _lidar3d_filter_intensity_use;
  float        _lidar3d_filter_intensity_range_sq;
  uint32_t     _lidar3d_filter_intensity_range_mm;
  int          _lidar3d_filter_intensity_threshold;
  int          _lidar3d_row_step;
  int          _lidar3d_col_step;

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
  void invalidatePoint(pt_t &point);

  visualization_msgs::MarkerArray plane_visualization(const vec3_t& plane_normal, float plane_d, const std_msgs::Header& header);
};
//}

}  // namespace mrs_pcl_tools
