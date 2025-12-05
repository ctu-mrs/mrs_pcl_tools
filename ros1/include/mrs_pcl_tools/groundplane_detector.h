#pragma once

/* includes //{ */

#include <mrs_pcl_tools/support.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <pcl_conversions/pcl_conversions.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/scope_timer.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <mrs_modules_msgs/PclToolsDiagnostics.h>

#include <boost/smart_ptr/make_shared_array.hpp>
#include <limits>

#include <tf2_eigen/tf2_eigen.h>

#include <mrs_lib/param_loader.h>

//}

namespace mrs_pcl_tools
{
using vec3_t = Eigen::Vector3f;
using vec4_t = Eigen::Vector4f;
using quat_t = Eigen::Quaternionf;

struct plane_t
{
  vec3_t normal;
  float  distance;
  plane_t(const vec3_t& normal, const float distance);
};

/* class GroundplaneDetector //{ */

class GroundplaneDetector {
public:
  struct groundplane_detection_config_t
  {
    std::string static_frame_id                    = "";
    bool        range_use                          = false;
    double      range_max_diff                     = 1.0;                  // metres
    double      range_max_diff_without_rangefinder = 1.5;                  // metres
    double      max_precrop_height                 = 1.0;                  // metres
    double      max_angle_diff                     = 15.0 / 180.0 * M_PI;  // 15 degrees
    double      max_inlier_dist                    = 3.0;                  // metres
    bool        publish_plane_marker               = false;

    groundplane_detection_config_t() = default;
    groundplane_detection_config_t(mrs_lib::ParamLoader& pl, const std::string& param_prefix);
    void loadParams(mrs_lib::ParamLoader& pl, const std::string& param_prefix);
  } m_cfg;

  GroundplaneDetector(const rclcpp::NodeOptions& options);

  void intialize(rclcpp::Node::SharedPtr nh_, const std::shared_ptr<mrs_lib::Transformer>& tfr, const groundplane_detection_config_t& cfg);

  void intialize(rclcpp::Node::SharedPtr nh_, const groundplane_detection_config_t& cfg);

  template <typename PC>
  [[nodiscard]] std::optional<plane_t> detectGroundplane(const typename boost::shared_ptr<const PC>& pc) const;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  const std::string NODE_NAME   = "GroundplaneDetector";
  bool              is_initialized = false;

  std::shared_ptr<mrs_lib::Transformer>         m_tfr                = nullptr;
  std::optional<rclcpp::Publisher<mrs_msgs::msg::messagename>::SharedPtr>  m_pub_detected_plane = std::nullopt;
  std::optional<rclcpp::Publisher<mrs_msgs::msg::messageName>::SharedPtr>  m_pub_inlier_points  = std::nullopt;
  mrs_lib::SubscribeHandler<sensor_msgs::msgs::Range> m_sh_range;

  visualization_msgs::MarkerArray plane_visualization(const vec3_t& plane_normal, float plane_d, const std_msgs::msgs::Header& header) const;
};

#include <mrs_pcl_tools/impl/groundplane_detector.hpp>

//}

}  // namespace mrs_pcl_tools
