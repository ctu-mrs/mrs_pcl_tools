#pragma once

#include <mrs_pcl_tools/support.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace mrs_pcl_tools
{

using vec3_t = Eigen::Vector3f;
using vec4_t = Eigen::Vector4f;
using quat_t = Eigen::Quaternionf;

struct plane_t
{
  vec3_t normal;
  float distance;
  plane_t(const vec3_t& normal, const float distance);
};

struct polygon_t
{
  geometry_msgs::msg::Point ptA;
  geometry_msgs::msg::Point ptB;
  geometry_msgs::msg::Point ptC;
  geometry_msgs::msg::Point ptD;
};

/* class GroundplaneDetector //{ */
class GroundplaneDetector
{
public:
  struct groundplane_detection_config_t
  {
    std::string static_frame_id = "";
    bool range_use = false;
    double range_max_diff = 1.0;
    // metres
    double range_max_diff_without_rangefinder = 1.5;  // metres
    double max_precrop_height = 1.0;                  // metres
    double max_angle_diff = 15.0 / 180.0 * M_PI;      // 15 degrees
    double max_inlier_dist = 3.0;                     // metres
    bool publish_plane_marker = false;

    groundplane_detection_config_t() = default;
    groundplane_detection_config_t(mrs_lib::ParamLoader& pl, const std::string& param_prefix);
    void loadParams(mrs_lib::ParamLoader& pl, const std::string& param_prefix);
  } m_cfg;

private:
  void m_add_borders_marker(const vec3_t& pos, const quat_t& quat, const polygon_t& poly,
                            const std_msgs::msg::Header& header, std::vector<visualization_msgs::msg::Marker>& markers) const;
  void m_add_plane_marker(const vec3_t& pos, const quat_t& quat, const polygon_t& poly,
                          const std_msgs::msg::Header& header, std::vector<visualization_msgs::msg::Marker>& markers) const;
  void m_add_normal_marker(const vec3_t& pos, const vec3_t& plane_normal, const std_msgs::msg::Header& header, std::vector<visualization_msgs::msg::Marker>& markers) const;


  visualization_msgs::msg::MarkerArray plane_visualization(const vec3_t& plane_normal, float plane_d,

                                                           const std_msgs::msg::Header& header) const;

private:
  const std::string NODE_NAME{ "GroundplaneDetector" };
  bool initialized{ false };

  std::shared_ptr<mrs_lib::Transformer> m_tfr;
  std::optional<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> m_pub_detected_plane;
  std::optional<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pub_inlier_points;
};
//}

}  // namespace mrs_pcl_tools

#include <mrs_pcl_tools/groundplane_detector.tpp>