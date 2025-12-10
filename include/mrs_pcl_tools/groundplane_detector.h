#pragma once

#include <mrs_pcl_tools/support.h>
#include <mrs_pcl_tools/groundplane_detector_core.h>

#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace mrs_pcl_tools
{

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
  static groundplane_detection_config_t loadCfg(mrs_lib::ParamLoader& pl, const std::string& param_prefix);

  void initialize(rclcpp::Node::SharedPtr node, const std::shared_ptr<mrs_lib::Transformer>& tfr,
                  const groundplane_detection_config_t& cfg);
  void initialize(rclcpp::Node::SharedPtr nh, const groundplane_detection_config_t& cfg);

  template <typename PC>
  [[nodiscard]] std::optional<plane_t> detectGroundplane(const std::shared_ptr<PC>& pc) const;

private:
  visualization_msgs::msg::MarkerArray m_planeVisualization(const vec3_t& plane_normal, float plane_d,
                                                            const std_msgs::msg::Header& header) const;

  void m_addBorderMarker(const vec3_t& pos, const quat_t& quat, const polygon_t& poly,
                         const std_msgs::msg::Header& header,
                         std::vector<visualization_msgs::msg::Marker>& markers) const;
  void m_addPlaneMarker(const vec3_t& pos, const quat_t& quat, const polygon_t& poly,
                        const std_msgs::msg::Header& header,
                        std::vector<visualization_msgs::msg::Marker>& markers) const;
  void m_addNormalMarker(const vec3_t& pos, const vec3_t& plane_normal, const std_msgs::msg::Header& header,
                         std::vector<visualization_msgs::msg::Marker>& markers) const;

private:
  groundplane_detection_config_t m_cfg;
  std::unique_ptr<GroundplaneDetectorCore> m_ground_plane_detector_;

  RosLogger* m_logger_;
  const std::string m_NODE_NAME{ "GroundplaneDetector" };
  bool m_initialized{ false };

  std::shared_ptr<mrs_lib::Transformer> m_tfr;

  std::optional<mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>> m_pub_detected_plane;
  std::optional<mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>> m_pub_inlier_points;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Range> m_sh_range;
};
//}

template <typename PC>
std::optional<plane_t> GroundplaneDetector::detectGroundplane(const std::shared_ptr<PC>& cloud) const
{
  if (!m_initialized)
  {
    m_logger_->error(m_NODE_NAME + ": not initialized, skipping.");
    return std::nullopt;
  }

  const PC& pc = *cloud;
  auto plane = m_ground_plane_detector_->detect(pc);

  //   // m_publishResult();
  //   // return plane_t({ fit_n, fit_d });
  return std::nullopt;
}

// template <typename PC>
// std::optional<plane_t> GroundplaneDetector::detectGroundplane(const typename boost::shared_ptr<const PC>& pc) const
// {
//   if (!m_initialized)
//   {
//     m_logger_->error(m_NODE_NAME + ": not initialized, skipping.");
//     return std::nullopt;
//   }

//   using pt_t = typename PC::PointType;
//   vec3_t ground_point(0, 0, 0);
//   vec3_t ground_normal(0, 0, 1);

//   // bool range_meas_used = m_tryEstimateGroundPoint(ground_point);
//   typename PC::Ptr pc_filtered = mrs_pcl_tools::filters::applyVoxelGridFilter(*m_logger_, pc, 0.5);

//   // if (!m_tryEstimateGroundNormal(ground_normal))
//   // {
//   //   m_logger_->warn(m_NODE_NAME + ": Could not get transformation from " + m_cfg.static_frame_id + " to " +
//   //                   pc->header.frame_id + ", ground plane may be imprecise.");
//   // }

//   // m_makePerpendicularPlaneModel(ground_normal);
//   // m_fitPlaneWithRansac();

//   // Eigen::VectorXf coeffs = *coeffs_opt;
//   // orientPlaneNormal(coeffs, ground_normal);

//   // vec3_t fit_n = coeffs.head<3>();
//   // float  fit_d = coeffs(3);

//   // m_isGroundPointInlier();

//   // m_publishResult();
//   // return plane_t({ fit_n, fit_d });

//   return plane_t({ ground_normal, 0 });
// }

}  // namespace mrs_pcl_tools
