#pragma once

#include <mrs_pcl_tools/support.h>
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

// /*//{ struct CommonHandlers_t */
// struct CommonHandlers_t
// {
//   std::shared_ptr<mrs_lib::ParamLoader> param_loader;
//   std::shared_ptr<mrs_lib::Transformer> transformer;

//   bool scope_timer_enabled;
//   std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger;

//   std::shared_ptr<Diagnostics_t> diagnostics;
// };

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

  void initialize(rclcpp::Node::SharedPtr node, const std::shared_ptr<mrs_lib::Transformer>& tfr,
                  const groundplane_detection_config_t& cfg);
  void initialize(rclcpp::Node::SharedPtr nh, const groundplane_detection_config_t& cfg);

  template <typename PC>
  [[nodiscard]] std::optional<plane_t> detectGroundplane(const typename boost::shared_ptr<const PC>& pc) const;

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

  // template <typename PC>
  // [[nodiscard]] bool m_tryEstimateGroundPoint(vec3_t& ground_point,
  //                                             const typename boost::shared_ptr<const PC>& pc) const;
  // template <typename PC>
  // [[nodiscard]] bool m_tryEstimateGroundNormal(vec3_t& ground_normal,
  //                                              const typename boost::shared_ptr<const PC>& pc) const;

  // template <typename PC>
  // typename pcl::SampleConsensusModelPerpendicularPlane<PC::PointType>::Ptr
  // m_makePerpendicularPlaneModel(const vec3_t& ground_normal);

  void m_fitPlaneWithRansac();
  void m_isGroundPointInlier();
  void m_publishResult();

private:
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
std::optional<plane_t> GroundplaneDetector::detectGroundplane(const typename boost::shared_ptr<const PC>& pc) const
{
  if (!m_initialized)
  {
    m_logger_->error(m_NODE_NAME + ": not initialized, skipping.");
    return std::nullopt;
  }

  using pt_t = typename PC::PointType;
  vec3_t ground_point(0, 0, 0);
  vec3_t ground_normal(0, 0, 1);

  // bool range_meas_used = m_tryEstimateGroundPoint(ground_point);
  typename PC::Ptr pc_filtered = mrs_pcl_tools::filters::applyVoxelGridFilter(*m_logger_, pc, 0.5);

  // if (!m_tryEstimateGroundNormal(ground_normal))
  // {
  //   m_logger_->warn(m_NODE_NAME + ": Could not get transformation from " + m_cfg.static_frame_id + " to " +
  //                   pc->header.frame_id + ", ground plane may be imprecise.");
  // }

  // m_makePerpendicularPlaneModel(ground_normal);
  // m_fitPlaneWithRansac();

  // Eigen::VectorXf coeffs = *coeffs_opt;
  // orientPlaneNormal(coeffs, ground_normal);

  // vec3_t fit_n = coeffs.head<3>();
  // float  fit_d = coeffs(3);

  // m_isGroundPointInlier();

  // m_publishResult();
  // return plane_t({ fit_n, fit_d });

  return plane_t({ ground_normal, 0 });
}

// /* m_makePerpendicularPlaneModel() //{ */
// template <typename PC>
// typename pcl::SampleConsensusModelPerpendicularPlane<PC::PointType>::Ptr
// GroundplaneDetector::m_makePerpendicularPlaneModel(const vec3_t& ground_normal)
// {
//   typename pcl::SampleConsensusModelPerpendicularPlane<PC::PointType>::Ptr model =
//       std::make_shared<pcl::SampleConsensusModelPerpendicularPlane<PC::PointType>>(pc_filtered, true);
//   model->setAxis(ground_normal);
//   model->setEpsAngle(m_cfg.max_angle_diff);
//   return model;
// }
// //}

// template <typename PC>
// bool GroundplaneDetector::m_tryEstimateGroundPoint(vec3_t& ground_point, const typename boost::shared_ptr<const PC>&
// pc)
// {
//   // try to deduce the ground point from the latest rangefinder measurement
//   bool range_meas_used = false;
//   // if (m_cfg.range_use && m_sh_range.hasMsg())
//   // {
//   //   const auto range_msg = m_sh_range.peekMsg();
//   //   if (range_msg->range > range_msg->min_range && range_msg->range < range_msg->max_range)
//   //   {
//   //     const vec3_t range_vec(range_msg->range, 0, 0);
//   //     const auto tf_opt = m_tfr->getTransform(range_msg->header.frame_id, pc->header.frame_id,
//   //     range_msg->header.stamp); if (tf_opt.has_value())
//   //     {
//   //       ground_point = tf2::transformToEigen(tf_opt.value().transform).template cast<float>() * range_vec;
//   //       range_meas_used = true;
//   //     }
//   //     else
//   //     {
//   //       ROS_WARN_STREAM_THROTTLE(1.0, "[" << NODE_NAME << "]: Could not get transformation from "
//   //                                         << range_msg->header.frame_id << " to " << pc->header.frame_id
//   //                                         << ", cannot use range measurement for ground plane point estimation.");
//   //     }
//   //   }
//   //   else
//   //   {
//   //     ROS_WARN_STREAM_THROTTLE(1.0, "[" << NODE_NAME
//   //                                       << "]: Range measurement is out of bounds, not using it for ground plane
//   //                                       point "
//   //                                          "estimation ("
//   //                                       << range_msg->range << " not in (" << range_msg->min_range << ", "
//   //                                       << range_msg->max_range << "))");
//   //   }
//   // }
//   return range_meas_used;
// }

#include <mrs_pcl_tools/groundplane_detector.tpp>

}  // namespace mrs_pcl_tools
