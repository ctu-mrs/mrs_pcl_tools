#pragma once

#include <mrs_pcl_tools/support.h>

namespace mrs_pcl_tools
{

using vec3_t = Eigen::Vector3f;
using vec4_t = Eigen::Vector4f;
using quat_t = Eigen::Quaternionf;

/* struct plane_t //{ */
struct plane_t
{
  vec3_t normal;
  float distance;
  plane_t(const vec3_t& normal, const float distance);
};
//}

/* struct groundplane_detection_config_t //{ */
struct groundplane_detection_config_t
{
  std::string static_frame_id = "";
  bool range_use = false;
  double range_max_diff = 1.0;                      // metres
  double range_max_diff_without_rangefinder = 1.5;  // metres
  double max_precrop_height = 1.0;                  // metres
  double max_angle_diff = 15.0 / 180.0 * M_PI;      // 15 degrees
  double max_inlier_dist = 3.0;                     // metres
  bool publish_plane_marker = false;

  groundplane_detection_config_t() = default;
};
//}

/* class GroundplaneDetectorCore //{ */
class GroundplaneDetectorCore
{
public:
  GroundplaneDetectorCore(ILogger& logger, const groundplane_detection_config_t& cfg) : m_cfg(cfg), m_logger(logger)
  {
  }

  template <typename PC>
  [[nodiscard]] std::optional<plane_t> detect(const std::shared_ptr<const PC>& pc) const;

private:
  void m_fitPlaneWithRansac();
  void m_isGroundPointInlier();
  void m_publishResult();

private:
  const groundplane_detection_config_t m_cfg;
  ILogger& m_logger;
};

/*//{ detect() */
template <typename PC>
[[nodiscard]] std::optional<plane_t> GroundplaneDetectorCore::detect(const std::shared_ptr<const PC>& pc) const
{
  using pt_t = typename PC::PointType;
  vec3_t ground_point(0, 0, 0);
  vec3_t ground_normal(0, 0, 1);

  typename PC::Ptr pc_filtered = mrs_pcl_tools::filters::applyVoxelGridFilter(m_logger, pc, 0.5);

  return std::nullopt;
}
/*//}*/


#include <mrs_pcl_tools/groundplane_detector_core.tpp>

}  // namespace mrs_pcl_tools