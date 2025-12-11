#pragma once

#include <mrs_pcl_tools/support.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

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
  [[nodiscard]] std::optional<plane_t> detect(const std::shared_ptr<const PC>& pc,
                                              const std::optional<vec3_t>& range_opt,
                                              const std::optional<Eigen::Affine3f>& static_frame_tf_opt) const;

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
[[nodiscard]] std::optional<plane_t>
GroundplaneDetectorCore::detect(const std::shared_ptr<const PC>& pc, const std::optional<vec3_t>& range_opt,
                                const std::optional<Eigen::Affine3f>& static_frame_tf_opt) const
{
  using pt_t = typename PC::PointType;
  vec3_t ground_point(0, 0, 0);
  vec3_t ground_normal(0, 0, 1);

  bool range_meas_used = false;
  if (m_cfg.range_use && range_opt.has_value())
  {
    ground_point = range_opt.value();
    range_meas_used = true;
  }

  // typename PC::Ptr pc_filtered = mrs_pcl_tools::filters::applyVoxelGridFilter(m_logger, pc, 0.5);
  typename PC::Ptr pc_filtered = std::make_shared<PC>();
  pcl::VoxelGrid<pt_t> vg;
  vg.setInputCloud(pc);
  vg.setLeafSize(0.5, 0.5, 0.5);
  vg.filter(*pc_filtered);

  // try to estimate the ground normal from the static frame
  if (static_frame_tf_opt.has_value())
  {
    ground_normal = static_frame_tf_opt.value() * vec3_t(0, 0, 1);
    if (!range_meas_used)
    {
      ground_point = static_frame_tf_opt.value() * vec3_t(0, 0, 0);
    }

    // crop out points above a certain height to reduce the number of non-ground-plane points
    const float plane_d = -ground_normal.dot(ground_point) - m_cfg.max_precrop_height;
    const vec4_t plane_params = -vec4_t(ground_normal.x(), ground_normal.y(), ground_normal.z(), plane_d);
    pcl::IndicesPtr inds_filtered = std::make_shared<pcl::Indices>();
    pcl::PlaneClipper3D<pt_t> pclip(plane_params);
    pclip.clipPointCloud3D(*pc_filtered, *inds_filtered);
    pcl::ExtractIndices<pt_t> ei;
    ei.setIndices(inds_filtered);
    ei.filterDirectly(pc_filtered);
  }
  else
  {
    m_logger.warn("Do not have transformation, ground plane may be imprecise.");
  }

   // prepare a SAC plane model with an angular constraint according to the estimated plane normal
  typename pcl::SampleConsensusModelPerpendicularPlane<pt_t>::Ptr model = std::make_shared<pcl::SampleConsensusModelPerpendicularPlane<pt_t>>(pc_filtered, true);
  model->setAxis(ground_normal);
  model->setEpsAngle(m_cfg.max_angle_diff);

  // fit the plane
  pcl::RandomSampleConsensus<pt_t> ransac(model);
  ransac.setDistanceThreshold(m_cfg.max_inlier_dist);
  if (!ransac.computeModel())
  {
    m_logger.error("Could not fit a ground-plane model! Skipping detection.");
    // ROS_ERROR_STREAM_THROTTLE(1.0, "[" << NODE_NAME << "]: Could not fit a ground-plane model! Skipping detection.");
    return std::nullopt;
  }

  // retreive the fitted model
  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);
  // orient the retrieved normal to point upwards in the static frame
  if (coeffs.block<3, 1>(0, 0).dot(ground_normal) < 0.0f)
  {
    coeffs = -coeffs;
  }
  // just some helper variables
  vec3_t fit_n = coeffs.block<3, 1>(0, 0);
  float fit_d = coeffs(3);

  // check if the assumed ground point is an inlier of the fitted plane
  const float ground_pt_dist = std::abs(fit_n.dot(ground_point) + fit_d);
  const float max_ground_pt_dist = range_meas_used ? m_cfg.range_max_diff : m_cfg.range_max_diff_without_rangefinder;
  if (ground_pt_dist > max_ground_pt_dist)
  {
    // m_logger.error("The RANSAC-fitted ground-plane model [" + )
    // ROS_WARN_STREAM_THROTTLE(1.0, "[" << NODE_NAME << "]: The RANSAC-fitted ground-plane model [" << coeffs.transpose() << "] is too far from the measured ground (" << ground_pt_dist << "m > " << max_ground_pt_dist << "m)! Using ground-plane based on the fixed frame.");
    const float plane_d = -ground_normal.dot(ground_point);
    coeffs << -ground_normal.x(), -ground_normal.y(), -ground_normal.z(), -plane_d;
    fit_n = coeffs.block<3, 1>(0, 0);
    fit_d = coeffs(3);
  }




  return std::nullopt;
}
/*//}*/

#include <mrs_pcl_tools/groundplane_detector_core.tpp>

}  // namespace mrs_pcl_tools