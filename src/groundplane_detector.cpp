#include "mrs_pcl_tools/GroundplaneDetector.h"

namespace mrs_pcl_tools
{
  using vec3_t = Eigen::Vector3f;
  using vec4_t = Eigen::Vector4f;
  using quat_t = Eigen::Quaternionf;

  GroundplaneDetector::groundplane_detection_config_t::groundplane_detection_config_t(mrs_lib::ParamLoader& pl, const std::string& param_prefix)
  {
    loadParams(pl, param_prefix);
  }

  void GroundplaneDetector::groundplane_detection_config_t::loadParams(mrs_lib::ParamLoader& pl, const std::string& param_prefix)
  {
    const std::string orig_prefix = pl.getPrefix();
    pl.setPrefix(param_prefix);
    pl.loadParam("static_frame_id", static_frame_id);
    pl.loadParam("range/use", range_use, false);
    pl.loadParam("range/max_difference", range_max_diff, 1.0);
    pl.loadParam("range/max_difference_without_rangefinder", range_max_diff_without_rangefinder, 1.5);
    pl.loadParam("max_precrop_height", max_precrop_height, std::numeric_limits<double>::infinity());
    pl.loadParam("ransac/max_inlier_distance", max_inlier_dist, 3.0);
    pl.loadParam("ransac/max_angle_difference", max_angle_diff, 15.0 / 180.0 * M_PI);
    pl.loadParam("publish_plane_marker", publish_plane_marker, false);
    pl.setPrefix(orig_prefix);
  }

  void GroundplaneDetector::initialize(ros::NodeHandle& nh, const std::shared_ptr<mrs_lib::Transformer>& tfr, const groundplane_detection_config_t& cfg)
  {
    m_cfg = cfg;
    m_tfr = tfr;

    if (m_cfg.range_use)
    {
      mrs_lib::SubscribeHandlerOptions shopts(nh);
      shopts.node_name          = NODE_NAME;
      shopts.no_message_timeout = ros::Duration(5.0);
      mrs_lib::construct_object(m_sh_range, shopts, "rangefinder_in");
    }

    if (cfg.publish_plane_marker)
      m_pub_detected_plane = nh.advertise<visualization_msgs::MarkerArray>("detected_groundplane", 10);

    initialized = true;
  }

  void GroundplaneDetector::initialize(ros::NodeHandle& nh, const groundplane_detection_config_t& cfg)
  {
    auto transformer = std::make_shared<mrs_lib::Transformer>(NODE_NAME);
    transformer->setLookupTimeout(ros::Duration(0.3));

    initialize(nh, transformer, cfg);
  }

}  // namespace mrs_pcl_tools
