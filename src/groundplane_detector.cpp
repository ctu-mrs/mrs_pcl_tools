#include <mrs_pcl_tools/groundplane_detector.h>

namespace mrs_pcl_tools
{

groundplane_detection_config_t GroundplaneDetector::loadCfg(mrs_lib::ParamLoader& pl,
                                                            const std::string& param_prefix)
{
  groundplane_detection_config_t cfg;
  const std::string orig_prefix = pl.getPrefix();
  pl.setPrefix(param_prefix);
  pl.loadParam("static_frame_id", cfg.static_frame_id);
  pl.loadParam("range/use", cfg.range_use, false);
  pl.loadParam("range/max_difference", cfg.range_max_diff, 1.0);
  pl.loadParam("range/max_difference_without_rangefinder", cfg.range_max_diff_without_rangefinder, 1.5);
  pl.loadParam("max_precrop_height", cfg.max_precrop_height, std::numeric_limits<double>::infinity());
  pl.loadParam("ransac/max_inlier_distance", cfg.max_inlier_dist, 3.0);
  pl.loadParam("ransac/max_angle_difference", cfg.max_angle_diff, 15.0 / 180.0 * M_PI);
  pl.loadParam("publish_plane_marker", cfg.publish_plane_marker, false);
  pl.setPrefix(orig_prefix);
  return cfg;
}

/* initialize() //{ */
void GroundplaneDetector::initialize(rclcpp::Node::SharedPtr nh_, const std::shared_ptr<mrs_lib::Transformer>& tfr,
                                     const groundplane_detection_config_t& cfg)
{
  m_logger_ = new RosLogger(nh_->get_logger());
  m_cfg = cfg;
  m_tfr = tfr;

  m_ground_plane_detector_ = std::make_unique<GroundplaneDetectorCore>(*m_logger_, m_cfg);

  if (m_cfg.range_use)
  {
    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node = nh_;
    shopts.node_name = nh_->get_name();
    shopts.no_message_timeout = rclcpp::Duration(std::chrono::duration<double>(5.0));
    mrs_lib::construct_object(m_sh_range, shopts, "~/rangefinder_in");
  }

  if (cfg.publish_plane_marker)
  {
    mrs_lib::PublisherHandlerOptions pubopts;
    pubopts.node = nh_;
    pubopts.qos = rclcpp::QoS(10);

    // clang-format off
    m_pub_detected_plane = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(pubopts, "~/detected_groundplane_out");
    m_pub_inlier_points = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(pubopts, "~/groundplane_inliers_out");
    // clang-format on
  }

  m_initialized = true;
}
//}

/* initialize() //{ */
void GroundplaneDetector::initialize(rclcpp::Node::SharedPtr nh_, const groundplane_detection_config_t& cfg)
{
  m_logger_ = new RosLogger(nh_->get_logger());

  auto transformer = std::make_shared<mrs_lib::Transformer>(nh_);
  transformer->setLookupTimeout(rclcpp::Duration(std::chrono::duration<double>(0.3)));

  initialize(nh_, transformer, cfg);
}
//}

/* m_planeVisualization() //{ */
visualization_msgs::msg::MarkerArray GroundplaneDetector::m_planeVisualization(
    const vec3_t& plane_normal, float plane_d, const std_msgs::msg::Header& header) const
{
  visualization_msgs::msg::MarkerArray ret;

  const quat_t quat = quat_t::FromTwoVectors(vec3_t::UnitZ(), plane_normal);
  const vec3_t pos = plane_normal * (-plane_d) / plane_normal.norm();

  const double size = 40.0;
  polygon_t poly;
  poly.ptA.x = size;
  poly.ptA.y = size;
  poly.ptA.z = 0;

  poly.ptB.x = -size;
  poly.ptB.y = size;
  poly.ptB.z = 0;

  poly.ptC.x = -size;
  poly.ptC.y = -size;
  poly.ptC.z = 0;

  poly.ptD.x = size;
  poly.ptD.y = -size;
  poly.ptD.z = 0;

  m_addBorderMarker(pos, quat, poly, header, ret.markers);
  m_addPlaneMarker(pos, quat, poly, header, ret.markers);
  m_addNormalMarker(pos, plane_normal, header, ret.markers);

  return ret;
}
//}

/* m_addBorderMarker() //{ */
void GroundplaneDetector::m_addBorderMarker(const vec3_t& pos, const quat_t& quat, const polygon_t& poly,
                                            const std_msgs::msg::Header& header,
                                            std::vector<visualization_msgs::msg::Marker>& markers) const
{
  visualization_msgs::msg::Marker borders_marker;
  borders_marker.header = header;

  borders_marker.ns = "borders";
  borders_marker.id = 0;
  borders_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  borders_marker.action = visualization_msgs::msg::Marker::ADD;

  borders_marker.pose.position.x = pos.x();
  borders_marker.pose.position.y = pos.y();
  borders_marker.pose.position.z = pos.z();

  borders_marker.pose.orientation.x = quat.x();
  borders_marker.pose.orientation.y = quat.y();
  borders_marker.pose.orientation.z = quat.z();
  borders_marker.pose.orientation.w = quat.w();

  borders_marker.scale.x = 0.1;

  borders_marker.color.a = 0.5;  // Don't forget to set the alpha!
  borders_marker.color.r = 0.0;
  borders_marker.color.g = 0.0;
  borders_marker.color.b = 1.0;

  borders_marker.points.push_back(poly.ptA);
  borders_marker.points.push_back(poly.ptB);

  borders_marker.points.push_back(poly.ptB);
  borders_marker.points.push_back(poly.ptC);

  borders_marker.points.push_back(poly.ptC);
  borders_marker.points.push_back(poly.ptD);

  borders_marker.points.push_back(poly.ptD);
  borders_marker.points.push_back(poly.ptA);

  markers.push_back(borders_marker);
}
//}

/* m_addPlaneMarker() //{ */
void GroundplaneDetector::m_addPlaneMarker(const vec3_t& pos, const quat_t& quat, const polygon_t& poly,
                                           const std_msgs::msg::Header& header,
                                           std::vector<visualization_msgs::msg::Marker>& markers) const
{
  visualization_msgs::msg::Marker plane_marker;
  plane_marker.header = header;

  plane_marker.ns = "plane";
  plane_marker.id = 1;
  plane_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  plane_marker.action = visualization_msgs::msg::Marker::ADD;

  plane_marker.pose.position.x = pos.x();
  plane_marker.pose.position.y = pos.y();
  plane_marker.pose.position.z = pos.z();

  plane_marker.pose.orientation.x = quat.x();
  plane_marker.pose.orientation.y = quat.y();
  plane_marker.pose.orientation.z = quat.z();
  plane_marker.pose.orientation.w = quat.w();

  plane_marker.scale.x = 1;
  plane_marker.scale.y = 1;
  plane_marker.scale.z = 1;

  plane_marker.color.a = 0.2;  // Don't forget to set the alpha!
  plane_marker.color.r = 0.0;
  plane_marker.color.g = 0.0;
  plane_marker.color.b = 1.0;

  // triangle ABC
  plane_marker.points.push_back(poly.ptA);
  plane_marker.points.push_back(poly.ptB);
  plane_marker.points.push_back(poly.ptC);

  // triangle ACD
  plane_marker.points.push_back(poly.ptA);
  plane_marker.points.push_back(poly.ptC);
  plane_marker.points.push_back(poly.ptD);
  markers.push_back(plane_marker);
}
//}

/* m_addNormalMarker() //{ */
void GroundplaneDetector::m_addNormalMarker(const vec3_t& pos, const vec3_t& plane_normal,
                                            const std_msgs::msg::Header& header,
                                            std::vector<visualization_msgs::msg::Marker>& markers) const
{
  visualization_msgs::msg::Marker normal_marker;
  normal_marker.header = header;

  normal_marker.ns = "normal";
  normal_marker.id = 2;
  normal_marker.type = visualization_msgs::msg::Marker::ARROW;
  normal_marker.action = visualization_msgs::msg::Marker::ADD;

  normal_marker.pose.position.x = pos.x();
  normal_marker.pose.position.y = pos.y();
  normal_marker.pose.position.z = pos.z();
  normal_marker.pose.orientation.w = 1.0;

  normal_marker.scale.x = 0.05;
  normal_marker.scale.y = 0.05;
  normal_marker.scale.z = 0.3;

  normal_marker.color.a = 0.5;  // Don't forget to set the alpha!
  normal_marker.color.r = 0.0;
  normal_marker.color.g = 0.0;
  normal_marker.color.b = 1.0;

  // direction
  geometry_msgs::msg::Point pt;
  normal_marker.points.push_back(pt);
  pt.x = plane_normal.x();
  pt.y = plane_normal.y();
  pt.z = plane_normal.z();
  normal_marker.points.push_back(pt);
  markers.push_back(normal_marker);
}
//}

// /* m_tryEstimateGroundNormal() //{ */
// bool GroundplaneDetector::m_tryEstimateGroundNormal(vec3_t& ground_normal)
// {
//   // ros::Time stamp;
//   // pcl_conversions::fromPCL(pc->header.stamp, stamp);
//   // const auto tf_opt = m_tfr->getTransform(m_cfg.static_frame_id, pc->header.frame_id, stamp);
//   // if (tf_opt.has_value())
//   // {
//   //   const Eigen::Affine3f tf = tf2::transformToEigen(tf_opt.value().transform).template cast<float>();
//   //   ground_normal = tf.rotation()*vec3_t(0, 0, 1);
//   //   // if the range measurement is not used for estimation of the ground point, assume that the static frame
//   starts
//   //   at ground level if (!range_meas_used)
//   //     ground_point = tf*vec3_t(0, 0, 0);

//   //   // crop out points above a certain height to reduce the number of non-ground-plane points
//   //   const float plane_d = -ground_normal.dot(ground_point)-m_cfg.max_precrop_height;
//   //   const vec4_t plane_params = -vec4_t(ground_normal.x(), ground_normal.y(), ground_normal.z(), plane_d);
//   //   pcl::IndicesPtr inds_filtered = boost::make_shared<pcl::Indices>();
//   //   pcl::PlaneClipper3D<pt_t> pclip(plane_params);
//   //   pclip.clipPointCloud3D(*pc_filtered, *inds_filtered);
//   //   pcl::ExtractIndices<pt_t> ei;
//   //   ei.setIndices(inds_filtered);
//   //   ei.filterDirectly(pc_filtered);
//   //   return true;
//   // }
//   return false;
// }
// //}


}  // namespace mrs_pcl_tools