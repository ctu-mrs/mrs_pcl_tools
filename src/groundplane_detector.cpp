#include <mrs_pcl_tools/groundplane_detector.h>

namespace mrs_pcl_tools
{

/* plane_visualization //{ */
visualization_msgs::msg::MarkerArray GroundplaneDetector::plane_visualization(const vec3_t& plane_normal, float plane_d,
                                                                              const std_msgs::msg::Header& header) const
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

  m_add_borders_marker(pos, quat, poly, header, ret.markers);
  m_add_plane_marker(pos, quat, poly, header, ret.markers);
  m_add_normal_marker(pos, plane_normal, header, ret.markers);

  return ret;
}
//}

/* m_add_borders_marker() //{ */
void GroundplaneDetector::m_add_borders_marker(const vec3_t& pos, const quat_t& quat, const polygon_t& poly,
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

/* m_add_normal_marker() //{ */
void GroundplaneDetector::m_add_plane_marker(const vec3_t& pos, const quat_t& quat, const polygon_t& poly,
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

/* m_add_normal_marker() //{ */
void GroundplaneDetector::m_add_normal_marker(const vec3_t& pos, const vec3_t& plane_normal,
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

/* groundplane_detection_config_t constructor //{ */
GroundplaneDetector::groundplane_detection_config_t::groundplane_detection_config_t(mrs_lib::ParamLoader& pl,
                                                                                    const std::string& param_prefix)
{
  loadParams(pl, param_prefix);
}
//}

/* loadParams() //{ */
void GroundplaneDetector::groundplane_detection_config_t::loadParams(mrs_lib::ParamLoader& pl,
                                                                     const std::string& param_prefix)
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
//}

/* plane_t() //{ */
plane_t::plane_t(const vec3_t& normal, const float distance)
  : normal(normal.normalized()), distance(distance / normal.norm())
{
}
//}

}  // namespace mrs_pcl_tools