#include <mrs_pcl_tools/groundplane_detector.h>

namespace mrs_pcl_tools
{
  /* plane_visualization //{ */
  visualization_msgs::MarkerArray GroundplaneDetector::plane_visualization(const vec3_t& plane_normal, float plane_d, const std_msgs::Header& header) const
  {
    visualization_msgs::MarkerArray ret;

    const quat_t quat = quat_t::FromTwoVectors(vec3_t::UnitZ(), plane_normal);
    const vec3_t pos = plane_normal * (-plane_d) / plane_normal.norm();

    const double size = 40.0;
    geometry_msgs::Point ptA;
    ptA.x = size;
    ptA.y = size;
    ptA.z = 0;
    geometry_msgs::Point ptB;
    ptB.x = -size;
    ptB.y = size;
    ptB.z = 0;
    geometry_msgs::Point ptC;
    ptC.x = -size;
    ptC.y = -size;
    ptC.z = 0;
    geometry_msgs::Point ptD;
    ptD.x = size;
    ptD.y = -size;
    ptD.z = 0;

    /* borders marker //{ */
    {
      visualization_msgs::Marker borders_marker;
      borders_marker.header = header;

      borders_marker.ns = "borders";
      borders_marker.id = 0;
      borders_marker.type = visualization_msgs::Marker::LINE_LIST;
      borders_marker.action = visualization_msgs::Marker::ADD;

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

      borders_marker.points.push_back(ptA);
      borders_marker.points.push_back(ptB);

      borders_marker.points.push_back(ptB);
      borders_marker.points.push_back(ptC);

      borders_marker.points.push_back(ptC);
      borders_marker.points.push_back(ptD);

      borders_marker.points.push_back(ptD);
      borders_marker.points.push_back(ptA);

      ret.markers.push_back(borders_marker);
    }
    //}

    /* plane marker //{ */
    {
      visualization_msgs::Marker plane_marker;
      plane_marker.header = header;

      plane_marker.ns = "plane";
      plane_marker.id = 1;
      plane_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      plane_marker.action = visualization_msgs::Marker::ADD;

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
      plane_marker.points.push_back(ptA);
      plane_marker.points.push_back(ptB);
      plane_marker.points.push_back(ptC);

      // triangle ACD
      plane_marker.points.push_back(ptA);
      plane_marker.points.push_back(ptC);
      plane_marker.points.push_back(ptD);
      ret.markers.push_back(plane_marker);
    }
    //}

    /* normal marker //{ */
    {
      visualization_msgs::Marker normal_marker;
      normal_marker.header = header;

      normal_marker.ns = "normal";
      normal_marker.id = 2;
      normal_marker.type = visualization_msgs::Marker::ARROW;
      normal_marker.action = visualization_msgs::Marker::ADD;

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
      geometry_msgs::Point pt;
      normal_marker.points.push_back(pt);
      pt.x = plane_normal.x();
      pt.y = plane_normal.y();
      pt.z = plane_normal.z();
      normal_marker.points.push_back(pt);
      ret.markers.push_back(normal_marker);
    }
    //}

    return ret;
  }
  //}

  plane_t::plane_t(const vec3_t& normal, const float distance)
    : normal(normal.normalized()), distance(distance/normal.norm())
  {
  }

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
    {
      m_pub_detected_plane = nh.advertise<visualization_msgs::MarkerArray>("detected_groundplane", 10);
      m_pub_inlier_points = nh.advertise<sensor_msgs::PointCloud2>("groundplane_inliers", 10);
    }

    initialized = true;
  }

  void GroundplaneDetector::initialize(ros::NodeHandle& nh, const groundplane_detection_config_t& cfg)
  {
    auto transformer = std::make_shared<mrs_lib::Transformer>(NODE_NAME);
    transformer->setLookupTimeout(ros::Duration(0.3));

    initialize(nh, transformer, cfg);
  }

}  // namespace mrs_pcl_tools
