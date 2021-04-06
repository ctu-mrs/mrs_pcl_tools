
/* plane_visualization //{ */
visualization_msgs::MarkerArray plane_visualization(const vec3_t& plane_normal, float plane_d, const std_msgs::Header& header)
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

/* removeBelowGround() //{ */

template <typename PC>
typename boost::shared_ptr<PC> RemoveBelowGroundFilter::applyInPlace(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed)
{
  typename PC::Ptr removed_pc_ptr = boost::make_shared<PC>();
  removed_pc_ptr->header = inout_pc->header;
  if (!initialized)
  {
    ROS_ERROR("[RemoveBelowGroundFilter]: not initialized, skipping.");
    return removed_pc_ptr;
  }

  using pt_t = typename PC::PointType;
  vec3_t ground_point(0,0,0);
  vec3_t ground_normal(0,0,1);

  // try to deduce the ground point from the latest rangefinder measurement
  bool range_meas_used = false;
  if (sh_range.hasMsg())
  {
    const auto range_msg = sh_range.getMsg();
    if (range_msg->range > range_msg->min_range && range_msg->range < range_msg->max_range)
    {
      const vec3_t range_vec(range_msg->range, 0,0);
      const auto tf_opt = transformer->getTransform(range_msg->header.frame_id, inout_pc->header.frame_id, range_msg->header.stamp);
      if (tf_opt.has_value())
      {
        ground_point = tf_opt->getTransformEigen().template cast<float>()*range_vec;
        range_meas_used = true;
      }
      else
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[RemoveBelowGroundFilter]: Could not get transformation from " << range_msg->header.frame_id << " to " << inout_pc->header.frame_id << ", cannot use range measurement for ground plane point estimation.");
      }
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[RemoveBelowGroundFilter]: Range measurement is out of bounds, not using it for ground plane point estimation (" << range_msg->range << " not in (" << range_msg->min_range << ", " << range_msg->max_range << "))");
    }
  }

  // prepare a new pointcloud that will contain the filtered points for ground fitting
  typename PC::Ptr pc_filtered = boost::make_shared<PC>();
  // simplify the fitting by filtering the input cloud a bit
  pcl::VoxelGrid<pt_t> vg;
  vg.setInputCloud(inout_pc);
  vg.setLeafSize(0.5, 0.5, 0.5);
  vg.filter(*pc_filtered);

  // try to estimate the ground normal from the static frame
  ros::Time stamp;
  pcl_conversions::fromPCL(inout_pc->header.stamp, stamp);
  const auto tf_opt = transformer->getTransform(static_frame_id, inout_pc->header.frame_id, stamp);
  if (tf_opt.has_value())
  {
    const Eigen::Affine3f tf = tf_opt->getTransformEigen().template cast<float>();
    ground_normal = tf.rotation()*vec3_t(0, 0, 1);
    // if the range measurement is not used for estimation of the ground point, assume that the static frame starts at ground level
    if (!range_meas_used)
      ground_point = tf*vec3_t(0, 0, 0);

    // crop out points above a certain height to reduce the number of non-ground-plane points
    const float plane_d = -ground_normal.dot(ground_point)-max_precrop_height;
    const vec4_t plane_params = -vec4_t(ground_normal.x(), ground_normal.y(), ground_normal.z(), plane_d);
    pcl::IndicesPtr inds_filtered = boost::make_shared<pcl::Indices>();
    pcl::PlaneClipper3D<pt_t> pclip(plane_params);
    pclip.clipPointCloud3D(*pc_filtered, *inds_filtered);
    pcl::ExtractIndices<pt_t> ei;
    ei.setIndices(inds_filtered);
    ei.filterDirectly(pc_filtered);
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "[RemoveBelowGroundFilter]: Could not get transformation from " << static_frame_id << " to " << inout_pc->header.frame_id << ", ground plane may be imprecise.");
  }

  // prepare a SAC plane model with an angular constraint according to the estimated plane normal
  typename pcl::SampleConsensusModelPerpendicularPlane<pt_t>::Ptr model = boost::make_shared<pcl::SampleConsensusModelPerpendicularPlane<pt_t>>(pc_filtered, true);
  model->setAxis(ground_normal);
  model->setEpsAngle(max_angle_diff);

  // actually fit the plane
  pcl::RandomSampleConsensus<pt_t> ransac(model);
  ransac.setDistanceThreshold(max_inlier_dist);
  if (!ransac.computeModel())
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[RemoveBelowGroundFilter]: Could not fit a ground-plane model! Skipping ground removal.");
    return removed_pc_ptr;
  }

  // retreive the fitted model
  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);
  // orient the retrieved normal to point upwards in the static frame
  if (coeffs.block<3, 1>(0, 0).dot(ground_normal) < 0.0f)
    coeffs = -coeffs;
  // just some helper variables
  const vec3_t fit_n = coeffs.block<3, 1>(0, 0);
  const float fit_d = coeffs(3);

  // check if the assumed ground point is an inlier of the fitted plane
  const float ground_pt_dist = std::abs(fit_n.dot(ground_point) + fit_d);
  if (ground_pt_dist >  plane_offset)
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[RemoveBelowGroundFilter]: The RANSAC-fitted ground-plane model [" << coeffs.transpose() << "] is too far from the measured ground (" << ground_pt_dist << "m > " << max_inlier_dist << "m)! Skipping ground removal.");
    return removed_pc_ptr;
  }

  std_msgs::Header header;
  pcl_conversions::fromPCL(inout_pc->header, header);

  if (pub_fitted_plane.has_value())
  {
    visualization_msgs::MarkerArray plane_msg = plane_visualization(fit_n, fit_d, header);
    pub_fitted_plane->publish(plane_msg);
  }

  // get indices of points above the plane
  const vec4_t plane_params(fit_n.x(), fit_n.y(), fit_n.z(), fit_d-plane_offset);
  pcl::IndicesPtr inds_filtered = boost::make_shared<pcl::Indices>();
  pcl::PlaneClipper3D<pt_t> pclip(plane_params);
  pclip.clipPointCloud3D(*inout_pc, *inds_filtered);

  // extract the points
  pcl::ExtractIndices<pt_t> ei;
  ei.setIndices(inds_filtered);
  ei.setInputCloud(inout_pc);
  if (return_removed)
  {
    ei.setNegative(true);
    ei.filter(*removed_pc_ptr);
    ei.setNegative(false);
  }
  ei.filter(*inout_pc);

  return removed_pc_ptr;
}

//}

