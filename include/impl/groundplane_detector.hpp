/* detectGroundplane() //{ */

template <typename PC>
std::optional<plane_t> GroundplaneDetector::detectGroundplane(const typename boost::shared_ptr<const PC>& pc) const
{
  if (!initialized)
  {
    ROS_ERROR_STREAM("[" << NODE_NAME << "]: not initialized, skipping.");
    return std::nullopt;
  }

  using pt_t = typename PC::PointType;
  vec3_t ground_point(0,0,0);
  vec3_t ground_normal(0,0,1);

  // try to deduce the ground point from the latest rangefinder measurement
  bool range_meas_used = false;
  if (m_cfg.range_use && m_sh_range.hasMsg())
  {
    const auto range_msg = m_sh_range.peekMsg();
    if (range_msg->range > range_msg->min_range && range_msg->range < range_msg->max_range)
    {
      const vec3_t range_vec(range_msg->range, 0,0);
      const auto tf_opt = m_tfr->getTransform(range_msg->header.frame_id, pc->header.frame_id, range_msg->header.stamp);
      if (tf_opt.has_value())
      {
        ground_point = tf2::transformToEigen(tf_opt.value().transform).template cast<float>()*range_vec;
        range_meas_used = true;
      }
      else
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[" << NODE_NAME << "]: Could not get transformation from " << range_msg->header.frame_id << " to " << pc->header.frame_id << ", cannot use range measurement for ground plane point estimation.");
      }
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[" << NODE_NAME << "]: Range measurement is out of bounds, not using it for ground plane point estimation (" << range_msg->range << " not in (" << range_msg->min_range << ", " << range_msg->max_range << "))");
    }
  }

  // prepare a new pointcloud that will contain the filtered points for ground fitting
  typename PC::Ptr pc_filtered = boost::make_shared<PC>();
  // simplify the fitting by filtering the input cloud a bit
  pcl::VoxelGrid<pt_t> vg;
  vg.setInputCloud(pc);
  vg.setLeafSize(0.5, 0.5, 0.5);
  vg.filter(*pc_filtered);

  // try to estimate the ground normal from the static frame
  ros::Time stamp;
  pcl_conversions::fromPCL(pc->header.stamp, stamp);
  const auto tf_opt = m_tfr->getTransform(m_cfg.static_frame_id, pc->header.frame_id, stamp);
  if (tf_opt.has_value())
  {
    const Eigen::Affine3f tf = tf2::transformToEigen(tf_opt.value().transform).template cast<float>();
    ground_normal = tf.rotation()*vec3_t(0, 0, 1);
    // if the range measurement is not used for estimation of the ground point, assume that the static frame starts at ground level
    if (!range_meas_used)
      ground_point = tf*vec3_t(0, 0, 0);

    // crop out points above a certain height to reduce the number of non-ground-plane points
    const float plane_d = -ground_normal.dot(ground_point)-m_cfg.max_precrop_height;
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
    ROS_WARN_STREAM_THROTTLE(1.0, "[" << NODE_NAME << "]: Could not get transformation from " << m_cfg.static_frame_id << " to " << pc->header.frame_id << ", ground plane may be imprecise.");
  }

  // prepare a SAC plane model with an angular constraint according to the estimated plane normal
  typename pcl::SampleConsensusModelPerpendicularPlane<pt_t>::Ptr model = boost::make_shared<pcl::SampleConsensusModelPerpendicularPlane<pt_t>>(pc_filtered, true);
  model->setAxis(ground_normal);
  model->setEpsAngle(m_cfg.max_angle_diff);

  // actually fit the plane
  pcl::RandomSampleConsensus<pt_t> ransac(model);
  ransac.setDistanceThreshold(m_cfg.max_inlier_dist);
  if (!ransac.computeModel())
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[" << NODE_NAME << "]: Could not fit a ground-plane model! Skipping detection.");
    return std::nullopt;
  }

  // retreive the fitted model
  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);
  // orient the retrieved normal to point upwards in the static frame
  if (coeffs.block<3, 1>(0, 0).dot(ground_normal) < 0.0f)
    coeffs = -coeffs;
  // just some helper variables
  vec3_t fit_n = coeffs.block<3, 1>(0, 0);
  float fit_d = coeffs(3);

  // check if the assumed ground point is an inlier of the fitted plane
  const float ground_pt_dist = std::abs(fit_n.dot(ground_point) + fit_d);
  const float max_ground_pt_dist = range_meas_used ? m_cfg.range_max_diff : m_cfg.range_max_diff_without_rangefinder;
  if (ground_pt_dist > max_ground_pt_dist)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "[" << NODE_NAME << "]: The RANSAC-fitted ground-plane model [" << coeffs.transpose() << "] is too far from the measured ground (" << ground_pt_dist << "m > " << max_ground_pt_dist << "m)! Using ground-plane based on the fixed frame.");
    const float plane_d = -ground_normal.dot(ground_point);
    coeffs << -ground_normal.x(), -ground_normal.y(), -ground_normal.z(), -plane_d;
    fit_n = coeffs.block<3, 1>(0, 0);
    fit_d = coeffs(3);
  }

  std_msgs::Header header;
  pcl_conversions::fromPCL(pc->header, header);

  if (m_pub_detected_plane.has_value())
  {
    visualization_msgs::MarkerArray plane_msg = plane_visualization(fit_n, fit_d, header);
    m_pub_detected_plane->publish(plane_msg);
  }

  if (m_pub_inlier_points.has_value() && m_pub_inlier_points->getNumSubscribers() > 0)
  {
    pcl::Indices in_inds;
    ransac.getInliers(in_inds);
    const typename PC::ConstPtr pub_pc = boost::make_shared<PC>(*pc, in_inds);
    m_pub_inlier_points->publish(pub_pc);
  }

  return plane_t({fit_n, fit_d});
}

//}
