/* removeBelowGround() //{ */

template <typename PC>
typename boost::shared_ptr<PC> RemoveBelowGroundFilter::applyInPlace(const typename boost::shared_ptr<PC>& inout_pc, const bool return_removed)
{
  typename PC::Ptr removed_pc_ptr = boost::make_shared<PC>();
  removed_pc_ptr->header = inout_pc->header;
  if (!initialized)
  {
    ROS_ERROR("[RemoveBelowGroundFilter]: not initialized, skipping.");
    return removed_pc_ptr;
  }

  using pt_t = typename PC::PointType;
  
  const typename boost::shared_ptr<const PC> const_pc(inout_pc);
  const auto plane_opt = m_ground_detector.detectGroundplane(const_pc);
  if (!plane_opt.has_value())
  {
    ROS_ERROR("[RemoveBelowGroundFilter]: failed to find ground plane, skipping.");
    return removed_pc_ptr;
  }
  const auto plane = plane_opt.value();
  const vec3_t fit_n = plane.normal;
  const float fit_d = plane.distance;

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
  ei.setKeepOrganized(keep_organized);
  ei.filter(*inout_pc);

  return removed_pc_ptr;
}

//}

