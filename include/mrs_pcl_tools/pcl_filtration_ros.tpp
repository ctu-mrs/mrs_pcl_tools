/*//{ m_cropBoxPointCloud() */
template <typename PC>
void PCLFiltration::m_cropBoxPointCloud(std::shared_ptr<PC>& inout_pc_ptr)
{
  Eigen::Affine3d tf = Eigen::Affine3d::Identity();

  if (!m_lidar_params.cropbox.frame_id.empty())
  {
    rclcpp::Time stamp;
    pcl_conversions::fromPCL(inout_pc_ptr->header.stamp, stamp);
    const auto tf_opt = m_transformer_->getTransform(inout_pc_ptr->header.frame_id, m_lidar_params.cropbox.frame_id, stamp);
    if (!tf_opt.has_value())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[PCLFiltration]: Could not find pointcloud transformation (from \"%s\" to \"%s\")! "
                           "Not applying CropBox filter.",
                           inout_pc_ptr->header.frame_id.c_str(), m_lidar_params.cropbox.frame_id.c_str());
      return;
    }
    tf = tf2::transformToEigen(tf_opt.value().transform);
  }
  m_pcl_filtration_core_->cropBoxPointCloud(inout_pc_ptr, tf.cast<float>());
}
/*//}*/

/*//{ m_logPointCloudStats() */
template <typename PC>
void PCLFiltration::m_logPointCloudStats(mrs_lib::ScopeTimer& timer, std::shared_ptr<PC>& inout_pc_ptr, const size_t height_before, const size_t width_before,
                                         const size_t points_before)
{
  const size_t height_after = inout_pc_ptr->height;
  const size_t width_after = inout_pc_ptr->width;
  size_t points_after = 0;
  if (m_lidar_params.keep_organized)
  {
    for (const auto& pt : *inout_pc_ptr)
      if (pcl::isFinite(pt))
        points_after++;
  } else
    points_after = inout_pc_ptr->size();

  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "[PCLFiltration] Processed 3D LIDAR data (run time: %.1f ms; points before: %lu, after: %lu; dim before: (w: %lu, h: %lu), after: (w: %lu, h: %lu)).",
      timer.getLifetime(), points_before, points_after, width_before, height_before, width_after, height_after);
}
/*//}*/
