namespace mrs_pcl_tools
{

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


  /*//{ m_processPointCloud() */
  template <typename PC>
  void PCLFiltration::m_processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, mrs_modules_msgs::msg::PclToolsDiagnostics& diag_msg)
  {
    typename PC::Ptr cloud = std::make_shared<PC>();
    pcl::fromROSMsg(*msg, *cloud);

    m_processMsg(cloud);
    diag_msg.cols_after = cloud->width;
    diag_msg.rows_after = cloud->height;
  }
  /*//}*/

  /*//{ m_processMsg() */
  template <typename PC>
  void PCLFiltration::m_processMsg(std::shared_ptr<PC>& inout_pc_ptr)
  {
    if (!inout_pc_ptr)
    {
      RCLCPP_WARN(this->get_logger(), "[PCLFiltration] Received null point cloud pointer. Skipping.");
      return;
    }

    mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer(m_node_, "PCLFiltration::process_msg", m_scope_timer_logger, m_scope_timer_enabled);

    const size_t height_before = inout_pc_ptr->height;
    const size_t width_before = inout_pc_ptr->width;
    const size_t points_before = inout_pc_ptr->size();

    if (m_lidar_params.downsample.use)
    {
      DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying downsampling");

      m_pcl_filtration_core_->downsample(inout_pc_ptr, m_lidar_params.downsample);
      m_pcl_filtration_core_->updateDownsampleParams(m_lidar_params.downsample);
    }

    const bool use_intensity_or_reflectivity = m_lidar_params.intensity.use || m_lidar_params.reflectivity.use;
    if (m_lidar_params.rangeclip.use)
    {
      DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying range-clipping");
      const bool publish_removed_far = m_pub_lidar_over_max_range.getNumSubscribers() > 0;
      typename PC::Ptr pcl_over_max_range;

      if (use_intensity_or_reflectivity)
      {
        DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying removeCloseAndFarAndLowFields");
        pcl_over_max_range = m_pcl_filtration_core_->removeCloseAndFarAndLowFields(inout_pc_ptr, false, publish_removed_far);
      } else
      {
        DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying removeCloseAndFar");
        pcl_over_max_range = m_pcl_filtration_core_->removeCloseAndFar(inout_pc_ptr, false, publish_removed_far);
      }
      m_publishOverMaxRange(pcl_over_max_range);
    } else if (use_intensity_or_reflectivity)
    {
      DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying removeCloseAndFar");
      m_pcl_filtration_core_->removeLowFields(inout_pc_ptr);
    }

    if (m_lidar_params.cropbox.use)
    {
      DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying cropbox filter");
      m_cropBoxPointCloud(inout_pc_ptr);
    }

    if (!m_lidar_params.keep_organized)
    {
      DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying removeInfinitePoints");
      m_pcl_filtration_core_->removeInfinitePoints(inout_pc_ptr);
      inout_pc_ptr->is_dense = true;
    }


    sensor_msgs::msg::PointCloud2 pcl_msg;
    pcl::toROSMsg(*inout_pc_ptr, pcl_msg);
    m_pub_lidar.publish(pcl_msg);


    m_logPointCloudStats(timer, inout_pc_ptr, height_before, width_before, points_before);
  }
  /*//}*/


  /*//{ publishOverMaxRange() */
  template <typename PC>
  void PCLFiltration::m_publishOverMaxRange(const std::shared_ptr<PC>& pc)
  {
    if (!pc || m_pub_lidar_over_max_range.getNumSubscribers() == 0)
      return;

    sensor_msgs::msg::PointCloud2 pcl_msg;
    pcl::toROSMsg(*pc, pcl_msg);
    m_pub_lidar_over_max_range.publish(pcl_msg);
  }
  /*//}*/

}  // namespace mrs_pcl_tools