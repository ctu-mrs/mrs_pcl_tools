namespace mrs_pcl_tools
{

  /*//{ downsample() */
  template <typename PC>
  void PCLFiltrationCore::downsample(std::shared_ptr<PC>& inout_pc_ptr, const DownsampleConfig& dp)
  {
    DEBUG_LOG(m_logger, "[PCLFiltration]: Applying downsampling");

    const size_t scale_row = dp.row_step;
    const size_t scale_col = dp.col_step;
    const size_t row_offset = [&] {
      if (dp.dynamic_row_selection_enabled)
      {
        return dp.dynamic_row_offset;
      }
      return static_cast<uint32_t>(dp.row_step - 1);
    }();

    if (!inout_pc_ptr)
    {
      m_logger.error("[PCLFiltration] Received null point cloud pointer. Skipping downsampling...");
      return;
    }
    if (inout_pc_ptr->height <= 1 || inout_pc_ptr->width <= 1)
    {
      m_logger.error("[PCLFiltration] Received unorganized pointcloud. Skipping downsampling...");
      return;
    }
    if (scale_row == 0 || scale_col == 0)
    {
      m_logger.error("[PCLFiltration] Division by zero. Skipping downsampling...");
      return;
    }
    if (row_offset >= inout_pc_ptr->height)
    {
      m_logger.error("[PCLFiltration] Row offset out of bounds. Skipping downsampling...");
      return;
    }

    using pt_t = typename PC::PointType;

    const auto [ring_exists, ring_offset] = getFieldOffset<pt_t>("ring");
    const uint8_t scale_row_uint8 = static_cast<uint8_t>(scale_row);

    const size_t height_before = inout_pc_ptr->height;
    const size_t width_before = inout_pc_ptr->width;
    // const size_t height_after = height_before / scale_row;
    // const size_t width_after = width_before / scale_col;
    // Note: to fix overflowing we have to ceil it
    const size_t height_after = (height_before - row_offset + scale_row - 1) / scale_row;
    const size_t width_after = (width_before + scale_col - 1) / scale_col;

    std::shared_ptr<PC> pc_out = std::make_shared<PC>(width_after, height_after);

    size_t r = 0;
    for (size_t j = row_offset; j < height_before; j += scale_row)
    {
      size_t c = 0;
      for (size_t i = 0; i < width_before; i += scale_col)
      {
        pt_t point = inout_pc_ptr->at(i, j);
        if (ring_exists)
        {
          const uint8_t ring = getFieldValue<uint8_t>(point, ring_offset);
          pcl::setFieldValue<pt_t, uint8_t>(point, ring_offset, ring / scale_row_uint8);
        }

        pc_out->at(c++, r) = point;
      }
      r++;
    }

    pc_out->header = inout_pc_ptr->header;
    pc_out->height = height_after;
    pc_out->width = width_after;
    pc_out->is_dense = inout_pc_ptr->is_dense;

    inout_pc_ptr = pc_out;
  }
  /*//}*/


  /*//{ invalidatePoint() */
  template <typename pt_t>
  void PCLFiltrationCore::invalidatePoint(pt_t& point)
  {
    point.x = m_lidar_params.invalid_value;
    point.y = m_lidar_params.invalid_value;
    point.z = m_lidar_params.invalid_value;
  }
  /*//}*/

  /*//{ invalidatePointsAtIndices() */
  template <typename PC>
  void PCLFiltrationCore::invalidatePointsAtIndices(const pcl::IndicesConstPtr& indices, std::shared_ptr<PC>& cloud)
  {
    for (auto it = indices->begin(); it != indices->end(); it++)
    {
      invalidatePoint(cloud->at(*it));
    }
  }
  /*//}*/

  /*//{ removeCloseAndFar() */
  template <typename PC>
  std::shared_ptr<PC> PCLFiltrationCore::removeCloseAndFar(std::shared_ptr<PC>& inout_pc_ptr, const bool return_removed_close, const bool return_removed_far)
  {
    DEBUG_LOG(m_logger, "[PCLFiltration]: Applying removeCloseAndFar");

    typename PC::Ptr removed_pc = std::make_shared<PC>();
    removed_pc->header = inout_pc_ptr->header;

    if (return_removed_close || return_removed_far)
      removed_pc->resize(inout_pc_ptr->size());
    size_t removed_it = 0;

    // Attempt to get the range field name's index
    LowFieldFilterConfig cfg = m_buildLowFieldConfig<PC>();

    for (auto& point : inout_pc_ptr->points)
    {
      RangeInfo range_info = m_computeRangeInfo(point, cfg.range_exists, cfg.range_offset);
      auto [invalid_range_close, invalid_range_far, ignored] = m_evaluateInvalidation(point, cfg, range_info);
      [[maybe_unused]] auto& _ = ignored;  // disable warning

      if (invalid_range_close || invalid_range_far)
      {
        if ((return_removed_far && invalid_range_far) || (return_removed_close && invalid_range_close))
          removed_pc->at(removed_it++) = point;
        invalidatePoint(point);
      }
    }
    removed_pc->resize(removed_it);

    return removed_pc;
  }
  /*//}*/

  /*//{ removeCloseAndFarAndLowFields() */
  template <typename PC>
  std::shared_ptr<PC> PCLFiltrationCore::removeCloseAndFarAndLowFields(std::shared_ptr<PC>& inout_pc_ptr, const bool clip_return_removed_close,
                                                                       const bool clip_return_removed_far)
  {
    DEBUG_LOG(m_logger, "[PCLFiltration]: Applying removeCloseAndFarAndLowFields");

    LowFieldFilterConfig cfg = m_buildLowFieldConfig<PC>();

    if (!cfg.intensity_exists && !cfg.reflectivity_exists)
    {
      return removeCloseAndFar(inout_pc_ptr, clip_return_removed_close, clip_return_removed_far);
    }

    typename PC::Ptr removed_pc = std::make_shared<PC>();
    removed_pc->header = inout_pc_ptr->header;

    if (clip_return_removed_close || clip_return_removed_far)
      removed_pc->resize(inout_pc_ptr->size());
    size_t removed_it = 0;


    for (auto& point : inout_pc_ptr->points)
    {
      RangeInfo range_info = m_computeRangeInfo(point, cfg.range_exists, cfg.range_offset);

      auto [invalid_range_close, invalid_range_far, invalid_field] = m_evaluateInvalidation(point, cfg, range_info);

      // check the invalidation condition
      if (invalid_range_close || invalid_range_far || invalid_field)
      {
        // check the removal condition
        if ((clip_return_removed_far && invalid_range_far) || (clip_return_removed_close && invalid_range_close))
        {
          removed_pc->at(removed_it++) = point;
        }

        invalidatePoint(point);
      }
    }
    removed_pc->resize(removed_it);

    return removed_pc;
  }
  /*//}*/

  /*//{ removeLowFields() */
  template <typename PC>
  std::shared_ptr<PC> PCLFiltrationCore::removeLowFields(std::shared_ptr<PC>& inout_pc_ptr)
  {
    DEBUG_LOG(m_logger, "[PCLFiltration]: Applying removeLowFields");

    LowFieldFilterConfig cfg = m_buildLowFieldConfig<PC>();

    // Prepare pointcloud of removed points
    typename PC::Ptr removed_pc = std::make_shared<PC>();
    removed_pc->header = inout_pc_ptr->header;
    size_t removed_it = 0;

    if (!cfg.intensity_exists && !cfg.reflectivity_exists)
    {
      return removed_pc;
    }

    for (auto& point : inout_pc_ptr->points)
    {
      RangeInfo range_info = m_computeRangeInfo(point, cfg.range_exists, cfg.range_offset);

      auto [ignored_1, ignored_2, invalid] = m_evaluateInvalidation(point, cfg, range_info);
      [[maybe_unused]] auto& _ignored_1 = ignored_1;  // disable warning
      [[maybe_unused]] auto& _ignored_2 = ignored_2;  // disable warning

      if (invalid)
      {
        invalidatePoint(point);
      }
    }
    removed_pc->resize(removed_it);

    return removed_pc;
  }
  /*//}*/

  /*//{ cropBoxPointCloud() */
  template <typename PC>
  void PCLFiltrationCore::cropBoxPointCloud(std::shared_ptr<PC>& inout_pc_ptr, const Eigen::Affine3f& tf)
  {
    vec4_t cb_min;
    cb_min.head<3>() = m_lidar_params.cropbox.min;
    cb_min.w() = -std::numeric_limits<float>::infinity();

    vec4_t cb_max;
    cb_max.head<3>() = m_lidar_params.cropbox.max;
    cb_max.w() = std::numeric_limits<float>::infinity();

    inout_pc_ptr =
        mrs_pcl_tools::filters::applyCropBox<PC>(m_logger, inout_pc_ptr, tf, cb_min, cb_max, m_lidar_params.keep_organized, m_lidar_params.cropbox.crop_inside);
  }
  /*//}*/

  /*//{ removeInfinitePoints() */
  template <typename PC>
  void PCLFiltrationCore::removeInfinitePoints(std::shared_ptr<PC>& inout_pc_ptr)
  {
    const auto orig_pc = inout_pc_ptr;
    inout_pc_ptr = std::make_shared<PC>();
    inout_pc_ptr->header = orig_pc->header;
    inout_pc_ptr->resize(orig_pc->size());
    size_t it = 0;
    for (const auto& pt : orig_pc->points)
    {
      if (pcl::isFinite(pt))
      {
        inout_pc_ptr->at(it++) = pt;
      }
    }
    inout_pc_ptr->resize(it);
  }
  /*//}*/

  /*//{ m_buildLowFieldConfig() */
  template <typename PC>
  LowFieldFilterConfig PCLFiltrationCore::m_buildLowFieldConfig() const
  {
    using pt_t = typename PC::PointType;
    LowFieldFilterConfig cfg;

    if (m_lidar_params.rangeclip.use)
    {
      std::tie(cfg.range_exists, cfg.range_offset) = getFieldOffset<pt_t>("range");
      if (cfg.range_exists)
        INFO_ONCE(m_logger, "[PCLFiltration] Found field name range in point type, will be using range from points.");
      else
        WARN_ONCE(m_logger, "[PCLFiltration] Unable to find field name range in point type, will be using calculated range.");
    }

    if (m_lidar_params.intensity.use)
    {
      std::tie(cfg.intensity_exists, cfg.intensity_offset) = getFieldOffset<pt_t>("intensity");
      if (cfg.intensity_exists)
        INFO_ONCE(m_logger, "[PCLFiltration] Found field name intensity in point type, will be using intensity for filtering.");
      else
        WARN_ONCE(m_logger, "[PCLFiltration] Unable to find field intensity in point type, will NOT be using intensity for filtering.");
    }

    if (m_lidar_params.reflectivity.use)
    {
      std::tie(cfg.reflectivity_exists, cfg.reflectivity_offset) = getFieldOffset<pt_t>("reflectivity");
      if (cfg.reflectivity_exists)
        INFO_ONCE(m_logger, "[PCLFiltration] Found field name reflectivity in point type, will be using reflectivity for filtering.");
      else
        WARN_ONCE(m_logger, "[PCLFiltration] Unable to find field reflectivity in point type, will NOT be using reflectivity for filtering.");
    }

    return cfg;
  }
  /*//}*/

  /*//{ m_computeRangeInfo() */
  template <typename PointT>
  RangeInfo PCLFiltrationCore::m_computeRangeInfo(const PointT& point, bool range_exists, std::size_t range_offset) const
  {
    RangeInfo range_info;

    if (range_exists)
    {
      // Get the range (in millimeters)
      range_info.has_mm = true;
      range_info.range_mm = getFieldValue<uint32_t>(point, range_offset);
      range_info.range_sq = static_cast<float>(range_info.range_mm * range_info.range_mm);
    }
    // otherwise, just calculate the range as the norm
    else
    {
      range_info.has_mm = false;
      const vec3_t pt = point.getArray3fMap();
      range_info.range_sq = pt.squaredNorm();
    }

    return range_info;
  }
  /*//}*/

  /*//{ evaluateInvalidation() */
  template <typename PointT>
  std::tuple<bool, bool, bool> PCLFiltrationCore::m_evaluateInvalidation(const PointT& point, const LowFieldFilterConfig& cfg,
                                                                         const RangeInfo& range_info) const
  {
    bool invalid_range_close = false;
    bool invalid_range_far = false;
    bool invalid_field = false;


    if (range_info.has_mm)
    {
      invalid_range_close = range_info.range_mm < m_lidar_params.rangeclip.min_mm;
      invalid_range_far = range_info.range_mm > m_lidar_params.rangeclip.max_mm;
    } else
    {
      invalid_range_close = range_info.range_sq < m_lidar_params.rangeclip.min_sq;
      invalid_range_far = range_info.range_sq > m_lidar_params.rangeclip.max_sq;
    }

    if (cfg.intensity_exists)
    {
      const float intensity = getFieldValue<float>(point, cfg.intensity_offset);
      if (range_info.has_mm)
        invalid_field = intensity < m_lidar_params.intensity.threshold && range_info.range_mm < m_lidar_params.intensity.range_mm;
      else
        invalid_field = intensity < m_lidar_params.intensity.threshold && range_info.range_sq < m_lidar_params.intensity.range_sq;
    }

    if (!invalid_field && cfg.reflectivity_exists)
    {
      const uint16_t reflefivity = getFieldValue<uint16_t>(point, cfg.reflectivity_offset);
      if (range_info.has_mm)
        invalid_field = reflefivity < m_lidar_params.reflectivity.threshold && range_info.range_mm < m_lidar_params.reflectivity.range_mm;
      else
        invalid_field = reflefivity < m_lidar_params.reflectivity.threshold && range_info.range_sq < m_lidar_params.reflectivity.range_sq;
    }

    return std::make_tuple(invalid_range_close, invalid_range_far, invalid_field);
  }
  /*//}*/


}  // namespace mrs_pcl_tools