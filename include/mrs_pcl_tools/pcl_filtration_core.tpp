/*//{ downsample() */
template <typename PC>
void PCLFiltrationCore::downsample(std::shared_ptr<PC>& inout_pc_ptr, const size_t scale_row, const size_t scale_col, const size_t row_offset)
{
  if (!inout_pc_ptr)
  {
    m_logger.error("[PCLFiltration] Received null point cloud pointer. Skipping downsampling...");
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
  const size_t height_after = height_before / scale_row;
  const size_t width_after = width_before / scale_col;

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
  using pt_t = typename PC::PointType;

  // Prepare pointcloud of removed points
  typename PC::Ptr removed_pc = std::make_shared<PC>();
  removed_pc->header = inout_pc_ptr->header;
  if (return_removed_close || return_removed_far)
    removed_pc->resize(inout_pc_ptr->size());
  size_t removed_it = 0;

  // Attempt to get the range field name's index
  const auto [range_exists, range_offset] = getFieldOffset<pt_t>("range");
  if (range_exists)
  {
    INFO_ONCE(m_logger, "[PCLFiltration] Found field name \"range\" in point type, will be using range from points.");
  } else
  {
    WARN_ONCE(m_logger, "[PCLFiltration] Unable to find field name \"range\" in point type, will be using calculated range.");
  }

  for (auto& point : inout_pc_ptr->points)
  {
    bool invalid_close = false;
    bool invalid_far = false;

    // if the range field is available, use it
    if (range_exists)
    {
      // Get the range (in millimeters)
      const auto range = getFieldValue<uint32_t>(point, range_offset);
      invalid_close = range < m_lidar_params.rangeclip.min_mm;
      invalid_far = range > m_lidar_params.rangeclip.max_mm;
    }
    // otherwise, just calculate the range as the norm
    else
    {
      const vec3_t pt = point.getArray3fMap();
      const float range_sq = pt.squaredNorm();
      invalid_close = range_sq < m_lidar_params.rangeclip.min_sq;
      invalid_far = range_sq > m_lidar_params.rangeclip.max_sq;
    }

    if (invalid_close || invalid_far)
    {
      if ((return_removed_far && invalid_far) || (return_removed_close && invalid_close))
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
  using pt_t = typename PC::PointType;

  typename PC::Ptr removed_pc = std::make_shared<PC>();
  removed_pc->header = inout_pc_ptr->header;
  if (clip_return_removed_close || clip_return_removed_far)
    removed_pc->resize(inout_pc_ptr->size());
  size_t removed_it = 0;


  // Attempt to get the intensity field name's index
  auto [intensity_exists, intensity_offset] = getFieldOffset<pt_t>("intensity");


  if (!intensity_exists)
  {
    return removeCloseAndFar(inout_pc_ptr, clip_return_removed_close, clip_return_removed_far);
  }


  bool filter_intensity = m_lidar_params.intensity.use;
  bool filter_reflectivity = m_lidar_params.reflectivity.use;

  std::size_t reflectivity_offset;

  // Attempt to get the fields' name indices
  if (filter_intensity)
  {
    INFO_ONCE(m_logger, "[PCLFiltration] Found field name \"intensity\" in point type, will be using intensity for filtering.");
    std::tie(filter_intensity, intensity_offset) = getFieldOffset<pt_t>("intensity");
  }
  if (filter_reflectivity)
  {
    INFO_ONCE(m_logger, "[PCLFiltration] Found field name \"reflectivity\" in point type, will be using reflectivity for filtering.");
    std::tie(filter_reflectivity, reflectivity_offset) = getFieldOffset<pt_t>("reflectivity");
  }

  // Attempt to get the range field name's index
  const auto [range_exists, range_offset] = getFieldOffset<pt_t>("range");
  if (range_exists)
  {
    INFO_ONCE(m_logger, "[PCLFiltration] Found field name \"range\" in point type, will be using range from points.");
  } else
  {
    WARN_ONCE(m_logger, "[PCLFiltration] Unable to find field name \"range\" in point type, will be using calculated range.");
  }


  for (auto& point : inout_pc_ptr->points)
  {

    bool invalid_range_close = false;
    bool invalid_range_far = false;
    bool invalid_field = false;

    // if the range field is available, use it
    if (range_exists)  // nevermind this condition inside a loop - the branch predictor will optimize this out easily
    {
      const auto range = getFieldValue<uint32_t>(point, range_offset);
      invalid_range_close = range < m_lidar_params.rangeclip.min_mm;
      invalid_range_far = range > m_lidar_params.rangeclip.max_mm;

      // Filter by field values
      if (filter_intensity)
      {
        const float intensity = getFieldValue<float>(point, intensity_offset);
        invalid_field = intensity < m_lidar_params.intensity.threshold && range < m_lidar_params.intensity.range_mm;
      }
      if (!invalid_field && filter_reflectivity)
      {
        const uint16_t reflectivity = getFieldValue<uint16_t>(point, reflectivity_offset);
        invalid_field = reflectivity < m_lidar_params.reflectivity.threshold && range < m_lidar_params.reflectivity.range_mm;
      }

    }
    // otherwise, just calculate the range as the norm
    else
    {
      const vec3_t pt = point.getArray3fMap();
      const float range_sq = pt.squaredNorm();
      invalid_range_close = range_sq < m_lidar_params.rangeclip.min_sq;
      invalid_range_far = range_sq > m_lidar_params.rangeclip.max_sq;

      // Filter by field values
      if (filter_intensity)
      {
        const float intensity = getFieldValue<float>(point, intensity_offset);
        invalid_field = intensity < m_lidar_params.intensity.threshold && range_sq < m_lidar_params.intensity.range_sq;
      }
      if (!invalid_field && filter_reflectivity)
      {
        const uint16_t reflectivity = getFieldValue<uint16_t>(point, reflectivity_offset);
        invalid_field = reflectivity < m_lidar_params.reflectivity.threshold && range_sq < m_lidar_params.reflectivity.range_sq;
      }
    }

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
  using pt_t = typename PC::PointType;

  // Prepare pointcloud of removed points
  typename PC::Ptr removed_pc = std::make_shared<PC>();
  removed_pc->header = inout_pc_ptr->header;

  size_t removed_it = 0;

  // Attempt to get the intensity field name's index
  auto [intensity_exists, intensity_offset] = getFieldOffset<pt_t>("intensity");

  if (!intensity_exists)
  {
    return removed_pc;
  }

  bool filter_intensity = m_lidar_params.intensity.use;
  bool filter_reflectivity = m_lidar_params.reflectivity.use;

  std::size_t reflectivity_offset;

  // Attempt to get the fields' name indices
  if (filter_intensity)
  {
    INFO_ONCE(m_logger, "[PCLFiltration] Found field name \"intensity\" in point type, will be using intensity for filtering.");
    std::tie(filter_intensity, intensity_offset) = getFieldOffset<pt_t>("intensity");
  }
  if (filter_reflectivity)
  {
    INFO_ONCE(m_logger, "[PCLFiltration] Found field name \"reflectivity\" in point type, will be using reflectivity for filtering.");
    std::tie(filter_reflectivity, reflectivity_offset) = getFieldOffset<pt_t>("reflectivity");
  }

  // Attempt to get the range field name's index
  const auto [range_exists, range_offset] = getFieldOffset<pt_t>("range");
  if (range_exists)
  {
    INFO_ONCE(m_logger, "[PCLFiltration] Found field name \"range\" in point type, will be using range from points.");
  } else
  {
    WARN_ONCE(m_logger, "[PCLFiltration] Unable to find field name \"range\" in point type, will be using calculated range.");
  }

  for (auto& point : inout_pc_ptr->points)
  {

    bool invalid = false;

    // Filter by field values
    if (filter_intensity)
    {
      const float intensity = getFieldValue<float>(point, intensity_offset);
      invalid = intensity < m_lidar_params.intensity.threshold;
    }
    if (!invalid && filter_reflectivity)
    {
      const uint16_t reflectivity = getFieldValue<uint16_t>(point, reflectivity_offset);
      invalid = reflectivity < m_lidar_params.reflectivity.threshold;
    }

    // check the removal condition
    if (invalid)
    {

      // if the range field is available, use it
      if (range_exists)
      {
        // Get the range (in millimeters)
        const auto range = getFieldValue<uint32_t>(point, range_offset);

        if (filter_intensity)
        {
          invalid = range < m_lidar_params.intensity.range_mm;
        }
        if (!invalid && filter_reflectivity)
        {
          invalid = range < m_lidar_params.reflectivity.range_mm;
        }
      }
      // otherwise, just calculate the range as the norm
      else
      {
        const vec3_t pt = point.getArray3fMap();
        const float range_sq = pt.squaredNorm();

        if (filter_intensity)
        {
          invalid = range_sq < m_lidar_params.intensity.range_sq;
        }
        if (!invalid && filter_reflectivity)
        {
          invalid = range_sq < m_lidar_params.reflectivity.range_sq;
        }
      }

      if (invalid)
      {
        invalidatePoint(point);
      }
    }
  }
  removed_pc->resize(removed_it);

  return removed_pc;
}
/*//}*/
