namespace mrs_pcl_tools
{

  namespace filters
  {

    /*//{ applyVoxelGridFilter() */
    template <typename PC_t>
    typename PC_t::Ptr applyVoxelGridFilter([[maybe_unused]] ILogger& logger, const typename PC_t::ConstPtr& cloud, const float resolution)
    {
      if (resolution <= 0.0f)
      {
        logger.error(
            "[mrs_pcl_tools::filters::applyVoxelGridFilter] Resolution is less than or equal to zero, not applying voxel "
            "grid filter.");
        return std::make_shared<PC_t>(*cloud);
      }

      pcl::VoxelGrid<typename PC_t::PointType> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(resolution, resolution, resolution);

      auto cloud_out = std::make_shared<PC_t>();
      vg.filter(*cloud_out);

      return cloud_out;
    }
    /*//}*/

    /*//{ applyRadiusOutlierFilter() */
    template <typename PC_t>
    typename PC_t::Ptr applyRadiusOutlierFilter([[maybe_unused]] ILogger& logger, const typename PC_t::ConstPtr& cloud, const float radius, const int neighbors,
                                                const bool keep_organized)
    {

      pcl::RadiusOutlierRemoval<typename PC_t::PointType> outrem;
      outrem.setInputCloud(cloud);
      outrem.setRadiusSearch(radius);
      outrem.setMinNeighborsInRadius(neighbors);
      outrem.setKeepOrganized(keep_organized);

      auto cloud_out = std::make_shared<PC_t>();
      outrem.filter(*cloud_out);

      return cloud_out;
    }
    /*//}*/

    /*//{ applyMinimumGridFilter() */
    template <typename PC_t>
    typename PC_t::Ptr applyMinimumGridFilter([[maybe_unused]] ILogger& logger, const typename PC_t::ConstPtr& cloud, const float resolution)
    {
      pcl::GridMinimum<typename PC_t::PointType> gmf(resolution);
      gmf.setInputCloud(cloud);

      auto cloud_out = std::make_shared<PC_t>();
      gmf.filter(*cloud_out);

      return cloud_out;
    }
    /*//}*/

    /*//{ applyBilateralFilter() */
    template <typename PC_t>
    typename PC_t::Ptr applyBilateralFilter([[maybe_unused]] ILogger& logger, const typename PC_t::ConstPtr& cloud, const float sigma_S, const float sigma_R)
    {
      if (cloud->width <= 1 || cloud->height <= 1)
      {
        logger.error("[mrs_pcl_tools::filters::applyBilateralFilter] Unorganized cloud given, not applying bilateral filter.");
        return cloud;
      }

      pcl::FastBilateralFilterOMP<typename PC_t::PointType> fbf;
      fbf.setInputCloud(cloud);
      fbf.setSigmaS(sigma_S);
      fbf.setSigmaR(sigma_R);

      auto cloud_out = std::make_shared<PC_t>();
      fbf.applyFilter(*cloud_out);

      return cloud_out;
    }
    /*//}*/

    /*//{ applyCropBox() */
    template <typename PC_t>
    typename PC_t::Ptr applyCropBox([[maybe_unused]] ILogger& logger, const typename PC_t::ConstPtr& cloud, const Eigen::Affine3f& transform,
                                    const Eigen::Vector4f& min, const Eigen::Vector4f& max, bool keep_organized, bool set_negative)
    {
      bool extract_removed_indices{false};
      if (keep_organized)
      {
        extract_removed_indices = true;
      }
      pcl::CropBox<typename PC_t::PointType> cb(extract_removed_indices);


      cb.setTransform(transform);
      cb.setKeepOrganized(keep_organized);
      cb.setNegative(set_negative);
      cb.setMin(min);
      cb.setMax(max);
      cb.setInputCloud(cloud);

      auto cloud_out = std::make_shared<PC_t>();
      cb.filter(*cloud_out);

      return cloud_out;
    }

    /*//}*/

  }  // namespace filters
}  // namespace mrs_pcl_tools