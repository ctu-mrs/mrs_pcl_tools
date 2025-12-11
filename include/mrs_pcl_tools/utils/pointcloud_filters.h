#pragma once

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/crop_box.h>

namespace mrs_pcl_tools
{

  namespace filters
  {

    template <typename PC_t>
    typename PC_t::Ptr applyVoxelGridFilter([[maybe_unused]] ILogger& logger, const typename PC_t::ConstPtr& cloud, const float resolution);

    template <typename PC_t>
    typename PC_t::Ptr applyRadiusOutlierFilter([[maybe_unused]] ILogger& logger, const typename PC_t::ConstPtr& cloud, const float radius, const int neighbors,
                                                const bool keep_organized = true);

    template <typename PC_t>
    typename PC_t::Ptr applyMinimumGridFilter([[maybe_unused]] ILogger& logger, const typename PC_t::ConstPtr& cloud, const float resolution);

    template <typename PC_t>
    typename PC_t::Ptr applyBilateralFilter([[maybe_unused]] ILogger& logger, const typename PC_t::ConstPtr& cloud, const float sigma_S, const float sigma_R);

    template <typename PC_t>
    typename PC_t::Ptr applyCropBox([[maybe_unused]] ILogger& logger, const typename PC_t::ConstPtr& cloud, const Eigen::Affine3f& transform,
                                    const Eigen::Vector4f& min, const Eigen::Vector4f& max, bool keep_organized, bool set_negative);

  }  // namespace filters

}  // namespace mrs_pcl_tools

#include <mrs_pcl_tools/utils/pointcloud_filters.tpp>
