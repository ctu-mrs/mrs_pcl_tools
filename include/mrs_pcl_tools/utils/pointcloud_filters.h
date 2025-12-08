#pragma once

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/fast_bilateral_omp.h>

namespace mrs_pcl_tools
{

namespace filters
{

template <typename PC_t>
typename std::shared_ptr<PC_t> applyVoxelGridFilter(ILogger& logger, std::shared_ptr<PC_t> const& cloud,
                                                    const float resolution);

template <typename PC_t>
typename std::shared_ptr<PC_t> applyRadiusOutlierFilter([[maybe_unused]] ILogger& logger,
                                                        std::shared_ptr<PC_t> const& cloud, const float radius,
                                                        const int neighbors, const bool keep_organized = true);

template <typename PC_t>
typename std::shared_ptr<PC_t> applyMinimumGridFilter([[maybe_unused]] ILogger& logger,
                                                      std::shared_ptr<PC_t> const& cloud, const float resolution);

template <typename PC_t>
typename std::shared_ptr<PC_t> applyBilateralFilter(ILogger& logger, std::shared_ptr<PC_t> const& cloud,
                                                    const float sigma_S, const float sigma_R);

}  // namespace filters

}  // namespace mrs_pcl_tools

#include <mrs_pcl_tools/utils/pointcloud_filters.tpp>
