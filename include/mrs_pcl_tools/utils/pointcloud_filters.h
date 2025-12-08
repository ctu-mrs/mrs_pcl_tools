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

/*//{ applyVoxelGridFilter() */
template <typename PC_t>
typename std::shared_ptr<PC_t> applyVoxelGridFilter(ILogger& logger, std::shared_ptr<PC_t> const& cloud,
                                                    const float resolution)
{
  if (resolution <= 0.0f)
  {
    logger.error(
        "[mrs_pcl_tools::filters::applyVoxelGridFilter] Resolution is less than or equal to zero, not applying voxel "
        "grid filter.");
    return cloud;
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
typename std::shared_ptr<PC_t> applyRadiusOutlierFilter([[maybe_unused]] ILogger& logger,
                                                        std::shared_ptr<PC_t> const& cloud, const float radius,
                                                        const int neighbors, const bool keep_organized = true)
{
  auto cloud_out = std::make_shared<PC_t>();

  pcl::RadiusOutlierRemoval<typename PC_t::PointType> outrem;
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(radius);
  outrem.setMinNeighborsInRadius(neighbors);
  outrem.setKeepOrganized(keep_organized);

  outrem.filter(*cloud_out);

  return cloud_out;
}
/*//}*/

/*//{ applyMinimumGridFilter() */
template <typename PC_t>
typename std::shared_ptr<PC_t> applyMinimumGridFilter([[maybe_unused]] ILogger& logger,
                                                      std::shared_ptr<PC_t> const& cloud, const float resolution)
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
typename std::shared_ptr<PC_t> applyBilateralFilter(ILogger& logger, std::shared_ptr<PC_t> const& cloud,
                                                    const float sigma_S, const float sigma_R)
{
  if (cloud->width <= 1 || cloud->height <= 1)
  {
    logger.error(
        "[mrs_pcl_tools::filters::applyBilateralFilter] Unorganized cloud given, not applying bilateral filter.");
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

}  // namespace filters

}  // namespace mrs_pcl_tools
