#pragma once

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/fast_bilateral_omp.h>


namespace filters
{

/*//{ applyVoxelGridFilter() */
template <typename PC_t>
typename std::shared_ptr<PC_t> applyVoxelGridFilter(std::shared_ptr<PC_t> const& cloud, const float resolution)
{
  if (resolution <= 0.0f)
  {
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
typename std::shared_ptr<PC_t> applyRadiusOutlierFilter(std::shared_ptr<PC_t> const& cloud, const float radius,
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
typename std::shared_ptr<PC_t> applyMinimumGridFilter(std::shared_ptr<PC_t> const& cloud, const float resolution)
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
typename std::shared_ptr<PC_t> applyBilateralFilter(std::shared_ptr<PC_t> const &cloud, const float sigma_S, const float sigma_R) {

  if (cloud->width <= 1 || cloud->height <= 1) {
    // ROS_ERROR("[mrs_pcl_tools::filters::applyBilateralFilter] Unorganized cloud given, not applying bilateral filter.");
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