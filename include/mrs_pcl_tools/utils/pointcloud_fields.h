#pragma once

#include <tuple>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>


namespace mrs_pcl_tools
{

  PC_NORM::Ptr estimateNormals(const PC::Ptr& cloud, const float& normal_est_radius);
  bool hasNormals(ILogger& logger, const std::string& pcd_file);

  template <typename pt_t>
  std::tuple<bool, std::size_t> getFieldOffset(const std::string& field_name);

  template <typename T, typename pt_t>
  T getFieldValue(const pt_t& point, std::size_t field_offset);

}  // namespace mrs_pcl_tools

#include <mrs_pcl_tools/utils/pointcloud_fields.tpp>