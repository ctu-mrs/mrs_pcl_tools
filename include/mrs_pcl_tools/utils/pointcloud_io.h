#pragma once

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>
#include <mrs_pcl_tools/utils/pointcloud_fields.h>


namespace mrs_pcl_tools
{

  std::optional<PC::Ptr> loadPcXYZ(ILogger& logger, const std::string& pcd_file);
  std::optional<PC_NORM::Ptr> loadPcNormals(ILogger& logger, const std::string& pcd_file);
  void savePCD(const std::string& pcd_file, const pcl::PCLPointCloud2& cloud, const bool binary = true);

  template <typename PC_t>
  bool loadCloud(ILogger& logger, const std::string& filepath, typename std::shared_ptr<PC_t> const& cloud, const bool verbose = true);

}  // namespace mrs_pcl_tools

#include <mrs_pcl_tools/utils/pointcloud_io.tpp>