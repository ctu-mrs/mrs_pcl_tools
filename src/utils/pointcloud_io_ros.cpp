#include <mrs_pcl_tools/utils/pointcloud_io_ros.h>

namespace mrs_pcl_tools
{

void savePCD(ILogger& logger, const std::string& pcd_file, const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg,
             const bool binary)
{
  logger.info(std::format("[PCLSupportLibrary] Saving PCD file ({}): {}", binary ? "binary" : "ascii", pcd_file));
  pcl::PCLPointCloud2 cloud;
  pcl_conversions::toPCL(*cloud_msg, cloud);
  mrs_pcl_tools::savePCD(pcd_file, cloud, binary);  // no-ros version
}

}  // namespace mrs_pcl_tools