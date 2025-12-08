#include <mrs_pcl_tools/utils/pointcloud_io.h>

namespace mrs_pcl_tools
{

/*//{ loadPcXYZ() */
std::optional<PC::Ptr> loadPcXYZ(ILogger& logger, const std::string& pcd_file)
{
  PC::Ptr pc = std::make_shared<PC>();
  
  logger.info("[PCLSupportLibrary] Reading pointcloud from path " + pcd_file);
  if (pcl::io::loadPCDFile<pt_XYZ>(pcd_file, *pc) < 0)
  {
    logger.error("[PCLSupportLibrary] Couldn't read PCD file from path: " + pcd_file);
    return nullptr;
  }
  logger.info("[PCLSupportLibrary] Loaded XYZ pcl with " + std::to_string(pc->points.size()) + " points.");
  return pc;
}
/*//}*/

/*//{ loadPcNormals() */
std::optional<PC_NORM::Ptr> loadPcNormals(ILogger& logger, const std::string& pcd_file)
{
  // Check if normals are present in the PCD file by looking at the header
  if (!hasNormals(logger, pcd_file))
  {
    return nullptr;
  }

  // Load PCD file
  PC_NORM::Ptr cloud = std::make_shared<PC_NORM>();
  logger.info("[PCLSupportLibrary] Loading normals from PCD file: " + pcd_file);
  if (pcl::io::loadPCDFile(pcd_file, *cloud) < 0)
  {
    logger.error("[PCLSupportLibrary] Couldn't read normals of PCD file: " + pcd_file);
    return nullptr;
  }

  logger.info("[PCLSupportLibrary] Loaded PCL normals with " + std::to_string(cloud->points.size()) + " points.");
  return cloud;
}
/*//}*/

bool hasNormals(ILogger& logger, const std::string& pcd_file)
{
  pcl::PCDReader reader_pcd;
  pcl::PCLPointCloud2 pc;
  if (reader_pcd.readHeader(pcd_file, pc) < 0)
  {
    logger.error("[PCLSupportLibrary] Couldn't read header of PCD file: " + pcd_file);
    return false;
  }

  unsigned int normal_fields = 0;
  bool curvature_field = false;
  for (const auto& field : pc.fields)
  {
    if (field.name.rfind("normal", 0) == 0)
    {
      normal_fields++;
    }
    else if (field.name == "curvature")
    {
      curvature_field = true;
    }
  }
  if (normal_fields != 3 || !curvature_field)
  {
    return false;
  }

  return true;
}

/*//{ savePCD() */
void savePCD(const std::string& pcd_file, const pcl::PCLPointCloud2& cloud, const bool binary)
{
  pcl::io::savePCDFile(pcd_file, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binary);
}
/*//}*/

}  // namespace mrs_pcl_tools