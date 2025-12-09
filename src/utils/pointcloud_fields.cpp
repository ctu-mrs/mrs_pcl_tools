#include <mrs_pcl_tools/utils/pointcloud_fields.h>

namespace mrs_pcl_tools
{

/*//{ estimateNormals() */
PC_NORM::Ptr estimateNormals(const PC::Ptr& cloud, const float& normal_est_radius)
{
  // XYZ type to XYZNormalPoint
  const PC_NORM::Ptr cloud_norm = std::make_shared<PC_NORM>();
  pcl::copyPointCloud(*cloud, *cloud_norm);

  // Estimate normals
  pcl::NormalEstimationOMP<pt_NORM, pt_NORM> nest;
  nest.setRadiusSearch(normal_est_radius);
  nest.setInputCloud(cloud_norm);
  nest.compute(*cloud_norm);

  return cloud_norm;
}
/*//}*/

/*//{ hasNormals() */
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
/*//}*/

}  // namespace mrs_pcl_tools
