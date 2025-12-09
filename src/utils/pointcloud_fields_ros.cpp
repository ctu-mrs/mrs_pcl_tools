#include <mrs_pcl_tools/utils/pointcloud_fields_ros.h>

namespace mrs_pcl_tools
{

/*//{ hasNormals() */
bool hasNormals(const std::vector<sensor_msgs::msg::PointField>& fields)
{
  // Check header for normals (normal_x, normal_y, normal_z, curvature)
  unsigned int normal_fields = 0;
  bool curvature_field = false;
  for (const auto& field : fields)
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

/*//{ hasNormals() */
bool hasNormals(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
{
  return hasNormals(cloud->fields);
}
/*//}*/

/*//{ hasField() */
bool hasField(const std::string& field, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  for (const auto& f : msg->fields)
  {
    if (f.name == field)
    {
      return true;
    }
  }
  return false;
}
/*//}*/

}  // namespace mrs_pcl_tools