# include <pcl_conversions/pcl_conversions.h>

// https://discourse.ros.org/t/optimized-ros-pcl-conversion/25833
namespace pcl_df
{

// Usage: substitute pcl::fromROSMsg with pcl_df::fromROSMsg in your code and you have done.
// You should experience a considerable speedup.
  
template<typename T>
void fromROSMsg(const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)
{
  // Code copied and pasted from the implementation of pcl::fromPCLPointCloud2
  
  // Copy info fields
  pcl_conversions::toPCL(cloud.header, pcl_cloud.header);
  pcl_cloud.width    = cloud.width;
  pcl_cloud.height   = cloud.height;
  pcl_cloud.is_dense = cloud.is_dense == 1;
  
  pcl::MsgFieldMap field_map;
  std::vector<pcl::PCLPointField> msg_fields;
  pcl_conversions::toPCL(cloud.fields, msg_fields);
  pcl::createMapping<T> (msg_fields, field_map);
  
  // Copy point data
  std::uint32_t num_points = cloud.width * cloud.height;
  pcl_cloud.points.resize (num_points);
  std::uint8_t* cloud_data = reinterpret_cast<std::uint8_t*>(&pcl_cloud.points[0]);
  
  // Check if we can copy adjacent points in a single memcpy.  We can do so if there
  // is exactly one field to copy and it is the same size as the source and destination
  // point types.
  if (field_map.size() == 1 &&
      field_map[0].serialized_offset == 0 &&
      field_map[0].struct_offset == 0 &&
      field_map[0].size == cloud.point_step &&
      field_map[0].size == sizeof(T))
  {
    std::uint32_t cloud_row_step = static_cast<std::uint32_t> (sizeof (T) * pcl_cloud.width);
    const std::uint8_t* msg_data = &cloud.data[0];
    // Should usually be able to copy all rows at once
    if (cloud.row_step == cloud_row_step)
    {
      memcpy (cloud_data, msg_data, cloud.data.size ());
    }
    else
    {
      for (std::uint32_t i = 0; i < cloud.height; ++i, cloud_data += cloud_row_step, msg_data += cloud.row_step)
        memcpy (cloud_data, msg_data, cloud_row_step);
    }
  }
  else
  {
    // If not, memcpy each group of contiguous fields separately
    for (std::uint32_t row = 0; row < cloud.height; ++row)
    {
      const std::uint8_t* row_data = &cloud.data[row * cloud.row_step];
      for (std::uint32_t col = 0; col < cloud.width; ++col)
      {
        const std::uint8_t* msg_data = row_data + col * cloud.point_step;
        for (const pcl::detail::FieldMapping& mapping : field_map)
        {
          memcpy (cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
        }
        cloud_data += sizeof (T);
      }
    }
  }
}

} // end namespace pcl_df
