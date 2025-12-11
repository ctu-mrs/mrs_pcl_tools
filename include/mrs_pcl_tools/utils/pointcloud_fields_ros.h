#pragma once

#include <tuple>

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mrs_pcl_tools
{

  bool hasNormals(const std::vector<sensor_msgs::msg::PointField>& fields);
  bool hasNormals(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
  bool hasField(const std::string& field, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

}  // namespace mrs_pcl_tools
