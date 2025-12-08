#pragma once

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>
#include <mrs_pcl_tools/utils/pointcloud_io.h>

// #include <pcl_ros/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mrs_pcl_tools
{

void savePCD(ILogger& logger, const std::string& pcd_file, const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg, const bool binary = true);

}  // namespace mrs_pcl_tools