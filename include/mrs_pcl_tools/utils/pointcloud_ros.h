#pragma once

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mrs_pcl_tools
{

template <typename T>
void publishCloud(ILogger& logger, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
                  const pcl::PointCloud<T>& cloud);

void publishCloudMsg(ILogger& logger, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);

}  // namespace mrs_pcl_tools

/*//{ publishCloud() */
template <typename T>
void mrs_pcl_tools::publishCloud(ILogger& logger,
                                 const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
                                 const pcl::PointCloud<T>& cloud)
{
  if (pub->get_subscription_count() > 0)
  {
    try
    {
      pub->publish(cloud);
    }
    catch (...)
    {
      logger.error("[mrs_pcl_tools::publishCloudMsg]: Exception caught during publishing on topic: " +
                   std::string(pub->get_topic_name()));
    }
  }
}
/*//}*/