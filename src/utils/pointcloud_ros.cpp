#include <mrs_pcl_tools/utils/pointcloud_ros.h>

namespace mrs_pcl_tools
{

/*//{ publishCloudMsg() */
void publishCloudMsg(ILogger& logger, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
{
  if (pub->get_subscription_count() > 0)
  {
    try
    {
      pub->publish(*cloud_msg);
    }
    catch (...)
    {
      logger.error("[mrs_pcl_tools::publishCloudMsg]: Exception caught during publishing on topic: " +
                   std::string(pub->get_topic_name()));
    }
  }
}
/*//}*/

}  // namespace mrs_pcl_tools