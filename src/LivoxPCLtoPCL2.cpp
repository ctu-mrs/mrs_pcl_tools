#include <LivoxPCLtoPCL2.h>

namespace mrs_pcl_tools
{

/* onInit() //{ */
void LivoxPCLtoPCL2::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  mrs_lib::SubscribeHandlerOptions shopts(nh);
  shopts.node_name          = "LivoxPCLtoPCL2";
  shopts.no_message_timeout = ros::Duration(5.0);

  _sub_lidar_livox_pcl = mrs_lib::SubscribeHandler<livox_ros_driver2::CustomMsg>(shopts, "lidar_in", &LivoxPCLtoPCL2::lidarLivoxCallback, this);

  _pub_lidar_pcl2 = nh.advertise<sensor_msgs::PointCloud2>("lidar_out", 1);

  NODELET_INFO_ONCE("[LivoxPCLtoPCL2] Nodelet initialized");

  is_initialized = true;
}
//}

/* lidarLivoxCallback() //{ */
// Convert livox_ros_driver2::CustomMsg to sensor_msgs::PointCloud2 message type
void LivoxPCLtoPCL2::lidarLivoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr msg) {

  if (!is_initialized) {
    return;
  }

  const sensor_msgs::PointCloud2::Ptr cloud_ros = boost::make_shared<sensor_msgs::PointCloud2>();

  convertLivoxPCLtoPCL2(cloud_ros, msg);

  _pub_lidar_pcl2.publish(cloud_ros);
}
//}

}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::LivoxPCLtoPCL2, nodelet::Nodelet);
