#include <PCLPublishCloudFileToNetwork.h>

namespace mrs_pcl_tools
{

/* onInit() //{ */
void PCLPublishCloudFileToNetwork::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Get parameters from config file
  std::string          file_cloud;
  mrs_lib::ParamLoader param_loader(nh, "PCLPublishCloudFileToNetwork");
  param_loader.loadParam("file_cloud", file_cloud);
  param_loader.loadParam("rate", _rate, 1.0f);
  param_loader.loadParam("frame_id", _frame_id, std::string("cloud_origin"));
  param_loader.loadParam("load_colors", _load_colors, false);

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCLPublishCloudFileToNetwork]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  bool        cloud_loaded = false;
  PC_RGB::Ptr cloud_rgb;

  if (_load_colors) {

    cloud_rgb    = boost::make_shared<PC_RGB>();
    cloud_loaded = mrs_pcl_tools::loadCloud(file_cloud, cloud_rgb, true);

  } else {

    const PC::Ptr cloud_xyz = boost::make_shared<PC>();
    cloud_loaded            = mrs_pcl_tools::loadCloud(file_cloud, cloud_xyz, true);

    if (cloud_loaded) {
      cloud_rgb = mrs_pcl_tools::visualization::colorizeCloud(cloud_xyz);
    }
  }

  if (!cloud_loaded) {
    NODELET_ERROR("[PCLPublishCloudFileToNetwork]: Cloud file (%s) was not loaded successfully, ending the node", file_cloud.c_str());
    ros::shutdown();
    return;
  }

  _cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_rgb, *_cloud_msg);
  _cloud_msg->header.frame_id = _frame_id;

  _pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);

  _is_initialized = true;

  _timer_publish = nh.createTimer(ros::Rate(_rate), &PCLPublishCloudFileToNetwork::publishCloud, this, false, true);
}
//}

/*//{ publishCloud() */
void PCLPublishCloudFileToNetwork::publishCloud([[maybe_unused]] const ros::TimerEvent &event) {
  const ros::Time now = ros::Time::now();
  if (_pub_cloud.getNumSubscribers() > 0) {
    _cloud_msg->header.stamp = now;
    _pub_cloud.publish(_cloud_msg);
  }
}
/*//}*/

}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCLPublishCloudFileToNetwork, nodelet::Nodelet);
