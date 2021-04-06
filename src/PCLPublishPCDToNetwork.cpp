#include "mrs_pcl_tools/PCLPublishPCDToNetwork.h"

namespace mrs_pcl_tools
{

/* onInit() //{ */
void PCLPublishPCDToNetwork::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Get parameters from config file
  std::string          pcd_path;
  mrs_lib::ParamLoader param_loader(nh, "PCLPublishPCDToNetwork");
  param_loader.loadParam("pcd", pcd_path);
  param_loader.loadParam("rate", _rate, 1.0f);
  param_loader.loadParam("resolution", _resolution, 0.2f);
  param_loader.loadParam("frame_id", _frame_id, std::string("pcd_origin"));

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCLPublishPCDToNetwork]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  auto ret = mrs_pcl_tools::loadPcXYZ(pcd_path);
  if (!ret) {
    NODELET_ERROR("[PCLPublishPCDToNetwork]: PCD file (%s) was not loaded successfully, ending the node", pcd_path.c_str());
    ros::shutdown();
  }

  PC_RGB::Ptr cloud_rgb = mrs_pcl_tools::visualization::colorizeCloud(ret.value());
  _cloud                = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_rgb, *_cloud);
  _cloud->header.frame_id = _frame_id;

  _cloud_vis = mrs_pcl_tools::visualization::getVisualizationMsg(_cloud, _resolution, _frame_id);

  _pub_cloud     = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
  _pub_cloud_vis = nh.advertise<visualization_msgs::MarkerArray>("cloud_vis_out", 1);

  _is_initialized = true;

  _timer_publish = nh.createTimer(ros::Rate(_rate), &PCLPublishPCDToNetwork::publishCloud, this, false, true);
}
//}

/*//{ publishCloud() */
void PCLPublishPCDToNetwork::publishCloud([[maybe_unused]] const ros::TimerEvent &event) {
  const ros::Time now = ros::Time::now();
  if (_pub_cloud.getNumSubscribers() > 0) {
    _cloud->header.stamp = now;
    _pub_cloud.publish(_cloud);
  }
  if (_pub_cloud_vis.getNumSubscribers() > 0) {
    for (unsigned int i = 0; i < _cloud_vis->markers.size(); i++) {
      _cloud_vis->markers[i].header.stamp = now;
      /* for (unsigned int j = 0; j < _cloud_vis->markers[i].points.size(); j++) { */
      /* ROS_INFO("scale: %0.2f, %0.2f, %0.2f, rgba: %0.2f, %0.2f, %0.2f, %0.2f", _cloud_vis->markers[i].scale.x, _cloud_vis->markers[i].scale.y, */
      /*          _cloud_vis->markers[i].scale.z, _cloud_vis->markers[i].colors[j].r, _cloud_vis->markers[i].colors[j].g, _cloud_vis->markers[i].colors[j].b,
       * _cloud_vis->markers[i].colors[j].a); */
      /* } */
    }
    _pub_cloud_vis.publish(_cloud_vis);
  }
}
/*//}*/

}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCLPublishPCDToNetwork, nodelet::Nodelet);
