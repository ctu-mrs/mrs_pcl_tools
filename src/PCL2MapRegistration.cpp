#include "PCL2MapRegistration.h"

namespace mrs_pcl_tools
{

/* onInit() //{ */
void PCL2MapRegistration::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Get parameters from config file
  mrs_lib::ParamLoader param_loader(nh, "PCL2MapRegistration");

  std::string path_map;
  std::string path_pcl;
  param_loader.loadParam("map", path_map);
  param_loader.loadParam("pcl", path_pcl);

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCL2MapRegistration]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  _pc_map  = load_pc(path_map);
  _pc_slam = load_pc(path_pcl);

  NODELET_INFO_ONCE("[PCL2MapRegistration] Nodelet initialized");

  is_initialized = true;
}
//}

/*//{ load_pc() */
PC::Ptr PCL2MapRegistration::load_pc(std::string path) {
  PC::Ptr pc = boost::make_shared<PC>();

  ROS_INFO("[PCL2MapRegistration] Reading pointcloud from path %s", path.c_str());
  if (pcl::io::loadPCDFile<pt_XYZ>(path, *pc) == -1) {
    ROS_ERROR("[PCL2MapRegistration] Couldn't read PCD file from path: %s.", path.c_str());
    ros::shutdown();
  } else {
    ROS_INFO("[PCL2MapRegistration] Loaded XYZ pcl with %ld points.", pc->points.size());
  }

  return pc;
}

/*//}*/

//
}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCL2MapRegistration, nodelet::Nodelet);
