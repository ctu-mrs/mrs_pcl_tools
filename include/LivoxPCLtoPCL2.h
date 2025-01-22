#pragma once

/* includes //{ */

#include <mrs_pcl_tools/support.h>

#include <mrs_lib/subscribe_handler.h>

#include <livox_ros_driver2/CustomMsg.h>

//}

namespace mrs_pcl_tools
{
/* class LivoxPCLtoPCL2 //{ */
class LivoxPCLtoPCL2 : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized = false;

  mrs_lib::SubscribeHandler<livox_ros_driver2::CustomMsg> _sub_lidar_livox_pcl;

  ros::Publisher _pub_lidar_pcl2;

  void lidarLivoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr msg);
};
//}

}  // namespace mrs_pcl_tools
