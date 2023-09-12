#pragma once

/* includes //{ */

#include <mrs_pcl_tools/support.h>

//}

namespace mrs_pcl_tools
{

/* class PCLPublishCloudFileToNetwork //{ */
class PCLPublishCloudFileToNetwork : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  // | ----------------------- Parameters ----------------------- |
  bool        _load_colors = false;
  float       _rate;
  std::string _frame_id;

  // | ------------------------ ROS msgs ------------------------ |
  sensor_msgs::PointCloud2::Ptr _cloud_msg;

  // | ------------------ Variables and objects ----------------- |
  bool           _is_initialized = false;
  ros::Publisher _pub_cloud;
  ros::Timer     _timer_publish;

  // | ------------------------ Functions ----------------------- |
  void publishCloud([[maybe_unused]] const ros::TimerEvent &event);
};
//}

}  // namespace mrs_pcl_tools
