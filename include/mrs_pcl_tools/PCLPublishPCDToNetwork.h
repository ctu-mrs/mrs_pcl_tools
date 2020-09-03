#pragma once

/* includes //{ */
#include "support.h"
//}

namespace mrs_pcl_tools
{

/* class PCLPublishPCDToNetwork //{ */
class PCLPublishPCDToNetwork : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool _is_initialized = false;

  ros::Publisher _pub_cloud;
  ros::Publisher _pub_cloud_vis;

  ros::Timer _timer_publish;

  float       _rate;
  float       _resolution;
  std::string _frame_id;

  sensor_msgs::PointCloud2::Ptr        _cloud;
  visualization_msgs::MarkerArray::Ptr _cloud_vis;

  void publishCloud([[maybe_unused]] const ros::TimerEvent &event);
};
//}

}  // namespace mrs_pcl_tools
