#pragma once

/* includes //{ */

// basic ros
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// timing
#include <ctime>
#include <cstdlib>
#include <chrono>

// pcl
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/common/common.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/passthrough.h>

// mrs_lib
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/scope_timer.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>

#include <mrs_msgs/PclToolsDiagnostics.h>

#include "mrs_pcl_tools/typedefs.h"

namespace mrs_pcl_tools
{

  /*//{ struct Diagnostics_t */
  struct Diagnostics_t
  {
    Diagnostics_t(ros::NodeHandle& nh) {
      _pub_diagnostics = nh.advertise<mrs_msgs::PclToolsDiagnostics>("diagnostics_out", 10);
    }

    void publish(const mrs_msgs::PclToolsDiagnosticsConstPtr& msg) const {
      if (_pub_diagnostics.getNumSubscribers() > 0) {
        try {
          _pub_diagnostics.publish(msg);
        }
        catch (...) {
          ROS_ERROR("[PCLFiltration] Failed to publish msg on topic (%s).", _pub_diagnostics.getTopic().c_str());
        }
      }
    }

  private:
    ros::Publisher _pub_diagnostics;
  };
  /*//}*/

  /*//{ struct CommonHandlers_t */
  struct CommonHandlers_t
  {
    std::shared_ptr<mrs_lib::ParamLoader> param_loader;
    std::shared_ptr<mrs_lib::Transformer> transformer;

    bool                                       scope_timer_enabled;
    std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger;

    std::shared_ptr<Diagnostics_t> diagnostics;
  };
  /*//}*/

}
