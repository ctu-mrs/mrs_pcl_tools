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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

#include <mrs_modules_msgs/PclToolsDiagnostics.h>

#ifdef COMPILE_WITH_OUSTER
// point types
#include <ouster_ros/point.h>
typedef ouster_ros::Point          pt_OS;
typedef pcl::PointCloud<pt_OS>     PC_OS;
#endif

//}

/*//{ typedefs */

typedef pcl::PointXYZ              pt_XYZ;
typedef pcl::PointXYZI             pt_XYZI;
typedef pcl::PointXYZRGB           pt_XYZRGB;
typedef pcl::PointNormal           pt_NORM;
typedef pcl::PointCloud<pt_XYZ>    PC;
typedef pcl::PointCloud<pt_XYZI>   PC_I;
typedef pcl::PointCloud<pt_XYZRGB> PC_RGB;
typedef pcl::PointCloud<pt_NORM>   PC_NORM;

//}

namespace mrs_pcl_tools
{

  /*//{ struct Diagnostics_t */
  struct Diagnostics_t
  {
    Diagnostics_t(ros::NodeHandle& nh) {
      _pub_diagnostics = nh.advertise<mrs_modules_msgs::PclToolsDiagnostics>("diagnostics_out", 10);
    }

    void publish(const mrs_modules_msgs::PclToolsDiagnosticsConstPtr& msg) const {
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
