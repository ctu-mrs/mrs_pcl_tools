#pragma once

// DO NOT MOVE THIS
#ifdef COMPILE_WITH_OUSTER
#define PCL_NO_PRECOMPILE
#endif
/* includes //{ */

// basic ros
// #include <ros/ros.h>

// #include <pluginlib/class_list_macros.h>

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
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/passthrough.h>

#include "mrs_pcl_tools/utils/i_logger.h"


//}

/*//{ typedefs */
typedef pcl::PointXYZ pt_XYZ;
typedef pcl::PointXYZI pt_XYZI;
typedef pcl::PointXYZRGB pt_XYZRGB;
typedef pcl::PointNormal pt_NORM;
typedef pcl::PointCloud<pt_XYZ> PC;
typedef pcl::PointCloud<pt_XYZI> PC_I;
typedef pcl::PointCloud<pt_XYZRGB> PC_RGB;
typedef pcl::PointCloud<pt_NORM> PC_NORM;

#ifdef COMPILE_WITH_OUSTER
// point types
#include <ouster_ros/os_point.h>
typedef ouster_ros::Point pt_OS;
typedef pcl::PointCloud<pt_OS> PC_OS;
#endif
//}


// }  // namespace mrs_pcl_tools
