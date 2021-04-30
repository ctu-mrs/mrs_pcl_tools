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

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// point types
#include <ouster_ros/point.h>

//}

/*//{ typedefs */

typedef pcl::PointXYZ              pt_XYZ;
typedef pcl::PointXYZI             pt_XYZI;
typedef pcl::PointXYZRGB           pt_XYZRGB;
typedef pcl::PointNormal           pt_NORM;
typedef ouster_ros::Point          pt_OS;
typedef pcl::PointCloud<pt_XYZ>    PC;
typedef pcl::PointCloud<pt_XYZI>   PC_I;
typedef pcl::PointCloud<pt_XYZRGB> PC_RGB;
typedef pcl::PointCloud<pt_NORM>   PC_NORM;
typedef pcl::PointCloud<pt_OS>     PC_OS;

//}
