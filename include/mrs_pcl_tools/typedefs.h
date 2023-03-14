#pragma once
#define PCL_NO_PRECOMPILE

#include "mrs_pcl_tools/point_types.h"
#include <pcl/point_cloud.h>

#ifdef COMPILE_WITH_OUSTER
// point types
#include <ouster_ros/point.h>
typedef ouster_ros::Point      pt_OS;
typedef pcl::PointCloud<pt_OS> PC_OS;
#endif

typedef pcl::PointXYZ              pt_XYZ;
typedef pcl::PointXYZI             pt_XYZI;
typedef pcl::PointXYZRGB           pt_XYZRGB;
typedef pcl::PointNormal           pt_NORM;
typedef pcl::PointCloud<pt_XYZ>    PC;
typedef pcl::PointCloud<pt_XYZI>   PC_I;
typedef pcl::PointCloud<pt_XYZRGB> PC_RGB;
typedef pcl::PointCloud<pt_NORM>   PC_NORM;
