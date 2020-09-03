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
#include <pcl/common/common.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/radius_outlier_removal.h>

// mrs_lib
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// point types
#include <os1_driver/ouster_ros/point_os1.h>

//}

/*//{ typedefs */

typedef pcl::PointXYZ              pt_XYZ;
typedef pcl::PointXYZI             pt_XYZI;
typedef pcl::PointXYZRGB           pt_XYZRGB;
typedef pcl::PointNormal           pt_NORM;
typedef ouster_ros::OS1::PointOS1  pt_OS1;
typedef pcl::PointCloud<pt_XYZ>    PC;
typedef pcl::PointCloud<pt_XYZI>   PC_I;
typedef pcl::PointCloud<pt_XYZRGB> PC_RGB;
typedef pcl::PointCloud<pt_NORM>   PC_NORM;
typedef pcl::PointCloud<pt_OS1>    PC_OS1;

//}
