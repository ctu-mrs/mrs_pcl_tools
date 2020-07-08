#pragma once

/* includes //{ */
// basic ros
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// timing
#include <chrono>

// pcl
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/grid_minimum.h>

// mrs_lib
#include <mrs_lib/param_loader.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// point types
#include <os1_driver/ouster_ros/point_os1.h>


//}

/*//{ typedefs */
typedef pcl::PointXYZ             pt_XYZ;
typedef pcl::PointXYZI            pt_XYZI;
typedef ouster_ros::OS1::PointOS1 pt_OS1;
typedef pcl::PointCloud<pt_XYZ>   PC;
typedef pcl::PointCloud<pt_XYZI>  PC_I;
typedef pcl::PointCloud<pt_OS1>   PC_OS1;
//}
