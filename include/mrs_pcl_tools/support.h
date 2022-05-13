#pragma once

#include "common_includes_and_typedefs.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/PointIndices.h>
#include <pcl/common/pca.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <ros/message_traits.h>
#include <visualization_msgs/MarkerArray.h>

namespace mrs_pcl_tools
{

std::optional<PC::Ptr>      loadPcXYZ(const std::string &pcd_file);
std::optional<PC_NORM::Ptr> loadPcNormals(const std::string &pcd_file);

/*//{ loadCloud() */
template <typename PC_t>
bool loadCloud(const std::string &filepath, typename boost::shared_ptr<PC_t> const &cloud, const bool verbose = true) {

  bool success = false;

  if (filepath.length() >= 3) {

    if (0 == filepath.compare(filepath.length() - 3, 3, "pcd")) {

      ROS_INFO_COND(verbose, "[PCLSupportLibrary] Reading PCD file from path %s", filepath.c_str());
      success = pcl::io::loadPCDFile<typename PC_t::PointType>(filepath, *cloud) == 0;

    } else if (0 == filepath.compare(filepath.length() - 3, 3, "ply")) {

      ROS_INFO_COND(verbose, "[PCLSupportLibrary] Reading PLY file from path %s", filepath.c_str());
      success = pcl::io::loadPLYFile<typename PC_t::PointType>(filepath, *cloud) == 0;

    } else {
      ROS_ERROR_COND(verbose, "[PCLSupportLibrary] Unknown format of cloud file: %s", filepath.c_str());
    }

  } else {
    ROS_ERROR_COND(verbose, "[PCLSupportLibrary] Could not read cloud from file. Path not given in valid format: %s", filepath.c_str());
  }

  ROS_INFO_COND(success && verbose, "[PCLSupportLibrary] Loaded point cloud with %ld points.", cloud->points.size());

  return success;
}
/*//}*/

void savePCD(const std::string &pcd_file, const sensor_msgs::PointCloud2::Ptr &cloud, const bool &binary = true);

PC_NORM::Ptr estimateNormals(const PC::Ptr &cloud, const float &normal_est_radius);

bool hasNormals(const std::vector<sensor_msgs::PointField> &fields);
bool hasNormals(const std::string &pcd_file);
bool hasNormals(const sensor_msgs::PointCloud2::ConstPtr &cloud);
bool hasField(const std::string &field, const sensor_msgs::PointCloud2::ConstPtr &msg);

template <typename T>
void publishCloud(const ros::Publisher &pub, const pcl::PointCloud<T> &cloud);

void publishCloudMsg(const ros::Publisher &pub, const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

void printEigenMatrix(const Eigen::Matrix4f &mat, const std::string &prefix = "");

// math functions
Eigen::Matrix4f getRotationMatrixAroundPoint(const Eigen::Matrix3f &rotation, const Eigen::Vector4f &point);

namespace filters
{

// Implement template functions here to force the compiler to instantiate all relevant template classes when linking the library

/*//{ applyVoxelGridFilter() */
template <typename PC_t>
typename boost::shared_ptr<PC_t> applyVoxelGridFilter(typename boost::shared_ptr<PC_t> const &cloud, const float resolution) {

  if (resolution <= 0.0f) {
    return cloud;
  }

  pcl::VoxelGrid<typename PC_t::PointType> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(resolution, resolution, resolution);

  boost::shared_ptr<PC_t> cloud_out = boost::make_shared<PC_t>();
  vg.filter(*cloud_out);

  return cloud_out;
}
/*//}*/

/*//{ applyRadiusOutlierFilter() */
template <typename PC_t>
typename boost::shared_ptr<PC_t> applyRadiusOutlierFilter(typename boost::shared_ptr<PC_t> const &cloud, const float radius, const int neighbors,
                                                          const bool keep_organized = true) {

  boost::shared_ptr<PC_t> cloud_out = boost::make_shared<PC_t>();

  // ROR requires finite points only
  /* boost::shared_ptr<std::vector<int>> finite_indices(new std::vector<int>()); */
  /* for (int i = 0; i < cloud->size(); i++) { */
  /*   if (cloud->at(i).getArray3fMap().allFinite()) { */
  /*     finite_indices->push_back(i); */
  /*   } */
  /* } */

  /* if (!finite_indices->empty()) { */
  pcl::RadiusOutlierRemoval<typename PC_t::PointType> outrem;
  outrem.setInputCloud(cloud);
  /* outrem.setIndices(finite_indices); */
  outrem.setRadiusSearch(radius);
  outrem.setMinNeighborsInRadius(neighbors);
  outrem.setKeepOrganized(keep_organized);

  outrem.filter(*cloud_out);
  /* } */

  return cloud_out;
}
/*//}*/

/*//{ applyMinimumGridFilter() */
template <typename PC_t>
typename boost::shared_ptr<PC_t> applyMinimumGridFilter(typename boost::shared_ptr<PC_t> const &cloud, const float resolution) {
  pcl::GridMinimum<typename PC_t::PointType> gmf(resolution);
  gmf.setInputCloud(cloud);

  boost::shared_ptr<PC_t> cloud_out = boost::make_shared<PC_t>();
  gmf.filter(*cloud_out);

  return cloud_out;
}
/*//}*/

/*//{ applyBilateralFilter() */
template <typename PC_t>
typename boost::shared_ptr<PC_t> applyBilateralFilter(typename boost::shared_ptr<PC_t> const &cloud, const float sigma_S, const float sigma_R) {

  if (cloud->width <= 1 || cloud->height <= 1) {
    ROS_ERROR("[mrs_pcl_tools::filters::applyBilateralFilter] Unorganized cloud given, not applying bilateral filter.");
    return cloud;
  }

  pcl::FastBilateralFilterOMP<typename PC_t::PointType> fbf;
  fbf.setInputCloud(cloud);
  fbf.setSigmaS(sigma_S);
  fbf.setSigmaR(sigma_R);

  boost::shared_ptr<PC_t> cloud_out = boost::make_shared<PC_t>();
  fbf.applyFilter(*cloud_out);

  return cloud_out;
}
/*//}*/

}  // namespace filters


namespace visualization
{
PC_RGB::Ptr         colorizeCloud(const PC::Ptr &cloud);
std_msgs::ColorRGBA heightToRGBA(double &height, const double &alpha = 1.0);
}  // namespace visualization

}  // namespace mrs_pcl_tools
