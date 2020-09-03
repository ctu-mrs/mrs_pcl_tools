#pragma once

#include "common_includes_and_typedefs.h"

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <visualization_msgs/MarkerArray.h>

namespace mrs_pcl_tools
{

// TODO: template
void applyVoxelGridFilter(const PC_NORM::Ptr &cloud_in, PC_NORM::Ptr &cloud_out, const float &leaf_size);
void applyVoxelGridFilter(PC_NORM::Ptr &cloud, const float &leaf_size);

std::optional<PC::Ptr>      loadPcXYZ(const std::string &pcd_file);
std::optional<PC_NORM::Ptr> loadPcNormals(const std::string &pcd_file);

void savePCD(const std::string &pcd_file, sensor_msgs::PointCloud2::ConstPtr &cloud, const bool &binary = true);

PC_NORM::Ptr estimateNormals(const PC::Ptr &cloud, const float &normal_est_radius);

bool hasNormals(const std::vector<sensor_msgs::PointField> &fields);
bool hasNormals(const std::string &pcd_file);
bool hasNormals(const sensor_msgs::PointCloud2::ConstPtr &cloud);
bool hasField(const std::string &field, const sensor_msgs::PointCloud2::ConstPtr &msg);

void publishCloud(const ros::Publisher &pub, const PC_NORM::Ptr &cloud);
void publishCloudMsg(const ros::Publisher &pub, const sensor_msgs::PointCloud2::Ptr &cloud_msg);

void printEigenMatrix(const Eigen::Matrix4f &mat, const std::string &prefix = "");

// math functions
Eigen::Matrix4f getRotationMatrixAroundPoint(const Eigen::Matrix3f &rotation, const Eigen::Vector4f &point);

namespace visualization
{
PC_RGB::Ptr                          colorizeCloud(const PC::Ptr &cloud);
visualization_msgs::MarkerArray::Ptr getVisualizationMsg(const sensor_msgs::PointCloud2::Ptr &cloud, const float &resolution, const std::string &frame_id);
visualization_msgs::MarkerArray::Ptr getVisualizationMsg(const PC::Ptr &cloud, const float &resolution, const std::string &frame_id);
visualization_msgs::MarkerArray::Ptr getVisualizationMsg(const std::shared_ptr<octomap::OcTree> &octree, const std::string &frame_id);
std_msgs::ColorRGBA                  heightToRGBA(double &height, const double &alpha = 1.0);
}  // namespace visualization

}  // namespace mrs_pcl_tools

/*//{ class TicToc */

class TicToc {
public:
  TicToc() {
    tic();
  }

  void tic() {
    start = std::chrono::system_clock::now();
  }

  double toc() {
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
    return elapsed_seconds.count() * 1000;
  }

  void toc_print(const std::string text) {
    ROS_INFO("%s: %0.2f ms", text.c_str(), toc());
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start;
};

/*//}*/