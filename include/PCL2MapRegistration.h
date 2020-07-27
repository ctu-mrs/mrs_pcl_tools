#pragma once

/* includes and typedefs //{ */
#include "common_includes_and_typedefs.h"

#include <mutex>
#include <tuple>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ndt.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "mrs_pcl_tools/pcl2map_registration_dynparamConfig.h"

typedef pcl::FPFHSignature33       feat_FPFH;
typedef pcl::PointCloud<feat_FPFH> PC_FPFH;
//}

namespace mrs_pcl_tools
{

/* class PCL2MapRegistration //{ */
class PCL2MapRegistration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized = false;

  ros::Publisher _pub_cloud_source;
  ros::Publisher _pub_cloud_target;
  ros::Publisher _pub_cloud_aligned;

  std::string _frame_map;
  std::string _path_map;
  std::string _path_pcl;

  int  _registration_method = 0;
  bool _use_init_guess      = false;

  std::mutex _mutex_registration;
  ros::Timer _timer_registration;
  float      _registration_period = 5.0f;

  float _fpfh_voxel_leaf           = 0.2;
  float _fpfh_search_rad           = 0.4;
  float _fpfh_similarity_threshold = 0.9;
  float _fpfh_inlier_fraction      = 0.25;
  int   _fpfh_ransac_max_iter      = 500;
  int   _fpfh_number_of_samples    = 3;
  int   _fpfh_corr_randomness      = 5;

  float _ndt_transformation_epsilon = 0.01f;
  float _ndt_step_size              = 0.2f;
  float _ndt_resolution             = 0.1f;
  int   _ndt_maximum_iterations     = 50;

  float _normal_estimation_radius = 0.02f;

  Eigen::Matrix4f _initial_guess = Eigen::Matrix4f::Identity();

  boost::recursive_mutex                                     config_mutex_;
  typedef mrs_pcl_tools::pcl2map_registration_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>                ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                       reconfigure_server_;

  PC_NORM::Ptr _pc_map;
  PC_NORM::Ptr _pc_slam;

  sensor_msgs::PointCloud2::Ptr _pc_map_msg  = boost::make_shared<sensor_msgs::PointCloud2>();
  sensor_msgs::PointCloud2::Ptr _pc_slam_msg = boost::make_shared<sensor_msgs::PointCloud2>();

  PC::Ptr      load_pc(const std::string &path);
  PC_NORM::Ptr load_pc_norm(const std::string &path);
  bool         load_pc_normals(const std::string &path, PC_NORM::Ptr &cloud);
  bool         pcd_file_has_normals(const std::string path);

  void callbackRegistration([[maybe_unused]] const ros::TimerEvent &event);
  void callbackReconfigure(Config &config, [[maybe_unused]] uint32_t level);

  std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> pcl2map_ndt(const PC_NORM::Ptr pc, const PC_NORM::Ptr pc_map);
  std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> pcl2map_fpfh(const PC_NORM::Ptr pc, const PC_NORM::Ptr pc_map);

  PC_NORM::Ptr estimate_normals(const PC::Ptr cloud, const float nest_radius);

  void publishCloud(const ros::Publisher pub, const PC_NORM::Ptr cloud);
  void publishCloudMsg(const ros::Publisher pub, const sensor_msgs::PointCloud2::Ptr cloud_msg);

  void printEigenMatrix(const Eigen::Matrix4f mat, const std::string prefix = "");
};
//}

}  // namespace mrs_pcl_tools
