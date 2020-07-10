#pragma once

/* includes //{ */
#include "common_includes_and_typedefs.h"

#include <mutex>
#include <tuple>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "mrs_pcl_tools/pcl2map_registration_dynparamConfig.h"

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

  std::mutex _mutex_registration;
  ros::Timer _timer_registration;
  float      _registration_period = 5.0f;

  float _ndt_transformation_epsilon = 0.01f;
  float _ndt_step_size              = 0.2f;
  float _ndt_resolution             = 0.1f;
  int   _ndt_maximum_iterations     = 50;

  Eigen::Matrix4f _initial_guess = Eigen::Matrix4f::Identity();

  boost::recursive_mutex                                     config_mutex_;
  typedef mrs_pcl_tools::pcl2map_registration_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>                ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                       reconfigure_server_;

  PC::Ptr      _pc_map;
  PC::Ptr      _pc_slam;
  PC_NORM::Ptr _pc_map_normals;
  bool         _map_has_normals;

  sensor_msgs::PointCloud2::Ptr _pc_map_msg  = boost::make_shared<sensor_msgs::PointCloud2>();
  sensor_msgs::PointCloud2::Ptr _pc_slam_msg = boost::make_shared<sensor_msgs::PointCloud2>();

  PC::Ptr load_pc(const std::string &path);
  bool    load_pc_normals(const std::string &path, PC_NORM::Ptr &cloud);

  void callbackRegistration([[maybe_unused]] const ros::TimerEvent &event);
  void callbackReconfigure(Config &config, [[maybe_unused]] uint32_t level);

  std::tuple<bool, float, Eigen::Matrix4f, PC::Ptr> pcl2map_ndt(PC::Ptr pc, PC::Ptr pc_map, Eigen::Matrix4f init_guess);

  void publishCloud(const ros::Publisher pub, PC::Ptr cloud);
  void publishCloudMsg(const ros::Publisher pub, sensor_msgs::PointCloud2::Ptr cloud_msg);
};
//}

}  // namespace mrs_pcl_tools
