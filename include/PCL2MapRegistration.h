#pragma once

/* includes and typedefs //{ */
#include "common_includes_and_typedefs.h"

#include <mutex>
#include <tuple>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#include <std_srvs/Trigger.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "mrs_pcl_tools/pcl2map_registration_dynparamConfig.h"
#include "mrs_pcl_tools/SrvRegisterPointCloudByName.h"

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
  ros::NodeHandle _nh;
  bool            _is_initialized = false;
  bool            _map_available  = false;

  ros::ServiceServer _srv_server_registration_offline;
  ros::ServiceServer _srv_server_registration_pointcloud2;

  ros::Publisher _pub_cloud_source;
  ros::Publisher _pub_cloud_target;
  ros::Publisher _pub_cloud_aligned;
  ros::Publisher _pub_registration_tf;

  std::string _frame_map;
  std::string _path_map;
  std::string _path_pcl;
  std::string _topic_pc2;

  const ros::Duration _SUBSCRIBE_MSG_TIMEOUT = ros::Duration(2.0f);

  int   _registration_method_initial     = 4;
  int   _registration_method_fine_tune   = 3;
  bool  _use_init_guess                  = false;
  float _clouds_voxel_leaf               = 0.6f;
  float _normal_estimation_radius        = 0.25f;
  float _cloud_correlation_z_crop_offset = 2.0f;
  float _min_convergence_score           = 0.5f;

  std::mutex _mutex_registration;

  tf2_ros::StaticTransformBroadcaster tf_broadcaster;
  tf2_msgs::TFMessage                 _tf_registration_msg;
  ros::Timer                          _timer_publish_registration_tf;
  void                                publishRegistrationTF([[maybe_unused]] const ros::TimerEvent &event);

  float _fpfh_search_rad           = 3.5;
  float _fpfh_similarity_threshold = 0.9;
  float _fpfh_inlier_fraction      = 0.05;
  int   _fpfh_ransac_max_iter      = 50000;
  int   _fpfh_number_of_samples    = 4;
  int   _fpfh_corr_randomness      = 15;

  float _ndt_transformation_epsilon = 0.01f;
  float _ndt_step_size              = 0.2f;
  float _ndt_resolution             = 0.1f;
  int   _ndt_maximum_iterations     = 50;

  float _gicp_max_corr_dist        = 5.0f;
  float _gicp_ransac_outl_rej_thrd = 0.3f;
  float _gicp_trans_eps            = 0.1f;
  int   _gicp_max_iter             = 50;
  int   _gicp_max_opt_iter         = 20;
  int   _gicp_ransac_iter          = 30;
  bool  _gicp_use_recip_corr       = false;

  float _icpn_max_corr_dist        = 4.0f;
  float _icpn_ransac_outl_rej_thrd = 0.95f;
  float _icpn_trans_eps            = 0.01f;
  float _icpn_eucld_fitn_eps       = 0.01f;
  int   _icpn_max_iter             = 1000;
  int   _icpn_ransac_iter          = 300;
  bool  _icpn_use_recip_corr       = false;

  unsigned int _sicpn_number_of_samples    = 8;
  float        _sicpn_max_corr_dist        = 7.0f;
  float        _sicpn_ransac_outl_rej_thrd = 0.8f;
  float        _sicpn_trans_eps            = 0.1;
  float        _sicpn_eucld_fitn_eps       = 0.1;
  int          _sicpn_max_iter             = 500;
  int          _sicpn_ransac_iter          = 150;
  bool         _sicpn_use_recip_corr       = false;

  Eigen::Matrix4f _initial_guess = Eigen::Matrix4f::Identity();

  boost::recursive_mutex                                     config_mutex_;
  typedef mrs_pcl_tools::pcl2map_registration_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>                ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                       reconfigure_server_;

  PC_NORM::Ptr _pc_map;
  PC_NORM::Ptr _pc_slam;

  PC::Ptr                     loadPcXYZ(const std::string &path);
  PC_NORM::Ptr                loadPcWithNormals(const std::string &path);
  std::optional<PC_NORM::Ptr> subscribeSinglePointCloudMsg(const std::string &topic);
  bool                        loadPcNormals(const std::string &path, PC_NORM::Ptr &cloud);
  bool                        hasNormals(const std::string path);
  bool                        hasNormals(const sensor_msgs::PointCloud2::ConstPtr &cloud);
  Eigen::Matrix4f             correlateCloudToCloud(PC_NORM::Ptr pc_src, PC_NORM::Ptr pc_targ);

  std::tuple<bool, std::string, Eigen::Matrix4f> registerCloudToCloud(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ);
  bool                                           callbackSrvRegisterOffline([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackSrvRegisterPointCloud(mrs_pcl_tools::SrvRegisterPointCloudByName::Request &req, mrs_pcl_tools::SrvRegisterPointCloudByName::Response &res);
  void callbackReconfigure(Config &config, [[maybe_unused]] uint32_t level);

  std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> pcl2map_ndt(const PC_NORM::Ptr pc, const PC_NORM::Ptr pc_map, const bool enable_init_guess = true);
  std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> pcl2map_fpfh(const PC_NORM::Ptr pc, const PC_NORM::Ptr pc_map, const bool enable_init_guess = true);
  std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> pcl2map_gicp(const PC_NORM::Ptr pc, const PC_NORM::Ptr pc_map, const bool enable_init_guess = true);
  std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> pcl2map_icpn(const PC_NORM::Ptr pc, const PC_NORM::Ptr pc_map, const bool enable_init_guess = true);
  std::tuple<bool, float, Eigen::Matrix4f, PC_NORM::Ptr> pcl2map_sicpn(const PC_NORM::Ptr pc, const PC_NORM::Ptr pc_map);

  PC_NORM::Ptr    estimateNormals(const PC::Ptr cloud, const float nest_radius);
  void            applyVoxelGridFilter(PC_NORM::Ptr cloud_in, const PC_NORM::Ptr cloud_out, const float leaf_size);
  void            applyRandomTransformation(PC_NORM::Ptr cloud);
  Eigen::Matrix4f getRotationMatrixAroundPoint(const Eigen::Matrix3f rotation, const Eigen::Vector4f point);

  void publishCloud(const ros::Publisher pub, const PC_NORM::Ptr cloud);
  void publishCloudMsg(const ros::Publisher pub, const sensor_msgs::PointCloud2::Ptr cloud_msg);

  void printEigenMatrix(const Eigen::Matrix4f mat, const std::string prefix = "");
};
//}

}  // namespace mrs_pcl_tools
