#pragma once

/* includes and typedefs //{ */
#include <mrs_pcl_tools/support.h>

#include <mutex>
#include <tuple>
#include <set>

#include <std_srvs/Trigger.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/conversions.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <mrs_pcl_tools/pcl2map_registration_dynparamConfig.h>
#include <mrs_pcl_tools/SrvRegisterPointCloudByName.h>
#include <mrs_pcl_tools/SrvRegisterPointCloudOffline.h>

#include <mrs_lib/scope_timer.h>

typedef pcl::FPFHSignature33       feat_FPFH;
typedef pcl::PointCloud<feat_FPFH> PC_FPFH;
//}

namespace mrs_pcl_tools
{

struct HULL
{
  bool                                     has_data = false;
  bool                                     concave;
  PC_NORM::Ptr                             cloud_hull;
  std::vector<std::pair<pt_NORM, pt_NORM>> edges;
  Eigen::Vector3f                          polyline_barycenter;
};

struct EigenVectors
{
  bool            valid = false;
  Eigen::Vector3f x;
  Eigen::Vector3f y;
  Eigen::Vector3f z;
};

struct RegistrationInput
{
  PC_NORM::Ptr cloud_source;
  PC_NORM::Ptr cloud_target;

  bool            has_origins = false;
  Eigen::Vector3f origin_source;
  Eigen::Vector3f origin_target;

  bool            enable_init_guess = true;
  Eigen::Matrix4f T_guess           = Eigen::Matrix4f::Identity();
};

struct RegistrationOutput
{
  bool            converged      = false;
  double          fitness_score  = 0.0;
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  PC_NORM::Ptr    cloud_aligned;
  std::string     status_msg;
};

/* class PCL2MapRegistration //{ */
class PCL2MapRegistration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  const ros::Duration _SUBSCRIBE_MSG_TIMEOUT = ros::Duration(2.5f);

  ros::NodeHandle _nh;
  bool            _is_initialized       = false;
  bool            _map_available        = false;
  bool            _pc_offline_available = false;

  std::mutex _mutex_registration;

  ros::ServiceServer _srv_server_registration_offline;
  ros::ServiceServer _srv_server_registration_pointcloud2;

  ros::Publisher _pub_cloud_source;
  ros::Publisher _pub_cloud_target;
  ros::Publisher _pub_cloud_aligned;

  ros::Publisher _pub_dbg_hull_src;
  ros::Publisher _pub_dbg_hull_target;
  ros::Publisher _pub_dbg_pca;

  std::string _frame_map;
  std::string _path_map;
  std::string _path_pcl;
  std::string _topic_pc2;
  std::string _cloud_correlation_method;

  int   _registration_method_initial;
  int   _registration_method_fine_tune;
  bool  _use_init_guess;
  float _normal_estimation_radius;
  float _min_convergence_score;

  float _preprocess_voxel_leaf;
  float _preprocess_ror_radius;
  int   _preprocess_ror_neighbors;

  float  _cloud_correlation_z_crop_offset;
  bool   _cloud_correlation_poly_bary_hull_concave;
  double _cloud_correlation_poly_bary_alpha;

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

  float _icpn_max_corr_dist;
  float _icpn_ransac_outl_rej_thrd;
  float _icpn_trans_eps;
  float _icpn_eucld_fitn_eps;
  int   _icpn_max_iter;
  int   _icpn_ransac_iter;
  bool  _icpn_use_recip_corr;

  int   _sicpn_number_of_samples;
  float _sicpn_max_corr_dist;
  float _sicpn_ransac_outl_rej_thrd;
  float _sicpn_trans_eps;
  float _sicpn_eucld_fitn_eps;
  int   _sicpn_max_iter;
  int   _sicpn_ransac_iter;
  bool  _sicpn_use_recip_corr;

  std::mutex _mutex_hull_map;
  HULL       _hull_concave_map;

  Eigen::Matrix4f _initial_guess = Eigen::Matrix4f::Identity();

  boost::recursive_mutex                                     config_mutex_;
  typedef mrs_pcl_tools::pcl2map_registration_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>                ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                       reconfigure_server_;

  PC_NORM::Ptr _pc_map;
  PC_NORM::Ptr _pc_offline;

  PC_NORM::Ptr                loadPcWithNormals(const std::string &pcd_file);
  std::optional<PC_NORM::Ptr> subscribeSinglePointCloudMsg(const std::string &topic);

  void                                        correlateCloudToCloud(RegistrationInput &input);
  std::pair<Eigen::Vector3f, Eigen::Vector3f> getCentroids(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ);
  std::pair<HULL, HULL>                       getHulls(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ);
  std::pair<EigenVectors, EigenVectors>       getEigenVectors(const PC_NORM::Ptr pc_src, const PC_NORM::Ptr pc_targ);

  Eigen::Vector3f getCentroid(const PC_NORM::Ptr pc_src);
  Eigen::Vector3f getPolylineBarycenter(const std::vector<std::pair<pt_NORM, pt_NORM>> &edges);
  EigenVectors    getEigenVectors(const PC_NORM::Ptr cloud);

  RegistrationOutput registerCloudToCloud(RegistrationInput &input);
  bool callbackSrvRegisterOffline(mrs_pcl_tools::SrvRegisterPointCloudOffline::Request &req, mrs_pcl_tools::SrvRegisterPointCloudOffline::Response &res);
  bool callbackSrvRegisterPointCloud(mrs_pcl_tools::SrvRegisterPointCloudByName::Request &req, mrs_pcl_tools::SrvRegisterPointCloudByName::Response &res);
  void callbackReconfigure(Config &config, [[maybe_unused]] uint32_t level);

  RegistrationOutput pcl2map_ndt(RegistrationInput &input);
  RegistrationOutput pcl2map_fpfh(RegistrationInput &input);
  RegistrationOutput pcl2map_gicp(RegistrationInput &input);
  RegistrationOutput pcl2map_icpn(RegistrationInput &input);
  RegistrationOutput pcl2map_sicpn(RegistrationInput &input);

  void applyRandomTransformation(const PC_NORM::Ptr cloud);
  void removeNans(PC_NORM::Ptr &cloud);

  bool checkNans(const PC_NORM::Ptr cloud, const std::string &ns = "");

  const Eigen::Matrix4f          translationYawToMatrix(const Eigen::Vector3f &translation, const float yaw);
  const Eigen::Matrix4f          translationToMatrix(const Eigen::Vector3f &translation);
  const geometry_msgs::Transform matrixToTfTransform(const Eigen::Matrix4f &mat);
  geometry_msgs::Point           toGeometryMsg(const Eigen::Vector3f &point);
  std_msgs::ColorRGBA            toColorMsg(const Eigen::Vector3f &rgb, const float alpha = 1.0);

  HULL getHull(const PC_NORM::Ptr pc, const bool concave, const double alpha = 0.1);
  void computeHull(const PC_NORM::Ptr cloud_pc_in, const PC_NORM::Ptr cloud_hull_out, std::vector<pcl::Vertices> &polygons, const bool concave,
                   const double alpha = 0.1);

  void transformHull(HULL &hull, const Eigen::Matrix4f &mat);
  void transformEigenVector(Eigen::Vector3f &vec, const Eigen::Affine3f &mat);
  void transformEigenVectors(EigenVectors &eigenvectors, const Eigen::Matrix4f &mat);
  void translateEigenVector(Eigen::Vector3f &vec, const Eigen::Matrix4f &mat);

  void publishCloud(const ros::Publisher &pub, const PC_NORM::Ptr pc, const Eigen::Matrix4f &transform = Eigen::Matrix4f::Identity());
  void publishHull(const ros::Publisher &pub, const HULL &hull, const Eigen::Vector3f &color_rgb);
  void publishPCA(const ros::Publisher &pub, const std::string &frame_id, const std::pair<EigenVectors, EigenVectors> &eigenvectors,
                  const std::pair<Eigen::Vector3f, Eigen::Vector3f> &origins);

  void print(const EigenVectors &eigenvectors, const std::string &ns = "");
};
//}

}  // namespace mrs_pcl_tools
