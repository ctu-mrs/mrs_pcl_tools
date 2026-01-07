#pragma once

/* includes //{ */
#include <mutex>

#include <mrs_pcl_tools/support.h>
#include <mrs_pcl_tools/pcl_filtration_core.h>
#include <mrs_pcl_tools/utils/ros_logger.h>
#include <mrs_pcl_tools/pcl_filtration_diagnostics.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/dynparam_mgr.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
//}

namespace mrs_pcl_tools
{
  using vec3_t = Eigen::Vector3f;
  using vec4_t = Eigen::Vector4f;
  using quat_t = Eigen::Quaternionf;
  using namespace std::literals::chrono_literals;


  class PCLFiltration : public rclcpp::Node
  {
  public:
    PCLFiltration(rclcpp::NodeOptions options);

  private:
    void m_timerInit();

    void m_readParams();
    void m_initDynParams();
    void m_initDynLidarParams();
    void m_readLidarParams();
    void m_readLidarGeneralParams();
    void m_readLidarClipParams();
    void m_readLidarCropboxParams();
    void m_readLidarDownSamplingParams();

    void m_initTransformer();
    void m_initScopeTimerLogger();
    void m_initLidarRepublishing();
    void m_lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    void callbackIntensityFilterEnable(const bool& param_value);
    void callbackIntensityFilterThreshold(const float& param_value);
    void callbackIntensityFilterRange(const float& param_value);

    template <typename PC>
    void m_publishOverMaxRange(const std::shared_ptr<PC>& pc);

    template <typename PC>
    void m_processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, mrs_modules_msgs::msg::PclToolsDiagnostics& diag_msg);

    template <typename PC>
    void m_processMsg(std::shared_ptr<PC>& inout_pc_ptr);

    template <typename PC>
    void m_cropBoxPointCloud(std::shared_ptr<PC>& inout_pc_ptr);

    template <typename PC>
    void m_logPointCloudStats(mrs_lib::ScopeTimer& timer, std::shared_ptr<PC>& inout_pc_ptr, const size_t height_before, const size_t width_before,
                              const size_t points_before);

  private:
    rclcpp::TimerBase::SharedPtr m_timer_init_;
    rclcpp::Node::SharedPtr m_node_;
    std::shared_ptr<mrs_lib::ParamLoader> m_param_loader_;
    std::shared_ptr<mrs_lib::Transformer> m_transformer_;
    std::shared_ptr<mrs_lib::DynparamMgr> m_dynparam_mgr_;
    std::shared_ptr<mrs_lib::ScopeTimerLogger> m_scope_timer_logger;
    std::shared_ptr<PclFiltrationDiagnostics> m_diagnostics_;

    std::shared_ptr<RosLogger> m_logger_;
    std::unique_ptr<PCLFiltrationCore> m_pcl_filtration_core_;

    mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> m_pub_lidar;
    mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> m_pub_lidar_over_max_range;
    mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2> m_sub_lidar;

    bool m_is_initialized{false};
    bool m_scope_timer_enabled{false};
    std::mutex m_mutex_drs_params;
    Lidar3DConfig m_lidar_params;
  };

}  // namespace mrs_pcl_tools

#include <mrs_pcl_tools/pcl_filtration_ros.tpp>
