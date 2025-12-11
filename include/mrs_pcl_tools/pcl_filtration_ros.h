#pragma once

/* includes //{ */
#include <mrs_pcl_tools/support.h>
#include <mrs_pcl_tools/pcl_filtration_core.h>
#include <mrs_pcl_tools/utils/ros_logger.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
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


    template <typename PC>
    void cropBoxPointCloud(const std::shared_ptr<PC>& inout_pc_ptr);

  private:
    void m_timerInit();
    void m_readParams();
    void m_readLidarParams();

  private:
    rclcpp::TimerBase::SharedPtr m_timer_init_;
    rclcpp::Node::SharedPtr m_node_;
    std::shared_ptr<mrs_lib::ParamLoader> m_param_loader_;
    std::shared_ptr<mrs_lib::Transformer> m_transformer_;
    std::shared_ptr<RosLogger> m_logger_;

    std::unique_ptr<PCLFiltrationCore> m_pcl_filtration_core_;

    Lidar3DConfig m_lidar_params;
  };

  template <typename PC>
  void PCLFiltration::cropBoxPointCloud(const std::shared_ptr<PC>& inout_pc_ptr)
  {
  }

}  // namespace mrs_pcl_tools