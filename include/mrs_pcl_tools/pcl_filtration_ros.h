#pragma once

/* includes //{ */
#include <mrs_pcl_tools/support.h>
#include <mrs_pcl_tools/pcl_filtration_core.h>
#include <mrs_pcl_tools/utils/ros_logger.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
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
    void m_readLidarGeneralParams();
    void m_readLidarClipParams();
    void m_readLidarCropboxParams();
    void m_readLidarDownSamplingParams();

    void m_initLidarRepublishing();

    void m_lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    template <typename PC>
    void m_processMsg(std::shared_ptr<PC>& inout_pc_ptr);

  private:
    rclcpp::TimerBase::SharedPtr m_timer_init_;
    rclcpp::Node::SharedPtr m_node_;
    std::shared_ptr<mrs_lib::ParamLoader> m_param_loader_;
    std::shared_ptr<mrs_lib::Transformer> m_transformer_;
    std::shared_ptr<RosLogger> m_logger_;

    mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> m_pub_lidar;
    mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> m_pub_lidar_over_max_range;
    mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2> m_sub_lidar;


    std::unique_ptr<PCLFiltrationCore> m_pcl_filtration_core_;

    Lidar3DConfig m_lidar_params;

    bool m_is_initialized{false};
  };

  template <typename PC>
  void PCLFiltration::cropBoxPointCloud(const std::shared_ptr<PC>& inout_pc_ptr)
  {
  }

}  // namespace mrs_pcl_tools