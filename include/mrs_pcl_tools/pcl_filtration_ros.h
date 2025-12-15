#pragma once

/* includes //{ */
#include <mrs_pcl_tools/support.h>
#include <mrs_pcl_tools/pcl_filtration_core.h>
#include <mrs_pcl_tools/utils/ros_logger.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/scope_timer.h>

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
    void m_readLidarParams();
    void m_readLidarGeneralParams();
    void m_readLidarClipParams();
    void m_readLidarCropboxParams();
    void m_readLidarDownSamplingParams();

    void m_initTransformer();
    void m_initScopeTimerLogger();

    void m_initLidarRepublishing();
    void m_lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

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

    std::shared_ptr<mrs_lib::ScopeTimerLogger> m_scope_timer_logger;
    bool m_scope_timer_enabled{false};

    std::shared_ptr<RosLogger> m_logger_;

    mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> m_pub_lidar;
    mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> m_pub_lidar_over_max_range;
    mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2> m_sub_lidar;


    std::unique_ptr<PCLFiltrationCore> m_pcl_filtration_core_;

    Lidar3DConfig m_lidar_params;

    bool m_is_initialized{false};
  };

  template <typename PC>
  void PCLFiltration::m_cropBoxPointCloud(std::shared_ptr<PC>& inout_pc_ptr)
  {
    Eigen::Affine3d tf = Eigen::Affine3d::Identity();

    if (!m_lidar_params.cropbox.frame_id.empty())
    {
      rclcpp::Time stamp;
      pcl_conversions::fromPCL(inout_pc_ptr->header.stamp, stamp);
      const auto tf_opt = m_transformer_->getTransform(inout_pc_ptr->header.frame_id, m_lidar_params.cropbox.frame_id, stamp);
      if (!tf_opt.has_value())
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[PCLFiltration]: Could not find pointcloud transformation (from \"%s\" to \"%s\")! "
                             "Not applying CropBox filter.",
                             inout_pc_ptr->header.frame_id.c_str(), m_lidar_params.cropbox.frame_id.c_str());
        return;
      }
      tf = tf2::transformToEigen(tf_opt.value().transform);
    }
    m_pcl_filtration_core_->cropBoxPointCloud(inout_pc_ptr, tf.cast<float>());
  }

  template <typename PC>
  void PCLFiltration::m_logPointCloudStats(mrs_lib::ScopeTimer& timer, std::shared_ptr<PC>& inout_pc_ptr, const size_t height_before, const size_t width_before,
                                           const size_t points_before)
  {
    const size_t height_after = inout_pc_ptr->height;
    const size_t width_after = inout_pc_ptr->width;
    size_t points_after = 0;
    if (m_lidar_params.keep_organized)
    {
      for (const auto& pt : *inout_pc_ptr)
        if (pcl::isFinite(pt))
          points_after++;
    } else
      points_after = inout_pc_ptr->size();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[PCLFiltration] Processed 3D LIDAR data (run time: %.1f ms; points before: %lu, after: %lu; dim before: (w: %lu, h: %lu), after: (w: %lu, h: %lu)).",
        timer.getLifetime(), points_before, points_after, width_before, height_before, width_after, height_after);
  }


}  // namespace mrs_pcl_tools