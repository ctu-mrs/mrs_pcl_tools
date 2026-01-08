#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <mrs_modules_msgs/msg/pcl_tools_diagnostics.hpp>

namespace mrs_pcl_tools
{

  class PclFiltrationDiagnostics
  {
  public:
    PclFiltrationDiagnostics(const rclcpp::Node::SharedPtr& node);
    void publish(const mrs_modules_msgs::msg::PclToolsDiagnostics& msg);

  private:
    const rclcpp::Node::SharedPtr m_nh_;
    rclcpp::Publisher<mrs_modules_msgs::msg::PclToolsDiagnostics>::SharedPtr m_pub_diagnostics_;
  };


}  // namespace mrs_pcl_tools