#include <mrs_pcl_tools/pcl_filtration_diagnostics.h>

namespace mrs_pcl_tools
{
  /*//{ PclFiltrationDiagnostics constructor */
  PclFiltrationDiagnostics::PclFiltrationDiagnostics(const rclcpp::Node::SharedPtr& node) : m_nh_(node)
  {
    m_pub_diagnostics_ = m_nh_->create_publisher<mrs_modules_msgs::msg::PclToolsDiagnostics>("~/diagnostics_out", 10);
  }
  /*//}*/

  /*//{ PclFiltrationDiagnostics publish()*/
  void PclFiltrationDiagnostics::publish(const mrs_modules_msgs::msg::PclToolsDiagnostics& msg)
  {
    if (m_pub_diagnostics_->get_subscription_count() > 0)
    {
      try
      {
        m_pub_diagnostics_->publish(msg);
      }
      catch (...)
      {
        RCLCPP_ERROR(m_nh_->get_logger(), "[PCLFiltration] Failed to publish msg on topic (%s).", m_pub_diagnostics_->get_topic_name());
      }
    }
  }
  /*//}*/

}  // namespace mrs_pcl_tools