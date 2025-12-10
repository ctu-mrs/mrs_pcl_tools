#include <rclcpp/rclcpp.hpp>
#include <mrs_pcl_tools/support.h>
#include <mrs_pcl_tools/groundplane_detector.h>

namespace mrs_pcl_tools
{

using namespace std::literals::chrono_literals;

pcl::PointCloud<pcl::PointXYZ>::Ptr getRandomPc()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = 100;
  cloud->height = 100;
  cloud->resize(cloud->width * cloud->height);

  for (auto& point : *cloud)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  return cloud;
}

class Testing : public rclcpp::Node
{
public:
  Testing(rclcpp::NodeOptions options) : Node("mrs_pcl_tools", options)
  {
    m_timer_init_ =
        this->create_wall_timer(std::chrono::duration<double>(0.1s), std::bind(&Testing::m_timerInit, this));
  }

private:
  void m_timerInit()
  {
    m_node_ = this->shared_from_this();
    m_param_loader_ = std::make_shared<mrs_lib::ParamLoader>(m_node_, m_node_->get_name());

    std::vector<std::string> config_files;
    m_param_loader_->loadParam("config_files", config_files);

    for (auto config_file : config_files)
    {
      RCLCPP_INFO(m_node_->get_logger(), "loading config file '%s'", config_file.c_str());
      m_param_loader_->addYamlFile(config_file);
    }

    m_transformer_ = std::make_shared<mrs_lib::Transformer>(m_node_);

    m_ground_detector.initialize(m_node_, m_transformer_,
                                 GroundplaneDetector::loadCfg(*m_param_loader_, "lidar3d/ground_removal/"));

    auto test_pc = getRandomPc();
    auto temp = m_ground_detector.detectGroundplane(test_pc);

    m_timer_init_->cancel();
  }

private:
  rclcpp::TimerBase::SharedPtr m_timer_init_;
  rclcpp::Node::SharedPtr m_node_;
  std::shared_ptr<mrs_lib::ParamLoader> m_param_loader_;
  std::shared_ptr<mrs_lib::Transformer> m_transformer_;

  GroundplaneDetector m_ground_detector;
};

}  // namespace mrs_pcl_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_pcl_tools::Testing)