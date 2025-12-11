#include <mrs_pcl_tools/pcl_filtration_ros.h>

namespace mrs_pcl_tools
{
  /*//{ PCLFiltrationCore constructor() */
  PCLFiltration::PCLFiltration(rclcpp::NodeOptions options) : Node("PCLFiltration", options)
  {
    m_timer_init_ = this->create_wall_timer(std::chrono::duration<double>(0.1s), std::bind(&PCLFiltration::m_timerInit, this));
  }
  /*//}*/

  /*//{ m_timerInit() */
  void PCLFiltration::m_timerInit()
  {
    m_node_ = this->shared_from_this();
    m_logger_ = std::make_shared<RosLogger>(this->get_logger());

    m_readParams();

    m_pcl_filtration_core_ = std::make_unique<PCLFiltrationCore>(*m_logger_);
    m_timer_init_->cancel();
  }
  /*//}*/

  /*//{ m_readParams() */
  void PCLFiltration::m_readParams()
  {
    m_param_loader_ = std::make_shared<mrs_lib::ParamLoader>(m_node_, m_node_->get_name());

    std::vector<std::string> config_files;
    m_param_loader_->loadParam("config_files", config_files);

    for (auto config_file : config_files)
    {
      RCLCPP_INFO(m_node_->get_logger(), "loading config file '%s'", config_file.c_str());
      m_param_loader_->addYamlFile(config_file);
    }

    m_readLidarParams();
  }
  /*//}*/

  /*//{ m_readLidarParams() */
  void PCLFiltration::m_readLidarParams()
  {
    m_param_loader_->loadParam("lidar3d/name", m_lidar_params.name, std::string("ouster"));
    m_param_loader_->loadParam("lidar3d/frequency", m_lidar_params.frequency);
    m_param_loader_->loadParam("lidar3d/vfov", m_lidar_params.vfov);

    m_param_loader_->loadParam("lidar3d/keep_organized", m_lidar_params.keep_organized, true);
    m_param_loader_->loadParam("lidar3d/republish", m_lidar_params.republish, false);
    m_param_loader_->loadParam("lidar3d/invalid_value", m_lidar_params.invalid_value, std::numeric_limits<float>::quiet_NaN());
    m_param_loader_->loadParam("lidar3d/clip/range/use", m_lidar_params.rangeclip.use, false);
    m_param_loader_->loadParam("lidar3d/clip/range/min", m_lidar_params.rangeclip.min_sq, 0.4f);
    m_param_loader_->loadParam("lidar3d/clip/range/max", m_lidar_params.rangeclip.max_sq, 100.0f);
    m_lidar_params.rangeclip.min_mm = m_lidar_params.rangeclip.min_sq * 1000;
    m_lidar_params.rangeclip.max_mm = m_lidar_params.rangeclip.max_sq * 1000;
    m_lidar_params.rangeclip.min_sq *= m_lidar_params.rangeclip.min_sq;
    m_lidar_params.rangeclip.max_sq *= m_lidar_params.rangeclip.max_sq;

    // load downsampling parameters
    m_param_loader_->loadParam("lidar3d/downsampling/dynamic_row_selection", m_lidar_params.dynamic_row_selection_enabled, false);
    m_param_loader_->loadParam("lidar3d/downsampling/row_step", m_lidar_params.downsample.row_step, 1);
    m_param_loader_->loadParam("lidar3d/downsampling/col_step", m_lidar_params.downsample.col_step, 1);

    // load dynamic row selection
    if (m_lidar_params.dynamic_row_selection_enabled && m_lidar_params.downsample.row_step > 1 && m_lidar_params.downsample.row_step % 2 != 0)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[PCLFiltration]: Dynamic selection of lidar rows is enabled, but `lidar_row_step` is not even and/or greater than 1. Ending node.");
      rclcpp::shutdown();
    }

    m_lidar_params.downsample.use =
        m_lidar_params.dynamic_row_selection_enabled || m_lidar_params.downsample.row_step > 1 || m_lidar_params.downsample.col_step > 1;
    if (m_lidar_params.downsample.use)
    {
      RCLCPP_INFO(this->get_logger(), "[PCLFiltration] Downsampling of input lidar data is enabled -> dynamically: %s, row step: %d, col step: %d",
                  m_lidar_params.dynamic_row_selection_enabled ? "true" : "false", m_lidar_params.downsample.row_step, m_lidar_params.downsample.col_step);
    } else
    {
      RCLCPP_INFO(this->get_logger(), "[PCLFiltration] Downsampling of input lidar data is disabled.");
    }

    // load cropbox parameters
    m_param_loader_->loadParam("lidar3d/cropbox/crop_inside", m_lidar_params.cropbox.crop_inside);
    m_param_loader_->loadParam("lidar3d/cropbox/frame_id", m_lidar_params.cropbox.frame_id, {});
    // m_param_loader_->loadMatrixStatic("lidar3d/cropbox/min", m_lidar_params.cropbox.min, -std::numeric_limits<float>::infinity() * vec3_t::Ones());
    // m_param_loader_->loadMatrixStatic("lidar3d/cropbox/max", m_lidar_params.cropbox.max, std::numeric_limits<float>::infinity() * vec3_t::Ones());

    // by default, use the cropbox filter if any of the crop coordinates is finite
    const bool cbox_use_default = m_lidar_params.cropbox.min.array().isFinite().any() || m_lidar_params.cropbox.max.array().isFinite().any();
    // the user can override this behavior by setting the "lidar3d/cropbox/use" parameter
    m_param_loader_->loadParam("lidar3d/cropbox/use", m_lidar_params.cropbox.use, cbox_use_default);


    m_param_loader_->loadParam("lidar3d/clip/intensity/use", m_lidar_params.intensity.use, false);
    m_param_loader_->loadParam("lidar3d/clip/intensity/threshold", m_lidar_params.intensity.threshold, std::numeric_limits<float>::max());
    m_param_loader_->loadParam("lidar3d/clip/intensity/range", m_lidar_params.intensity.range_sq, std::numeric_limits<float>::max());
    m_lidar_params.intensity.range_mm = m_lidar_params.intensity.range_sq * 1000;
    m_lidar_params.intensity.range_sq *= m_lidar_params.intensity.range_sq;

    m_param_loader_->loadParam("lidar3d/clip/reflectivity/use", m_lidar_params.reflectivity.use, false);
    m_param_loader_->loadParam("lidar3d/clip/reflectivity/range", m_lidar_params.reflectivity.range_sq, std::numeric_limits<float>::max());
    const int lidar3d_filter_reflectivity_threshold =
        m_param_loader_->loadParam2("lidar3d/clip/reflectivity/threshold", static_cast<int>(std::numeric_limits<uint16_t>::max()));
    m_lidar_params.reflectivity.threshold = static_cast<uint16_t>(lidar3d_filter_reflectivity_threshold);
    m_lidar_params.reflectivity.range_mm = m_lidar_params.reflectivity.range_sq * 1000;
    m_lidar_params.reflectivity.range_sq *= m_lidar_params.reflectivity.range_sq;
  }
  /*//}*/

}  // namespace mrs_pcl_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_pcl_tools::PCLFiltration)