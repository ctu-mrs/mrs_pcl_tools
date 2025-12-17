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
    m_initTransformer();
    m_initScopeTimerLogger();
    m_diagnostics_ = std::make_shared<PclFiltrationDiagnostics>(m_node_);


    if (m_lidar_params.republish)
    {
      m_initLidarRepublishing();
    }

    m_pcl_filtration_core_ = std::make_unique<PCLFiltrationCore>(*m_logger_);
    m_pcl_filtration_core_->loadLidarParams(m_lidar_params);

    m_timer_init_->cancel();
    m_is_initialized = true;
  }
  /*//}*/

  /*//{ m_initScopeTimerLogger() */
  void PCLFiltration::m_initScopeTimerLogger()
  {
    m_param_loader_->loadParam("scope_timer/enable", m_scope_timer_enabled, false);

    const std::string time_logger_filepath = m_param_loader_->loadParam2("scope_timer/log_filename", std::string(""));
    m_scope_timer_logger = std::make_shared<mrs_lib::ScopeTimerLogger>(m_node_, time_logger_filepath, m_scope_timer_enabled);
  }
  /*//}*/

  /*//{ m_initTransformer() */
  void PCLFiltration::m_initTransformer()
  {
    const auto uav_name = m_param_loader_->loadParam2<std::string>("uav_name");
    m_transformer_ = std::make_shared<mrs_lib::Transformer>(m_node_);
    m_transformer_->setDefaultPrefix(uav_name);
    m_transformer_->setLookupTimeout(rclcpp::Duration::from_seconds(0.05));
    m_transformer_->retryLookupNewest(false);

    m_lidar_params.cropbox.frame_id = m_transformer_->resolveFrame(m_lidar_params.cropbox.frame_id);
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

    if (!m_param_loader_->loadedSuccessfully())
    {
      RCLCPP_ERROR(this->get_logger(), "[PCLFiltration]: Some compulsory parameters were not loaded successfully, ending the node");
      rclcpp::shutdown();
    }
  }
  /*//}*/

  /*//{ m_readLidarParams() */
  void PCLFiltration::m_readLidarParams()
  {
    m_readLidarGeneralParams();
    m_readLidarClipParams();
    m_readLidarCropboxParams();
    m_readLidarDownSamplingParams();
  }
  /*//}*/

  /*//{ m_readLidarGeneralParams() */
  void PCLFiltration::m_readLidarGeneralParams()
  {
    m_param_loader_->loadParam("lidar3d/name", m_lidar_params.name, std::string("ouster"));
    m_param_loader_->loadParam("lidar3d/frequency", m_lidar_params.frequency);
    m_param_loader_->loadParam("lidar3d/vfov", m_lidar_params.vfov);

    m_param_loader_->loadParam("lidar3d/keep_organized", m_lidar_params.keep_organized, true);
    m_param_loader_->loadParam("lidar3d/republish", m_lidar_params.republish, false);
    m_param_loader_->loadParam("lidar3d/invalid_value", m_lidar_params.invalid_value, std::numeric_limits<float>::quiet_NaN());

    int temp_dynamic_row_offset;
    m_param_loader_->loadParam("lidar3d/dynamic_row_offset", temp_dynamic_row_offset, 0);
    m_lidar_params.dynamic_row_offset = temp_dynamic_row_offset;

  } /*//}*/

  /*//{ m_readLidarClipParams() */
  void PCLFiltration::m_readLidarClipParams()
  {
    m_param_loader_->loadParam("lidar3d/clip/range/use", m_lidar_params.rangeclip.use, false);
    m_param_loader_->loadParam("lidar3d/clip/range/min", m_lidar_params.rangeclip.min_sq, 0.4f);
    m_param_loader_->loadParam("lidar3d/clip/range/max", m_lidar_params.rangeclip.max_sq, 100.0f);
    m_lidar_params.rangeclip.min_mm = m_lidar_params.rangeclip.min_sq * 1000;
    m_lidar_params.rangeclip.max_mm = m_lidar_params.rangeclip.max_sq * 1000;
    m_lidar_params.rangeclip.min_sq *= m_lidar_params.rangeclip.min_sq;
    m_lidar_params.rangeclip.max_sq *= m_lidar_params.rangeclip.max_sq;

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

  /*//{ m_readLidarCropboxParams() */
  void PCLFiltration::m_readLidarCropboxParams()
  {
    m_param_loader_->loadParam("lidar3d/cropbox/crop_inside", m_lidar_params.cropbox.crop_inside);
    m_param_loader_->loadParam("lidar3d/cropbox/frame_id", m_lidar_params.cropbox.frame_id, {});

    Eigen::Vector3d temp_min;
    m_param_loader_->loadMatrixStatic("lidar3d/cropbox/min", temp_min, -std::numeric_limits<float>::infinity() * Eigen::Vector3d::Ones());
    m_lidar_params.cropbox.min = temp_min.cast<float>();
    Eigen::Vector3d temp_max;
    m_param_loader_->loadMatrixStatic("lidar3d/cropbox/max", temp_max, std::numeric_limits<float>::infinity() * Eigen::Vector3d::Ones());
    m_lidar_params.cropbox.max = temp_max.cast<float>();

    // by default, use the cropbox filter if any of the crop coordinates is finite
    const bool cbox_use_default = m_lidar_params.cropbox.min.array().isFinite().any() || m_lidar_params.cropbox.max.array().isFinite().any();
    // the user can override this behavior by setting the "lidar3d/cropbox/use" parameter
    m_param_loader_->loadParam("lidar3d/cropbox/use", m_lidar_params.cropbox.use, cbox_use_default);
  }
  /*//}*/

  /*//{ m_readLidarDownSamplingParams() */
  void PCLFiltration::m_readLidarDownSamplingParams()
  {
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
  }
  /*//}*/

  /*//{ m_initLidarRepublishing() */
  void PCLFiltration::m_initLidarRepublishing()
  {
    if (m_lidar_params.downsample.row_step <= 0 || m_lidar_params.downsample.col_step <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "[PCLFiltration]: Downsampling row/col steps for 3D lidar must be >=1, ending nodelet.");
      rclcpp::shutdown();
    }

    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node = m_node_;
    shopts.node_name = m_node_->get_name();
    shopts.no_message_timeout = rclcpp::Duration::from_seconds(5.0);
    m_sub_lidar = mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>(shopts, "~/lidar_in", &PCLFiltration::m_lidarCallback, this);

    mrs_lib::PublisherHandlerOptions pubopts;
    pubopts.node = m_node_;
    pubopts.qos = rclcpp::QoS(1);

    m_pub_lidar = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(pubopts, "~/lidar_out");
    m_pub_lidar_over_max_range = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(pubopts, "~/lidar_over_max_range_out");
  }
  /*//}*/

  /*//{ m_lidarCallback() */
  void PCLFiltration::m_lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (!m_lidar_params.republish || !m_is_initialized)
    {
      return;
    }

    if (msg->width % m_lidar_params.downsample.col_step != 0 || msg->height % m_lidar_params.downsample.row_step != 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[PCLFiltration] Step-based downsampling of 3D lidar data would create nondeterministic results. "
                  "Data (w: %d, h: %d) with downsampling step (w: %d, h: %d) would leave some samples untouched. "
                  "Skipping lidar frame.",
                  msg->width, msg->height, m_lidar_params.downsample.col_step, m_lidar_params.downsample.row_step);
      return;
    }

    mrs_modules_msgs::msg::PclToolsDiagnostics diag_msg;
    diag_msg.sensor_name = m_lidar_params.name;
    diag_msg.stamp = msg->header.stamp;
    diag_msg.sensor_type = mrs_modules_msgs::msg::PclToolsDiagnostics::SENSOR_TYPE_LIDAR_3D;
    diag_msg.cols_before = msg->width;
    diag_msg.rows_before = msg->height;
    diag_msg.frequency = m_lidar_params.frequency;
    diag_msg.vfov = m_lidar_params.vfov;

    const bool is_ouster_type = hasField("range", msg) && hasField("ring", msg) && hasField("t", msg);
    if (is_ouster_type)
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "[PCLFiltration] Received first 3D LIDAR message. Point type: ouster_ros::Point.");

#ifdef COMPILE_WITH_OUSTER
      m_processPointCloud<PC_OS>(msg, diag_msg);
#else
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "[PCLFiltration] 3D LiDAR message comes from an Ouster sensor, but this package was not compiled with the Ouster flag. Please "
                            "rebuild the package with: --cmake-args -DCOMPILE_WITH_OUSTER=ON");
#endif
    } else
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "[PCLFiltration] Received first 3D LIDAR message. Point type: pcl::PointXYZI.");
      m_processPointCloud<PC_I>(msg, diag_msg);
    }

    m_diagnostics_->publish(diag_msg);
  }
  /*//}*/

  /*//{ m_processPointCloud() */
  template <typename PC>
  void PCLFiltration::m_processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, mrs_modules_msgs::msg::PclToolsDiagnostics& diag_msg)
  {
    typename PC::Ptr cloud = std::make_shared<PC>();
    pcl::fromROSMsg(*msg, *cloud);
    m_processMsg(cloud);
    diag_msg.cols_after = cloud->width;
    diag_msg.rows_after = cloud->height;
  }
  /*//}*/

  /*//{ m_processMsg() */
  template <typename PC>
  void PCLFiltration::m_processMsg(std::shared_ptr<PC>& inout_pc_ptr)
  {
    if (!inout_pc_ptr)
    {
      RCLCPP_WARN(this->get_logger(), "[PCLFiltration] Received null point cloud pointer. Skipping.");
      return;
    }

    mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer(m_node_, "PCLFiltration::process_msg", m_scope_timer_logger, m_scope_timer_enabled);

    const size_t height_before = inout_pc_ptr->height;
    const size_t width_before = inout_pc_ptr->width;
    const size_t points_before = inout_pc_ptr->size();

    if (m_lidar_params.downsample.use)
    {
      DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying downsampling");
      const size_t row_offset = m_lidar_params.dynamic_row_selection_enabled ? m_lidar_params.dynamic_row_offset : m_lidar_params.downsample.row_step - 1;
      m_pcl_filtration_core_->downsample(inout_pc_ptr, m_lidar_params.downsample.row_step, m_lidar_params.downsample.col_step, row_offset);
    }

    const bool use_intensity_or_reflectivity = m_lidar_params.intensity.use || m_lidar_params.reflectivity.use;
    if (m_lidar_params.rangeclip.use)
    {
      DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying range-clipping");
      const bool publish_removed_far = m_pub_lidar_over_max_range.getNumSubscribers() > 0;

      if (use_intensity_or_reflectivity)
      {
        DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying removeCloseAndFarAndLowFields");
        const typename PC::Ptr pcl_over_max_range = m_pcl_filtration_core_->removeCloseAndFarAndLowFields(inout_pc_ptr, false, publish_removed_far);
        if (publish_removed_far)
        {
          sensor_msgs::msg::PointCloud2 pcl_msg;
          pcl::toROSMsg(*pcl_over_max_range, pcl_msg);
          m_pub_lidar_over_max_range.publish(pcl_msg);
        }
      } else
      {
        DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying removeCloseAndFar");
        const typename PC::Ptr pcl_over_max_range = m_pcl_filtration_core_->removeCloseAndFar(inout_pc_ptr, false, publish_removed_far);
        if (publish_removed_far)
        {
          sensor_msgs::msg::PointCloud2 pcl_msg;
          pcl::toROSMsg(*pcl_over_max_range, pcl_msg);
          m_pub_lidar_over_max_range.publish(pcl_msg);
        }
      }
    } else if (use_intensity_or_reflectivity)
    {
      DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying removeCloseAndFar");
      m_pcl_filtration_core_->removeLowFields(inout_pc_ptr);
    } else
    {
    }

    if (m_lidar_params.cropbox.use)
    {
      DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying crobox filter");
      m_cropBoxPointCloud(inout_pc_ptr);
    }

    if (!m_lidar_params.keep_organized)
    {
      DEBUG_LOG(*m_logger_, "[PCLFiltration]: Applying removeInfinitePoints");
      m_pcl_filtration_core_->removeInfinitePoints(inout_pc_ptr);
    }
    inout_pc_ptr->is_dense = !m_lidar_params.keep_organized;

    sensor_msgs::msg::PointCloud2 pcl_msg;
    pcl::toROSMsg(*inout_pc_ptr, pcl_msg);
    m_pub_lidar.publish(pcl_msg);

    if (m_lidar_params.dynamic_row_selection_enabled)
    {
      m_lidar_params.dynamic_row_offset++;
      m_lidar_params.dynamic_row_offset %= m_lidar_params.downsample.row_step;
    }

    m_logPointCloudStats(timer, inout_pc_ptr, height_before, width_before, points_before);
  }
  /*//}*/

}  // namespace mrs_pcl_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_pcl_tools::PCLFiltration)