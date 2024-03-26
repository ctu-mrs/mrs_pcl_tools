
/* class SensorDepthCamera */

/*//{ initialize() */
void SensorDepthCamera::initialize(const ros::NodeHandle& nh, const std::shared_ptr<CommonHandlers_t> common_handlers, const std::string& prefix,
                                   const std::string& name) {

  _nh              = nh;
  _common_handlers = common_handlers;
  sensor_name      = name;

  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/frequency", frequency);
  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/vfov", vfov);
  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/keep_ordered", keep_ordered);

  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/mask/enabled", mask_use, false);
  if (mask_use) {
    _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/mask/topic/mask_in", mask_in);
    _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/mask/topic/depth_out", masked_out);
  }

  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/filter/downsample/step/col", downsample_step_col, 1);
  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/filter/downsample/step/row", downsample_step_row, 1);
  if (downsample_step_col < 1) {
    downsample_step_col = 1;
  }
  if (downsample_step_row < 1) {
    downsample_step_row = 1;
  }

  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/filter/range_clip/min", range_clip_min, 0.0f);
  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/filter/range_clip/max", range_clip_max, std::numeric_limits<float>::max());
  range_clip_use    = range_clip_max > 0.0f && range_clip_max > range_clip_min;
  replace_nan_depth = range_clip_use ? 10.0f * range_clip_max : 1000.0f;

  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/filter/voxel_grid/resolution", voxel_grid_resolution, 0.0f);
  voxel_grid_use = voxel_grid_resolution > 0.0f;

  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/filter/radius_outlier/radius", radius_outlier_radius, 0.0f);
  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/filter/radius_outlier/neighbors", radius_outlier_neighbors, 0);
  radius_outlier_use = radius_outlier_radius > 0.0f && radius_outlier_neighbors > 0;

  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/filter/minimum_grid/resolution", minimum_grid_resolution, 0.0f);
  minimum_grid_use = minimum_grid_resolution > 0.0f;

  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/filter/bilateral/sigma_S", bilateral_sigma_S, 0.0f);
  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/filter/bilateral/sigma_R", bilateral_sigma_R, 0.0f);
  bilateral_use = bilateral_sigma_S > 0.0f && bilateral_sigma_R > 0.0f;

  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/topic/depth_in", depth_in);
  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/topic/depth_camera_info_in", depth_camera_info_in);
  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/topic/points_out", points_out);
  _common_handlers->param_loader->loadParam("depth/" + sensor_name + "/topic/points_over_max_range_out", points_over_max_range_out, std::string(""));


  publish_over_max_range = points_over_max_range_out.size() > 0 && range_clip_use;

  if (prefix.size() > 0) {

    depth_in             = "/" + prefix + "/" + depth_in;
    depth_camera_info_in = "/" + prefix + "/" + depth_camera_info_in;
    points_out           = "/" + prefix + "/" + points_out;

    if (publish_over_max_range) {
      points_over_max_range_out = "/" + prefix + "/" + points_over_max_range_out;
    }

    if (mask_use) {
      mask_in             = "/" + prefix + "/" + mask_in;
      masked_out             = "/" + prefix + "/" + masked_out;
    }
  }

  // Start subscribe handler for depth camera info
  mrs_lib::SubscribeHandlerOptions shopts(_nh);
  shopts.node_name  = "SensorDepthCamera::CameraInfo::" + sensor_name;
  shopts.threadsafe = true;
  sh_camera_info    = mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo>(shopts, depth_camera_info_in);
  mrs_lib::construct_object(sh_camera_info, shopts, depth_camera_info_in, ros::Duration(20.0), &SensorDepthCamera::process_camera_info_msg, this);

  if (mask_use) {
    sh_mask    = mrs_lib::SubscribeHandler<sensor_msgs::Image>(shopts, mask_in);
    mrs_lib::construct_object(sh_mask, shopts, mask_in, ros::Duration(1.0), &SensorDepthCamera::process_mask_msg, this);
  }

  initialized = true;
}
/*//}*/

/*//{ convertDepthToCloud() */
// Depth conversion inspired by:
// https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h#L48
template <typename T>
void SensorDepthCamera::convertDepthToCloud(const sensor_msgs::Image::ConstPtr& depth_msg, PC::Ptr& out_pc, PC::Ptr& removed_pc,
                                            const bool return_removed_close, const bool return_removed_far, const bool replace_nans, const bool keep_ordered) {

  if (keep_ordered) {

    convertDepthToCloudOrdered<T>(depth_msg, out_pc, removed_pc, return_removed_close, return_removed_far, replace_nans);

  } else {

    convertDepthToCloudUnordered<T>(depth_msg, out_pc, removed_pc, return_removed_close, return_removed_far, replace_nans);
  }
}
/*//}*/

/*//{ convertDepthToCloudUnordered() */
// Depth conversion inspired by:
// https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h#L48
template <typename T>
void SensorDepthCamera::convertDepthToCloudUnordered(const sensor_msgs::Image::ConstPtr& depth_msg, PC::Ptr& out_pc, PC::Ptr& removed_pc,
                                                     const bool return_removed_close, const bool return_removed_far, const bool replace_nans) {

  const unsigned int max_points_count = (image_height / downsample_step_row) * (image_width / downsample_step_col);

  out_pc = boost::make_shared<PC>();
  out_pc->resize(max_points_count);
  if (return_removed_close || return_removed_far) {
    removed_pc = boost::make_shared<PC>();
  }
  removed_pc->resize(max_points_count);

  const T*  depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  const int row_step  = downsample_step_row * depth_msg->step / sizeof(T);

  size_t converted_it = 0;
  size_t removed_it   = 0;

  for (int v = 0; v < (int)depth_msg->height; v += downsample_step_row, depth_row += row_step) {
    for (int u = 0; u < (int)depth_msg->width; u += downsample_step_col) {
      const auto depth_raw = depth_row[u];
      const bool valid     = DepthTraits<T>::valid(depth_raw);

      if (!valid) {

        if (replace_nans) {
          imagePointToCloudPoint(u, v, replace_nan_depth, removed_pc->points.at(removed_it++));
        }

        continue;
      }

      const float depth         = DepthTraits<T>::toMeters(depth_raw);
      const bool  invalid_close = depth < range_clip_min;
      const bool  invalid_far   = depth > range_clip_max;

      // Convert to point cloud points and optionally clip range
      if (!range_clip_use || (!invalid_close && !invalid_far)) {

        imagePointToCloudPoint(u, v, depth, out_pc->points.at(converted_it++));

      } else if (range_clip_use && ((return_removed_close && invalid_close) || (return_removed_far && invalid_far))) {

        imagePointToCloudPoint(u, v, depth, removed_pc->points.at(removed_it++));
      }
    }
  }

  // Fill headers
  out_pc->width    = converted_it;
  out_pc->height   = converted_it > 0 ? 1 : 0;
  out_pc->is_dense = true;
  pcl_conversions::toPCL(depth_msg->header, out_pc->header);
  out_pc->points.resize(converted_it);

  // Publish removed depth msg data
  if (return_removed_close || return_removed_far) {
    removed_pc->width    = removed_it;
    removed_pc->height   = removed_it > 0 ? 1 : 0;
    removed_pc->is_dense = true;
    removed_pc->header   = out_pc->header;
    removed_pc->points.resize(removed_it);
  }
}
/*//}*/

/*//{ convertDepthToCloudOrdered() */
// Depth conversion inspired by:
// https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h#L48
template <typename T>
void SensorDepthCamera::convertDepthToCloudOrdered(const sensor_msgs::Image::ConstPtr& depth_msg, PC::Ptr& out_pc, PC::Ptr& removed_pc,
                                                   const bool return_removed_close, const bool return_removed_far, const bool replace_nans) {

  const bool         return_removed = return_removed_close || return_removed_far;
  const unsigned int W              = image_width / downsample_step_col;
  const unsigned int H              = image_height / downsample_step_row;

  out_pc           = boost::make_shared<PC>(W, H);
  out_pc->width    = W;
  out_pc->height   = H;
  out_pc->is_dense = false;
  pcl_conversions::toPCL(depth_msg->header, out_pc->header);

  if (return_removed) {
    removed_pc = boost::make_shared<PC>();
  }
  removed_pc->resize(W * H);

  const T*  depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  const int row_step  = downsample_step_row * depth_msg->step / sizeof(T);

  size_t removed_it = 0;

  unsigned int row = 0;

  for (int v = 0; v < (int)depth_msg->height; v += downsample_step_row, depth_row += row_step) {

    unsigned int col = 0;

    for (int u = 0; u < (int)depth_msg->width; u += downsample_step_col) {

      const auto depth_raw = depth_row[u];
      const bool valid     = DepthTraits<T>::valid(depth_raw);
      float      depth     = DepthTraits<T>::toMeters(depth_raw);

      const bool invalid_close = depth < range_clip_min;
      const bool invalid_far   = depth > range_clip_max;

      if (!valid) {
        // fill invalid points with NaN
        out_pc->at(col, row).x = std::numeric_limits<float>::quiet_NaN();
        out_pc->at(col, row).y = std::numeric_limits<float>::quiet_NaN();
        out_pc->at(col, row).z = std::numeric_limits<float>::quiet_NaN();
      } else if (range_clip_use && (invalid_close || invalid_far)) {
        // fill clipped points with NaN
        out_pc->at(col, row).x = std::numeric_limits<float>::quiet_NaN();
        out_pc->at(col, row).y = std::numeric_limits<float>::quiet_NaN();
        out_pc->at(col, row).z = std::numeric_limits<float>::quiet_NaN();
        // return removed points in a separate pointcloud
        if ((return_removed_close && invalid_close) || (return_removed_far && invalid_far)) {
          imagePointToCloudPoint(u, v, depth, removed_pc->points.at(removed_it++));
        }
      } else {
        // fill valid points
        imagePointToCloudPoint(u, v, depth, out_pc->at(col, row));
      }

      col++;
    }

    row++;
  }

  // Fill headers
  if (return_removed) {
    removed_pc->width    = removed_it;
    removed_pc->height   = removed_it > 0 ? 1 : 0;
    removed_pc->is_dense = true;
    removed_pc->header   = out_pc->header;
    removed_pc->points.resize(removed_it);
  }
}
/*//}*/

/*//{ imagePointToCloudPoint() */
void SensorDepthCamera::imagePointToCloudPoint(const int x, const int y, const float depth, pt_XYZ& point) {

  const float dc = depth * focal_length_inverse;

  point.x = (x - image_center_x) * dc;
  point.y = (y - image_center_y) * dc;
  point.z = depth;
}
/*//}*/

/*//{ process_depth_msg() */
void SensorDepthCamera::process_depth_msg(const sensor_msgs::Image::ConstPtr msg) {

  if (!initialized) {
    return;
  }

  if (mask_use && !got_mask_msg) {
    ROS_WARN_THROTTLE(1.0, "[%s]: waiting for mask image msg", ros::this_node::getName().c_str());
    return;
  }

  const std::string         scope_label = "PCLFiltration::process_depth_msg::" + sensor_name;
  const mrs_lib::ScopeTimer timer       = mrs_lib::ScopeTimer(scope_label, _common_handlers->scope_timer_logger, _common_handlers->scope_timer_enabled);

  PC::Ptr     cloud, cloud_over_max_range;
  const auto& depth_msg_raw = msg;

  sensor_msgs::Image::Ptr masked_msg;
  if (mask_use) {
    masked_msg = applyMask(depth_msg_raw); 
    pub_masked_depth.publish(*masked_msg);
  }
  
  const auto& depth_msg = mask_use ? boost::make_shared<sensor_msgs::Image>(*masked_msg) : depth_msg_raw;

  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || depth_msg->encoding == sensor_msgs::image_encodings::MONO16) {
    convertDepthToCloud<uint16_t>(depth_msg, cloud, cloud_over_max_range, false, publish_over_max_range, false, keep_ordered);
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convertDepthToCloud<float>(depth_msg, cloud, cloud_over_max_range, false, publish_over_max_range, false, keep_ordered);
  } else {
    ROS_ERROR_THROTTLE(5.0, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  const mrs_modules_msgs::PclToolsDiagnostics::Ptr diag_msg = boost::make_shared<mrs_modules_msgs::PclToolsDiagnostics>();
  diag_msg->sensor_name                             = sensor_name;
  diag_msg->stamp                                   = depth_msg->header.stamp;
  diag_msg->sensor_type                             = mrs_modules_msgs::PclToolsDiagnostics::SENSOR_TYPE_DEPTH_CAMERA;
  diag_msg->cols_before                             = cloud->width;
  diag_msg->rows_before                             = cloud->height;
  diag_msg->frequency                               = frequency;
  diag_msg->vfov                                    = vfov;

  // Apply filters to the original cloud (beware, the filters are applied in sequential order: no parallelization)
  if (voxel_grid_use) {
    cloud = mrs_pcl_tools::filters::applyVoxelGridFilter(cloud, voxel_grid_resolution);
  }
  if (radius_outlier_use) {
    cloud = mrs_pcl_tools::filters::applyRadiusOutlierFilter(cloud, radius_outlier_radius, radius_outlier_neighbors);
  }
  if (bilateral_use) {
    cloud = mrs_pcl_tools::filters::applyBilateralFilter(cloud, bilateral_sigma_S, bilateral_sigma_R);
  }
  if (minimum_grid_use) {
    cloud = mrs_pcl_tools::filters::applyMinimumGridFilter(cloud, minimum_grid_resolution);
  }

  try {
    pub_points.publish(cloud);
    if (publish_over_max_range) {
      pub_points_over_max_range.publish(cloud_over_max_range);
    }
  }
  catch (...) {
  }

  diag_msg->cols_after = cloud->width;
  diag_msg->rows_after = cloud->height;
  _common_handlers->diagnostics->publish(diag_msg);
}
/*//}*/

/*//{ process_camera_info_msg() */
void SensorDepthCamera::process_camera_info_msg(const sensor_msgs::CameraInfo::ConstPtr msg) {

  if (!initialized || has_camera_info)
    return;

  // Store data required for 2D -> 3D projection
  image_width    = msg->width;
  image_height   = msg->height;
  focal_length   = msg->K.at(0);
  image_center_x = msg->K.at(2);
  image_center_y = msg->K.at(5);

  const bool valid = image_width > 0 && image_height > 0 && focal_length > 0.0 && image_center_x >= 0.0 && image_center_y >= 0.0;

  if (valid) {

    // Shutdown this subscribe handler
    has_camera_info = true;
    sh_camera_info.stop();

    ROS_INFO(
        "[SensorDepthCamera::process_camera_info_msg::%s] Received valid depth camera info (width: %d, height: %d, focal length: %0.2f, cx: %0.1f, cy: "
        "%0.1f)",
        sensor_name.c_str(), image_width, image_height, focal_length, image_center_x, image_center_y);

    focal_length_inverse = 1.0 / focal_length;

    // Advertise publishers
    pub_points = _nh.advertise<sensor_msgs::PointCloud2>(points_out, 1);
    if (publish_over_max_range) {
      pub_points_over_max_range = _nh.advertise<sensor_msgs::PointCloud2>(points_over_max_range_out, 1);
    }

    // Start subscribe handler for depth data
    mrs_lib::SubscribeHandlerOptions shopts(_nh);
    shopts.node_name  = "SensorDepthCamera::Image::" + sensor_name;
    shopts.threadsafe = true;
    sh_depth          = mrs_lib::SubscribeHandler<sensor_msgs::Image>(shopts, depth_in);
    mrs_lib::construct_object(sh_depth, shopts, depth_in, ros::Duration(1.0), &SensorDepthCamera::process_depth_msg, this);
  }
}
/*//}*/

/*//{ process_mask_msg() */
void SensorDepthCamera::process_mask_msg(const sensor_msgs::Image::ConstPtr msg) {

  if (!initialized) {
    return;
  }

  const std::string         scope_label = "PCLFiltration::process_mask_msg::" + sensor_name;
  const mrs_lib::ScopeTimer timer       = mrs_lib::ScopeTimer(scope_label, _common_handlers->scope_timer_logger, _common_handlers->scope_timer_enabled);

  mask_msg = msg;
  got_mask_msg = true;

  pub_masked_depth = _nh.advertise<sensor_msgs::Image>(masked_out, 1);

  sh_mask.stop();

}
/*//}*/

/*//{ applyMask() */
sensor_msgs::Image::Ptr SensorDepthCamera::applyMask(const sensor_msgs::Image::ConstPtr& depth_msg) {

  sensor_msgs::Image::Ptr depth_masked = boost::make_shared<sensor_msgs::Image>(*depth_msg);

  const int col_step = depth_msg->step / depth_msg->width;
  const int row_step  = depth_msg->step;

  const int col_step_mask = mask_msg->step / mask_msg->width;
  const int row_step_mask  = mask_msg->step;

  for (int v = 0; v < (int)depth_msg->height; v += 1) {

    unsigned int depth_col = 0;

    for (int u = 0; u < (int)depth_msg->width; u += 1) {

      if (mask_msg->data[v*row_step_mask+u*col_step_mask] < 1) {

        for (int b = 0; b < col_step; b++) {
          depth_masked->data[v*row_step+u*col_step+b] = 0;
        }

      }

    }

  }
  return depth_masked;
}
/*//}*/
