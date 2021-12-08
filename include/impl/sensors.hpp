
/* class SensorDepthCamera */

/*//{ initialize() */
void SensorDepthCamera::initialize(const ros::NodeHandle& nh, mrs_lib::ParamLoader& pl, const std::shared_ptr<mrs_lib::Transformer>& transformer,
                                   const std::string& prefix, const std::string& name) {

  _nh          = nh;
  _transformer = transformer;
  sensor_name  = name;

  pl.loadParam("depth/" + sensor_name + "/filter/downsample/step/col", downsample_step_col, 1);
  pl.loadParam("depth/" + sensor_name + "/filter/downsample/step/row", downsample_step_row, 1);
  if (downsample_step_col < 1)
    downsample_step_col = 1;
  if (downsample_step_row < 1)
    downsample_step_row = 1;

  pl.loadParam("depth/" + sensor_name + "/filter/range_clip/min", range_clip_min, 0.0f);
  pl.loadParam("depth/" + sensor_name + "/filter/range_clip/max", range_clip_max, std::numeric_limits<float>::max());
  range_clip_use    = range_clip_max > 0.0f && range_clip_max > range_clip_min;
  replace_nan_depth = range_clip_use ? 10.0f * range_clip_max : 1000.0f;

  pl.loadParam("depth/" + sensor_name + "/filter/voxel_grid/resolution", voxel_grid_resolution, 0.0f);
  voxel_grid_use = voxel_grid_resolution > 0.0f;

  pl.loadParam("depth/" + sensor_name + "/filter/radius_outlier/radius", radius_outlier_radius, 0.0f);
  pl.loadParam("depth/" + sensor_name + "/filter/radius_outlier/neighbors", radius_outlier_neighbors, 0);
  radius_outlier_use = radius_outlier_radius > 0.0f && radius_outlier_neighbors > 0;

  pl.loadParam("depth/" + sensor_name + "/filter/minimum_grid/resolution", minimum_grid_resolution, 0.0f);
  minimum_grid_use = minimum_grid_resolution > 0.0f;

  pl.loadParam("depth/" + sensor_name + "/filter/bilateral/sigma_S", bilateral_sigma_S, 0.0f);
  pl.loadParam("depth/" + sensor_name + "/filter/bilateral/sigma_R", bilateral_sigma_R, 0.0f);
  bilateral_use = bilateral_sigma_S > 0.0f && bilateral_sigma_R > 0.0f;

  pl.loadParam("depth/" + sensor_name + "/topic/depth_in", depth_in);
  pl.loadParam("depth/" + sensor_name + "/topic/depth_camera_info_in", depth_camera_info_in);
  pl.loadParam("depth/" + sensor_name + "/topic/points_out", points_out);
  pl.loadParam("depth/" + sensor_name + "/topic/points_over_max_range_out", points_over_max_range_out, std::string(""));

  publish_over_max_range = points_over_max_range_out.size() > 0 && range_clip_use;

  if (prefix.size() > 0) {

    depth_in             = "/" + prefix + "/" + depth_in;
    depth_camera_info_in = "/" + prefix + "/" + depth_camera_info_in;
    points_out           = "/" + prefix + "/" + points_out;

    if (publish_over_max_range)
      points_over_max_range_out = "/" + prefix + "/" + points_over_max_range_out;
  }

  // Start subscribe handler for depth camera info
  mrs_lib::SubscribeHandlerOptions shopts(_nh);
  shopts.node_name  = "SensorDepthCamera::CameraInfo::" + sensor_name;
  shopts.threadsafe = true;
  sh_camera_info    = mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo>(shopts, depth_camera_info_in);
  mrs_lib::construct_object(sh_camera_info, shopts, depth_camera_info_in, ros::Duration(20.0), &SensorDepthCamera::process_camera_info_msg, this);

  initialized = true;
}
/*//}*/

/*//{ setScopeTimerLogger() */
void SensorDepthCamera::setScopeTimerLogger(const std::shared_ptr<mrs_lib::ScopeTimerLogger> logger_ptr, const bool enable_scope_timer) {
  _enable_scope_timer = enable_scope_timer;
  _scope_time_logger  = logger_ptr;
}
/*//}*/

/*//{ convertDepthToCloud() */
// Depth conversion inspired by:
// https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h#L48
template <typename T>
void SensorDepthCamera::convertDepthToCloud(const sensor_msgs::Image::ConstPtr& depth_msg, PC::Ptr& out_pc, PC::Ptr& removed_pc,
                                            const bool return_removed_close, const bool return_removed_far, const bool replace_nans) {

  // TODO: keep ordered

  const unsigned int max_points_count = (image_height / downsample_step_row) * (image_width / downsample_step_col);

  out_pc = boost::make_shared<PC>();
  out_pc->resize(max_points_count);
  if (return_removed_close || return_removed_far)
    removed_pc = boost::make_shared<PC>();
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
        if (replace_nans)
          imagePointToCloudPoint(u, v, replace_nan_depth, removed_pc->points.at(removed_it++));
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

/*//{ imagePointToCloudPoint() */
void SensorDepthCamera::imagePointToCloudPoint(const int x, const int y, const float depth, pt_XYZ& point) {

  const float dc = depth * focal_length_inverse;

  point.x = (x - image_center_x) * dc;
  point.y = (y - image_center_y) * dc;
  point.z = depth;
}
/*//}*/

/*//{ process_depth_msg() */
void SensorDepthCamera::process_depth_msg(mrs_lib::SubscribeHandler<sensor_msgs::Image>& sh) {

  if (!initialized)
    return;

  const std::string   scope_label = "PCLFiltration::process_depth_msg::" + sensor_name;
  mrs_lib::ScopeTimer timer       = mrs_lib::ScopeTimer(scope_label, _scope_time_logger, _enable_scope_timer);

  PC::Ptr     cloud, cloud_over_max_range;
  const auto& depth_msg = sh.getMsg();

  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || depth_msg->encoding == sensor_msgs::image_encodings::MONO16) {
    convertDepthToCloud<uint16_t>(depth_msg, cloud, cloud_over_max_range, false, publish_over_max_range, false);
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convertDepthToCloud<float>(depth_msg, cloud, cloud_over_max_range, false, publish_over_max_range, false);
  } else {
    ROS_ERROR_THROTTLE(5.0, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

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
    if (publish_over_max_range)
      pub_points_over_max_range.publish(cloud_over_max_range);
  }
  catch (...) {
  }
}
/*//}*/

/*//{ process_camera_info_msg() */
void SensorDepthCamera::process_camera_info_msg(mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo>& sh) {

  if (!initialized || has_camera_info)
    return;

  const auto msg = sh.getMsg();

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
    if (publish_over_max_range)
      pub_points_over_max_range = _nh.advertise<sensor_msgs::PointCloud2>(points_over_max_range_out, 1);

    // Start subscribe handler for depth data
    mrs_lib::SubscribeHandlerOptions shopts(_nh);
    shopts.node_name  = "SensorDepthCamera::Image::" + sensor_name;
    shopts.threadsafe = true;
    sh_depth          = mrs_lib::SubscribeHandler<sensor_msgs::Image>(shopts, depth_in);
    mrs_lib::construct_object(sh_depth, shopts, depth_in, ros::Duration(1.0), &SensorDepthCamera::process_depth_msg, this);
  }
}
/*//}*/
