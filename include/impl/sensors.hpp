
/* class SensorDepthCamera */

/*//{ initialize() */
void SensorDepthCamera::initialize(ros::NodeHandle& nh, mrs_lib::ParamLoader& pl, const std::string& prefix, const std::string& name) {
  _nh         = nh;
  sensor_name = name;

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
  pl.loadParam("depth/" + sensor_name + "/topic/landing_spot_detection_out", landing_spot_detection_out, std::string(""));
  pl.loadParam("depth/" + sensor_name + "/topic/landing_spot_detection_dbg_out", landing_spot_detection_dbg_out, std::string(""));

  publish_over_max_range = points_over_max_range_out.size() > 0 && range_clip_use;

  pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/use", landing_spot_detection_use, false);
  if (landing_spot_detection_use && landing_spot_detection_out.size() > 0) {
    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/square_size", landing_spot_detection_square_size);
    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/plane_z_normal_threshold", landing_spot_detection_z_normal_threshold);
    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/ransac_distance_threshold", landing_spot_detection_ransac_distance_threshold);
    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/frame_step", landing_spot_detection_frame_step, 1);
  }

  if (prefix.size() > 0) {

    depth_in                       = "/" + prefix + "/" + depth_in;
    depth_camera_info_in           = "/" + prefix + "/" + depth_camera_info_in;
    points_out                     = "/" + prefix + "/" + points_out;
    landing_spot_detection_out     = "/" + prefix + "/" + landing_spot_detection_out;
    landing_spot_detection_dbg_out = "/" + prefix + "/" + landing_spot_detection_dbg_out;

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

  if (landing_spot_detection_use && landing_spot_detection_frame++ % landing_spot_detection_frame_step == 0) {

    const bool ret_dbg_pcl                                                     = pub_landing_spot_dbg_pcl.getNumSubscribers() > 0;
    const auto& [landing_feasible, plane_normal_z, spot_pos_in_world, dbg_pcl] = detectLandingPosition(cloud, ret_dbg_pcl);

    // Publish if found
    if (landing_feasible) {

      const darpa_mrs_msgs::LandingSpot::Ptr ls_msg = boost::make_shared<darpa_mrs_msgs::LandingSpot>();

      ls_msg->stamp             = depth_msg->header.stamp;
      ls_msg->safe_landing_spot = spot_pos_in_world;
      ls_msg->normal_z          = plane_normal_z;

      try {
        pub_landing_spot_detection.publish(ls_msg);
      }
      catch (...) {
      }
    }

    // Publish debug cloud
    if (ret_dbg_pcl && dbg_pcl) {
      try {
        pub_landing_spot_dbg_pcl.publish(dbg_pcl);
      }
      catch (...) {
      }
    }
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
    pub_points = _nh.advertise<sensor_msgs::PointCloud2>(points_out, 10);
    if (publish_over_max_range)
      pub_points_over_max_range = _nh.advertise<sensor_msgs::PointCloud2>(points_over_max_range_out, 10);
    if (landing_spot_detection_use) {
      pub_landing_spot_detection = _nh.advertise<darpa_mrs_msgs::LandingSpot>(landing_spot_detection_out, 1);
      pub_landing_spot_dbg_pcl   = _nh.advertise<sensor_msgs::PointCloud2>(landing_spot_detection_dbg_out, 1);
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

/*//{ detectLandingPosition() */
std::tuple<bool, float, geometry_msgs::Point, PC_RGB::Ptr> SensorDepthCamera::detectLandingPosition(const PC::Ptr& cloud, const bool ret_dbg_pcl) {

  size_t      pc_dbg_it = 0;
  PC_RGB::Ptr pc_dbg;

  // Crop square of landing spot size in data
  PC::Ptr     square = boost::make_shared<PC>();
  const float d      = landing_spot_detection_square_size / 2.0f;

  // TODO: TEST Filter data by xy distance (keep middle-area square of edge size 2*d)
  pcl::CropBox<pt_XYZ> filter;
  filter.setMin(Eigen::Vector4f(-d, -d, 0.0f, 1.0f));
  filter.setMax(Eigen::Vector4f(d, d, 100.0f, 1.0f));
  filter.setInputCloud(cloud);
  filter.filter(*square);

  if (square->size() < 4)
    return std::make_tuple(false, -1.0f, geometry_msgs::Point(), pc_dbg);

  if (ret_dbg_pcl) {

    pc_dbg         = boost::make_shared<PC_RGB>();
    pc_dbg->header = cloud->header;

    // Publish square as red points
    pc_dbg->resize(square->size());
    for (const auto& p : square->points) {
      pc_dbg->at(pc_dbg_it).x   = p.x;
      pc_dbg->at(pc_dbg_it).y   = p.y;
      pc_dbg->at(pc_dbg_it).z   = p.z;
      pc_dbg->at(pc_dbg_it).r   = 255;
      pc_dbg->at(pc_dbg_it).g   = 0;
      pc_dbg->at(pc_dbg_it++).b = 0;
    }
  }

  // TODO: TEST Match planar surface on cropped data
  pcl::ModelCoefficients       coefficients;
  pcl::PointIndices            inliers;
  pcl::SACSegmentation<pt_XYZ> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(landing_spot_detection_ransac_distance_threshold);
  seg.setInputCloud(square);
  seg.segment(inliers, coefficients);

  if (inliers.indices.size() == 0)
    return std::make_tuple(false, -1.0f, geometry_msgs::Point(), pc_dbg);

  if (ret_dbg_pcl) {

    // Publish plane inliers as green points
    pc_dbg->resize(square->size() + inliers.indices.size());
    for (const auto& i : inliers.indices) {
      const auto& p             = square->at(i);
      pc_dbg->at(pc_dbg_it).x   = p.x;
      pc_dbg->at(pc_dbg_it).y   = p.y;
      pc_dbg->at(pc_dbg_it).z   = p.z;
      pc_dbg->at(pc_dbg_it).r   = 0;
      pc_dbg->at(pc_dbg_it).g   = 255;
      pc_dbg->at(pc_dbg_it++).b = 0;
    }
  }

  // TODO: Transform data to world
  geometry_msgs::Point spot_pos_in_world;

  // TODO: Decide if planar surface is ground or not
  const float n_z = std::fabs(coefficients.values[2]);
  if (std::fabs(n_z - 1.0f) < landing_spot_detection_z_normal_threshold) {
    // TODO: find center and return it as point
    return std::make_tuple(true, n_z, spot_pos_in_world, pc_dbg);
  }

  return std::make_tuple(false, -1.0f, geometry_msgs::Point(), pc_dbg);
}
/*//}*/
