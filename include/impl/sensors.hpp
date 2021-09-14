
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
  pl.loadParam("depth/" + sensor_name + "/topic/landing_spot_detection_out", landing_spot_detection_out, std::string(""));
  pl.loadParam("depth/" + sensor_name + "/topic/landing_spot_detection_dbg_out", landing_spot_detection_dbg_out, std::string(""));

  publish_over_max_range = points_over_max_range_out.size() > 0 && range_clip_use;

  pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/use", landing_spot_detection_use, false);
  if (landing_spot_detection_use && landing_spot_detection_out.size() > 0) {

    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/world_frame", frame_world);

    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/square/size", landing_spot_detection_square_size);
    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/square/max_ratio", landing_spot_detection_square_max_ratio);
    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/plane_detection/normal_z_threshold", landing_spot_detection_z_normal_threshold);
    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/plane_detection/ransac_distance_threshold",
                 landing_spot_detection_ransac_distance_threshold);
    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/plane_detection/min_inliers_ratio", landing_spot_detection_min_inliers_ratio);
    pl.loadParam("depth/" + sensor_name + "/landing_spot_detection/frame_step", landing_spot_detection_frame_step, 1);

    _seg_plane_ptr = std::make_unique<pcl::SACSegmentation<pt_XYZ>>();
    _seg_plane_ptr->setModelType(pcl::SACMODEL_PLANE);
    _seg_plane_ptr->setMethodType(pcl::SAC_RANSAC);
    _seg_plane_ptr->setDistanceThreshold(landing_spot_detection_ransac_distance_threshold);
  }

  if (prefix.size() > 0) {

    depth_in                       = "/" + prefix + "/" + depth_in;
    depth_camera_info_in           = "/" + prefix + "/" + depth_camera_info_in;
    points_out                     = "/" + prefix + "/" + points_out;
    landing_spot_detection_out     = "/" + prefix + "/" + landing_spot_detection_out;
    landing_spot_detection_dbg_out = "/" + prefix + "/" + landing_spot_detection_dbg_out;

    if (publish_over_max_range)
      points_over_max_range_out = "/" + prefix + "/" + points_over_max_range_out;

    frame_world = prefix + "/" + frame_world;
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

      ls_msg->safe_landing_spot.header.frame_id = frame_world;
      ls_msg->safe_landing_spot.header.stamp    = depth_msg->header.stamp;
      ls_msg->safe_landing_spot.point           = spot_pos_in_world;
      ls_msg->normal_z                          = plane_normal_z;

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
    pub_points = _nh.advertise<sensor_msgs::PointCloud2>(points_out, 1);
    if (publish_over_max_range)
      pub_points_over_max_range = _nh.advertise<sensor_msgs::PointCloud2>(points_over_max_range_out, 1);
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

  // Filter data by xy distance (keep middle-area square of edge size 2*d)
  pcl::CropBox<pt_XYZ> filter;
  filter.setMin(Eigen::Vector4f(-d, -d, 0.0f, 1.0f));
  filter.setMax(Eigen::Vector4f(d, d, 100.0f, 1.0f));
  filter.setInputCloud(cloud);
  filter.filter(*square);

  if (ret_dbg_pcl) {

    pc_dbg         = boost::make_shared<PC_RGB>();
    pc_dbg->header = cloud->header;

    // Publish square as red points
    pc_dbg->resize(square->size());
    for (const auto& p : square->points)
      pc_dbg->at(pc_dbg_it++) = colorPointXYZ(p, 255, 0, 0);
  }

  if (square->size() < 4) {
    ROS_INFO_COND(ret_dbg_pcl, "[LandingSpotDetection] Square points cloud is empty. Not safe to land here.");
    return std::make_tuple(false, -1.0f, geometry_msgs::Point(), pc_dbg);
  }

  // Make sure the width of the square is the true width, not a smaller one
  pcl::PointXYZ pt_min, pt_max;
  pcl::getMinMax3D(*square, pt_min, pt_max);
  const float height          = (pt_max.y - pt_min.y);  // In optical frame
  const float width           = (pt_max.x - pt_min.x);  // In optical frame
  const float min_square_size = 0.9f * landing_spot_detection_square_size;
  if (width < min_square_size || height < min_square_size) {
    ROS_INFO("[LandingSpotDetection] Square with expected size (%.1f) is too small (width: %.1f, height: %.1f). Not safe to land here.",
             landing_spot_detection_square_size, width, height);
    return std::make_tuple(false, -1.0f, geometry_msgs::Point(), pc_dbg);
  }

  // If square data take a large portion of the entire scan, ignore the scan
  const float square_points_ratio = float(square->size()) / float(cloud->size());
  if (square_points_ratio > landing_spot_detection_square_max_ratio) {
    ROS_INFO_COND(ret_dbg_pcl, "[LandingSpotDetection] Square points ratio (%0.1f) to given cloud is low. Not safe to land here.", square_points_ratio);
    return std::make_tuple(false, -1.0f, geometry_msgs::Point(), pc_dbg);
  }

  // Match planar surface on cropped data
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices      inliers;
  _seg_plane_ptr->setInputCloud(square);
  _seg_plane_ptr->segment(inliers, coefficients);

  // If a plane was found but the inliers portion is low, ignore the scan
  const float plane_inliers_ratio = float(inliers.indices.size()) / float(square->size());
  if (plane_inliers_ratio < landing_spot_detection_min_inliers_ratio) {
    ROS_INFO_COND(ret_dbg_pcl, "[LandingSpotDetection] Plane inliers ratio (%0.1f) to given square is low. Not safe to land here.", plane_inliers_ratio);
    return std::make_tuple(false, -1.0f, geometry_msgs::Point(), pc_dbg);
  }

  // Find sensor->world TF
  const auto ret_data_in_world_tf = _transformer->getTransform(cloud->header.frame_id, frame_world, ros::Time(0), true);
  if (!ret_data_in_world_tf) {
    ROS_ERROR_THROTTLE(1.0, "[PCLFiltration] Could not transform depth image from sensor frame (%s) to world frame (%s) on camera (%s).",
                       cloud->header.frame_id.c_str(), frame_world.c_str(), sensor_name.c_str());
    return std::make_tuple(true, -1.0f, geometry_msgs::Point(), pc_dbg);
  }

  // Transform plane coefficients to world
  const Eigen::Affine3d T_affine        = ret_data_in_world_tf.value().getTransformEigen();
  const Eigen::Matrix4f T_data_in_world = T_affine.matrix().cast<float>();
  const Eigen::Vector4f coefficients_in_sensor =
      Eigen::Vector4f(coefficients.values.at(0), coefficients.values.at(1), coefficients.values.at(2), coefficients.values.at(3));
  // https://stackoverflow.com/questions/7685495/transforming-a-3d-plane-using-a-4x4-matrix/7706849
  const Eigen::Vector4f coefficients_in_world = (T_data_in_world.inverse().transpose() * coefficients_in_sensor).normalized();

  // Decide if planar surface is ground or not
  const float n_z               = std::fabs(coefficients_in_world.z());
  const bool  safe_landing_spot = n_z > landing_spot_detection_z_normal_threshold;

  if (!safe_landing_spot) {
    ROS_INFO_COND(ret_dbg_pcl, "[LandingSpotDetection] Plane normal z-axis component is too steep (%0.1f). Not safe to land here.", n_z);
    return std::make_tuple(false, n_z, geometry_msgs::Point(), pc_dbg);
  }

  if (ret_dbg_pcl)
    pc_dbg->resize(square->size() + inliers.indices.size());

  // Store centroid of inlier points + fill inliers with green color
  Eigen::Vector4f centroid_in_sensor = Eigen::Vector4f::Zero();
  for (const auto& i : inliers.indices) {
    const auto& p = square->at(i);
    centroid_in_sensor.x() += p.x;
    centroid_in_sensor.y() += p.y;
    centroid_in_sensor.z() += p.z;

    if (ret_dbg_pcl)
      pc_dbg->at(pc_dbg_it++) = colorPointXYZ(p, 0, 255, 0);
  }

  centroid_in_sensor /= float(inliers.indices.size());
  centroid_in_sensor.w() = 1.0;

  if (ret_dbg_pcl)
    pc_dbg->push_back(colorPointXYZ(centroid_in_sensor, 0, 0, 255));

  // Transform centroid to world
  const Eigen::Vector4f centroid_in_world = T_data_in_world * centroid_in_sensor;
  geometry_msgs::Point  pt_centroid_in_world;
  pt_centroid_in_world.x = centroid_in_world.x();
  pt_centroid_in_world.y = centroid_in_world.y();
  pt_centroid_in_world.z = centroid_in_world.z();

  return std::make_tuple(true, n_z, pt_centroid_in_world, pc_dbg);
}
/*//}*/

/*//{ colorPointXYZ() */
pt_XYZRGB SensorDepthCamera::colorPointXYZ(const pt_XYZ& point, const uint8_t& ch_r, const uint8_t& ch_g, const uint8_t& ch_b) {
  return colorPointXYZ(point.x, point.y, point.z, ch_r, ch_g, ch_b);
}

pt_XYZRGB SensorDepthCamera::colorPointXYZ(const Eigen::Vector4f& point, const uint8_t& ch_r, const uint8_t& ch_g, const uint8_t& ch_b) {
  return colorPointXYZ(point.x(), point.y(), point.z(), ch_r, ch_g, ch_b);
}

pt_XYZRGB SensorDepthCamera::colorPointXYZ(const float& x, const float& y, const float& z, const uint8_t& ch_r, const uint8_t& ch_g, const uint8_t& ch_b) {
  pt_XYZRGB pt_rgb;
  pt_rgb.x = x;
  pt_rgb.y = y;
  pt_rgb.z = z;
  pt_rgb.r = ch_r;
  pt_rgb.g = ch_g;
  pt_rgb.b = ch_b;
  return pt_rgb;
}
/*//}*/
