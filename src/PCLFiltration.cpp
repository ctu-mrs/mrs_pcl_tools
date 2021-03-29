#include "mrs_pcl_tools/PCLFiltration.h"
#include <limits>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/pcl_base.h>
#include <pcl/point_traits.h>

// 1: new ouster_ros driver; 0: old os1_driver
#define OUSTER_ORDERING_TRANSPOSE 1

namespace mrs_pcl_tools
{

/* onInit() //{ */
void PCLFiltration::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Set PCL verbosity level to errors and higher
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  // Get parameters from config file
  mrs_lib::ParamLoader param_loader(nh, "PCLFiltration");

  const auto uav_name = param_loader.loadParam2<std::string>("uav_name");

  /* 3D LIDAR */
  param_loader.loadParam("lidar3d/republish", _lidar3d_republish, false);
  param_loader.loadParam("lidar3d/range_clip/use", _lidar3d_rangeclip_use, false);
  param_loader.loadParam("lidar3d/range_clip/min", _lidar3d_rangeclip_min_sq, 0.4f);
  param_loader.loadParam("lidar3d/range_clip/max", _lidar3d_rangeclip_max_sq, 100.0f);
  _lidar3d_rangeclip_min_mm = _lidar3d_rangeclip_min_sq * 1000;
  _lidar3d_rangeclip_max_mm = _lidar3d_rangeclip_max_sq * 1000;
  _lidar3d_rangeclip_min_sq *= _lidar3d_rangeclip_min_sq;
  _lidar3d_rangeclip_max_sq *= _lidar3d_rangeclip_max_sq;

  // load downsampling parameters
  param_loader.loadParam("lidar3d/downsampling/dynamic_row_selection", _lidar3d_dynamic_row_selection_enabled, false);
  param_loader.loadParam("lidar3d/downsampling/row_step", _lidar3d_row_step, 1);
  param_loader.loadParam("lidar3d/downsampling/col_step", _lidar3d_col_step, 1);

  // load cropbox parameters
  param_loader.loadParam("lidar3d/ground_removal/use", _lidar3d_groundremoval_use, false);
  param_loader.loadParam("lidar3d/ground_removal/range/use", _lidar3d_groundremoval_range_use, false);
  param_loader.loadParam("lidar3d/ground_removal/frame_id", _lidar3d_groundremoval_frame_id, {});
  param_loader.loadParam("lidar3d/ground_removal/max_height", _lidar3d_groundremoval_max_height, std::numeric_limits<float>::infinity());
  param_loader.loadParam("lidar3d/ground_removal/ransac/max_inlier_distance", _lidar3d_groundremoval_ransac_max_inlier_dist, 3.0f);
  param_loader.loadParam("lidar3d/ground_removal/ransac/max_angle_difference", _lidar3d_groundremoval_ransac_max_angle_diff, float(15.0/180.0*M_PI));
  param_loader.loadParam("lidar3d/ground_removal/offset", _lidar3d_groundremoval_offset, 1.0f);
  if (!_lidar3d_groundremoval_use && _lidar3d_groundremoval_range_use)
    ROS_WARN("[PCLFiltration]: Ignoring the \"lidar3d/ground_removal/range/use\" parameter because \"lidar3d/ground_removal/use\" is set to false.");
  if (_lidar3d_groundremoval_use && _lidar3d_groundremoval_range_use)
  {
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh = nh;
    shopts.node_name = "PCLFiltration";
    shopts.no_message_timeout = ros::Duration(5.0);
    mrs_lib::construct_object(_sh_range, shopts, "rangefinder_in");
  }
  /* param_loader.loadParam("lidar3d/ground_removal", _lidar3d_cropbox_min.x(), -std::numeric_limits<float>::infinity()); */

  // load cropbox parameters
  param_loader.loadParam("lidar3d/cropbox/frame_id", _lidar3d_cropbox_frame_id, {});
  param_loader.loadParam("lidar3d/cropbox/min/x", _lidar3d_cropbox_min.x(), -std::numeric_limits<float>::infinity());
  param_loader.loadParam("lidar3d/cropbox/min/y", _lidar3d_cropbox_min.y(), -std::numeric_limits<float>::infinity());
  param_loader.loadParam("lidar3d/cropbox/min/z", _lidar3d_cropbox_min.z(), -std::numeric_limits<float>::infinity());
  _lidar3d_cropbox_min.w() = -std::numeric_limits<float>::infinity();

  param_loader.loadParam("lidar3d/cropbox/max/x", _lidar3d_cropbox_max.x(), std::numeric_limits<float>::infinity());
  param_loader.loadParam("lidar3d/cropbox/max/y", _lidar3d_cropbox_max.y(), std::numeric_limits<float>::infinity());
  param_loader.loadParam("lidar3d/cropbox/max/z", _lidar3d_cropbox_max.z(), std::numeric_limits<float>::infinity());
  _lidar3d_cropbox_max.w() = std::numeric_limits<float>::infinity();

  // by default, use the cropbox filter if any of the crop coordinates is finite
  const bool cbox_use_default = _lidar3d_cropbox_min.array().isFinite().any() || _lidar3d_cropbox_max.array().isFinite().any();
  // the user can override this behavior by setting the "lidar3d/cropbox/use" parameter
  param_loader.loadParam("lidar3d/cropbox/use", _lidar3d_cropbox_use, cbox_use_default);

  // load dynamic row selection
  if (_lidar3d_dynamic_row_selection_enabled && _lidar3d_row_step % 2 != 0) {
    NODELET_ERROR("[PCLFiltration]: Dynamic selection of lidar rows is enabled, but `lidar_row_step` is not even. Ending nodelet.");
    ros::shutdown();
  }

  param_loader.loadParam("lidar3d/filter/intensity/enable", _lidar3d_filter_intensity_en, false);
  param_loader.loadParam("lidar3d/filter/intensity/threshold", _lidar3d_filter_intensity_thrd, std::numeric_limits<int>::max());
  param_loader.loadParam("lidar3d/filter/intensity/range", _lidar3d_filter_intensity_range_sq, std::numeric_limits<float>::max());
  _lidar3d_filter_intensity_range_mm = _lidar3d_filter_intensity_range_sq * 1000;
  _lidar3d_filter_intensity_range_sq *= _lidar3d_filter_intensity_range_sq;

  /* Depth camera */
  param_loader.loadParam("depth/republish", _depth_republish, true);
  param_loader.loadParam("depth/pcl2_over_max_range", _depth_pcl2_over_max_range, false);
  param_loader.loadParam("depth/min_range", _depth_min_range_sq, 0.0f);
  param_loader.loadParam("depth/max_range", _depth_max_range_sq, 8.0f);
  param_loader.loadParam("depth/voxel_resolution", _depth_voxel_resolution, 0.0f);
  param_loader.loadParam("depth/minimum_grid_resolution", _depth_minimum_grid_resolution, 0.03f);
  param_loader.loadParam("depth/use_bilateral", _depth_use_bilateral, false);
  param_loader.loadParam("depth/bilateral_sigma_S", _depth_bilateral_sigma_S, 5.0f);
  param_loader.loadParam("depth/bilateral_sigma_R", _depth_bilateral_sigma_R, 5e-3f);
  param_loader.loadParam("depth/radius_outlier_filter/radius", _depth_radius_outlier_filter_radius, 0.0f);
  param_loader.loadParam("depth/radius_outlier_filter/neighbors", _depth_radius_outlier_filter_neighbors, 0);
  _depth_min_range_sq *= _depth_min_range_sq;
  _depth_max_range_sq *= _depth_max_range_sq;
  _depth_use_radius_outlier_filter = _depth_radius_outlier_filter_radius > 0.0f && _depth_radius_outlier_filter_neighbors > 0;

  /* RPLidar */
  param_loader.loadParam("rplidar/republish", _rplidar_republish, false);
  param_loader.loadParam("rplidar/voxel_resolution", _rplidar_voxel_resolution, 0.0f);

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR("[PCLFiltration]: Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  // setup transformer
  _transformer = mrs_lib::Transformer("PCLFiltration", uav_name);
  _lidar3d_cropbox_frame_id = _transformer.resolveFrameName(_lidar3d_cropbox_frame_id);


  if (_lidar3d_republish) {

    if (_lidar3d_row_step <= 0 || _lidar3d_col_step <= 0) {
      NODELET_ERROR("[PCLFiltration]: Downsampling row/col steps for 3D lidar must be >=1, ending nodelet.");
      ros::shutdown();
    }

    _sub_lidar3d                = nh.subscribe("lidar3d_in", 10, &PCLFiltration::lidar3dCallback, this, ros::TransportHints().tcpNoDelay());
    _pub_lidar3d                = nh.advertise<sensor_msgs::PointCloud2>("lidar3d_out", 10);
    _pub_lidar3d_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("lidar3d_over_max_range_out", 10);
    if (_lidar3d_groundremoval_use)
    {
      _pub_lidar3d_below_ground = nh.advertise<sensor_msgs::PointCloud2>("lidar3d_below_ground_out", 10);
      _pub_fitted_plane = nh.advertise<visualization_msgs::MarkerArray>("lidar3d_fitted_plane", 10);
    }
    if (_lidar3d_groundremoval_range_use)
      _pub_ground_point = nh.advertise<geometry_msgs::PointStamped>("lidar3d_ground_point_out", 10);
  }

  if (_depth_republish) {
    _sub_depth                = nh.subscribe("depth_in", 1, &PCLFiltration::depthCallback, this, ros::TransportHints().tcpNoDelay());
    _pub_depth                = nh.advertise<sensor_msgs::PointCloud2>("depth_out", 10);
    _pub_depth_over_max_range = nh.advertise<sensor_msgs::PointCloud2>("depth_over_max_range_out", 10);
  }

  if (_rplidar_republish) {
    _sub_rplidar = nh.subscribe("rplidar_in", 1, &PCLFiltration::rplidarCallback, this, ros::TransportHints().tcpNoDelay());
    _pub_rplidar = nh.advertise<sensor_msgs::LaserScan>("rplidar_out", 10);
  }

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh));
  /* reconfigure_server_->updateConfig(last_drs_config); */
  ReconfigureServer::CallbackType f = boost::bind(&PCLFiltration::callbackReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  NODELET_INFO_ONCE("[PCLFiltration] Nodelet initialized");

  is_initialized = true;
}
//}

/* callbackReconfigure() //{ */
void PCLFiltration::callbackReconfigure(Config &config, [[maybe_unused]] uint32_t level) {
  if (!is_initialized) {
    return;
  }
  NODELET_INFO("[PCLFiltration] Reconfigure callback.");

  _lidar3d_filter_intensity_en       = config.lidar3d_filter_intensity_en;
  _lidar3d_filter_intensity_thrd     = config.lidar3d_filter_intensity_thrd;
  _lidar3d_filter_intensity_range_sq = std::pow(config.lidar3d_filter_intensity_rng, 2);
}
//}

/* lidar3dCallback() //{ */
void PCLFiltration::lidar3dCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (!is_initialized || !_lidar3d_republish)
    return;

  if (msg->width % _lidar3d_col_step != 0 || msg->height % _lidar3d_row_step != 0) {
    NODELET_WARN(
        "[PCLFiltration] Step-based downsampling of 3D lidar data would create nondeterministic results. Data (w: %d, h: %d) with downsampling step (w: %d, "
        "h: %d) would leave some samples untouched. Skipping lidar frame.",
        msg->width, msg->height, _lidar3d_col_step, _lidar3d_row_step);
    return;
  }

  const bool is_ouster = hasField("range", msg) && hasField("ring", msg) && hasField("t", msg);
  if (is_ouster)
  {
    NODELET_INFO_ONCE("[PCLFiltration] Received first 3D LIDAR message. Point type: ouster_ros::Point.");
    PC_OS::Ptr cloud = boost::make_shared<PC_OS>();
    pcl::fromROSMsg(*msg, *cloud);
    process_msg(cloud);
  }
  else
  {
    NODELET_INFO_ONCE("[PCLFiltration] Received first 3D LIDAR message. Point type: pcl::PointXYZI.");
    PC_I::Ptr cloud = boost::make_shared<PC_I>();
    pcl::fromROSMsg(*msg, *cloud);
    process_msg(cloud);
  }
}

template <typename PC>
void PCLFiltration::process_msg(typename boost::shared_ptr<PC> pc_ptr)
{
  TicToc t;
  const size_t height_before = pc_ptr->height;
  const size_t width_before = pc_ptr->width;
  const size_t points_before = pc_ptr->size();

  if (_lidar3d_groundremoval_use)
  {
    const bool publish_removed = _pub_lidar3d_below_ground.getNumSubscribers() > 0;
    const typename PC::Ptr pcl_below_ground = removeBelowGround(pc_ptr, publish_removed);
    if (publish_removed)
      _pub_lidar3d_below_ground.publish(pcl_below_ground);
  }

  if (_lidar3d_rangeclip_use)
  {
    const bool publish_removed = _pub_lidar3d_over_max_range.getNumSubscribers() > 0;
    const typename PC::Ptr pcl_over_max_range = removeCloseAndFar(pc_ptr, publish_removed);
    if (publish_removed)
      _pub_lidar3d_over_max_range.publish(pcl_over_max_range);
  }

  if (_lidar3d_cropbox_use)
    cropBoxPointCloud(pc_ptr);

  _pub_lidar3d.publish(pc_ptr);

  if (_lidar3d_dynamic_row_selection_enabled)
  {
    _lidar3d_dynamic_row_selection_offset++;
    _lidar3d_dynamic_row_selection_offset %= _lidar3d_row_step;
  }

  const size_t height_after = pc_ptr->height;
  const size_t width_after = pc_ptr->width;
  const size_t points_after = pc_ptr->size();
  NODELET_INFO_THROTTLE(
      5.0, "[PCLFiltration] Processed 3D LIDAR data (run time: %0.1f ms; points before: %lu, after: %lu; dim before: (w: %lu, h: %lu), after: (w: %lu, h: %lu)).",
      t.toc(), points_before, points_after, width_before, height_before, width_after, height_after);
}
//}

/* depthCallback() //{ */
void PCLFiltration::depthCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  if (is_initialized && _depth_republish) {
    NODELET_INFO_ONCE("[PCLFiltration] Received first depth camera message.");
    TicToc t;

    unsigned int points_before = msg->height * msg->width;

    std::pair<PC::Ptr, PC::Ptr> clouds = removeCloseAndFarPointCloudXYZ(msg, _depth_pcl2_over_max_range, _depth_min_range_sq, _depth_max_range_sq);

    PC::Ptr pcl = clouds.first;

    // Voxel grid sampling
    if (_depth_voxel_resolution > 0.0f) {
      /* NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] \t - Applied voxel sampling filter (res: %0.2f m).", _depth_voxel_resolution); */
      pcl::VoxelGrid<pt_XYZ> vg;
      vg.setInputCloud(pcl);
      vg.setLeafSize(_depth_voxel_resolution, _depth_voxel_resolution, _depth_voxel_resolution);
      vg.filter(*pcl);
    }

    // Radius outlier filter
    if (_depth_use_radius_outlier_filter) {
      pcl::RadiusOutlierRemoval<pt_XYZ> outrem;
      outrem.setInputCloud(pcl);
      outrem.setRadiusSearch(_depth_radius_outlier_filter_radius);
      outrem.setMinNeighborsInRadius(_depth_radius_outlier_filter_neighbors);
      outrem.setKeepOrganized(true);
      outrem.filter(*pcl);
    }

    // Bilateral filter
    if (_depth_use_bilateral) {
      /* NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] \t - Applied fast bilateral OMP filter (sigma S: %0.2f, sigma R: %0.2f).", _depth_bilateral_sigma_S, */
      /*                       _depth_bilateral_sigma_R); */
      pcl::FastBilateralFilterOMP<pt_XYZ> fbf;
      fbf.setInputCloud(pcl);
      fbf.setSigmaS(_depth_bilateral_sigma_S);
      fbf.setSigmaR(_depth_bilateral_sigma_R);
      fbf.applyFilter(*pcl);
    }

    // Grid minimum
    if (_depth_minimum_grid_resolution > 0.0f) {
      /* NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] \t - Applied minimum grid filter (res: %0.2f m).", _depth_minimum_grid_resolution); */
      pcl::GridMinimum<pt_XYZ> gmf(_depth_minimum_grid_resolution);
      gmf.setInputCloud(pcl);
      gmf.filter(*pcl);
    }

    // Publish data
    publishCloud(_pub_depth, *pcl);

    // Publish data over max range
    if (_depth_pcl2_over_max_range) {
      publishCloud(_pub_depth_over_max_range, *clouds.second);
    }

    NODELET_INFO_THROTTLE(5.0, "[PCLFiltration] Processed depth camera data (run time: %0.1f ms; points before: %d, after: %ld).", t.toc(), points_before,
                          pcl->points.size());
  }
}
//}

/* rplidarCallback() //{ */
void PCLFiltration::rplidarCallback([[maybe_unused]] const sensor_msgs::LaserScan::ConstPtr msg) {
  if (is_initialized && _rplidar_republish) {
    NODELET_INFO_ONCE("[PCLFiltration] Received first RPLidar message.");
    NODELET_WARN_THROTTLE(2.0, "[PCLFiltration] rplidarCallback() not implemented!");
  }
}
//}

template <typename pt_t>
std::tuple<bool, std::size_t> getFieldOffset(const std::string& field_name)
{
  std::vector<pcl::PCLPointField> fields;
  const int range_idx = pcl::getFieldIndex<pt_t>(field_name, fields);
  if (range_idx == -1)
    return {false, 0};
  const std::size_t range_offset = fields.at(range_idx).offset;
  return {true, range_offset};
}

template <typename T, typename pt_t>
T getFieldValue(const pt_t& point, std::size_t field_offset)
{
  /* T val; */
  /* pcl::getFieldValue(point, field_offset, val); */
  const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*>(&(point));
  T field_value = 0;
  memcpy(&field_value, pt_data + field_offset, sizeof(T));
  return field_value;
}

/*//{ removeCloseAndFar() */
template <typename PC>
typename boost::shared_ptr<PC> PCLFiltration::removeCloseAndFar(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed)
{
  using pt_t = typename PC::PointType;

  // Prepare pointcloud of removed points
  typename PC::Ptr removed_pc = boost::make_shared<PC>();
  removed_pc->header = inout_pc->header;
  if (return_removed)
    removed_pc->resize(inout_pc->size());
  size_t removed_it = 0;

  // Attempt to get the range field name's index
  const auto [range_exists, range_offset] = getFieldOffset<pt_t>("range");
  if (range_exists)
    ROS_WARN_ONCE("[PCLFiltration] Unable to find field name \"range\" in point type, will be using calculated range.");
  else
    ROS_INFO_ONCE("[PCLFiltration] Found field name \"range\" in point type, will be using range from points.");

  for (auto& point : inout_pc->points)
  {
    bool invalid = false;

    // if the range field is available, use it
    if (range_exists)
    {
      // Get the range (in millimeters)
      const auto range = getFieldValue<uint32_t>(point, range_offset);
      invalid = range < _lidar3d_rangeclip_min_mm || range > _lidar3d_rangeclip_max_mm;
    }
    // otherwise, just calculate the range as the norm
    else
    {
      const vec3_t pt = point.getArray3fMap();
      const float range_sq = pt.squaredNorm();
      invalid = range_sq < _lidar3d_rangeclip_min_sq || range_sq > _lidar3d_rangeclip_max_sq;
    }

    if (invalid)
    {
      if (return_removed)
        removed_pc->at(removed_it++) = point;
      invalidatePoint(point);
    }
  }

  return removed_pc;
}
/*//}*/

/*//{ removeCloseAndFarAndLowIntensity() */
template <typename PC>
typename boost::shared_ptr<PC> PCLFiltration::removeCloseAndFarAndLowIntensity(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed)
{
  using pt_t = typename PC::PointType;

  // Prepare pointcloud of removed points
  typename PC::Ptr removed_pc = boost::make_shared<PC>();
  removed_pc->header = inout_pc->header;
  if (return_removed)
    removed_pc->resize(inout_pc->size());
  size_t removed_it = 0;

  // Attempt to get the intensity field name's index
  const auto [intensity_exists, intensity_offset] = getFieldOffset<pt_t>("intensity");
  if (intensity_exists)
  {
    ROS_WARN("[PCLFiltration] Unable to find field name \"intensity\" in point type.");
    return removeCloseAndFar(inout_pc, return_removed);
  }

  // Attempt to get the range field name's index
  const auto [range_exists, range_offset] = getFieldOffset<pt_t>("range");
  if (range_exists)
    ROS_WARN_ONCE("[PCLFiltration] Unable to find field name \"range\" in point type, will be using calculated range.");
  else
    ROS_INFO_ONCE("[PCLFiltration] Found field name \"range\" in point type, will be using range from points.");

  for (auto& point : inout_pc->points)
  {
    // Get the intensity
    const auto intensity = getFieldValue<float>(point, intensity_offset);

    bool invalid_range = false;
    bool invalid_intensity = false;
    // if the range field is available, use it
    if (range_exists) // nevermind this condition inside a loop - the branch predictor will optimize this out easily
    {
      const auto range = getFieldValue<uint32_t>(point, range_offset);
      invalid_range = range < _lidar3d_rangeclip_min_mm || range > _lidar3d_rangeclip_max_mm;
      invalid_intensity = intensity < _lidar3d_filter_intensity_thrd && range < _lidar3d_filter_intensity_range_mm;
    }
    // otherwise, just calculate the range as the norm
    else
    {
      const vec3_t pt = point.getArray3fMap();
      const float range_sq = pt.squaredNorm();
      invalid_range = range_sq < _lidar3d_rangeclip_min_sq || range_sq > _lidar3d_rangeclip_max_sq;
      invalid_intensity = intensity < _lidar3d_filter_intensity_thrd && range_sq < _lidar3d_filter_intensity_range_sq;
    }

    // check the removal condition
    if (invalid_range || invalid_intensity)
    {
      if (return_removed)
        removed_pc->at(removed_it++) = point;
      invalidatePoint(point);
    }
  }

  return removed_pc;
}
/*//}*/

/*//{ removeLowIntensity() */
template <typename PC>
typename boost::shared_ptr<PC> PCLFiltration::removeLowIntensity(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed)
{
  using pt_t = typename PC::PointType;

  // Prepare pointcloud of removed points
  typename PC::Ptr removed_pc = boost::make_shared<PC>();
  removed_pc->header = inout_pc->header;
  if (return_removed)
    removed_pc->resize(inout_pc->size());
  size_t removed_it = 0;

  // Attempt to get the intensity field name's index
  const auto [intensity_exists, intensity_offset] = getFieldOffset<pt_t>("intensity");
  if (intensity_exists)
  {
    ROS_WARN("[PCLFiltration] Unable to find field name \"intensity\" in point type.");
    return removed_pc;
  }

  // Attempt to get the range field name's index
  const auto [range_exists, range_offset] = getFieldOffset<pt_t>("range");
  if (range_exists)
    ROS_WARN_ONCE("[PCLFiltration] Unable to find field name \"range\" in point type, will be using calculated range.");
  else
    ROS_INFO_ONCE("[PCLFiltration] Found field name \"range\" in point type, will be using range from points.");

  for (auto& point : inout_pc->points)
  {
    const auto intensity = getFieldValue<float>(point, intensity_offset);
    // check the removal condition
    if (intensity < _lidar3d_filter_intensity_thrd)
    {
      bool invalid = false;

      // if the range field is available, use it
      if (range_exists)
      {
        // Get the range (in millimeters)
        const auto range = getFieldValue<uint32_t>(point, range_offset);
        invalid = range < _lidar3d_filter_intensity_range_mm;
      }
      // otherwise, just calculate the range as the norm
      else
      {
        const vec3_t pt = point.getArray3fMap();
        const float range_sq = pt.squaredNorm();
        invalid = range_sq < _lidar3d_filter_intensity_range_sq;
      }

      if (invalid)
      {
        if (return_removed)
          removed_pc->at(removed_it++) = point;
        invalidatePoint(point);
      }
    }
  }

  return removed_pc;
}

/*//}*/

/* removeBelowGround() //{ */

template <typename PC>
typename boost::shared_ptr<PC> PCLFiltration::removeBelowGround(typename boost::shared_ptr<PC>& inout_pc, const bool return_removed)
{
  using pt_t = typename PC::PointType;
  vec3_t ground_point(0,0,0);
  vec3_t ground_normal(0,0,1);
  const auto orig_size = inout_pc->size();
  typename PC::Ptr removed_pc_ptr = boost::make_shared<PC>();
  removed_pc_ptr->header = inout_pc->header;

  // try to deduce the ground point from the latest rangefinder measurement
  bool range_meas_used = false;
  if (_lidar3d_groundremoval_range_use && _sh_range.hasMsg())
  {
    const sensor_msgs::RangeConstPtr range_msg = _sh_range.getMsg();
    if (range_msg->range > range_msg->min_range && range_msg->range < range_msg->max_range)
    {
      const vec3_t range_vec(range_msg->range, 0,0);
      const auto tf_opt = _transformer.getTransform(range_msg->header.frame_id, inout_pc->header.frame_id, range_msg->header.stamp);
      if (tf_opt.has_value())
      {
        ground_point = tf_opt->getTransformEigen().template cast<float>()*range_vec;
        range_meas_used = true;
      }
      else
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[PCLFiltration]: Could not get transformation from " << range_msg->header.frame_id << " to " << inout_pc->header.frame_id << ", cannot use range measurement for ground plane point estimation.");
      }
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[PCLFiltration]: Range measurement is out of bounds, not using it for ground plane point estimation (" << range_msg->range << " not in (" << range_msg->min_range << ", " << range_msg->max_range << "))");
    }
  }

  // prepare a new pointcloud that will contain the filtered points for ground fitting
  typename PC::Ptr pc_filtered = boost::make_shared<PC>();
  // simplify the fitting by filtering the input cloud a bit
  pcl::VoxelGrid<pt_t> vg;
  vg.setInputCloud(inout_pc);
  vg.setLeafSize(0.5, 0.5, 0.5);
  vg.filter(*pc_filtered);

  // try to estimate the ground normal from the static frame
  ros::Time stamp;
  pcl_conversions::fromPCL(inout_pc->header.stamp, stamp);
  const auto tf_opt = _transformer.getTransform(_lidar3d_groundremoval_frame_id, inout_pc->header.frame_id, stamp);
  if (tf_opt.has_value())
  {
    const Eigen::Affine3f tf = tf_opt->getTransformEigen().template cast<float>();
    ground_normal = tf.rotation()*vec3_t(0, 0, 1);
    // if the range measurement is not used for estimation of the ground point, assume that the static frame starts at ground level
    if (!range_meas_used)
      ground_point = tf*vec3_t(0, 0, 0);

    // crop out points above a certain height to reduce the number of non-ground-plane points
    const float plane_d = -ground_normal.dot(ground_point)-_lidar3d_groundremoval_max_height;
    const vec4_t plane_params = -vec4_t(ground_normal.x(), ground_normal.y(), ground_normal.z(), plane_d);
    pcl::IndicesPtr inds_filtered = boost::make_shared<pcl::Indices>();
    pcl::PlaneClipper3D<pt_t> pclip(plane_params);
    pclip.clipPointCloud3D(*pc_filtered, *inds_filtered);
    pcl::ExtractIndices<pt_t> ei;
    ei.setIndices(inds_filtered);
    ei.filterDirectly(pc_filtered);
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "[PCLFiltration]: Could not get transformation from " << _lidar3d_groundremoval_frame_id << " to " << inout_pc->header.frame_id << ", ground plane may be imprecise.");
  }

  // prepare a SAC plane model with an angular constraint according to the estimated plane normal
  typename pcl::SampleConsensusModelPerpendicularPlane<pt_t>::Ptr model = boost::make_shared<pcl::SampleConsensusModelPerpendicularPlane<pt_t>>(pc_filtered, true);
  model->setAxis(ground_normal);
  model->setEpsAngle(_lidar3d_groundremoval_ransac_max_angle_diff);

  // actually fit the plane
  pcl::RandomSampleConsensus<pt_t> ransac(model);
  ransac.setDistanceThreshold(_lidar3d_groundremoval_ransac_max_inlier_dist);
  if (!ransac.computeModel())
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[PCLFiltration]: Could not fit a ground-plane model! Skipping ground removal.");
    return removed_pc_ptr;
  }

  // retreive the fitted model
  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);
  // orient the retrieved normal to point upwards in the static frame
  if (coeffs.block<3, 1>(0, 0).dot(ground_normal) < 0.0f)
    coeffs = -coeffs;
  // just some helper variables
  const vec3_t fit_n = coeffs.block<3, 1>(0, 0);
  const float fit_d = coeffs(3);

  // check if the assumed ground point is an inlier of the fitted plane
  const float ground_pt_dist = std::abs(fit_n.dot(ground_point) + fit_d);
  if (ground_pt_dist >  _lidar3d_groundremoval_offset)
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[PCLFiltration]: The RANSAC-fitted ground-plane model [" << coeffs.transpose() << "] is too far from the measured ground (" << ground_pt_dist << "m > " << _lidar3d_groundremoval_ransac_max_inlier_dist << "m)! Skipping ground removal.");
    return removed_pc_ptr;
  }

  std_msgs::Header header;
  pcl_conversions::fromPCL(inout_pc->header, header);

  geometry_msgs::PointStamped pt_msg;
  pt_msg.header = header;
  pt_msg.point.x = ground_point.x();
  pt_msg.point.y = ground_point.y();
  pt_msg.point.z = ground_point.z();
  _pub_ground_point.publish(pt_msg);

  visualization_msgs::MarkerArray plane_msg = plane_visualization(fit_n, fit_d, header);
  _pub_fitted_plane.publish(plane_msg);

  // get indices of points above the plane
  const vec4_t plane_params(fit_n.x(), fit_n.y(), fit_n.z(), fit_d-_lidar3d_groundremoval_offset);
  pcl::IndicesPtr inds_filtered = boost::make_shared<pcl::Indices>();
  pcl::PlaneClipper3D<pt_t> pclip(plane_params);
  pclip.clipPointCloud3D(*inout_pc, *inds_filtered);

  // extract the points
  pcl::ExtractIndices<pt_t> ei;
  ei.setIndices(inds_filtered);
  ei.setInputCloud(inout_pc);
  if (return_removed)
  {
    ei.setNegative(true);
    ei.filter(*removed_pc_ptr);
    ei.setNegative(false);
  }
  ei.filter(*inout_pc);

  return removed_pc_ptr;
}

//}

/* cropBoxPointCloud() //{ */

template <typename PC>
void PCLFiltration::cropBoxPointCloud(boost::shared_ptr<PC>& inout_pc)
{
  pcl::CropBox<typename PC::PointType> cb;

  if (!_lidar3d_cropbox_frame_id.empty())
  {
    ros::Time stamp;
    pcl_conversions::fromPCL(inout_pc->header.stamp, stamp);
    const auto tf_opt = _transformer.getTransform(inout_pc->header.frame_id, _lidar3d_cropbox_frame_id, stamp);
    if (tf_opt.has_value())
    {
      const Eigen::Affine3d tf = tf_opt->getTransformEigen();
      cb.setTransform(tf.cast<float>());
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[PCLFiltration]: Could not find pointcloud transformation (from \"" << inout_pc->header.frame_id << "\" to \"" << _lidar3d_cropbox_frame_id << "\") ! Not applying CropBox filter.");
      return;
    }
  }

  cb.setNegative(false);
  cb.setMin(_lidar3d_cropbox_min);
  cb.setMax(_lidar3d_cropbox_max);
  cb.setInputCloud(inout_pc);
  cb.filter(*inout_pc);
}

//}

/*//{ removeCloseAndFarPointCloudXYZ() */
std::pair<PC::Ptr, PC::Ptr> PCLFiltration::removeCloseAndFarPointCloudXYZ(const sensor_msgs::PointCloud2::ConstPtr &msg, const bool &ret_cloud_over_max_range,
                                                                          const float &min_range_sq, const float &max_range_sq) {

  // Convert to pcl object
  PC::Ptr cloud                = boost::make_shared<PC>();
  PC::Ptr cloud_over_max_range = boost::make_shared<PC>();
  pcl::fromROSMsg(*msg, *cloud);

  unsigned int j          = 0;
  unsigned int k          = 0;
  unsigned int cloud_size = cloud->points.size();

  if (ret_cloud_over_max_range) {
    cloud_over_max_range->header = cloud->header;
    cloud_over_max_range->points.resize(cloud_size);
  }

  for (unsigned int i = 0; i < cloud_size; i++) {

    float range_sq =
        cloud->points.at(i).x * cloud->points.at(i).x + cloud->points.at(i).y * cloud->points.at(i).y + cloud->points.at(i).z * cloud->points.at(i).z;

    if (range_sq < min_range_sq) {
      continue;

    } else if (range_sq <= max_range_sq) {

      cloud->points.at(j++) = cloud->points.at(i);

    } else if (ret_cloud_over_max_range && cloud->points.at(i).getArray3fMap().allFinite()) {

      cloud_over_max_range->points.at(k++) = cloud->points.at(i);
    }
  }

  // Resize both clouds
  if (j != cloud_size) {
    cloud->points.resize(j);
  }
  cloud->height   = 1;
  cloud->width    = static_cast<uint32_t>(j);
  cloud->is_dense = false;


  if (ret_cloud_over_max_range) {
    if (k != cloud_size) {
      cloud_over_max_range->points.resize(k);
    }
    cloud_over_max_range->header   = cloud->header;
    cloud_over_max_range->height   = 1;
    cloud_over_max_range->width    = static_cast<uint32_t>(k);
    cloud_over_max_range->is_dense = false;
  }

  return std::make_pair(cloud, cloud_over_max_range);
}
/*//}*/

/*//{ invalidatePoint() */
template <typename pt_t>
void PCLFiltration::invalidatePoint(pt_t &point, const float inv_value)
{
  point.x = inv_value;
  point.y = inv_value;
  point.z = inv_value;
}
/*//}*/

/*//{ publishCloud() */
template <typename T>
void PCLFiltration::publishCloud(const ros::Publisher &pub, const pcl::PointCloud<T> cloud) {
  if (pub.getNumSubscribers() > 0) {
    try {
      pub.publish(cloud);
    }
    catch (...) {
      NODELET_ERROR("[PCLFiltration]: Exception caught during publishing on topic: %s", pub.getTopic().c_str());
    }
  }
}
/*//}*/

  /* plane_visualization //{ */
  visualization_msgs::MarkerArray PCLFiltration::plane_visualization(const vec3_t& plane_normal, float plane_d, const std_msgs::Header& header)
  {
    visualization_msgs::MarkerArray ret;

    const quat_t quat = quat_t::FromTwoVectors(vec3_t::UnitZ(), plane_normal);
    const vec3_t pos = plane_normal * (-plane_d) / plane_normal.norm();

    const double size = 40.0;
    geometry_msgs::Point ptA;
    ptA.x = size;
    ptA.y = size;
    ptA.z = 0;
    geometry_msgs::Point ptB;
    ptB.x = -size;
    ptB.y = size;
    ptB.z = 0;
    geometry_msgs::Point ptC;
    ptC.x = -size;
    ptC.y = -size;
    ptC.z = 0;
    geometry_msgs::Point ptD;
    ptD.x = size;
    ptD.y = -size;
    ptD.z = 0;

    /* borders marker //{ */
    {
      visualization_msgs::Marker borders_marker;
      borders_marker.header = header;

      borders_marker.ns = "borders";
      borders_marker.id = 0;
      borders_marker.type = visualization_msgs::Marker::LINE_LIST;
      borders_marker.action = visualization_msgs::Marker::ADD;

      borders_marker.pose.position.x = pos.x();
      borders_marker.pose.position.y = pos.y();
      borders_marker.pose.position.z = pos.z();

      borders_marker.pose.orientation.x = quat.x();
      borders_marker.pose.orientation.y = quat.y();
      borders_marker.pose.orientation.z = quat.z();
      borders_marker.pose.orientation.w = quat.w();

      borders_marker.scale.x = 0.1;

      borders_marker.color.a = 0.5;  // Don't forget to set the alpha!
      borders_marker.color.r = 0.0;
      borders_marker.color.g = 0.0;
      borders_marker.color.b = 1.0;

      borders_marker.points.push_back(ptA);
      borders_marker.points.push_back(ptB);

      borders_marker.points.push_back(ptB);
      borders_marker.points.push_back(ptC);

      borders_marker.points.push_back(ptC);
      borders_marker.points.push_back(ptD);

      borders_marker.points.push_back(ptD);
      borders_marker.points.push_back(ptA);

      ret.markers.push_back(borders_marker);
    }
    //}

    /* plane marker //{ */
    {
      visualization_msgs::Marker plane_marker;
      plane_marker.header = header;

      plane_marker.ns = "plane";
      plane_marker.id = 1;
      plane_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      plane_marker.action = visualization_msgs::Marker::ADD;

      plane_marker.pose.position.x = pos.x();
      plane_marker.pose.position.y = pos.y();
      plane_marker.pose.position.z = pos.z();

      plane_marker.pose.orientation.x = quat.x();
      plane_marker.pose.orientation.y = quat.y();
      plane_marker.pose.orientation.z = quat.z();
      plane_marker.pose.orientation.w = quat.w();

      plane_marker.scale.x = 1;
      plane_marker.scale.y = 1;
      plane_marker.scale.z = 1;

      plane_marker.color.a = 0.2;  // Don't forget to set the alpha!
      plane_marker.color.r = 0.0;
      plane_marker.color.g = 0.0;
      plane_marker.color.b = 1.0;

      // triangle ABC
      plane_marker.points.push_back(ptA);
      plane_marker.points.push_back(ptB);
      plane_marker.points.push_back(ptC);

      // triangle ACD
      plane_marker.points.push_back(ptA);
      plane_marker.points.push_back(ptC);
      plane_marker.points.push_back(ptD);
      ret.markers.push_back(plane_marker);
    }
    //}

    /* normal marker //{ */
    {
      visualization_msgs::Marker normal_marker;
      normal_marker.header = header;

      normal_marker.ns = "normal";
      normal_marker.id = 2;
      normal_marker.type = visualization_msgs::Marker::ARROW;
      normal_marker.action = visualization_msgs::Marker::ADD;

      normal_marker.pose.position.x = pos.x();
      normal_marker.pose.position.y = pos.y();
      normal_marker.pose.position.z = pos.z();
      normal_marker.pose.orientation.w = 1.0;

      normal_marker.scale.x = 0.05;
      normal_marker.scale.y = 0.05;
      normal_marker.scale.z = 0.3;

      normal_marker.color.a = 0.5;  // Don't forget to set the alpha!
      normal_marker.color.r = 0.0;
      normal_marker.color.g = 0.0;
      normal_marker.color.b = 1.0;

      // direction
      geometry_msgs::Point pt;
      normal_marker.points.push_back(pt);
      pt.x = plane_normal.x();
      pt.y = plane_normal.y();
      pt.z = plane_normal.z();
      normal_marker.points.push_back(pt);
      ret.markers.push_back(normal_marker);
    }
    //}

    return ret;
  }
  //}

}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::PCLFiltration, nodelet::Nodelet);
