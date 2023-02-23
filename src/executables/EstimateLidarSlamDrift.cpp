#include <mrs_pcl_tools/support.h>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <tuple>
#include <optional>
#include <iostream>
#include <sys/stat.h>
#include <fstream>
#include <iomanip>

/*//{ struct ARGUMENTS */
struct ARGUMENTS
{
  bool initialized = false;

  std::string              path_rosbag;
  std::string              topic_cloud;
  std::string              mapping_origin;
  std::string              pcd_target;
  std::string              txt_trajectory_odom;
  std::string              txt_trajectory_gt;
  float                    traj_step_dist              = 0.0f;
  double                   traj_step_time              = 0.0f;
  double                   start_time                  = 0.0;
  double                   end_time                    = std::numeric_limits<double>::max();
  double                   cloud_buffer_sec            = 5.0;
  double                   post_registration_delay_sec = 0.0;
  double                   min_points_dist_from_origin = 0.0;
  geometry_msgs::Transform tf_target_in_map_origin;
  bool                     invert_tf_target_in_map_origin = false;

  void print() {

    if (initialized) {
      ROS_INFO("Target pcd: %s", pcd_target.c_str());
      ROS_INFO("Rosbag path: %s", path_rosbag.c_str());
      ROS_INFO("Topic cloud: %s", topic_cloud.c_str());
      ROS_INFO("Mapping origin: %s", mapping_origin.c_str());
      ROS_INFO("Transform map->target: xyz (%0.1f, %0.1f, %0.1f), xyzw (%0.1f, %0.1f, %0.1f, %0.1f), inverted: %s", tf_target_in_map_origin.translation.x,
               tf_target_in_map_origin.translation.y, tf_target_in_map_origin.translation.z, tf_target_in_map_origin.rotation.x,
               tf_target_in_map_origin.rotation.y, tf_target_in_map_origin.rotation.z, tf_target_in_map_origin.rotation.w,
               invert_tf_target_in_map_origin ? "true" : "false");
      ROS_INFO("Trajectory odom out: %s", txt_trajectory_odom.c_str());
      ROS_INFO("Trajectory gt out: %s", txt_trajectory_gt.c_str());
      ROS_INFO("Trajectory distance step: %0.2f; time step: %0.2f", traj_step_dist, traj_step_time);
      ROS_INFO("Point cloud buffer: %0.2f s", cloud_buffer_sec);
      ROS_INFO("Post-registration delay: %0.2f s", post_registration_delay_sec);
      ROS_INFO("Minimum distance of points from their origin: %0.2f m", min_points_dist_from_origin);
      ROS_INFO("Start time offset: %0.2f", start_time);
      ROS_INFO("End time offset: %0.2f", end_time);
    }
  }
};
/*//}*/

/*//{ struct TRAJECTORY_POINT */
struct TRAJECTORY_POINT
{
  Eigen::Matrix4f transformation;
  Eigen::Matrix4f transformed_pose;
  Eigen::Matrix4f untransformed_pose;
  double          sample_time;
};
/*//}*/

/*//{ struct BUFFER_QUEUE */
struct BUFFER_QUEUE
{
private:
  std::vector<std::pair<ros::Time, PC::Ptr>> clouds;

public:
  ros::Duration buffer_size_secs;
  std::string   cloud_frame;

  void insertCloud(const ros::Time &stamp, const PC::Ptr &cloud) {

    const bool buffer = buffer_size_secs.toSec() > 0.0;

    if (!buffer) {

      clouds.clear();

    } else if (!clouds.empty()) {

      const ros::Time stamp_latest = clouds.back().first;

      // Check for sequentiality of data
      if (stamp <= stamp_latest) {
        ROS_ERROR("Trying to insert cloud with old time stamp to buffer queue. The data should be in correct order, skipping.");
        return;
      }

      // Remove old clouds
      const ros::Time stamp_min = stamp - buffer_size_secs;
      clouds.erase(std::remove_if(clouds.begin(), clouds.end(), [&stamp_min](const auto &pair_time_cloud) { return pair_time_cloud.first < stamp_min; }),
                   clouds.end());
    }

    // Insert new cloud
    clouds.push_back({stamp, cloud});
  }

  PC::Ptr getCloudsAsOne() {
    const PC::Ptr cloud_merged = boost::make_shared<PC>();

    for (const auto &pair_time_cloud : clouds) {
      *cloud_merged += *pair_time_cloud.second;
    }

    pcl_conversions::toPCL(clouds.back().first, cloud_merged->header.stamp);
    cloud_merged->header.frame_id = cloud_frame;

    return cloud_merged;
  }
};
/*//}*/

/*//{ printHelp() */
void printHelp() {
  ROS_ERROR(
      "Ground truth trajectory will be estimated from given drifting trajectory using lidar data, dynamic transformation map->lidar, and ground truth map.");
  ROS_ERROR("Reads robot trajectory and map from rosbag.");
  ROS_ERROR("Transformation of target cloud to the mapping origin has to be provided.");

  ROS_ERROR("Usage:");
  ROS_ERROR(
      "   rosrun mrs_pcl_tools estimate_lidar_slam_drift rosbag.bag topic_cloud mapping_origin tf_map_in_target.mat target.pcd trajectory_odom_out.txt "
      "trajectory_gt_out.txt [...]");

  ROS_ERROR("Arguments:");
  ROS_ERROR(" rosbag.bag:              rosbag containing sensor data (point cloud) and /tf topic with mapping_origin->cloud_origin transformation");
  ROS_ERROR(" topic_cloud:             point cloud topic (supported types: sensor_msgs/PointCloud2)");
  ROS_ERROR(" mapping_origin:          mapping origin (string)");
  ROS_ERROR(
      " tf_map_in_target.mat:    transformation from the target cloud to the mapping origin (expected format: 4x4 matrix (4 rows, cols separated by space))");
  ROS_ERROR(" target.pcd:              target point cloud (ground truth)");
  ROS_ERROR(" trajectory_odom_out.txt: robot estimated trajectory (from the rosbag)");
  ROS_ERROR(" trajectory_gt_out.txt:   corrected (real) trajectory of the robot");

  ROS_ERROR("Optional arguments:");
  ROS_ERROR(" --traj-step-dist:               sampling of trajectory by distance, used if greater than 0.0 (default: 0.0 m)");
  ROS_ERROR(" --traj-step-time:               sampling of trajectory by time, used if greater than 0.0 (default: 0.0 s)");
  ROS_ERROR(" --cloud-buffer:                 buffer length of cloud data in seconds (default: 5.0 s)");
  ROS_ERROR(" --start-time:                   start time offset of the rosbag (default: 0.0 s)");
  ROS_ERROR(" --end-time:                     end time offset of the rosbag (default: full duration)");
  ROS_ERROR(" --invert-transform:             if the initial map in target transformation should be inverted (default: false)");
  ROS_ERROR(" --post-registration-delay:      delay before processing next scan sample (default: 0.0 s)");
  ROS_ERROR(" --min-points-dist-from-origin:  minimal distance of points from their origin (default: 0.0 m)");
}
/*//}*/

/*//{ eigenMatrixToPoseMsg() */
geometry_msgs::Pose eigenMatrixToPoseMsg(const Eigen::Matrix4f &mat) {
  geometry_msgs::Pose msg;
  msg.position.x = mat(0, 3);
  msg.position.y = mat(1, 3);
  msg.position.z = mat(2, 3);

  const mrs_lib::AttitudeConverter atti = mrs_lib::AttitudeConverter(mat.block<3, 3>(0, 0).cast<double>());
  msg.orientation                       = atti;

  return msg;
}
/*//}*/

/*//{ eigenMatrixToTransformMsg() */
geometry_msgs::Transform eigenMatrixToTransformMsg(const Eigen::Matrix4f &mat) {

  geometry_msgs::Transform transform;

  transform.translation.x = mat(0, 3);
  transform.translation.y = mat(1, 3);
  transform.translation.z = mat(2, 3);

  const mrs_lib::AttitudeConverter atti            = mrs_lib::AttitudeConverter(mat.block<3, 3>(0, 0).cast<double>());
  const Eigen::Quaterniond         atti_normalized = Eigen::Quaterniond(atti).normalized();
  transform.rotation.x                             = atti_normalized.x();
  transform.rotation.y                             = atti_normalized.y();
  transform.rotation.z                             = atti_normalized.z();
  transform.rotation.w                             = atti_normalized.w();

  return transform;
}
/*//}*/

/*//{ loadStaticTransformFromFile() */
std::optional<Eigen::Matrix4d> loadStaticTransformFromFile(const std::string &filepath) {

  ROS_INFO("Loading matrix from: %s", filepath.c_str());

  struct stat buffer;
  if (stat(filepath.c_str(), &buffer) == 0) {

    std::ifstream infile(filepath);
    std::string   line;

    unsigned int l = 0;

    Eigen::Matrix4d mat;
    unsigned int    row = 0;

    while (std::getline(infile, line) && row < 4) {

      if (line.rfind("#", 0) == 0) {
        continue;
      }

      std::istringstream  iss(line);
      std::vector<double> line_numbers;
      std::copy(std::istream_iterator<double>(iss), std::istream_iterator<double>(), std::back_inserter(line_numbers));

      if (line_numbers.size() != 4) {
        ROS_WARN("[Loading matrix] Skipping line %d (size: %ld) as it does not contain data in the expected format of matrix row.", l, line_numbers.size());
      } else {

        mat(row, 0) = line_numbers[0];
        mat(row, 1) = line_numbers[1];
        mat(row, 2) = line_numbers[2];
        mat(row, 3) = line_numbers[3];

        row++;
      }

      l++;
    }

    if (row == 4) {

      // Normalize rotation
      const mrs_lib::AttitudeConverter atti = mrs_lib::AttitudeConverter(mat.block<3, 3>(0, 0));
      mat.block<3, 3>(0, 0)                 = Eigen::Quaterniond(atti).normalized().toRotationMatrix();

      return mat;
    }

    ROS_ERROR("[Loading matrix] Did not find 4 rows in the file.");

  } else {
    ROS_ERROR("[Loading matrix] File path (%s) does not exist.", filepath.c_str());
  }

  return std::nullopt;
}
/*//}*/

/*//{ parseArguments() */
bool parseArguments(int argc, char **argv, ARGUMENTS &args) {

  if (argc < 8) {
    printHelp();
    return false;
  }

  args.path_rosbag         = argv[1];
  args.topic_cloud         = argv[2];
  args.mapping_origin      = argv[3];
  args.pcd_target          = argv[5];
  args.txt_trajectory_odom = argv[6];
  args.txt_trajectory_gt   = argv[7];

  const auto tf = loadStaticTransformFromFile(argv[4]);
  if (!tf) {
    std::cerr << "Could not load transform of target map in the mapping origin. Ending." << std::endl;
    return false;
  }

  for (unsigned int i = 8; i < argc;) {
    int i_inc = 2;

    const auto option = std::string(argv[i]);
    if (option == "--traj-step-dist") {
      args.traj_step_dist = float(std::atof(argv[i + 1]));
    } else if (option == "--traj-step-time") {
      args.traj_step_time = std::atof(argv[i + 1]);
    } else if (option == "--start-time") {
      args.start_time = std::atof(argv[i + 1]);
    } else if (option == "--end-time") {
      args.end_time = std::atof(argv[i + 1]);
    } else if (option == "--cloud-buffer") {
      args.cloud_buffer_sec = std::atof(argv[i + 1]);
    } else if (option == "--post-registration-delay") {
      args.post_registration_delay_sec = std::atof(argv[i + 1]);
    } else if (option == "--min-points-dist-from-origin") {
      args.min_points_dist_from_origin = std::atof(argv[i + 1]);
    } else if (option == "--invert-transform") {
      args.invert_tf_target_in_map_origin = true;
      i_inc                               = 1;
    } else {
      std::cerr << "Unknown option: " << option.c_str() << ". Ending." << std::endl;
      return false;
    }

    i += i_inc;
  }

  const Eigen::Matrix4d tf_target_in_map_origin = tf.value();
  if (args.invert_tf_target_in_map_origin) {

    const double det = tf_target_in_map_origin.determinant();
    if (det == 0.0) {
      ROS_ERROR("Input matrix is not invertible!");
      ros::shutdown();
      return false;
    }

    const Eigen::Matrix4d tf_target_in_map_origin_inverted = tf_target_in_map_origin.inverse();
    mrs_pcl_tools::printEigenMatrix(tf_target_in_map_origin_inverted.cast<float>(), "Target in map (result of inversion):");

    args.tf_target_in_map_origin = eigenMatrixToTransformMsg(tf_target_in_map_origin_inverted.cast<float>());

  } else {

    mrs_pcl_tools::printEigenMatrix(tf_target_in_map_origin.cast<float>(), "Target in map:");
    args.tf_target_in_map_origin = eigenMatrixToTransformMsg(tf_target_in_map_origin.cast<float>());
  }

  args.initialized = true;

  return true;
}
/*//}*/

/* -------------------- Global variables -------------------- */

const std::string GLOBAL_FRAME = "global_origin";

pcl::IterativeClosestPoint<pt_XYZ, pt_XYZ> _icp;
pcl::CropBox<pt_XYZ>                       _filter_box;

ros::Publisher _pub_pose_odom;
ros::Publisher _pub_pose_gt;
ros::Publisher _pub_pc_source_local;
ros::Publisher _pub_pc_target_global;
ros::Publisher _pub_pc_target_local;
ros::Publisher _pub_pc_aligned_local;
ros::Publisher _pub_path_untransformed;
ros::Publisher _pub_path_transformed;
ros::Publisher _pub_paths_correlation;

std::unique_ptr<tf2_ros::Buffer>            tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;

ros::Time rosbag_start_time;
ros::Time rosbag_end_time;


/* -------------------- Functions -------------------- */

/*//{ saveTrajectory() */
void saveTrajectory(const std::string &filepath, const std::vector<TRAJECTORY_POINT> &data, const bool transformed_data) {

  ROS_INFO("Saving trajectory to: %s", filepath.c_str());

  const unsigned int precision = 5;

  std::ofstream outfile(filepath);

  if (outfile.is_open()) {

    outfile << "# timestamp x y z qx qy qz qw" << std::endl;

    for (const auto &dato : data) {

      const Eigen::Matrix4f &pose = transformed_data ? dato.transformed_pose : dato.untransformed_pose;

      const mrs_lib::AttitudeConverter atti = mrs_lib::AttitudeConverter(pose.block<3, 3>(0, 0).cast<double>());
      const geometry_msgs::Quaternion  quat = atti;

      outfile << std::fixed << std::setprecision(precision) << dato.sample_time << " ";
      outfile << std::fixed << std::setprecision(precision) << pose(0, 3) << " ";
      outfile << std::fixed << std::setprecision(precision) << pose(1, 3) << " ";
      outfile << std::fixed << std::setprecision(precision) << pose(2, 3) << " ";
      outfile << std::fixed << std::setprecision(precision) << quat.x << " ";
      outfile << std::fixed << std::setprecision(precision) << quat.y << " ";
      outfile << std::fixed << std::setprecision(precision) << quat.z << " ";
      outfile << std::fixed << std::setprecision(precision) << quat.w << std::endl;
    }

  } else {
    ROS_ERROR("Could not open output trajectory file: %s", filepath.c_str());
  }
}
/*//}*/

/*//{ cropCloud() */
PC::Ptr cropCloud(const PC::Ptr &cloud, const Eigen::Vector3f &box_center, const float &xy_crop_dist, const float &z_crop_dist) {
  PC::Ptr cloud_cropped = boost::make_shared<PC>();

  _filter_box.setMin(Eigen::Vector4f(box_center.x() - xy_crop_dist, box_center.y() - xy_crop_dist, box_center.z() - z_crop_dist, 1.0));
  _filter_box.setMax(Eigen::Vector4f(box_center.x() + xy_crop_dist, box_center.y() + xy_crop_dist, box_center.z() + z_crop_dist, 1.0));

  _filter_box.setInputCloud(cloud);
  _filter_box.filter(*cloud_cropped);

  return cloud_cropped;
}
/*//}*/

/*//{ registerClouds() */
std::optional<std::tuple<Eigen::Matrix4f, PC::Ptr>> registerClouds(const PC::Ptr &source, const PC::Ptr &target, const Eigen::Matrix4f &initial_guess) {

  PC::Ptr aligned = boost::make_shared<PC>();

  _icp.setInputSource(source);
  _icp.setInputTarget(target);

  _icp.align(*aligned, initial_guess);

  if (!_icp.hasConverged()) {
    ROS_WARN("ICP has not converged, change your parameters.");
    return std::nullopt;
  }

  return std::make_tuple(_icp.getFinalTransformation(), aligned);
}
/*//}*/

/*//{ clipCloud() */
PC::Ptr clipCloud(const PC::Ptr &cloud_in, const double min_dist) {
  const PC::Ptr cloud_out = boost::make_shared<PC>();

  // Copy points in given range
  for (const auto &p : cloud_in->points) {
    if (std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) > min_dist) {
      cloud_out->points.push_back(p);
    }
  }

  cloud_out->header   = cloud_in->header;
  cloud_out->height   = 1;
  cloud_out->width    = cloud_out->points.size();
  cloud_out->is_dense = true;
  return cloud_out;
}
/*//}*/

/*//{ estimateGroundTruthTrajectoryFromRosbag() */
std::vector<TRAJECTORY_POINT> estimateGroundTruthTrajectoryFromRosbag(const rosbag::Bag &bag, const PC::Ptr &pc_target, const ARGUMENTS &args) {

  std::vector<TRAJECTORY_POINT> drift_data;

  nav_msgs::Path::Ptr path_transformed   = boost::make_shared<nav_msgs::Path>();
  nav_msgs::Path::Ptr path_untransformed = boost::make_shared<nav_msgs::Path>();
  path_transformed->header.frame_id      = GLOBAL_FRAME;
  path_untransformed->header.frame_id    = GLOBAL_FRAME;

  visualization_msgs::MarkerArray::Ptr path_correlation = boost::make_shared<visualization_msgs::MarkerArray>();
  visualization_msgs::Marker           marker;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.header.frame_id    = GLOBAL_FRAME;
  marker.ns                 = "correlation";
  marker.scale.x            = 0.05;
  marker.type               = visualization_msgs::Marker::LINE_LIST;
  marker.color.a            = 1.0;
  marker.color.g            = 1.0;
  marker.pose.orientation.w = 1.0;
  path_correlation->markers = {marker};

  // Preset sample selection variables
  ros::Time       stamp_prev;
  Eigen::Vector3f position_prev;
  Eigen::Matrix4f T_prev;
  unsigned int    sample_count      = 0;
  float           trajectory_length = 0.0f;

  // Preset global objects parameters
  const float resolution = 0.2f;
  _icp.setMaxCorrespondenceDistance(15.0);
  _icp.setMaximumIterations(5000);
  _icp.setTransformationEpsilon(0.01);
  _icp.setEuclideanFitnessEpsilon(0.01);
  _icp.setRANSACIterations(1000);
  _icp.setRANSACOutlierRejectionThreshold(0.8);
  _icp.setUseReciprocalCorrespondences(false);

  // Prepare target cloud
  const PC::Ptr cloud_target = mrs_pcl_tools::filters::applyVoxelGridFilter(pc_target, resolution);

  // Create n-sec buffer for laser data
  BUFFER_QUEUE cloud_buffer;
  cloud_buffer.buffer_size_secs = ros::Duration(args.cloud_buffer_sec);
  cloud_buffer.cloud_frame      = GLOBAL_FRAME;

  ROS_INFO("Reading cloud messages on cloud_topic: %s", args.topic_cloud.c_str());
  rosbag::View view = rosbag::View(bag, rosbag::TopicQuery(args.topic_cloud));

  // Iterate over rosbag
  for (const rosbag::MessageInstance &msg : view) {

    if (!ros::ok()) {
      ros::shutdown();
      return drift_data;
    }

    // Read point cloud topic
    const sensor_msgs::PointCloud2::Ptr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
    if (!cloud_msg) {
      ROS_WARN("Skipping msg: could not instantiate it to a sensor_msgs::PointCloud2.");
      continue;
    }
    ROS_INFO_ONCE("Found atleast one valid message of type (%s).", msg.getDataType().c_str());

    // Filter msgs before specified start time of reading
    const ros::Time stamp = cloud_msg->header.stamp;
    if (stamp < rosbag_start_time || stamp > rosbag_end_time) {
      ROS_INFO("Skipping msg: timestamp (%.2f) is out of the area of interest (%.2f, %.2f).", stamp.toSec(), rosbag_start_time.toSec(),
               rosbag_end_time.toSec());
      continue;
    }
    ROS_INFO_ONCE("Found atleast one valid message with valid time.");

    // Find current pose of cloud origin in world
    geometry_msgs::TransformStamped lidar_in_world_geom_tf;
    try {
      lidar_in_world_geom_tf = tf_buffer->lookupTransform(GLOBAL_FRAME, cloud_msg->header.frame_id, stamp);
    }
    catch (tf2::TransformException ex) {
      ROS_WARN("Failed to lookup transform (target: %s, source: %s, time: %0.2f)", GLOBAL_FRAME.c_str(), cloud_msg->header.frame_id.c_str(), stamp.toSec());
      continue;
    }
    ROS_INFO_ONCE("Found atleast one valid transformation of lidar in world.");

    // Convert msg to PCL
    PC::Ptr cloud = boost::make_shared<PC>();
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Clip range of the clouds
    if (args.min_points_dist_from_origin > 0.0) {
      cloud = clipCloud(cloud, args.min_points_dist_from_origin);
    }

    // Transform lidar to world
    Eigen::Affine3d lidar_in_world_eigen = Eigen::Affine3d::Identity();
    tf::transformMsgToEigen(lidar_in_world_geom_tf.transform, lidar_in_world_eigen);
    pcl::transformPointCloud(*cloud, *cloud, lidar_in_world_eigen);

    // Insert cloud in world to buffer queue
    cloud_buffer.insertCloud(stamp, cloud);

    // Get current position of lidar in world
    const Eigen::Vector3f position = Eigen::Vector3f(lidar_in_world_geom_tf.transform.translation.x, lidar_in_world_geom_tf.transform.translation.y,
                                                     lidar_in_world_geom_tf.transform.translation.z);

    // Perform registration in first iteration
    bool perform_registration = sample_count++ == 0;
    if (perform_registration) {
      stamp_prev    = stamp;
      position_prev = position;
      T_prev        = Eigen::Matrix4f::Identity();
    }
    // Other registrations should be performed after some time passes and the lidar moves
    else {
      const float  d_position = (position - position_prev).norm();
      const double d_time     = (stamp - stamp_prev).toSec();
      perform_registration    = d_time > args.traj_step_time && d_position > args.traj_step_dist;
      trajectory_length += d_position;
    }

    /*//{ Perform registration*/
    if (perform_registration) {

      // Prepare source cloud
      const PC::Ptr cloud_source = mrs_pcl_tools::filters::applyVoxelGridFilter(cloud_buffer.getCloudsAsOne(), resolution);

      /*//{ Publish source and target clouds before registration */
      if (_pub_pc_source_local.getNumSubscribers() > 0) {
        _pub_pc_source_local.publish(cloud_source);
      }
      if (_pub_pc_target_local.getNumSubscribers() > 0) {
        _pub_pc_target_local.publish(cloud_target);
      }
      if (_pub_pc_target_global.getNumSubscribers() > 0) {
        _pub_pc_target_global.publish(cloud_target);
      }
      /*//}*/

      // Find mutual transformations
      const auto ret = registerClouds(cloud_source, cloud_target, T_prev);

      // Fill return object
      if (ret) {
        const auto [T, cloud_aligned] = ret.value();

        const Eigen::Matrix4f pose = lidar_in_world_eigen.matrix().cast<float>();

        TRAJECTORY_POINT dato;
        dato.transformation     = T;
        dato.transformed_pose   = T * pose;
        dato.untransformed_pose = pose;
        dato.sample_time        = stamp.toSec();
        drift_data.push_back(dato);

        /*//{ Publish rviz visualizations */

        // Publish aligned cloud
        if (_pub_pc_aligned_local.getNumSubscribers() > 0) {
          _pub_pc_aligned_local.publish(cloud_aligned);
        }

        // Publish nav_msgs::Odometry msgs
        nav_msgs::Odometry odom_estimated;
        nav_msgs::Odometry odom_gt;
        odom_estimated.header.frame_id = GLOBAL_FRAME;
        odom_estimated.header.stamp    = stamp;
        odom_estimated.child_frame_id  = cloud_msg->header.frame_id;
        odom_estimated.pose.pose       = eigenMatrixToPoseMsg(dato.untransformed_pose);
        odom_gt.header                 = odom_estimated.header;
        odom_gt.child_frame_id         = cloud_msg->header.frame_id;
        odom_gt.pose.pose              = eigenMatrixToPoseMsg(dato.transformed_pose);

        if (_pub_pose_odom.getNumSubscribers() > 0 || _pub_pose_gt.getNumSubscribers() > 0) {
          try {
            _pub_pose_odom.publish(odom_estimated);
            _pub_pose_gt.publish(odom_gt);
          }
          catch (...) {
            ROS_ERROR("Exception caught during publishing the (un)transformed odometries.");
          }
        }

        // Publish nav_msgs::Path msgs
        geometry_msgs::PoseStamped pose_msg_transformed;
        geometry_msgs::PoseStamped pose_msg_untransformed;
        pose_msg_transformed.header   = path_transformed->header;
        pose_msg_untransformed.header = path_untransformed->header;
        pose_msg_transformed.pose     = odom_gt.pose.pose;
        pose_msg_untransformed.pose   = odom_estimated.pose.pose;
        path_transformed->poses.push_back(pose_msg_transformed);
        path_untransformed->poses.push_back(pose_msg_untransformed);

        geometry_msgs::Point point_msg_transformed;
        geometry_msgs::Point point_msg_untransformed;
        point_msg_transformed   = odom_gt.pose.pose.position;
        point_msg_untransformed = odom_estimated.pose.pose.position;
        path_correlation->markers.at(0).points.push_back(point_msg_transformed);
        path_correlation->markers.at(0).points.push_back(point_msg_untransformed);

        if (_pub_path_untransformed.getNumSubscribers() > 0 || _pub_path_transformed.getNumSubscribers() > 0 ||
            _pub_paths_correlation.getNumSubscribers() > 0) {
          try {
            _pub_path_untransformed.publish(path_untransformed);
            _pub_path_transformed.publish(path_transformed);
            _pub_paths_correlation.publish(path_correlation);
          }
          catch (...) {
            ROS_ERROR("Exception caught during publishing the (un)transformed trajectories.");
          }
        }
        /*//}*/

        T_prev = T;
      }

      stamp_prev    = stamp;
      position_prev = position;

      if (args.post_registration_delay_sec > 0.0) {
        ros::Duration(args.post_registration_delay_sec).sleep();
      }
    }
    /*//}*/
  }

  return drift_data;
}
/*//}*/

int main(int argc, char **argv) {

  ARGUMENTS args;
  if (!parseArguments(argc, argv, args)) {
    return -1;
  }

  ros::init(argc, argv, "EstimateLidarSlamDrift");
  ros::NodeHandle nh("EstimateLidarSlamDrift");
  ros::Time::waitForValid();

  args.print();

  // Load target cloud
  const auto ret_target = mrs_pcl_tools::loadPcXYZ(args.pcd_target);
  if (!ret_target) {
    ROS_ERROR("Could not load the target pcd file.");
    return -1;
  }

  // Common steps for both methods
  _pub_pose_odom          = nh.advertise<nav_msgs::Odometry>("odometry/estimated", 1);
  _pub_pose_gt            = nh.advertise<nav_msgs::Odometry>("odometry/ground_truth", 1);
  _pub_pc_target_global   = nh.advertise<sensor_msgs::PointCloud2>("target/global", 1);
  _pub_pc_source_local    = nh.advertise<sensor_msgs::PointCloud2>("source/local", 1);
  _pub_pc_target_local    = nh.advertise<sensor_msgs::PointCloud2>("target/local", 1);
  _pub_pc_aligned_local   = nh.advertise<sensor_msgs::PointCloud2>("aligned/local", 1);
  _pub_path_untransformed = nh.advertise<nav_msgs::Path>("trajectory/untransformed", 1);
  _pub_path_transformed   = nh.advertise<nav_msgs::Path>("trajectory/transformed", 1);
  _pub_paths_correlation  = nh.advertise<visualization_msgs::MarkerArray>("trajectory/correlation", 1);

  /*//{ Open rosbag */
  rosbag::Bag bag;
  try {
    ROS_INFO("Opening rosbag: %s", args.path_rosbag.c_str());
    bag.open(args.path_rosbag, rosbag::bagmode::Read);
  }
  catch (...) {
    ROS_ERROR("Couldn't open rosbag: %s", args.path_rosbag.c_str());
    ros::shutdown();
    return -2;
  }
  /*//}*/

  /*//{ Fill transform buffer with all the transforms from the rosbag */
  ROS_INFO("Filling transform buffer.");
  rosbag::View tf_dynamic = rosbag::View(bag, rosbag::TopicQuery(std::vector<std::string>{"/tf"}));
  rosbag::View tf_static  = rosbag::View(bag, rosbag::TopicQuery(std::vector<std::string>{"/tf_static"}));

  rosbag::View        view(bag);
  const ros::Time     bag_begin_time        = view.getBeginTime();
  const double        rosbag_start_time_sec = bag_begin_time.toSec() + args.start_time;
  const ros::Time     bag_end_time          = view.getEndTime();
  const ros::Duration bag_duration          = bag_end_time - bag_begin_time;
  const double        rosbag_end_time_sec   = bag_begin_time.toSec() + std::fmin(bag_duration.toSec(), args.end_time);

  rosbag_start_time.fromSec(rosbag_start_time_sec);
  rosbag_end_time.fromSec(rosbag_end_time_sec);

  tf_buffer   = std::make_unique<tf2_ros::Buffer>(ros::Duration(bag_duration.toSec()));
  tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  // Read static tfs
  for (const rosbag::MessageInstance &msg : tf_static) {

    if (!ros::ok()) {
      ros::shutdown();
      return -1;
    }

    const tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg) {
      for (const auto &transform : tf_msg->transforms) {
        tf_buffer->setTransform(transform, "default_authority", true);
      }
    }
  }

  // Read dynamic tfs
  for (const rosbag::MessageInstance &msg : tf_dynamic) {

    if (!ros::ok()) {
      ros::shutdown();
      return -1;
    }

    if (msg.getTime() < rosbag_start_time || msg.getTime() > rosbag_end_time) {
      continue;
    }

    const tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg) {
      for (const auto &transform : tf_msg->transforms) {
        tf_buffer->setTransform(transform, "default_authority", false);
      }
    }
  }

  // Insert initial transformation map->world
  geometry_msgs::TransformStamped tf;
  tf.header.stamp    = rosbag_start_time;
  tf.header.frame_id = args.mapping_origin;
  tf.child_frame_id  = GLOBAL_FRAME;
  tf.transform       = args.tf_target_in_map_origin;
  const bool succ    = tf_buffer->setTransform(tf, "default_authority", true);
  if (!succ) {
    ROS_ERROR("Couldn't set static global transform.");
    ros::shutdown();
    return -2;
  }

  /*//}*/

  // Find real trajectory using the cloud data within rosbag
  const auto &cloud_target                 = ret_target.value();
  cloud_target->header.frame_id            = GLOBAL_FRAME;
  const std::vector<TRAJECTORY_POINT> data = estimateGroundTruthTrajectoryFromRosbag(bag, cloud_target, args);

  // Close rosbag
  bag.close();

  // Save trajectory to text file
  saveTrajectory(args.txt_trajectory_gt, data, true);
  saveTrajectory(args.txt_trajectory_odom, data, false);

  return 0;
}
