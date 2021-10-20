#include <mrs_pcl_tools/support.h>

#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>
#include <nav_msgs/Path.h>

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

  std::string     path_rosbag;
  std::string     topic_cloud;
  std::string     mapping_origin;
  std::string     pcd_target;
  std::string     txt_trajectory_out;
  float           traj_step_dist   = 0.0f;
  float           traj_step_time   = 0.0f;
  double          start_time       = 0.0;
  double          cloud_buffer_sec = 5.0;
  Eigen::Matrix4f tf_map_in_target_origin;

  void print() {

    if (initialized) {
      ROS_INFO("Target pcd: %s", pcd_target.c_str());
      ROS_INFO("Rosbag path: %s", path_rosbag.c_str());
      ROS_INFO("Topic cloud: %s", topic_cloud.c_str());
      ROS_INFO("Mapping origin: %s", mapping_origin.c_str());
      mrs_pcl_tools::printEigenMatrix(tf_map_in_target_origin, "Transform target->map");
      ROS_INFO("Trajectory out: %s", txt_trajectory_out.c_str());
      ROS_INFO("Trajectory distance step: %0.2f; time step: %0.2f", traj_step_dist, traj_step_time);
      ROS_INFO("Point cloud buffer: %0.2f s", cloud_buffer_sec);
      ROS_INFO("Start time offset: %0.2f", start_time);
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
  unsigned int    sample_index;
  double          sample_time;
  float           eucl_distance_from_origin;
  float           distance_on_trajectory;
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
      "   rosrun mrs_pcl_tools estimate_cloud_to_cloud_drift rosbag.bag topic_cloud mapping_origin tf_map_in_target.txt target.pcd trajectory_out.txt [...]");

  ROS_ERROR("Arguments:");
  ROS_ERROR(" rosbag.bag:            rosbag containing sensor data (point cloud) and /tf topic with mapping_origin->cloud_origin transformation");
  ROS_ERROR(" topic_cloud:           point cloud topic (supported types: sensor_msgs/PointCloud2)");
  ROS_ERROR(" mapping_origin:        mapping origin (string)");
  ROS_ERROR(" tf_map_in_target.txt:  transformation from the target cloud to the mapping origin (expected format: (x, y, z, qx, qy, qz, qw))");
  ROS_ERROR(" target.pcd:            target point cloud (ground truth)");
  ROS_ERROR(" trajectory_out.txt:    corrected (real) trajectory of the robot");

  ROS_ERROR("Optional arguments:");
  ROS_ERROR(" --traj-step-dist:      sampling of trajectory by distance, used if greater than 0.0 (default: 0.0 m)");
  ROS_ERROR(" --traj-step-time:      sampling of trajectory by time, used if greater than 0.0 (default: 0.0 s)");
  ROS_ERROR(" --cloud-buffer:        buffer length of cloud data in seconds (default: 5.0 s)");
  ROS_ERROR(" --start-time:          start time offset of the rosbag (default: 0.0 s)");
}
/*//}*/

/*//{ loadMatrix() */
std::optional<Eigen::Matrix4f> loadMatrix(const std::string &filepath) {

  ROS_INFO("Loading matrix from: %s", filepath.c_str());

  struct stat buffer;
  if (stat(filepath.c_str(), &buffer) == 0) {

    std::ifstream infile(filepath);
    std::string   line;

    unsigned int l = 0;

    while (std::getline(infile, line)) {

      if (line.rfind("#", 0) == 0) {
        continue;
      }

      std::istringstream  iss(line);
      std::vector<double> line_numbers;
      std::copy(std::istream_iterator<double>(iss), std::istream_iterator<double>(), std::back_inserter(line_numbers));

      if (line_numbers.size() != 7) {
        ROS_WARN("[Loading matrix] Skipping line %d (size: %ld) as it does not contain data in the expected format (x, y, z, qx, qy, qz, qw).", l,
                 line_numbers.size());
      } else {

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

        T(0, 3) = line_numbers[0];
        T(1, 3) = line_numbers[1];
        T(2, 3) = line_numbers[2];

        geometry_msgs::Quaternion quat;
        quat.x = line_numbers[3];
        quat.y = line_numbers[4];
        quat.z = line_numbers[5];
        quat.w = line_numbers[6];

        mrs_lib::AttitudeConverter atti = mrs_lib::AttitudeConverter(quat);
        T.block<3, 3>(0, 0)             = Eigen::Matrix3d(atti).cast<float>();

        return T;
      }

      l++;
    }

  } else {
    ROS_ERROR("File path (%s) does not exist.", filepath.c_str());
  }

  return std::nullopt;
}
/*//}*/

/*//{ parseArguments() */
bool parseArguments(int argc, char **argv, ARGUMENTS &args) {

  if (argc < 7) {
    printHelp();
    return false;
  }

  args.path_rosbag        = argv[1];
  args.topic_cloud        = argv[2];
  args.mapping_origin     = argv[3];
  args.pcd_target         = argv[5];
  args.txt_trajectory_out = argv[6];

  const auto tf = loadMatrix(argv[4]);
  if (!tf) {
    std::cerr << "Could not load transform of mapping in the target origin. Ending." << std::endl;
    return false;
  }
  args.tf_map_in_target_origin = tf.value();

  for (unsigned int i = 7; i < argc; i += 2) {
    const auto option = std::string(argv[i]);
    if (option == "--traj-step-dist") {
      args.traj_step_dist = float(std::atof(argv[i + 1]));
    } else if (option == "--traj-step-time") {
      args.traj_step_time = float(std::atof(argv[i + 1]));
    } else if (option == "--start-time") {
      args.start_time = std::atof(argv[i + 1]);
    } else if (option == "--cloud-buffer") {
      args.cloud_buffer_sec = std::atof(argv[i + 1]);
    } else {
      std::cerr << "Unknown option: " << option.c_str() << ". Ending." << std::endl;
      return false;
    }
  }

  args.initialized = true;

  return true;
}
/*//}*/

/* -------------------- Global variables -------------------- */
pcl::IterativeClosestPoint<pt_XYZ, pt_XYZ> _icp;
pcl::CropBox<pt_XYZ>                       _filter_box;

ros::Publisher _pub_pc_source_local;
ros::Publisher _pub_pc_target_global;
ros::Publisher _pub_pc_target_local;
ros::Publisher _pub_pc_aligned_local;
ros::Publisher _pub_path_untransformed;
ros::Publisher _pub_path_transformed;

/* -------------------- Functions -------------------- */
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

/*//{ saveTrajectory() */
void saveTrajectory(const std::string &filepath, const std::vector<TRAJECTORY_POINT> &data) {

  ROS_INFO("Saving trajectory to: %s", filepath.c_str());

  const unsigned int precision = 5;

  std::ofstream outfile(filepath);

  if (outfile.is_open()) {

    outfile << "# timestamp x y z qx qy qz qw" << std::endl;

    for (const auto &dato : data) {

      const mrs_lib::AttitudeConverter atti = mrs_lib::AttitudeConverter(dato.transformed_pose.block<3, 3>(0, 0).cast<double>());
      const geometry_msgs::Quaternion  quat = atti;

      outfile << std::fixed << std::setprecision(precision) << dato.sample_time << " ";
      outfile << std::fixed << std::setprecision(precision) << dato.transformed_pose(0, 3) << " ";
      outfile << std::fixed << std::setprecision(precision) << dato.transformed_pose(1, 3) << " ";
      outfile << std::fixed << std::setprecision(precision) << dato.transformed_pose(2, 3) << " ";
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

/*//{ estimateGroundTruthTrajectoryFromRosbag() */
std::vector<TRAJECTORY_POINT> estimateGroundTruthTrajectoryFromRosbag(const PC::Ptr &pc_target, const ARGUMENTS &args) {
  // TODO: estimate trajectory from rosbag and read scan data per time (do not use global map which can be very bad)

  std::vector<TRAJECTORY_POINT> drift_data;

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

  args.print();

  // Load target cloud
  const auto ret_target = mrs_pcl_tools::loadPcXYZ(args.pcd_target);
  if (!ret_target) {
    ROS_ERROR("Could not load the target pcd file.");
    return -1;
  }

  // Common steps for both methods
  _pub_pc_target_global   = nh.advertise<sensor_msgs::PointCloud2>("target/global", 1);
  _pub_pc_source_local    = nh.advertise<sensor_msgs::PointCloud2>("source/local", 1);
  _pub_pc_target_local    = nh.advertise<sensor_msgs::PointCloud2>("target/local", 1);
  _pub_pc_aligned_local   = nh.advertise<sensor_msgs::PointCloud2>("aligned/local", 1);
  _pub_path_untransformed = nh.advertise<nav_msgs::Path>("trajectory/untransformed", 1);
  _pub_path_transformed   = nh.advertise<nav_msgs::Path>("trajectory/transformed", 1);

  // Find real trajectory using the cloud data within rosbag
  const std::vector<TRAJECTORY_POINT> data = estimateGroundTruthTrajectoryFromRosbag(ret_target.value(), args);

  // save trajectory
  saveTrajectory(args.txt_trajectory_out, data);

  return 0;
}
