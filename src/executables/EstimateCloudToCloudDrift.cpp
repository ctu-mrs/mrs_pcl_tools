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

void printHelp() {
  ROS_ERROR(
      "Syntax is: `rosrun mrs_pcl_tools estimate_cloud_to_cloud_drift source.pcd target.pcd trajectory_in.txt trajectory_out.txt [traj_step, crop_dist]`");
  ROS_ERROR(
      "Ground truth trajectory will be estimated from given drifting trajectory using two PCD maps. 1) source cloud (i.e., SLAM map output) and 2) target "
      "cloud (i.e., reference 3D scan). The registration parameters are hard-coded.");
  ROS_ERROR("IMPORTANT: Method expects source/target clouds to be pre-registered and the input trajectory to be in the frame of the source point cloud.");
  ROS_ERROR("Arguments:");
  ROS_ERROR("- source.pcd:         source point cloud (with drift)");
  ROS_ERROR("- target.pcd:         target point cloud (ground truth)");
  ROS_ERROR("- trajectory_in.txt:  trajectory of the robot (expected format: time x y z [qx qy qz qw]");
  ROS_ERROR("- trajectory_out.txt: corrected trajectory of the robot");
  ROS_ERROR("- traj_step:          sampling of trajectory by distance, used if greater than 0.0 (optional, default: -1.0 m)");
  ROS_ERROR(
      "- crop_dist:          crop distance of global->local clouds conversion before their local registration (optional, default: [xy: 10.0 m, z: 3 * xy])");
  ROS_ERROR("- start_time:         start time offset of the trajectory (optional, default: 0.0 s)");
}

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

pcl::IterativeClosestPoint<pt_XYZ, pt_XYZ> _icp;
pcl::CropBox<pt_XYZ>                       _filter_box;

ros::Publisher _pub_pc_source_global;
ros::Publisher _pub_pc_source_local;
ros::Publisher _pub_pc_target_global;
ros::Publisher _pub_pc_target_local;
ros::Publisher _pub_pc_aligned_local;
ros::Publisher _pub_path_untransformed;
ros::Publisher _pub_path_transformed;

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

/*//{ loadTrajectory() */
std::vector<std::tuple<double, Eigen::Matrix4f>> loadTrajectory(const std::string &filepath) {

  ROS_INFO("Loading trajectory from: %s", filepath.c_str());

  std::vector<std::tuple<double, Eigen::Matrix4f>> trajectory;

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

      if (line_numbers.size() != 4 && line_numbers.size() != 8) {
        ROS_WARN("[Loading trajectory] Skipping line %d (size: %ld) as it does not contain data in the expected format (time, x, y, z [, qx, qy, qz, qw]).", l,
                 line_numbers.size());

      } else {

        Eigen::Matrix4f           T = Eigen::Matrix4f::Identity();
        geometry_msgs::Quaternion quat;

        if (line_numbers.size() == 4) {
          quat.w = 1.0;
        } else {
          quat.x = line_numbers[4];
          quat.y = line_numbers[5];
          quat.z = line_numbers[6];
          quat.w = line_numbers[7];
        }

        mrs_lib::AttitudeConverter atti = mrs_lib::AttitudeConverter(quat);
        T.block<3, 3>(0, 0)             = Eigen::Matrix3d(atti).cast<float>();

        T(0, 3) = line_numbers[1];
        T(1, 3) = line_numbers[2];
        T(2, 3) = line_numbers[3];

        trajectory.push_back(std::make_tuple(line_numbers[0], T));
      }

      l++;
    }

  } else {
    ROS_ERROR("File path (%s) does not exist.", filepath.c_str());
  }

  return trajectory;
}
/*//}*/

/*//{ saveTrajectory() */
void saveTrajectory(const std::string &filepath, const std::vector<TRAJECTORY_POINT> &data) {

  ROS_INFO("Saving trajectory to: %s", filepath.c_str());

  const unsigned int precision = 5;

  std::ofstream outfile(filepath);

  if (outfile.is_open()) {

    for (const auto &dato : data) {

      const mrs_lib::AttitudeConverter atti = mrs_lib::AttitudeConverter(dato.transformed_pose.block<3, 3>(0, 0).cast<double>());
      const geometry_msgs::Quaternion  quat = atti;

      outfile << std::fixed << std::setprecision(20) << dato.sample_time << " ";
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
std::optional<std::tuple<Eigen::Matrix4f, PC::Ptr>> registerClouds(const PC::Ptr &source, const PC::Ptr &target) {

  PC::Ptr aligned = boost::make_shared<PC>();

  _icp.setInputSource(source);
  _icp.setInputTarget(target);

  _icp.align(*aligned, Eigen::Matrix4f::Identity());

  if (!_icp.hasConverged()) {
    ROS_WARN("ICP has not converged, change your parameters.");
    return std::nullopt;
  }

  return std::make_tuple(_icp.getFinalTransformation(), aligned);
}
/*//}*/

/*//{ estimateGroundTruthTrajectory() */
std::vector<TRAJECTORY_POINT> estimateGroundTruthTrajectory(const std::vector<std::tuple<double, Eigen::Matrix4f>> &trajectory, const PC::Ptr &pc_source,
                                                            const PC::Ptr &pc_target, const float &traj_step, const float &crop_dist,
                                                            const double &start_time) {

  std::vector<TRAJECTORY_POINT> drift_data;

  const std::string frame = "origin";

  nav_msgs::Path::Ptr path_transformed   = boost::make_shared<nav_msgs::Path>();
  nav_msgs::Path::Ptr path_untransformed = boost::make_shared<nav_msgs::Path>();
  path_transformed->header.frame_id      = frame;
  path_untransformed->header.frame_id    = frame;

  // Preset global objects parameters
  _icp.setMaxCorrespondenceDistance(15.0);
  _icp.setMaximumIterations(5000);
  _icp.setTransformationEpsilon(0.1);
  _icp.setEuclideanFitnessEpsilon(0.1);
  _icp.setRANSACIterations(500);
  _icp.setRANSACOutlierRejectionThreshold(0.8);
  _icp.setUseReciprocalCorrespondences(false);

  const float xy_crop_dist = crop_dist / 2.0f;
  const float z_crop_dist  = 3.0f * crop_dist / 2.0f;

  const auto [t_origin, pose_origin]      = trajectory.front();
  const Eigen::Vector3f position_origin   = Eigen::Vector3f(pose_origin(0, 3), pose_origin(1, 3), pose_origin(2, 3));
  const double          time_min          = t_origin + start_time;
  Eigen::Vector3f       position_prev     = position_origin;
  float                 trajectory_length = 0.0f;
  unsigned int          sample_count      = 0;

  ROS_INFO("Estimating ground truth trajectory:");
  for (unsigned int i = 0; i < trajectory.size(); i++) {

    if (!ros::ok()) {
      ros::shutdown();
      break;
    }

    const auto [t, pose]           = trajectory.at(i);
    const Eigen::Vector3f position = Eigen::Vector3f(pose(0, 3), pose(1, 3), pose(2, 3));

    const float d_position = (position - position_prev).norm();
    trajectory_length += d_position;

    // Discretize trajectory by distance per traj_step
    if ((i > 0 && d_position < traj_step) || t < time_min) {
      continue;
    }

    ROS_INFO("- %d/%ld.", i, trajectory.size());

    // Crop both clouds
    const PC::Ptr pc_source_local = cropCloud(pc_source, position, xy_crop_dist, z_crop_dist);
    const PC::Ptr pc_target_local = cropCloud(pc_target, position, xy_crop_dist, z_crop_dist);

    if (pc_source_local->empty() || pc_target_local->empty()) {
      ROS_WARN("Cropped part of the source or target clouds are empty, skipping frame.");
      continue;
    }

    pc_source->header.frame_id       = frame;
    pc_target->header.frame_id       = frame;
    pc_source_local->header.frame_id = frame;
    pc_target_local->header.frame_id = frame;

    // Publish clouds
    if (sample_count++ % 10 == 0) {
      mrs_pcl_tools::publishCloud(_pub_pc_source_global, pc_source);
      mrs_pcl_tools::publishCloud(_pub_pc_target_global, pc_target);
    }
    mrs_pcl_tools::publishCloud(_pub_pc_source_local, pc_source_local);
    mrs_pcl_tools::publishCloud(_pub_pc_target_local, pc_target_local);

    // Find mutual transformations
    const auto ret = registerClouds(pc_source_local, pc_target_local);

    // Fill return object
    if (ret) {
      const auto [T, pc_aligned_local] = ret.value();

      TRAJECTORY_POINT dato;
      dato.transformation            = T;
      dato.transformed_pose          = T * pose;
      dato.untransformed_pose        = pose;
      dato.sample_time               = t;
      dato.sample_index              = i;
      dato.eucl_distance_from_origin = (position - position_origin).norm();
      dato.distance_on_trajectory    = trajectory_length;
      drift_data.push_back(dato);

      // Publish aligned cloud
      mrs_pcl_tools::publishCloud(_pub_pc_aligned_local, pc_aligned_local);

      // Publish nav_msgs::Path msgs
      geometry_msgs::PoseStamped pose_msg_transformed;
      geometry_msgs::PoseStamped pose_msg_untransformed;
      pose_msg_transformed.header   = path_transformed->header;
      pose_msg_untransformed.header = path_untransformed->header;
      pose_msg_transformed.pose     = eigenMatrixToPoseMsg(dato.transformed_pose);
      pose_msg_untransformed.pose   = eigenMatrixToPoseMsg(dato.untransformed_pose);
      path_transformed->poses.push_back(pose_msg_transformed);
      path_untransformed->poses.push_back(pose_msg_untransformed);

      if (_pub_path_untransformed.getNumSubscribers() > 0 || _pub_path_transformed.getNumSubscribers() > 0) {
        try {
          _pub_path_untransformed.publish(path_untransformed);
          _pub_path_transformed.publish(path_transformed);
        }
        catch (...) {
          ROS_ERROR("[AloamMapping]: Exception caught during publishing the trajectories.");
        }
      }
    }

    position_prev = position;
  }

  return drift_data;
}
/*//}*/

int main(int argc, char **argv) {
  if (argc < 5) {
    printHelp();
    return -1;
  }

  ros::init(argc, argv, "EstimateCloudToCloudDrift");
  ros::NodeHandle nh("EstimateCloudToCloudDrift");

  // open rosbag
  const std::string pcd_source         = argv[1];
  const std::string pcd_target         = argv[2];
  const std::string txt_trajectory_in  = argv[3];
  const std::string txt_trajectory_out = argv[4];
  float             traj_step          = -1.0f;
  float             crop_dist          = 10.0f;
  double            start_time         = 0.0;
  if (argc == 6) {
    traj_step = std::atof(argv[5]);
  } else if (argc == 7) {
    traj_step = std::atof(argv[5]);
    crop_dist = std::atof(argv[6]);
  } else if (argc == 8) {
    traj_step  = std::atof(argv[5]);
    crop_dist  = std::atof(argv[6]);
    start_time = std::atof(argv[7]);
  }

  ROS_INFO("Source pcd: %s", pcd_source.c_str());
  ROS_INFO("Target pcd: %s", pcd_target.c_str());
  ROS_INFO("Trajectory in: %s", txt_trajectory_in.c_str());
  ROS_INFO("Trajectory out: %s", txt_trajectory_out.c_str());
  ROS_INFO("Trajectory distance step: %0.2f", traj_step);
  ROS_INFO("Clouds crop distance -> xy: %0.2f, z: %0.2f", crop_dist, crop_dist * 3.0f);

  // load trajectory
  std::vector<std::tuple<double, Eigen::Matrix4f>> trajectory = loadTrajectory(txt_trajectory_in);
  if (trajectory.size() == 0) {
    ROS_ERROR("Loaded trajectory is empty. Ending the node.");
    return 0;
  }

  // load pcds
  const auto ret_source = mrs_pcl_tools::loadPcXYZ(pcd_source);
  const auto ret_target = mrs_pcl_tools::loadPcXYZ(pcd_target);

  if (!ret_source || !ret_target) {
    ROS_ERROR("Could not load one of the source/target sources.");
    return -1;
  }

  // Advertise publishers
  _pub_pc_source_global   = nh.advertise<sensor_msgs::PointCloud2>("source/global", 1);
  _pub_pc_target_global   = nh.advertise<sensor_msgs::PointCloud2>("target/global", 1);
  _pub_pc_source_local    = nh.advertise<sensor_msgs::PointCloud2>("source/local", 1);
  _pub_pc_target_local    = nh.advertise<sensor_msgs::PointCloud2>("target/local", 1);
  _pub_pc_aligned_local   = nh.advertise<sensor_msgs::PointCloud2>("aligned/local", 1);
  _pub_path_untransformed = nh.advertise<nav_msgs::Path>("trajectory/untransformed", 1);
  _pub_path_transformed   = nh.advertise<nav_msgs::Path>("trajectory/transformed", 1);

  // for traj_step in trajectory, crop maps and compute icp source->target
  std::vector<TRAJECTORY_POINT> data = estimateGroundTruthTrajectory(trajectory, ret_source.value(), ret_target.value(), traj_step, crop_dist, start_time);

  // save trajectory
  saveTrajectory(txt_trajectory_out, data);

  return 0;
}
