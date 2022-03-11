#include <mrs_pcl_tools/support.h>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/convolution.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <eigen_conversions/eigen_msg.h>

#include <tuple>
#include <optional>
#include <iostream>
#include <sys/stat.h>
#include <fstream>
#include <iomanip>

/*//{ struct TRAJECTORY */
struct SAMPLE
{
  double                   time;
  geometry_msgs::Transform tf;
};

struct TRAJECTORY
{
  std::vector<SAMPLE> samples;

  const void insert(const double time, const geometry_msgs::Transform &tf) {
    SAMPLE sample;
    sample.time = time;
    sample.tf   = tf;
    samples.push_back(sample);
  }

  const size_t size() {
    return samples.size();
  }
};
/*//}*/

/*//{ struct ARGUMENTS */
struct ARGUMENTS
{
  bool initialized = false;

  std::string path_rosbag;
  std::string topic_points;
  std::string topic_poses = "";
  std::string file_poses  = "";
  std::string file_cloud_out;
  std::string node_name              = "ConcatenateLidarScans";
  double      max_tf_dt              = 0.005;
  double      voxel_resolution_scans = 0.0;
  double      voxel_resolution_map   = 0.0;
  double      start_time             = 0.0;
  double      end_time               = std::numeric_limits<double>::max();
  double      points_min_dist        = 0.0;
  double      points_max_dist        = 1000.0;

  void print() {

    if (initialized) {
      ROS_INFO("Rosbag: %s", path_rosbag.c_str());
      ROS_INFO("Topic points: %s", topic_points.c_str());
      ROS_INFO("Topic with poses: %s", topic_poses.c_str());
      ROS_INFO("File with poses: %s", file_poses.c_str());
      ROS_INFO("PCD/PLY out: %s", file_cloud_out.c_str());
      ROS_INFO("Rosbag start time offset: %0.2f", start_time);
      ROS_INFO("Rosbag end time offset: %0.2f", end_time);
      ROS_INFO("Maximal dt between scans and corresponding TF: %0.4f s", max_tf_dt);
      ROS_INFO("Minimum distance of points from their origin: %0.2f m", points_min_dist);
      ROS_INFO("Maximum distance of points from their origin: %0.2f m", points_max_dist);
      ROS_INFO("Voxel resolution -> scans: %0.2f m, map: %0.2f m", voxel_resolution_scans, voxel_resolution_map);
    }
  }
};
/*//}*/

/*//{ printHelp() */
void printHelp() {
  ROS_ERROR("Single scan will be built by concatenating LiDAR scans (from rosbag) and a sequence of poses (from rosbag or file).");

  ROS_ERROR("Usage:");
  ROS_ERROR("   rosrun mrs_pcl_tools concatenate_lidar_scans rosbag.bag topic_points point_cloud.ply [...]");

  ROS_ERROR("Arguments:");
  ROS_ERROR(" rosbag.bag:              rosbag containing sensor data (point cloud)");
  ROS_ERROR(" topic_points:            point cloud topic (supported types: sensor_msgs/PointCloud2)");
  ROS_ERROR(" point_cloud.ply:         path to store the output point cloud in ply/pcd format (string)");
  ROS_ERROR(" --poses-file:            text file with poses in format [timestamp x y z qx qy qz qw]");
  ROS_ERROR(" --poses-topic:           topic with poses (supported types: nav_msgs/Odometry)");

  ROS_ERROR("Optional arguments:");
  ROS_ERROR(" --start-time:                   start time offset of the rosbag (default: 0.0 s)");
  ROS_ERROR(" --end-time:                     end time offset of the rosbag (default: full duration)");
  ROS_ERROR(" --max-tf-dt:                    maximal dt between transform and scan timestamps (default: 0.02 s)");
  ROS_ERROR(" --voxel-resolution-scans:       voxel resolution of integrated scans (default: 0.0 m)");
  ROS_ERROR(" --voxel-resolution-map:         voxel resolution of output file (default: 0.0 m)");
  ROS_ERROR(" --points-min-dist:              minimal distance of points from their origin (default: 0.0 m)");
  ROS_ERROR(" --points-max-dist:              maximal distance of points from their origin (default: 0.0 m)");
}
/*//}*/

/*//{ parseArguments() */
bool parseArguments(int argc, char **argv, ARGUMENTS &args) {

  if (argc < 6) {
    printHelp();
    return false;
  }

  args.path_rosbag    = argv[1];
  args.topic_points   = argv[2];
  args.file_cloud_out = argv[3];

  for (unsigned int i = 4; i < argc;) {
    int i_inc = 2;

    const auto option = std::string(argv[i]);
    if (option == "--poses-topic") {
      args.topic_poses = argv[i + 1];
    } else if (option == "--poses-file") {
      args.file_poses = argv[i + 1];
    } else if (option == "--start-time") {
      args.start_time = std::atof(argv[i + 1]);
    } else if (option == "--end-time") {
      args.end_time = std::atof(argv[i + 1]);
    } else if (option == "--max-tf-dt") {
      args.max_tf_dt = std::atof(argv[i + 1]);
    } else if (option == "--voxel-resolution-scans") {
      args.voxel_resolution_scans = std::atof(argv[i + 1]);
    } else if (option == "--voxel-resolution-map") {
      args.voxel_resolution_map = std::atof(argv[i + 1]);
    } else if (option == "--points-min-dist") {
      args.points_min_dist = std::atof(argv[i + 1]);
    } else if (option == "--points-max-dist") {
      args.points_max_dist = std::atof(argv[i + 1]);
    } else if (option == "--node-name") {
      args.node_name = argv[i + 1];
    } else {
      std::cerr << "Unknown option: " << option.c_str() << ". Ending." << std::endl;
      return false;
    }

    i += i_inc;
  }

  if (args.file_poses.empty() && args.topic_poses.empty()) {
    std::cerr << "Text file with poses nor a pose topic were given." << std::endl;
    printHelp();
    return false;
  } else if (!args.file_poses.empty() && !args.topic_poses.empty()) {
    std::cerr << "Both text file with poses and a pose topic were given. Specify just one of them, I cannot decide for you what data to use." << std::endl;
    printHelp();
    return false;
  }

  args.initialized = true;

  return true;
}
/*//}*/

/* -------------------- Global variables -------------------- */

const std::string GLOBAL_FRAME = "global_origin";
const std::string CHILD_FRAME  = "sensor_origin";

ros::Publisher _pub_cloud_global;
ros::Publisher _pub_cloud_latest;
ros::Publisher _pub_odom;
ros::Publisher _pub_path;

std::unique_ptr<tf2_ros::Buffer>            tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;

ros::Time rosbag_start_time;
ros::Time rosbag_end_time;


/* -------------------- Functions -------------------- */

/*//{ loadTrajectoryFile() */
TRAJECTORY loadTrajectoryFile(const std::string &filepath) {

  ROS_INFO("Loading trajectory from: %s", filepath.c_str());

  TRAJECTORY trajectory;

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

        geometry_msgs::Transform tf_msg;

        tf_msg.translation.x = line_numbers[1];
        tf_msg.translation.y = line_numbers[2];
        tf_msg.translation.z = line_numbers[3];

        if (line_numbers.size() == 4) {
          tf_msg.rotation.w = 1.0;
        } else {
          tf_msg.rotation.x = line_numbers[4];
          tf_msg.rotation.y = line_numbers[5];
          tf_msg.rotation.z = line_numbers[6];
          tf_msg.rotation.w = line_numbers[7];
        }

        trajectory.insert(line_numbers[0], tf_msg);
      }

      l++;
    }

  } else {
    ROS_ERROR("File path (%s) does not exist.", filepath.c_str());
  }

  return trajectory;
}
/*//}*/

/*//{ clipCloud() */
PC::Ptr clipCloud(const PC::Ptr &cloud_in, const double min_dist, const double max_dist) {

  if (min_dist <= 0.0) {
    return cloud_in;
  }

  const PC::Ptr cloud_out = boost::make_shared<PC>();

  // Copy points in given range
  for (const auto &p : cloud_in->points) {
    const double norm = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (norm > min_dist && norm < max_dist) {
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

/*//{ concatenateClouds() */
bool concatenateClouds(PC::Ptr &cloud_out, const rosbag::Bag &bag, const ARGUMENTS &args) {

  const nav_msgs::Path::Ptr path_msg = boost::make_shared<nav_msgs::Path>();
  path_msg->header.frame_id          = GLOBAL_FRAME;

  ROS_INFO("Reading cloud messages on cloud_topic: %s", args.topic_points.c_str());
  rosbag::View view = rosbag::View(bag, rosbag::TopicQuery(args.topic_points));

  // Iterate over rosbag
  for (const rosbag::MessageInstance &msg : view) {

    // Handle ctrl+c
    if (!ros::ok()) {
      ros::shutdown();
      return false;
    }

    // Read point cloud topic
    const sensor_msgs::PointCloud2::Ptr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
    if (!cloud_msg) {
      continue;
    }
    ROS_INFO_ONCE("Found atleast one valid message of type (%s).", msg.getDataType().c_str());

    // Filter msgs before specified start time of reading
    const ros::Time stamp = cloud_msg->header.stamp;
    if (stamp < rosbag_start_time || stamp > rosbag_end_time) {
      continue;
    }
    ROS_INFO_ONCE("Found atleast one valid message with valid time.");

    // Find current pose of cloud origin in world
    geometry_msgs::TransformStamped lidar_in_world_geom_tf;
    try {
      lidar_in_world_geom_tf = tf_buffer->lookupTransform(GLOBAL_FRAME, CHILD_FRAME, stamp);
    }
    catch (tf2::TransformException ex) {
      ROS_WARN("Failed to lookup transform (target: %s, source: %s, time: %0.2f)", GLOBAL_FRAME.c_str(), CHILD_FRAME.c_str(), stamp.toSec());
      continue;
    }

    if (std::fabs((lidar_in_world_geom_tf.header.stamp - stamp).toSec()) > args.max_tf_dt) {
      ROS_WARN("Did not find close enough transform for cloud (cloud stamp: %.1f, tf stamp: %.1f, max_tf_dt: %0.1f)", cloud_msg->header.stamp.toSec(),
               lidar_in_world_geom_tf.header.stamp.toSec(), args.max_tf_dt);
      continue;
    }

    ROS_INFO_ONCE("Found atleast one valid transformation of lidar in world.");

    // Convert msg to PCL
    PC::Ptr scan = boost::make_shared<PC>();
    pcl::fromROSMsg(*cloud_msg, *scan);

    // Clip range of the clouds
    scan = clipCloud(scan, args.points_min_dist, args.points_max_dist);
    scan = mrs_pcl_tools::filters::applyVoxelGridFilter(scan, float(args.voxel_resolution_scans));

    // Transform lidar to world
    Eigen::Affine3d lidar_in_world_eigen = Eigen::Affine3d::Identity();
    tf::transformMsgToEigen(lidar_in_world_geom_tf.transform, lidar_in_world_eigen);
    pcl::transformPointCloud(*scan, *scan, lidar_in_world_eigen);

    // Concatenate
    *cloud_out += *scan;
    /* cloud_out = mrs_pcl_tools::filters::applyVoxelGridFilter(cloud_out, float(args.voxel_resolution_map)); */

    /*//{ Publish */
    if (_pub_cloud_latest.getNumSubscribers() > 0) {
      scan->header.frame_id = GLOBAL_FRAME;
      _pub_cloud_latest.publish(scan);
    }
    if (_pub_cloud_global.getNumSubscribers() > 0) {
      _pub_cloud_global.publish(cloud_out);
    }
    if (_pub_odom.getNumSubscribers() > 0) {
      const nav_msgs::Odometry::Ptr odom_msg = boost::make_shared<nav_msgs::Odometry>();
      odom_msg->header.stamp                 = stamp;
      odom_msg->header.frame_id              = GLOBAL_FRAME;
      odom_msg->child_frame_id               = CHILD_FRAME;
      odom_msg->pose.pose.position.x         = lidar_in_world_geom_tf.transform.translation.x;
      odom_msg->pose.pose.position.y         = lidar_in_world_geom_tf.transform.translation.y;
      odom_msg->pose.pose.position.z         = lidar_in_world_geom_tf.transform.translation.z;
      odom_msg->pose.pose.orientation        = lidar_in_world_geom_tf.transform.rotation;

      _pub_odom.publish(odom_msg);
    }

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id  = GLOBAL_FRAME;
    pose_msg.header.stamp     = stamp;
    pose_msg.pose.position.x  = lidar_in_world_geom_tf.transform.translation.x;
    pose_msg.pose.position.y  = lidar_in_world_geom_tf.transform.translation.y;
    pose_msg.pose.position.z  = lidar_in_world_geom_tf.transform.translation.z;
    pose_msg.pose.orientation = lidar_in_world_geom_tf.transform.rotation;
    path_msg->poses.push_back(pose_msg);

    if (_pub_path.getNumSubscribers() > 0) {
      _pub_path.publish(path_msg);
    }
    /*//}*/
  }

  ROS_INFO_COND(args.voxel_resolution_map > 0.0, "Applying voxel grid filter to map. Resolution: %.1f m", args.voxel_resolution_map);
  cloud_out = mrs_pcl_tools::filters::applyVoxelGridFilter(cloud_out, float(args.voxel_resolution_map));
  return true;
}
/*//}*/

/*//{ saveCloud() */
bool saveCloud(const PC::Ptr &cloud, const std::string &filepath) {

  if (filepath.length() >= 3) {
    if (0 == filepath.compare(filepath.length() - 3, 3, "pcd")) {
      ROS_INFO("Saving pcd file to: %s", filepath.c_str());
      return pcl::io::savePCDFileASCII(filepath, *cloud) == 0;
    } else if (0 == filepath.compare(filepath.length() - 3, 3, "ply")) {
      ROS_INFO("Saving ply file to: %s", filepath.c_str());
      return pcl::io::savePLYFileASCII(filepath, *cloud) == 0;
    }
  }

  return false;
}
/*//}*/

int main(int argc, char **argv) {

  ARGUMENTS args;
  if (!parseArguments(argc, argv, args)) {
    return -1;
  }

  ros::init(argc, argv, args.node_name);
  ros::NodeHandle nh(args.node_name);
  ros::Time::waitForValid();

  args.print();

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  // Common steps for both methods
  _pub_odom         = nh.advertise<nav_msgs::Odometry>("pose", 1);
  _pub_path         = nh.advertise<nav_msgs::Path>("path", 1);
  _pub_cloud_global = nh.advertise<sensor_msgs::PointCloud2>("cloud/global", 1);
  _pub_cloud_latest = nh.advertise<sensor_msgs::PointCloud2>("cloud/latest", 1);

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

  /*//{ Fill transform buffer with all the transforms from the file/topic */
  ROS_INFO("Filling transform buffer.");
  rosbag::View tf_dynamic = rosbag::View(bag, rosbag::TopicQuery(std::vector<std::string>{"/tf"}));

  const double        rosbag_start_time_sec = tf_dynamic.getBeginTime().toSec() + args.start_time;
  const ros::Duration bag_duration          = tf_dynamic.getEndTime() - tf_dynamic.getBeginTime();
  const double        rosbag_end_time_sec   = tf_dynamic.getBeginTime().toSec() + std::fmin(bag_duration.toSec(), args.end_time);

  rosbag_start_time.fromSec(rosbag_start_time_sec);
  rosbag_end_time.fromSec(rosbag_end_time_sec);

  tf_buffer   = std::make_unique<tf2_ros::Buffer>(ros::Duration(bag_duration.toSec()));
  tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  ROS_INFO("Rosbag start time: %.1f s, duration: %.1f s", rosbag_start_time_sec, bag_duration.toSec());

  /*//{ Load poses from topic */
  if (!args.topic_poses.empty()) {
    ROS_INFO("Loading poses from topic: %s", args.topic_poses.c_str());
    rosbag::View view_poses = rosbag::View(bag, rosbag::TopicQuery(std::vector<std::string>{args.topic_poses}));

    for (const rosbag::MessageInstance &msg : view_poses) {

      if (!ros::ok()) {
        ros::shutdown();
        return -1;
      }

      if (msg.getTime() < rosbag_start_time || msg.getTime() > rosbag_end_time) {
        continue;
      }

      if (msg.getDataType() == "nav_msgs/Odometry") {

        const nav_msgs::Odometry::ConstPtr odom_msg = msg.instantiate<nav_msgs::Odometry>();
        if (odom_msg) {
          geometry_msgs::TransformStamped tf_msg;
          tf_msg.header                  = odom_msg->header;
          tf_msg.header.frame_id         = GLOBAL_FRAME;  // Overwrite for visualization purposes
          tf_msg.child_frame_id          = CHILD_FRAME;
          tf_msg.transform.rotation      = odom_msg->pose.pose.orientation;
          tf_msg.transform.translation.x = odom_msg->pose.pose.position.x;
          tf_msg.transform.translation.y = odom_msg->pose.pose.position.y;
          tf_msg.transform.translation.z = odom_msg->pose.pose.position.z;
          tf_buffer->setTransform(tf_msg, "default_authority", false);
        }

      } else if (msg.getDataType() == "geometry_msgs/PoseStamped") {

        const geometry_msgs::PoseStamped::ConstPtr pose_msg = msg.instantiate<geometry_msgs::PoseStamped>();
        if (pose_msg) {
          geometry_msgs::TransformStamped tf_msg;
          tf_msg.header                  = pose_msg->header;
          tf_msg.header.frame_id         = GLOBAL_FRAME;  // Overwrite for visualization purposes
          tf_msg.child_frame_id          = CHILD_FRAME;
          tf_msg.transform.rotation      = pose_msg->pose.orientation;
          tf_msg.transform.translation.x = pose_msg->pose.position.x;
          tf_msg.transform.translation.y = pose_msg->pose.position.y;
          tf_msg.transform.translation.z = pose_msg->pose.position.z;
          tf_buffer->setTransform(tf_msg, "default_authority", false);
        }

      } else {
        ROS_ERROR("Unsupported message type: %s", msg.getDataType().c_str());
        ros::shutdown();
        return -1;
      }
    }

  } else {

    const TRAJECTORY trajectory = loadTrajectoryFile(args.file_poses);
    for (const SAMPLE &sample : trajectory.samples) {

      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = ros::Time(sample.time);

      if (tf_msg.header.stamp < rosbag_start_time || tf_msg.header.stamp > rosbag_end_time) {
        continue;
      }

      tf_msg.header.frame_id = GLOBAL_FRAME;
      tf_msg.child_frame_id  = CHILD_FRAME;
      tf_msg.transform       = sample.tf;

      tf_buffer->setTransform(tf_msg, "default_authority", false);
    }
  }
  /*//}*/

  /*//}*/

  // Build cloud
  PC::Ptr cloud          = boost::make_shared<PC>();
  cloud->header.frame_id = GLOBAL_FRAME;
  bool succ              = concatenateClouds(cloud, bag, args);

  // Close rosbag
  bag.close();

  // Save cloud
  if (succ) {
    succ = saveCloud(cloud, args.file_cloud_out);
    if (!succ) {
      ROS_ERROR("Could not save cloud of size (%ld) to: %s", cloud->size(), args.file_cloud_out.c_str());
    }
  }

  return 0;
}
