#include <mrs_pcl_tools/support.h>

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>

const std::string TYPE_SENSOR_MSGS_PC2   = "sensor_msgs/PointCloud2";
const std::string TYPE_VIS_MSGS_MARK_ARR = "visualization_msgs/MarkerArray";
const std::string TYPE_OCTOMAP           = "octomap_msgs/Octomap";

// Default parameters
bool        use_only_last_message  = true;
bool        use_voxel_filter       = false;
bool        use_transform_to_frame = false;
float       voxel_resolution       = 0.0f;
std::string transform_frame        = "";

// Variables
std::unique_ptr<tf2_ros::Buffer>            tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;

struct DATA
{
  sensor_msgs::PointCloud2::Ptr   cloud;
  geometry_msgs::TransformStamped transform;
};

void printHelp() {
  ROS_ERROR("Syntax is: rosrun mrs_pcl_tools rosbag_cloud_to_pcd rosbag.bag topic output.pcd [...]");
  ROS_ERROR("Optional parameters:");
  ROS_ERROR("--use_all_messages; switches between all messages and the last one only (default)");
  ROS_ERROR("--transform_to_frame string; used if non-empty, needs data on /tf and /tf_static (default: \"\")");
  ROS_ERROR("--voxel_resolution float; used if >0 (default: 0.0)");
  ROS_ERROR("Supported topic types are: [%s, %s, %s]", TYPE_SENSOR_MSGS_PC2.c_str(), TYPE_VIS_MSGS_MARK_ARR.c_str(), TYPE_OCTOMAP.c_str());
}

/*//{ vis_msg_to_cloud_msg() */
sensor_msgs::PointCloud2::Ptr vis_msg_to_cloud_msg(const visualization_msgs::MarkerArray::ConstPtr& vis_msg) {
  sensor_msgs::PointCloud2::Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();

  const PC_RGB::Ptr cloud = boost::make_shared<PC_RGB>();
  for (const auto& marker : vis_msg->markers) {
    for (unsigned int i = 0; i < marker.points.size(); i++) {
      uint8_t r = 255.0f * marker.colors[i].r;
      uint8_t g = 255.0f * marker.colors[i].g;
      uint8_t b = 255.0f * marker.colors[i].b;

      pt_XYZRGB p(r, g, b);
      p.x = marker.points[i].x;
      p.y = marker.points[i].y;
      p.z = marker.points[i].z;
      cloud->points.push_back(p);
    }
  }

  cloud->is_dense = false;
  cloud->width    = cloud->points.size();
  cloud->height   = 1;

  pcl::toROSMsg(*cloud, *cloud_msg);
  cloud_msg->header = vis_msg->markers.at(0).header;
  return cloud_msg;
}
/*//}*/

/*//{ octomap_msg_to_cloud_msg() */
sensor_msgs::PointCloud2::Ptr octomap_msg_to_cloud_msg(const octomap_msgs::Octomap::ConstPtr& octomap_msg) {

  octomap::AbstractOcTree* tree_ptr;
  if (octomap_msg->binary) {
    tree_ptr = octomap_msgs::binaryMsgToMap(*octomap_msg);
  } else {
    tree_ptr = octomap_msgs::fullMsgToMap(*octomap_msg);
  }

  if (!tree_ptr) {
    return nullptr;
  }

  const std::shared_ptr<octomap::OcTree> octree  = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree_ptr));
  const auto&                            vis_msg = mrs_pcl_tools::visualization::getVisualizationMsg(octree, octomap_msg->header.frame_id);

  return vis_msg_to_cloud_msg(vis_msg);
}
/*//}*/

/*//{ loadParams() */
std::tuple<int, std::string, std::string, std::string> loadParams(int argc, char** argv) {

  if (argc < 4)
    return std::make_tuple(-1, std::string(), std::string(), std::string());

  for (int i = 4; i < argc;) {
    const std::string option = argv[i];

    int i_inc = 2;

    if (option == "--use_all_messages") {

      use_only_last_message = false;

      i_inc = 1;

    } else if (option == "--transform_to_frame") {

      if (i + 1 >= argc) {
        ROS_ERROR("Value of optional parameter (%s) unspecified. Check help.", option.c_str());
        return std::make_tuple(-1, std::string(), std::string(), std::string());
      }

      transform_frame        = std::string(argv[i + 1]);
      use_transform_to_frame = !transform_frame.empty();

    } else if (option == "--voxel_resolution") {

      if (i + 1 >= argc) {
        ROS_ERROR("Value of optional parameter (%s) unspecified. Check help.", option.c_str());
        return std::make_tuple(-1, std::string(), std::string(), std::string());
      }

      voxel_resolution = std::atof(argv[i + 1]);
      use_voxel_filter = voxel_resolution > 0.0f;

    } else {

      ROS_ERROR("Unknown option (%s). Check help.", option.c_str());
      return std::make_tuple(-1, std::string(), std::string(), std::string());
    }

    i += i_inc;
  }

  const std::string rosbag_filename = argv[1];
  const std::string topic           = argv[2];
  const std::string pcd_filename    = argv[3];
  return std::make_tuple(0, rosbag_filename, topic, pcd_filename);
}
/*//}*/

/*//{ processData() */
sensor_msgs::PointCloud2::Ptr processData(const std::vector<DATA>& data) {

  // TODO: Do not throw away color information
  // TODO: Include some registration ICP to fit better scan by scan

  const auto data_size = data.size();
  ROS_INFO("Processing:");
  ROS_INFO(" - number of data samples: %ld", data_size);
  ROS_INFO_COND(use_voxel_filter, " - voxel grid filter -> resolution: %0.2f m", voxel_resolution);
  ROS_INFO_COND(use_transform_to_frame, " - frame transformation -> to frame: %s", transform_frame.c_str());

  PC::Ptr cloud_whole = boost::make_shared<PC>();

  ros::Time start_time = ros::Time::now();
  for (int i = 0; i < data_size; i++) {

    if ((i + 1) % 500 == 0) {
      const ros::Time now = ros::Time::now();
      ROS_INFO("%i/%ld (took: %0.1f ms)", i, data_size, (now - start_time).toSec() * 1000.0);
      start_time = now;
    }

    const auto& dato = data.at(i);

    // Transform to the desired frame
    if (use_transform_to_frame)
      tf2::doTransform(*dato.cloud, *dato.cloud, dato.transform);

    PC::Ptr cloud = boost::make_shared<PC>();
    pcl::fromROSMsg(*dato.cloud, *cloud);

    *cloud_whole += *cloud;
  }

  if (use_voxel_filter) {
    ROS_INFO("Applying voxel filter. This may take a while.");
    cloud_whole = mrs_pcl_tools::filters::applyVoxelGridFilter(cloud_whole, voxel_resolution);
  }

  sensor_msgs::PointCloud2::Ptr cloud_whole_msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_whole, *cloud_whole_msg);
  return cloud_whole_msg;
}
/*//}*/

/*//{ main() */
int main(int argc, char** argv) {

  ros::init(argc, argv, "RosbagCloudToPCD");
  ros::NodeHandle nh = ros::NodeHandle("~");

  const auto& [ret, rosbag_filename, cloud_topic, pcd_filename] = loadParams(argc, argv);
  if (ret < 0) {
    printHelp();
    return ret;
  }

  // open rosbag
  rosbag::Bag bag;
  try {
    ROS_INFO("Opening rosbag: %s", rosbag_filename.c_str());
    bag.open(rosbag_filename, rosbag::bagmode::Read);
  }
  catch (...) {
    ROS_ERROR("Couldn't open rosbag: %s", rosbag_filename.c_str());
    return -2;
  }

  /*//{ Fill transform buffer with all the transforms */
  if (use_transform_to_frame) {

    std::vector<std::string> tf_topics    = {"/tf", "/tf_static"};
    rosbag::View             tf_view      = rosbag::View(bag, rosbag::TopicQuery(tf_topics));
    const ros::Duration      bag_duration = tf_view.getEndTime() - tf_view.getBeginTime();
    /* std::cout << "[Fusion]: ROS bag time: " << (bag_end_time - bag_begin_time).toSec() << "(s)" << std::endl; */

    tf_buffer   = std::make_unique<tf2_ros::Buffer>(ros::Duration(2.0 * bag_duration.toSec()));
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    BOOST_FOREACH (rosbag::MessageInstance const m, tf_view) {
      tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
      if (tf_msg) {
        const bool is_static = m.getTopic() == "/tf_static" || m.getTopic() == "tf_static";
        for (auto transform : tf_msg->transforms) {
          tf_buffer->setTransform(transform, "default_authority", is_static);
        }
      }
    }
  }
  /*//}*/

  /*//{ Read cloud messages */
  std::vector<DATA> data_vec;
  rosbag::View      view = rosbag::View(bag, rosbag::TopicQuery(cloud_topic));

  ROS_INFO("Reading cloud messages on cloud_topic: %s", cloud_topic.c_str());
  BOOST_FOREACH (rosbag::MessageInstance const msg, view) {

    DATA data;
    bool data_are_valid = false;

    if (msg.getDataType() == TYPE_SENSOR_MSGS_PC2) {
      const sensor_msgs::PointCloud2::Ptr cloud = msg.instantiate<sensor_msgs::PointCloud2>();
      if (cloud) {
        data.cloud     = cloud;
        data_are_valid = true;
      }
    } else if (msg.getDataType() == TYPE_VIS_MSGS_MARK_ARR) {
      const visualization_msgs::MarkerArray::ConstPtr ma = msg.instantiate<visualization_msgs::MarkerArray>();
      if (ma) {
        data.cloud     = vis_msg_to_cloud_msg(ma);
        data_are_valid = true;
      }
    } else if (msg.getDataType() == TYPE_OCTOMAP) {
      const octomap_msgs::Octomap::ConstPtr ma = msg.instantiate<octomap_msgs::Octomap>();

      if (ma) {
        const auto cloud = octomap_msg_to_cloud_msg(ma);
        if (cloud) {
          data.cloud     = cloud;
          data_are_valid = true;
        }
      }
    }

    if (data_are_valid) {
      ROS_INFO_ONCE("Found atleast one valid message of type (%s).", msg.getDataType().c_str());

      if (use_transform_to_frame) {
        try {
          data.transform = tf_buffer->lookupTransform(transform_frame, data.cloud->header.frame_id, data.cloud->header.stamp);
        }
        catch (tf2::TransformException ex) {
          ROS_WARN("Failed to lookup transform (from: %s, to: %s, time: %0.2f)", transform_frame.c_str(), data.cloud->header.frame_id.c_str(),
                   data.cloud->header.stamp.toSec());
          continue;
        }
      }

      if (use_only_last_message && !data_vec.empty()) {
        data_vec.at(0) = data;
      } else {
        data_vec.push_back(data);
      }
    }
  }
  bag.close();

  if (data_vec.empty()) {
    ROS_ERROR("Did not find any message of type (%s) on topic (%s).", TYPE_VIS_MSGS_MARK_ARR.c_str(), cloud_topic.c_str());
    return -1;
  }
  /*//}*/

  const auto& cloud = processData(data_vec);
  mrs_pcl_tools::savePCD(pcd_filename, cloud);
  return 0;
}
/*//}*/
