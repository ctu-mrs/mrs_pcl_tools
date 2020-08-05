#include <mrs_pcl_tools/support.h>

#include <visualization_msgs/MarkerArray.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <variant>

const std::string TYPE_SENSOR_MSGS_PC2   = "sensor_msgs/PointCloud2";
const std::string TYPE_VIS_MSGS_MARK_ARR = "visualization_msgs/MarkerArray";

struct DATA
{
  sensor_msgs::PointCloud2::ConstPtr cloud;
  ros::Time                          t;
  bool                               cloud_valid;
};

void printHelp() {
  ROS_ERROR("Syntax is: rosrun mrs_pcl_tools rosbag_cloud_to_pcd rosbag.bag output.pcd topic");
  ROS_ERROR("Supported topic types are: [%s, %s]", TYPE_SENSOR_MSGS_PC2.c_str(), TYPE_VIS_MSGS_MARK_ARR.c_str());
  ROS_ERROR("Note: only last message on the topic will be saved to output.pcd");
}

sensor_msgs::PointCloud2::Ptr vis_msg_to_cloud_msg(visualization_msgs::MarkerArray::ConstPtr vis_msg) {
  sensor_msgs::PointCloud2::Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();

  PC_RGB::Ptr cloud = boost::make_shared<PC_RGB>();
  for (auto marker : vis_msg->markers) {
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
  return cloud_msg;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    printHelp();
    return -1;
  }

  ros::init(argc, argv, "RosbagCloudToPCD");

  // open rosbag
  const std::string rosbag_filename = argv[1];
  const std::string pcd_filename    = argv[2];
  const std::string topic           = argv[3];
  rosbag::Bag       bag;
  try {
    ROS_INFO("Opening rosbag: %s", rosbag_filename.c_str());
    bag.open(rosbag_filename, rosbag::bagmode::Read);
  }
  catch (...) {
    ROS_ERROR("Couldn't open rosbag: %s", rosbag_filename.c_str());
    return -2;
  }

  // read messages
  DATA         data;
  rosbag::View view = rosbag::View(bag, rosbag::TopicQuery(topic));
  ROS_INFO("Reading topic: %s", topic.c_str());
  BOOST_FOREACH (rosbag::MessageInstance const msg, view) {
    if (msg.getDataType() == TYPE_SENSOR_MSGS_PC2) {
      sensor_msgs::PointCloud2::ConstPtr cloud = msg.instantiate<sensor_msgs::PointCloud2>();
      if (cloud) {
        ROS_INFO_ONCE("Found atleast one valid %s message.", TYPE_SENSOR_MSGS_PC2.c_str());
        data.cloud       = cloud;
        data.t           = data.cloud->header.stamp;
        data.cloud_valid = true;
      }
    } else if (msg.getDataType() == TYPE_VIS_MSGS_MARK_ARR) {
      visualization_msgs::MarkerArray::ConstPtr ma = msg.instantiate<visualization_msgs::MarkerArray>();
      if (ma) {
        ROS_INFO_ONCE("Found atleast one valid %s message.", TYPE_VIS_MSGS_MARK_ARR.c_str());
        data.cloud       = vis_msg_to_cloud_msg(ma);
        data.t           = data.cloud->header.stamp;
        data.cloud_valid = true;
      }
    }
  }
  bag.close();

  if (data.cloud_valid) {
    mrs_pcl_tools::savePCD(pcd_filename, data.cloud);
  }

  return 0;
}
