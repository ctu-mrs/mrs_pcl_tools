#include "mrs_pcl_tools/support.h"

namespace mrs_pcl_tools
{

/*//{ applyVoxelGridFilter() */
void applyVoxelGridFilter(const PC_NORM::Ptr &cloud_in, PC_NORM::Ptr &cloud_out, const float &leaf_size) {
  pcl::VoxelGrid<pt_NORM> grid;
  grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  grid.setInputCloud(cloud_in);
  grid.filter(*cloud_out);
}

void applyVoxelGridFilter(PC_NORM::Ptr &cloud, const float &leaf_size) {
  applyVoxelGridFilter(cloud, cloud, leaf_size);
}
/*//}*/

/*//{ loadPcXYZ() */
std::optional<PC::Ptr> loadPcXYZ(const std::string &pcd_file) {

  PC::Ptr pc = boost::make_shared<PC>();

  ROS_INFO("[PCLSupportLibrary] Reading pointcloud from path %s", pcd_file.c_str());
  if (pcl::io::loadPCDFile<pt_XYZ>(pcd_file, *pc) < 0) {
    ROS_ERROR("[PCLSupportLibrary] Couldn't read PCD file from path: %s.", pcd_file.c_str());
    return nullptr;
  }
  ROS_INFO("[PCLSupportLibrary] Loaded XYZ pcl with %ld points.", pc->points.size());

  return pc;
}
/*//}*/

/*//{ loadPcNormals() */
std::optional<PC_NORM::Ptr> loadPcNormals(const std::string &pcd_file) {
  // Check if normals are present in the PCD file by looking at the header
  if (!hasNormals(pcd_file)) {
    return nullptr;
  }

  // Load PCD file
  PC_NORM::Ptr cloud = boost::make_shared<PC_NORM>();
  ROS_INFO("[PCLSupportLibrary] Loading normals from PCD file: %s.", pcd_file.c_str());
  if (pcl::io::loadPCDFile(pcd_file, *cloud) < 0) {
    ROS_ERROR("[PCLSupportLibrary] Couldn't read normals of PCD file: %s.", pcd_file.c_str());
    return nullptr;
  }

  ROS_INFO("[PCLSupportLibrary] Loaded PCL normals with %ld points.", cloud->points.size());
  return cloud;
}
/*//}*/

/*//{ savePCD() */
void savePCD(const std::string &pcd_file, sensor_msgs::PointCloud2::ConstPtr &cloud_msg, const bool &binary) {
  ROS_INFO("[PCLSupportLibrary] Saving PCD file (%s): %s", (binary) ? "binary" : "ascii", pcd_file.c_str());
  pcl::PCLPointCloud2 cloud;
  pcl_conversions::toPCL(*cloud_msg, cloud);
  pcl::io::savePCDFile(pcd_file, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binary);
}
/*//}*/

/*//{ estimateNormals() */
PC_NORM::Ptr estimateNormals(const PC::Ptr &cloud, const float &normal_est_radius) {

  // XYZ type to XYZNormalPoint
  PC_NORM::Ptr cloud_norm = boost::make_shared<PC_NORM>();
  pcl::copyPointCloud(*cloud, *cloud_norm);

  // Estimate normals
  pcl::NormalEstimationOMP<pt_NORM, pt_NORM> nest;
  nest.setRadiusSearch(normal_est_radius);
  nest.setInputCloud(cloud_norm);
  nest.compute(*cloud_norm);

  return cloud_norm;
}
/*//}*/

/*//{ hasNormals() */
bool hasNormals(const std::vector<sensor_msgs::PointField> &fields) {
  // Check header for normals (normal_x, normal_y, normal_z, curvature)
  unsigned int normal_fields   = 0;
  bool         curvature_field = false;
  for (auto field : fields) {
    if (field.name.rfind("normal", 0) == 0) {
      normal_fields++;
    } else if (field.name == "curvature") {
      curvature_field = true;
    }
  }
  if (normal_fields != 3 || !curvature_field) {
    return false;
  }

  return true;
}

bool hasNormals(const std::string &pcd_file) {

  // Read header of PCD file
  pcl::PCDReader      reader_pcd;
  pcl::PCLPointCloud2 pc;
  if (reader_pcd.readHeader(pcd_file, pc) < 0) {
    ROS_ERROR("[PCLSupportLibrary] Couldn't read header of PCD file: %s.", pcd_file.c_str());
    return false;
  }

  // pcl::PCLPointField to sensor_msgs::PointField
  std::vector<sensor_msgs::PointField> fields;
  for (auto pc_field : pc.fields) {
    sensor_msgs::PointField field;
    field.name = pc_field.name;
    fields.push_back(field);
  }

  return hasNormals(fields);
}

bool hasNormals(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
  return hasNormals(cloud->fields);
}
/*//}*/

/*//{ hasField() */
bool hasField(const std::string &field, const sensor_msgs::PointCloud2::ConstPtr &msg) {
  for (auto f : msg->fields) {
    if (f.name == field) {
      return true;
    }
  }
  return false;
}
/*//}*/

/*//{ publishCloud() */
void publishCloud(const ros::Publisher &pub, const PC_NORM::Ptr &cloud) {
  if (pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud, *cloud_msg);
    publishCloudMsg(pub, cloud_msg);
  }
}
/*//}*/

/*//{ publishCloud() */
void publishCloud(const ros::Publisher &pub, const PC::Ptr &cloud) {
  if (pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud, *cloud_msg);
    publishCloudMsg(pub, cloud_msg);
  }
}
/*//}*/

/*//{ publishCloudMsg() */
void publishCloudMsg(const ros::Publisher &pub, const sensor_msgs::PointCloud2::Ptr &cloud_msg) {
  try {
    pub.publish(cloud_msg);
  }
  catch (...) {
    ROS_ERROR("[PCLSupportLibrary]: Exception caught during publishing on topic: %s", pub.getTopic().c_str());
  }
}
/*//}*/

/*//{ printEigenMatrix() */
void printEigenMatrix(const Eigen::Matrix4f &mat, const std::string &prefix) {
  const std::string          st = (prefix.size() > 0) ? prefix : "Eigen matrix:";
  mrs_lib::AttitudeConverter atti(mat.block<3, 3>(0, 0).cast<double>());
  ROS_INFO("[PCLSupportLibrary] %s", st.c_str());
  ROS_INFO("      | %2.3f %2.3f %2.3f |", mat(0, 0), mat(0, 1), mat(0, 2));
  ROS_INFO("R   = | %2.3f %2.3f %2.3f |", mat(1, 0), mat(1, 1), mat(1, 2));
  ROS_INFO("      | %2.3f %2.3f %2.3f |", mat(2, 0), mat(2, 1), mat(2, 2));
  ROS_INFO("RPY = < %2.3f, %2.3f, %2.3f >", atti.getRoll(), atti.getPitch(), atti.getYaw());
  ROS_INFO("t   = < %2.3f, %2.3f, %2.3f >", mat(0, 3), mat(1, 3), mat(2, 3));
}
/*//}*/

/*//{ getRotationMatrixAroundPoint() */
Eigen::Matrix4f getRotationMatrixAroundPoint(const Eigen::Matrix3f &rotation, const Eigen::Vector4f &point) {
  Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T2 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T3 = Eigen::Matrix4f::Identity();

  // To the point
  T1(0, 3) = -point.x();
  T1(1, 3) = -point.y();
  T1(2, 3) = -point.z();

  // Rotate
  T2.block<3, 3>(0, 0) = rotation;

  // Back to the origin
  T3(0, 3) = point.x();
  T3(1, 3) = point.y();
  T3(2, 3) = point.z();

  return T3 * T2 * T1;
}
/*//}*/

/*//{ PcToOctree() */
std::shared_ptr<octomap::OcTree> PcToOctree(const PC::ConstPtr &cloud, const float &resolution) {
  if (cloud->points.size() == 0) {
    ROS_WARN("[%s]: Cannot create octree from 0 points.", ros::this_node::getName().c_str());
    return nullptr;
  } else if (!std::isfinite(resolution) || resolution <= 0.0f) {
    ROS_ERROR("[%s]: Undefined resolution in mrs_pcl_tools::PcToOctree().", ros::this_node::getName().c_str());
    return nullptr;
  }

  std::shared_ptr<octomap::OcTree> tree = std::make_shared<octomap::OcTree>(resolution);
  for (auto p : cloud->points) {
    tree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
  }
  tree->updateInnerOccupancy();
  return tree;
}
/*//}*/

/*//{ overload PcToOctree() */
std::shared_ptr<octomap::OcTree> PcToOctree(const sensor_msgs::PointCloud2::ConstPtr &cloud, const float &resolution) {
  PC::Ptr pc = boost::make_shared<PC>();
  pcl::fromROSMsg(*cloud, *pc);

  return PcToOctree(pc, resolution);
}
/*//}*/

namespace visualization
{
/*//{ colorizeCloud() */
PC_RGB::Ptr colorizeCloud(const PC::Ptr &cloud_xyz) {
  pt_XYZ min_xyz;
  pt_XYZ max_xyz;
  pcl::getMinMax3D(*cloud_xyz, min_xyz, max_xyz);
  const float min_z = min_xyz.z;
  const float max_z = max_xyz.z;

  const unsigned int size      = cloud_xyz->points.size();
  PC_RGB::Ptr        cloud_rgb = boost::make_shared<PC_RGB>();
  cloud_rgb->points.resize(size);
  cloud_rgb->width    = size;
  cloud_rgb->height   = 1;
  cloud_rgb->is_dense = false;

  for (unsigned int i = 0; i < size; i++) {
    const float         z      = cloud_xyz->points.at(i).z;
    double              height = (1.0 - std::fmin(std::fmax((z - min_z) / (max_z - min_z), 0.0f), 1.0f));
    std_msgs::ColorRGBA color  = heightToRGBA(height, 1.0);
    pt_XYZRGB           p;
    p.x              = cloud_xyz->points.at(i).x;
    p.y              = cloud_xyz->points.at(i).y;
    p.z              = z;
    p.r              = color.r;
    p.g              = color.g;
    p.b              = color.b;
    p.a              = 1.0;
    cloud_rgb->at(i) = p;
  }

  cloud_rgb->header = cloud_xyz->header;
  return cloud_rgb;
}
/*//}*/

/* //{ getVisualizationMsg() */
visualization_msgs::MarkerArray::Ptr getVisualizationMsg(const std::shared_ptr<octomap::OcTree> &octree, const std::string &frame_id) {
  visualization_msgs::MarkerArray::Ptr msg = boost::make_shared<visualization_msgs::MarkerArray>();

  const int    tree_depth = octree->getTreeDepth();
  double       min_x;
  double       min_y;
  double       min_z;
  double       max_x;
  double       max_y;
  double       max_z;
  const double visualization_color_factor = 1.0;

  msg->markers.resize(tree_depth + 1);
  octree->getMetricMin(min_x, min_y, min_z);
  octree->getMetricMax(max_x, max_y, max_z);

  for (octomap::OcTree::iterator it = octree->begin(tree_depth), end = octree->end(); it != end; ++it) {
    if (octree->isNodeOccupied(*it)) {
      const unsigned int idx = it.getDepth();

      geometry_msgs::Point cube_center;
      cube_center.x = it.getX();
      cube_center.y = it.getY();
      cube_center.z = it.getZ();

      double height = (1.0 - std::min(std::max((cube_center.z - min_z) / (max_z - min_z), 0.0), 1.0)) * visualization_color_factor;
      msg->markers[idx].colors.push_back(heightToRGBA(height));
      msg->markers[idx].points.push_back(cube_center);
    }
  }

  const ros::Time ros_time = ros::Time::now();
  for (unsigned i = 0; i < msg->markers.size(); ++i) {
    const double size               = octree->getNodeSize(i);
    msg->markers[i].header.frame_id = frame_id;
    msg->markers[i].header.stamp    = ros_time;
    msg->markers[i].ns              = "marker";
    msg->markers[i].id              = i;
    msg->markers[i].type            = visualization_msgs::Marker::CUBE_LIST;
    msg->markers[i].scale.x         = size;
    msg->markers[i].scale.y         = size;
    msg->markers[i].scale.z         = size;
    if (msg->markers[i].points.size() > 0) {
      msg->markers[i].action = visualization_msgs::Marker::ADD;
    } else {
      msg->markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }

  return msg;
}
//}/*

/* //{ overload getVisualizationMsg() */
visualization_msgs::MarkerArray::Ptr getVisualizationMsg(const sensor_msgs::PointCloud2::Ptr &cloud, const float &resolution, const std::string &frame_id) {
  return getVisualizationMsg(PcToOctree(cloud, resolution), frame_id);
}
visualization_msgs::MarkerArray::Ptr getVisualizationMsg(const PC::Ptr &cloud, const float &resolution, const std::string &frame_id) {
  return getVisualizationMsg(PcToOctree(cloud, resolution), frame_id);
}
//}/*

/*//{ heightToRGBA() */
std_msgs::ColorRGBA heightToRGBA(double &h, const double &a) {

  std_msgs::ColorRGBA color;
  color.a = a;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int    i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}
/*//}*/

}  // namespace visualization

}  // namespace mrs_pcl_tools
