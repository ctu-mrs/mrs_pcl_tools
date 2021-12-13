#include <mrs_pcl_tools/support.h>

/*//{ printHelp() */
void printHelp() {
  ROS_ERROR("Normals will be estimated for given PCD file.");
  ROS_ERROR("Usage:");
  ROS_ERROR("   rosrun mrs_pcl_tools pcd_estimate_normals input.pcd output.pcd estimation_radius");

  ROS_ERROR("Arguments:");
  ROS_ERROR(" input.pcd:         input PCD file");
  ROS_ERROR(" output.pcd:        output PCD file");
  ROS_ERROR(" estimation_radius: normal estimation radius (float)");
}
/*//}*/

/*//{ struct ARGUMENTS */
struct ARGUMENTS
{
  bool        initialized = false;
  std::string pcd_input;
  std::string pcd_output;
  float       estimation_radius;

  void print() {
    if (initialized) {
      ROS_INFO("Input PCD: %s", pcd_input.c_str());
      ROS_INFO("Output PCD: %s", pcd_output.c_str());
      ROS_INFO("Normal estimation radius: %0.2f", estimation_radius);
    }
  }
};
/*//}*/

/*//{ parseArguments() */
bool parseArguments(int argc, char **argv, ARGUMENTS &args) {
  if (argc < 4) {
    printHelp();
    return false;
  }

  args.pcd_input         = argv[1];
  args.pcd_output        = argv[2];
  args.estimation_radius = float(std::atof(argv[3]));

  args.initialized = true;

  return true;
}
/*//}*/

/* -------------------- Functions -------------------- */
int main(int argc, char **argv) {

  ARGUMENTS args;
  if (!parseArguments(argc, argv, args)) {
    return -1;
  }

  ros::init(argc, argv, "PcdEstimateNormals");

  args.print();

  const auto ret_input = mrs_pcl_tools::loadPcXYZ(args.pcd_input);
  if (!ret_input) {
    ROS_ERROR("Could not load input PCD file.");
    return -1;
  }

  ROS_INFO("[PCL2MapRegistration] Estimating normals of the input PCD.");
  const PC_NORM::Ptr pc_norm = mrs_pcl_tools::estimateNormals(ret_input.value(), args.estimation_radius);

  const sensor_msgs::PointCloud2::Ptr pc_norm_msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*pc_norm, *pc_norm_msg);

  mrs_pcl_tools::savePCD(args.pcd_output, pc_norm_msg);

  return 0;
}
