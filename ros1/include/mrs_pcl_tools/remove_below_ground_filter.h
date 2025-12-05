#pragma once

/* includes //{ */

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <pcl_conversions/pcl_conversions.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/scope_timer.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <mrs_modules_msgs/msg/pcl_tools_diagnostics.hpp>

#include <boost/smart_ptr/make_shared_array.hpp>
#include <limits>

#include <tf2_eigen/tf2_eigen.h>

#include <mrs_pcl_tools/common_includes_and_typedefs.h>
#include <mrs_pcl_tools/groundplane_detector.h>

//}

namespace mrs_pcl_tools
{
using vec3_t = Eigen::Vector3f;
using vec4_t = Eigen::Vector4f;
using quat_t = Eigen::Quaternionf;

struct CommonHandlers_t;

/* class RemoveBelowGroundFilter //{ */

class RemoveBelowGroundFilter{
public:
  void initialize(rclcpp::Node::SharedPtr nh_, const std::shared_ptr<CommonHandlers_t> common_handlers);

  bool used() const {
    return initialized;
  }

  template <typename PC>
  typename boost::shared_ptr<PC> applyInPlace(const typename boost::shared_ptr<PC>& inout_pc, const bool return_removed = false);

private:
  bool initialized = false;

  GroundplaneDetector m_ground_detector;

  std::shared_ptr<mrs_lib::Transformer> transformer = nullptr;

  bool   keep_organized = false;
  double plane_offset   = 1.0;  // metres
};

#include <mrs_pcl_tools/impl/remove_below_ground_filter.hpp>

//}

}  // namespace mrs_pcl_tools
