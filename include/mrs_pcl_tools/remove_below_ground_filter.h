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

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Transform.h>

#include <visualization_msgs/MarkerArray.h>

#include <mrs_msgs/PclToolsDiagnostics.h>

#include <boost/smart_ptr/make_shared_array.hpp>
#include <limits>

#include <tf2_eigen/tf2_eigen.h>

#include "mrs_pcl_tools/common_includes.h"
#include "mrs_pcl_tools/groundplane_detector.h"

//}

namespace mrs_pcl_tools
{
  using vec3_t = Eigen::Vector3f;
  using vec4_t = Eigen::Vector4f;
  using quat_t = Eigen::Quaternionf;

  struct CommonHandlers_t;

  /* class RemoveBelowGroundFilter //{ */

  class RemoveBelowGroundFilter
  {
  public:
    void initialize(ros::NodeHandle& nh, const std::shared_ptr<CommonHandlers_t> common_handlers);

    bool used() const
    {
      return initialized;
    }

    template <typename PC>
    typename boost::shared_ptr<PC> applyInPlace(const typename boost::shared_ptr<PC>& inout_pc, const bool return_removed = false);

  private:
    bool initialized = false;

    GroundplaneDetector m_ground_detector;

    std::shared_ptr<mrs_lib::Transformer> transformer = nullptr;

    bool keep_organized = false;
    double plane_offset = 1.0;  // metres
  };

#include <impl/remove_below_ground_filter.hpp>

  //}

}  // namespace mrs_pcl_tools
