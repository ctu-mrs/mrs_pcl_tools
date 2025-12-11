#pragma once

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>
#include <std_msgs/msg/color_rgba.hpp>

namespace mrs_pcl_tools
{

  namespace visualization
  {

    PC_RGB::Ptr colorizeCloud(const PC::Ptr& cloud_xyz);
    std_msgs::msg::ColorRGBA heightToRGBA(double& height, const double& alpha = 1.0);

  }  // namespace visualization

}  // namespace mrs_pcl_tools
