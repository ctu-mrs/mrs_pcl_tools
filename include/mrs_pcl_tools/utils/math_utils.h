#pragma once

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>


namespace mrs_pcl_tools
{

void printEigenMatrix(const Eigen::Matrix4f& mat, const std::string& prefix = "");

Eigen::Matrix4f getRotationMatrixAroundPoint(const Eigen::Matrix3f& rotation, const Eigen::Vector4f& point);

}  // namespace mrs_pcl_tools