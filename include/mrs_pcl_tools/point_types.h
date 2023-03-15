#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace pandar_pointcloud
{
/** Euclidean Pandar128 coordinate, including intensity and ring number. */
struct PointXYZIR
{
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float         intensity;         ///< laser intensity reading
  std::uint16_t ring;              ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;


struct PointXYZIT
{
  PCL_ADD_POINT4D
  float         intensity;
  double        timestamp;
  std::uint16_t ring;              ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

struct PointXYZITd
{
  double       x;
  double       y;
  double       z;
  std::uint8_t intensity;
  double       timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

struct PointXYZd
{
  double x;
  double y;
  double z;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

struct PointXYZRGBd
{
  double x;
  double y;
  double z;
  PCL_ADD_RGB
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
};  // namespace pandar_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZITd,
                                  (double, x, x)(double, y, y)(double, z, z)(std::uint8_t, intensity, intensity)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZd, (double, x, x)(double, y, y)(double, z, z))
POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZRGBd, (double, x, x)(double, y, y)(double, z, z)(float, rgb, rgb))
