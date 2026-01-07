#pragma once

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>

namespace mrs_pcl_tools
{
  using vec3_t = Eigen::Vector3f;
  using vec4_t = Eigen::Vector4f;
  using quat_t = Eigen::Quaternionf;

  struct RangeClipConfig
  {
    bool use;
    float min_sq;
    float max_sq;
    uint32_t min_mm;
    uint32_t max_mm;
  };

  struct InertialClipConfig
  {
    bool use;
    vec4_t min;
    vec4_t max;
  };

  struct IntensityFilterConfig
  {
    bool use;
    float range_sq;
    uint32_t range_mm;
    float threshold;
  };

  struct ReflectivityFilterConfig
  {
    bool use;
    float range_sq;
    uint32_t range_mm;
    uint16_t threshold;
  };

  struct CropBoxConfig
  {
    bool use;
    bool crop_inside;
    std::string frame_id;
    vec3_t min;
    vec3_t max;
  };

  struct DownsampleConfig
  {
    bool use;
    uint32_t dynamic_row_offset;
    bool dynamic_row_selection_enabled;
    int row_step;
    int col_step;
  };

  struct Lidar3DConfig
  {
    std::string name;
    float frequency;
    float vfov;
    bool keep_organized;
    bool republish;
    float invalid_value;

    RangeClipConfig rangeclip;
    InertialClipConfig inertclip;
    IntensityFilterConfig intensity;
    ReflectivityFilterConfig reflectivity;
    CropBoxConfig cropbox;
    DownsampleConfig downsample;
  };

}  // namespace mrs_pcl_tools