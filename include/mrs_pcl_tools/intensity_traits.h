#pragma once

#include <mrs_pcl_tools/support.h>

/*
Encapsulate differences between processing float, uint16_t
and uint8_t intensities in RGBDI data
*/

namespace mrs_pcl_tools
{

  // Primary template
  template <typename T>
  struct IntensityTraits
  {
  };

  /*//{ IntensityTraits<uint8_t> */
  template <>
  struct IntensityTraits<uint8_t>
  {
    static inline bool valid(uint8_t intensity)
    {
      return intensity != 0;
    }
    static inline float toFloat(uint8_t intensity)
    {
      return float(intensity);
    }
    static inline uint8_t fromFloat(float intensity)
    {
      return intensity + 0.5f;
    }
  };
  /*//}*/

  /*//{ IntensityTraits<uint16_t> */
  template <>
  struct IntensityTraits<uint16_t>
  {
    static inline bool valid(uint16_t intensity)
    {
      return intensity != 0;
    }
    static inline float toFloat(uint16_t intensity)
    {
      return float(intensity);
    }
    static inline uint16_t fromFloat(float intensity)
    {
      return intensity + 0.5f;
    }
  };
  /*//}*/

  /*//{ IntensityTraits<float> */
  template <>
  struct IntensityTraits<float>
  {
    static inline bool valid(float intensity)
    {
      return std::isfinite(intensity);
    }
    static inline float toFloat(float intensity)
    {
      return intensity;
    }
    static inline float fromFloat(float intensity)
    {
      return intensity;
    }
  };
  /*//}*/

}  // namespace mrs_pcl_tools