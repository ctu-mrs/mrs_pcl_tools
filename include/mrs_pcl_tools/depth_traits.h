#pragma once

#include <mrs_pcl_tools/support.h>

/*
Encapsulate differences between processing float
and uint16_t depths in RGBD data
*/

namespace mrs_pcl_tools
{


  // Primary template
  template <typename T>
  struct DepthTraits
  {
  };

  /*//{ DepthTraits<uint8_t> */
  template <>
  struct DepthTraits<uint8_t>
  {
    static inline bool valid(uint8_t depth)
    {
      return depth != 0;
    }
    static inline float toMeters(uint8_t depth)
    {
      return float(depth) * 0.001f;
    }  // originally mm
    static inline uint8_t fromMeters(float depth)
    {
      return (depth * 1000.0f) + 0.5f;
    }
  };
  /*//}*/

  /*//{ DepthTraits<uint16_t> */
  template <>
  struct DepthTraits<uint16_t>
  {
    static inline bool valid(uint16_t depth)
    {
      return depth != 0;
    }
    static inline float toMeters(uint16_t depth)
    {
      return float(depth) * 0.001f;
    }  // originally mm
    static inline uint16_t fromMeters(float depth)
    {
      return (depth * 1000.0f) + 0.5f;
    }
  };
  /*//}*/

  /*//{ DepthTraits<float> */
  template <>
  struct DepthTraits<float>
  {
    static inline bool valid(float depth)
    {
      return std::isfinite(depth);
    }
    static inline float toMeters(float depth)
    {
      return depth;
    }
    static inline float fromMeters(float depth)
    {
      return depth;
    }
  };
  /*//}*/


}  // namespace mrs_pcl_tools
