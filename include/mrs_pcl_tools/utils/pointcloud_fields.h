#pragma once

#include <tuple>

#include <mrs_pcl_tools/utils/common_includes_and_typedefs.h>

namespace mrs_pcl_tools
{

template <typename pt_t>
std::tuple<bool, std::size_t> getFieldOffset(const std::string& field_name);

template <typename T, typename pt_t>
T getFieldValue(const pt_t& point, std::size_t field_offset);

}  // namespace mrs_pcl_tools

#include <mrs_pcl_tools/utils/pointcloud_fields.tpp>