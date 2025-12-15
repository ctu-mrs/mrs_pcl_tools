#pragma once

/* includes //{ */
#include <mrs_pcl_tools/support.h>
#include <mrs_pcl_tools/lidar_3d_config.h>
//}

namespace mrs_pcl_tools
{
  using vec3_t = Eigen::Vector3f;
  using vec4_t = Eigen::Vector4f;
  using quat_t = Eigen::Quaternionf;

  class PCLFiltrationCore
  {
  public:
    PCLFiltrationCore(ILogger& logger);

    void loadLidarParams(const Lidar3DConfig& params);

    template <typename PC>
    void downsample(std::shared_ptr<PC>& inout_pc_ptr, const size_t scale_row, const size_t scale_col, const size_t row_offset);

    template <typename PC>
    void removeInfinitePoints(std::shared_ptr<PC>& inout_pc_ptr);

    template <typename pt_t>
    void invalidatePoint(pt_t& point);

    template <typename PC>
    void invalidatePointsAtIndices(const pcl::IndicesConstPtr& indices, std::shared_ptr<PC>& cloud);

    template <typename PC>
    void cropBoxPointCloud(std::shared_ptr<PC>& inout_pc_ptr, const Eigen::Affine3f& tf);

    template <typename PC>
    std::shared_ptr<PC> removeCloseAndFar(std::shared_ptr<PC>& inout_pc_ptr, const bool return_removed_close, const bool return_removed_far);

    template <typename PC>
    std::shared_ptr<PC> removeCloseAndFarAndLowFields(std::shared_ptr<PC>& inout_pc_ptr, const bool clip_return_removed_close,
                                                      const bool clip_return_removed_far);

    template <typename PC>
    std::shared_ptr<PC> removeLowFields(std::shared_ptr<PC>& inout_pc_ptr);

  private:
    ILogger& m_logger;
    Lidar3DConfig m_lidar_params;
  };


#include <mrs_pcl_tools/pcl_filtration_core.tpp>

}  // namespace mrs_pcl_tools
