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

  struct LowFieldFilterConfig
  {
    bool range_exists{false};
    std::size_t range_offset{0};

    bool intensity_exists{false};
    std::size_t intensity_offset{0};

    bool reflectivity_exists{false};
    std::size_t reflectivity_offset{0};
  };

  struct RangeInfo
  {
    bool has_mm = false;  // true => range_mm valid
    uint32_t range_mm = 0;
    float range_sq = 0.f;  // always valid
  };


  class PCLFiltrationCore
  {
  public:
    PCLFiltrationCore(ILogger& logger);

    void loadLidarParams(const Lidar3DConfig& params);
    void updateDownsampleParams(DownsampleConfig& downsample_params);

    template <typename PC>
    void downsample(std::shared_ptr<PC>& inout_pc_ptr, const DownsampleConfig& downsample_params);

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
    std::shared_ptr<PC> removeLowFields(std::shared_ptr<PC>& inout_pc_ptr, const bool return_removed);

  private:
    template <typename PC>
    LowFieldFilterConfig m_buildLowFieldConfig() const;

    template <typename PointT>
    RangeInfo m_computeRangeInfo(const PointT& point, bool range_exists, std::size_t range_offset) const;

    template <typename PointT>
    std::tuple<bool, bool, bool> m_evaluateInvalidation(const PointT& point, const LowFieldFilterConfig& cfg, const RangeInfo& range_info) const;

  private:
    ILogger& m_logger;
    Lidar3DConfig m_lidar_params;
  };

}  // namespace mrs_pcl_tools

#include <mrs_pcl_tools/pcl_filtration_core.tpp>
