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
    PCLFiltrationCore(ILogger& logger) : m_logger(logger)
    {
    }


    template <typename PC>
    void cropBoxPointCloud(const std::shared_ptr<PC>& inout_pc_ptr, const Eigen::Affine3f& tf);

  private:
    ILogger& m_logger;
    Lidar3DConfig m_lidar_params;
  };


  template <typename PC>
  void PCLFiltrationCore::cropBoxPointCloud(const std::shared_ptr<PC>& inout_pc_ptr, const Eigen::Affine3f& tf)
  {
    // TODO: ros version has to do rest of the stuff
    if (!m_lidar_params.cropbox.frame_id.empty())
    {
      auto filtered_cloud = mrs_pcl_tools::filters::applyCropBox<PC>(m_logger, inout_pc_ptr, tf, m_lidar_params.cropbox.min, m_lidar_params.cropbox.max,
                                                                     m_lidar_params.keep_organized, m_lidar_params.cropbox.crop_inside);
    }
  }


}  // namespace mrs_pcl_tools
