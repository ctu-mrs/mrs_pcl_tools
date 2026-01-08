#include <mrs_pcl_tools/pcl_filtration_core.h>

namespace mrs_pcl_tools
{
  /*//{ PCLFiltrationCore constructor */
  PCLFiltrationCore::PCLFiltrationCore(ILogger& logger) : m_logger(logger)
  {
  }
  /*//}*/

  // TODO: remove this and put it in constructor later on
  /*//{ loadLidarParams() */
  void PCLFiltrationCore::loadLidarParams(const Lidar3DConfig& params)
  {
    m_lidar_params = params;
  }
  /*//}*/

  /*//{ updateDownsampleParams() */
  void PCLFiltrationCore::updateDownsampleParams(DownsampleConfig& dp)
  {
    if (dp.dynamic_row_selection_enabled && dp.row_step > 0)
    {
      dp.dynamic_row_offset++;
      dp.dynamic_row_offset %= dp.row_step;
    }
  }
  /*//}*/

}  // namespace mrs_pcl_tools