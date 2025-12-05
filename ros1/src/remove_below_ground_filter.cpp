#include <mrs_pcl_tools/remove_below_ground_filter.h>

namespace mrs_pcl_tools
{

  void RemoveBelowGroundFilter::initialize(ros::NodeHandle& nh, const std::shared_ptr<CommonHandlers_t> common_handlers)
  {
    common_handlers->param_loader->loadParamReusable<bool>("keep_organized", keep_organized, false);
    common_handlers->param_loader->loadParam("ground_removal/plane_offset", plane_offset, 1.0);
    const auto pfx = common_handlers->param_loader->getPrefix();

    if (common_handlers->transformer == nullptr)
    {
      common_handlers->param_loader->setPrefix("");
      const auto uav_name = common_handlers->param_loader->loadParamReusable2<std::string>("uav_name");
      common_handlers->param_loader->setPrefix(pfx);
      this->transformer = std::make_shared<mrs_lib::Transformer>("RemoveBelowGroundFilter");
      this->transformer->setDefaultPrefix(uav_name);
      this->transformer->setLookupTimeout(ros::Duration(0.3));
      this->transformer->retryLookupNewest(true);
    } else
    {
      this->transformer = common_handlers->transformer;
    }

    m_ground_detector.initialize(nh,
        this->transformer, GroundplaneDetector::groundplane_detection_config_t(*common_handlers->param_loader, pfx + "ground_removal/"));

    initialized = true;
  }

}  // namespace mrs_pcl_tools

