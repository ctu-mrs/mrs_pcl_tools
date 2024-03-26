#include <mrs_pcl_tools/filters.h>

namespace mrs_pcl_tools
{

/*//{ class: PointCloudFilters */
PointCloudFilters::PointCloudFilters(const std::shared_ptr<mrs_lib::ParamLoader> param_loader, const std::string& ns) {

  // TODO: Add the rest of filters using this convention: downsample, range_clip, radius outlier, minimum grid, bilateral

  std::string prefix = "";
  if (!ns.empty()) {
    prefix = ns + "/";
  }

  // | ---------------------- Load filters ---------------------- |
  const std::vector<std::string> sequence = param_loader->loadParamReusable2<std::vector<std::string>>(prefix + "cloud_filter/sequence", std::vector<std::string>());
  std::vector<std::string>       valid_filters;

  valid_filters.reserve(sequence.size());
  _filters.reserve(sequence.size());

  for (const auto& name : sequence) {

    std::shared_ptr<AbstractFilter> filter;

    if (name == "voxel_grid") {

      const float res = param_loader->loadParamReusable2<float>(prefix + "cloud_filter/voxel_grid/resolution");

      filter = std::make_shared<VoxelFilter>(res);

    } else if (name == "NormS") {

      const double res_az = param_loader->loadParamReusable2<double>(prefix + "cloud_filter/NormS/resolution/azimuth");
      const double res_el = param_loader->loadParamReusable2<double>(prefix + "cloud_filter/NormS/resolution/elevation");

      filter = std::make_shared<NormSFilter>(res_az, res_el);

    } else if (name == "CovS") {

      const int         count       = param_loader->loadParamReusable2<int>(prefix + "cloud_filter/CovS/count");
      const std::string torque_norm = param_loader->loadParamReusable2<std::string>(prefix + "cloud_filter/CovS/torque_norm", "L1");

      filter = std::make_shared<CovSFilter>(size_t(count), torque_norm);

    } else {

      ROS_WARN("[PointCloudFilters] Unknown filter: %s. Skipping.", name.c_str());
    }

    if (filter && filter->isValid()) {
      valid_filters.push_back(name);
      _filters.push_back(filter);
    }
  }

  if (!valid_filters.empty()) {
    ROS_INFO("[PointCloudFilters][ns=%s] Using %ld filters in this order:", ns.c_str(), valid_filters.size());
    for (const auto& filter : valid_filters) {
      ROS_INFO("[PointCloudFilters]   - %s", filter.c_str());
    }
  }
}

void PointCloudFilters::applyFilters(typename boost::shared_ptr<PC>& inout_pc_ptr) {
  for (const auto& filter : _filters) {
    filter->filter(inout_pc_ptr);
  }
}
/*//}*/

/*//{ class: AbstractFilter */
bool AbstractFilter::isValid() const {
  return _params_valid;
}
/*//}*/

/*//{ class: VoxelFilter */
VoxelFilter::VoxelFilter(const float resolution) {
  _resolution = resolution;
  if (resolution < 0.0) {
    ROS_ERROR("[VoxelFilter] Invalid parameter resolution: %.1f (must be positive).", resolution);
    _params_valid = false;
  }
}

void VoxelFilter::filter(typename boost::shared_ptr<PC>& inout_pc) const {
  
  // TODO: Replace PCL voxel filter with such that does not use centroids but real points (such as in KISS-ICP)

  pcl::VoxelGrid<typename PC::PointType> vg;
  vg.setInputCloud(inout_pc);
  vg.setLeafSize(_resolution, _resolution, _resolution);

  /* const boost::shared_ptr<PC> cloud_out = boost::make_shared<PC>(); */
  vg.filter(*inout_pc);

  /* inout_pc = cloud_out; */
}
/*//}*/

/*//{ class: NormSFilter */
NormSFilter::NormSFilter(const double resolution_azimuth, const double resolution_elevation) {
  _res_az = resolution_azimuth;
  _res_el = resolution_elevation;

  if (_res_az < 0.0 || _res_el < 0.0) {
    ROS_ERROR("[NormSFilter] Invalid resolution in azimuth | elevation: %.1f | %.1f (both must be positive).", _res_az, _res_el);
    _params_valid = false;
  }
}

void NormSFilter::filter(typename boost::shared_ptr<PC>& inout_pc) const {
  // TODO: implement
}
/*//}*/

/*//{ class: CovSFilter */
CovSFilter::CovSFilter(const size_t count, const std::string& torque_norm) {
  _count = count;
  if (torque_norm == "L1") {
    _torque_norm = TORQUE_NORM::L1;
  } else if (torque_norm == "Lavg") {
    _torque_norm = TORQUE_NORM::Lavg;
  } else if (torque_norm == "Lmax") {
    _torque_norm = TORQUE_NORM::Lmax;
  } else {
    ROS_ERROR("[CovSFilter] Invalid parameter torque_norm: %s.", torque_norm.c_str());
    _params_valid = false;
  }
}

void CovSFilter::filter(typename boost::shared_ptr<PC>& inout_pc) const {
  // TODO: implement
  
  pcl::NormalEstimation<PC::PointType, pcl::Normal> ne;
  ne.setInputCloud(inout_pc);

  const pcl::search::KdTree<PC::PointType>::Ptr tree = boost::make_shared<pcl::search::KdTree<PC::PointType>>();
  ne.setSearchMethod(tree);

  const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(5);
  ne.compute(*normals);

  /* ROS_INFO("inout_pc size, normals size: %ld, %ld", inout_pc->size(), normals->size()); */

}
/*//}*/

}  // namespace mrs_pcl_tools
