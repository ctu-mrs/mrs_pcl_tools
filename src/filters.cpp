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
  const std::vector<std::string> sequence =
      param_loader->loadParamReusable2<std::vector<std::string>>(prefix + "cloud_filter/sequence", std::vector<std::string>());
  std::vector<std::string> valid_filters;

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

/*//{ CovSFilter() */
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
/*//}*/

/*//{ filter() */
void CovSFilter::filter(typename boost::shared_ptr<PC>& inout_pc) const {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("CovSFilter", nullptr, false);

  // Implementation adapted from:
  // https://github.com/norlab-ulaval/libpointmatcher/blob/master/pointmatcher/DataPointsFilters/CovarianceSampling.cpp

  using Matrix66 = Eigen::Matrix<double, 6, 6>;
  using Vector6  = Eigen::Matrix<double, 6, 1>;
  using Vector3  = Eigen::Matrix<double, 3, 1>;

  const size_t nbPoints = inout_pc->size();
  const size_t nbSample = _count;

  if (nbSample >= nbPoints) {
    return;
  }

  // | --------------------- Compute Normals -------------------- |
  pcl::NormalEstimation<PC::PointType, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(inout_pc);

  const pcl::search::KdTree<PC::PointType>::Ptr tree = boost::make_shared<pcl::search::KdTree<PC::PointType>>();
  normal_estimator.setSearchMethod(tree);

  const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normal_estimator.setKSearch(_norm_est_K);
  normal_estimator.compute(*normals);

  timer.checkpoint("normal estimation");

  /* ROS_INFO("inout_pc size, normals size: %ld, %ld", inout_pc->size(), normals->size()); */

  // | ---------------- Covariance-space Sampling --------------- |

  ///---- Part A, as we compare the cloud with himself, the overlap is 100%, so we keep all points

  // Compute centroid
  Vector3 center = Vector3::Zero();
  for (size_t i = 0; i < nbPoints; ++i) {
    const auto& p = inout_pc->at(i);
    center.x() += p.x;
    center.y() += p.y;
    center.z() += p.z;
  }
  center /= double(nbPoints);

  // Compute torque normalization
  double Lnorm = 1.0;  // L1 normalization

  if (_torque_norm == TORQUE_NORM::Lavg) {

    Lnorm = 0.0;
    for (size_t i = 0; i < nbPoints; ++i) {

      const auto&   p     = inout_pc->at(i);
      const Vector3 point = Vector3(p.x, p.y, p.z);

      Lnorm += (point - center).norm();
    }
    Lnorm /= nbPoints;
  }
  // TODO:: add Lmax option
  else if (_torque_norm == TORQUE_NORM::Lmax) {
    ROS_WARN("[CovSFilter] Lmax option not implemented, using L1.");
    /*   const Vector3 minValues = cloud.features.rowwise().minCoeff(); */
    /*   const Vector3 maxValues = cloud.features.rowwise().maxCoeff(); */
    /*   const Vector3 radii     = maxValues.head(3) - minValues.head(3); */

    /*   Lnorm = radii.maxCoeff() / 2.; */
  }

  // A.3 - Compute 6x6 covariance matrix + EigenVectors
  auto computeCovariance = [Lnorm, nbPoints, &inout_pc, &center, &normals](Matrix66& cov) -> void {
    // Compute F matrix, see Eq. (4)
    Eigen::Matrix<double, 6, Eigen::Dynamic> F(6, nbPoints);

    for (size_t i = 0; i < nbPoints; ++i) {

      const auto&   n_p = normals->at(i);
      const Vector3 ni  = Vector3(n_p.normal_x, n_p.normal_y, n_p.normal_z);

      // Handle that normal estimation can produce inf/nan if the PCA fails
      if (!ni.allFinite()) {

        for (int k = 0; k < 6; k++) {
          F(k, i) = 0;
        }

      } else {

        const auto&   p_pcl = inout_pc->at(i);
        const Vector3 point = Vector3(p_pcl.x, p_pcl.y, p_pcl.z);
        const Vector3 p     = point - center;  // pi-c

        // compute (1 / L) * (pi - c) x ni
        F.template block<3, 1>(0, i) = (1. / Lnorm) * p.cross(ni);
        // set ni part
        F.template block<3, 1>(3, i) = ni;
      }
    }

    // Compute the covariance matrix Cov = FF'
    cov = F * F.transpose();
  };

  Matrix66 covariance;
  computeCovariance(covariance);

  const Eigen::EigenSolver<Matrix66> solver(covariance);
  const Matrix66                     eigenVe = solver.eigenvectors().real();
  const Vector6                      eigenVa = solver.eigenvalues().real();

  ///---- Part B
  // B.1 - Compute the v-6 for each candidate
  std::vector<Vector6, Eigen::aligned_allocator<Vector6>> v;  // v[i] = [(pi-c) x ni ; ni ]'
  v.resize(nbPoints);

  for (size_t i = 0; i < nbPoints; ++i) {

    const auto& p   = inout_pc->at(i);
    const auto& n_p = normals->at(i);

    const Vector3 ni = Vector3(n_p.normal_x, n_p.normal_y, n_p.normal_z);
    if (!ni.allFinite()) {

      v[i] = Vector6::Zero();

    } else {

      const Vector3 point = Vector3(p.x, p.y, p.z);
      const Vector3 p     = point - center;  // pi-c

      v[i].template block<3, 1>(0, 0) = (1. / Lnorm) * p.cross(ni);
      v[i].template block<3, 1>(3, 0) = ni;
    }
  }

  // B.2 - Compute the 6 sorted list based on dot product (vi . Xk) = magnitude, with Xk the kth-EigenVector
  std::vector<std::list<std::pair<int, double>>> L;  // contain list of pair (index, magnitude) contribution to the eigens vectors
  L.resize(6);

  // sort by decreasing magnitude
  auto comp = [](const std::pair<int, double>& p1, const std::pair<int, double>& p2) -> bool { return p1.second > p2.second; };

  for (size_t k = 0; k < 6; ++k) {
    for (size_t i = 0; i < nbPoints; ++i) {
      L[k].push_back(std::make_pair(i, std::fabs(v[i].dot(eigenVe.template block<6, 1>(0, k)))));
    }
    L[k].sort(comp);
  }

  std::vector<double> t(6, 0.0);                       // contains the sums of squared magnitudes
  std::vector<bool>   sampledPoints(nbPoints, false);  // maintain flag to avoid resampling the same point in an other list

  // Add point iteratively till we got the desired number of point
  for (size_t i = 0; i < nbSample; ++i) {

    // B.3 - Equally constrained all eigen vectors
    //  magnitude contribute to t_i where i is the indice of th least contrained eigen vector

    // Find least constrained vector
    size_t k = 0;
    for (size_t i = 0; i < 6; ++i) {
      if (t[k] > t[i]) {
        k = i;
      }
    }

    // Add the point from the top of the list corresponding to the dimension to the set of samples
    while (sampledPoints[L[k].front().first]) {
      L[k].pop_front();  // remove already sampled point
    }

    // Get index to keep
    const size_t idxToKeep = static_cast<size_t>(L[k].front().first);
    L[k].pop_front();

    sampledPoints[idxToKeep] = true;  // set flag to avoid resampling

    // B.4 - Update the running total
    for (size_t k = 0; k < 6; ++k) {
      const double magnitude = v[idxToKeep].dot(eigenVe.template block<6, 1>(0, k));
      t[k] += (magnitude * magnitude);
    }

    /* pc_out->points.push_back(inout_pc->at(idxToKeep)); */
  }

  // Invalidate points
  for (size_t i = 0; i < nbPoints; i++) {

    if (!sampledPoints[i]) {
      auto& p = inout_pc->at(i);
      p.x     = INVALID_VALUE;
      p.y     = INVALID_VALUE;
      p.z     = INVALID_VALUE;
    }
  }
}
/*//}*/

/*//}*/

}  // namespace mrs_pcl_tools
