#include <mrs_pcl_tools/filters.h>
#include <tsl/robin_map.h> // needs to be here in case filters.h is included from external files

namespace mrs_pcl_tools
{

/*//{ class: PointCloudFilters */

/*//{ PointCloudFilters() */
PointCloudFilters::PointCloudFilters(const std::shared_ptr<mrs_lib::ParamLoader> param_loader, const std::string& ns) {

  // TODO: Add the rest of filters using this convention: downsample, range_clip, radius outlier, minimum grid, bilateral

  std::string prefix = "";
  if (!ns.empty()) {
    prefix = ns + "/";
  }

  const bool enable_scope_timer = param_loader->loadParamReusable2(prefix + "cloud_filter/scope_timer/enabled", false);

  // | ---------------------- Load filters ---------------------- |
  const std::vector<std::string> sequence =
      param_loader->loadParamReusable2<std::vector<std::string>>(prefix + "cloud_filter/sequence", std::vector<std::string>());
  std::vector<std::string> valid_filters;

  valid_filters.reserve(sequence.size());
  _filters.reserve(sequence.size());

  for (const auto& name : sequence) {

    std::shared_ptr<AbstractFilter> filter;

    if (name == "voxel_grid") {

      const float       res    = param_loader->loadParamReusable2<float>(prefix + "cloud_filter/voxel_grid/resolution");
      const std::string method = param_loader->loadParamReusable2<std::string>(prefix + "cloud_filter/voxel_grid/method", "centroid");

      filter = std::make_shared<VoxelFilter>(res, method, enable_scope_timer);

    } else if (name == "NormS") {

      const int    count      = param_loader->loadParamReusable2<int>(prefix + "cloud_filter/NormS/count");
      const double resolution = param_loader->loadParamReusable2<double>(prefix + "cloud_filter/NormS/resolution");

      filter = std::make_shared<NormSFilter>(size_t(count), resolution, enable_scope_timer);

    } else if (name == "CovS") {

      const int         count       = param_loader->loadParamReusable2<int>(prefix + "cloud_filter/CovS/count");
      const std::string torque_norm = param_loader->loadParamReusable2<std::string>(prefix + "cloud_filter/CovS/torque_norm", "L1");

      filter = std::make_shared<CovSFilter>(size_t(count), torque_norm, enable_scope_timer);

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
/*//}*/

/*//{ applyFilters() */
void PointCloudFilters::applyFilters(typename boost::shared_ptr<PC>& inout_pc, const bool remove_nans) {
  for (const auto& filter : _filters) {
    filter->filter(inout_pc);
  }

  if (remove_nans) {
    std::vector<int> ind;
    inout_pc->is_dense = false;
    pcl::removeNaNFromPointCloud(*inout_pc, *inout_pc, ind);
  }
}
/*//}*/

/*//}*/

/*//{ class: AbstractFilter */

/*//{ isValid() */
bool AbstractFilter::isValid() const {
  return _params_valid;
}
/*//}*/

/*//{ estimateNormals() */
pcl::PointCloud<pcl::Normal>::Ptr AbstractFilter::estimateNormals(const typename boost::shared_ptr<PC>& in_pc, const size_t nn_K) const {

  pcl::NormalEstimation<PC::PointType, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(in_pc);

  const pcl::search::KdTree<PC::PointType>::Ptr tree = boost::make_shared<pcl::search::KdTree<PC::PointType>>();
  normal_estimator.setSearchMethod(tree);

  const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normal_estimator.setKSearch(nn_K);
  normal_estimator.compute(*normals);

  return normals;
}
/*//}*/

/*//}*/

/*//{ class: VoxelFilter */

/*//{ VoxelFilter() */
VoxelFilter::VoxelFilter(const float resolution, const std::string& method, const bool enable_scope_timer) {
  _resolution         = resolution;
  _enable_scope_timer = enable_scope_timer;

  if (method == "hashmap") {
    _method = METHOD::HASHMAP;
  } else if (method == "centroid") {
    _method = METHOD::CENTROID;
  } else {
    ROS_ERROR("[VoxelFilter] Invalid method: %s (valid: hashmap, centroid).", method.c_str());
    _params_valid = false;
  }

  if (resolution < 0.0) {
    ROS_ERROR("[VoxelFilter] Invalid parameter resolution: %.1f (must be positive).", resolution);
    _params_valid = false;
  }
}
/*//}*/

/*//{ filter() */
void VoxelFilter::filter(typename boost::shared_ptr<PC>& inout_pc) const {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("VoxelFilter", nullptr, _enable_scope_timer);

  // Use PCL as it "centroids" all points within a voxel
  if (_method == METHOD::CENTROID) {

    pcl::VoxelGrid<typename PC::PointType> vg;
    vg.setInputCloud(inout_pc);
    vg.setLeafSize(_resolution, _resolution, _resolution);

    vg.filter(*inout_pc);

  }

  // Use code from KISS-ICP which keeps real point within a voxel
  else if (_method == METHOD::HASHMAP) {

    // prepare new output cloud
    const boost::shared_ptr<PC> pc_out = boost::make_shared<PC>();
    pc_out->reserve(inout_pc->size());

    using Voxel = Eigen::Vector3i;

    // init map (key: Voxel, value: pt_XYZ, hashing: VoxelHash)
    tsl::robin_map<Voxel, pt_XYZ, VoxelHash> grid;
    grid.reserve(inout_pc->size());

    // voxelize: keep first point inserted into the voxel (we could easily keep the most middle point or multiple points if needed)
    for (const auto& point : inout_pc->points) {
      const auto voxel = Voxel((Eigen::Vector3f(point.x, point.y, point.z) / _resolution).cast<int>());
      if (grid.contains(voxel)) {
        continue;
      }
      pc_out->push_back(point);
      grid.insert({voxel, point});
    }

    pc_out->header   = inout_pc->header;
    pc_out->width    = pc_out->size();
    pc_out->height   = pc_out->size() > 0 ? 1 : 0;
    pc_out->is_dense = true;

    inout_pc = pc_out;
  }
}
/*//}*/

/*//}*/

/*//{ class: NormSFilter */

/*//{ NormSFilter() */
NormSFilter::NormSFilter(const size_t count, const double resolution, const bool enable_scope_timer) {
  _resolution         = resolution;
  _count              = count;
  _enable_scope_timer = enable_scope_timer;

  if (_resolution < 0.0) {
    ROS_ERROR("[NormSFilter] Invalid resolution: %.1f (must be positive).", _resolution);
    _params_valid = false;
  }

  nbBucket = std::size_t(std::ceil(2.0 * M_PI / _resolution) * std::ceil(M_PI / _resolution));
}
/*//}*/

/*//{ filter() */
void NormSFilter::filter(typename boost::shared_ptr<PC>& inout_pc) const {
  // Implementation adapted from:
  // https://github.com/norlab-ulaval/libpointmatcher/blob/master/pointmatcher/DataPointsFilters/NormalSpace.cpp

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("NormSFilter", nullptr, _enable_scope_timer);

  const size_t nbPoints = inout_pc->size();
  const size_t nbSample = _count;

  if (nbSample >= nbPoints) {
    return;
  }

  const auto& normals = estimateNormals(inout_pc, _norm_est_K);
  timer.checkpoint("normal estimation");

  std::mt19937 gen(time(nullptr));

  // bucketed normal space
  std::vector<std::vector<int>> idBuckets;
  idBuckets.resize(nbBucket);

  // Generate a random sequence of indices so that elements are placed in buckets in random order
  std::vector<std::size_t> randIdcs(nbPoints);
  std::iota(randIdcs.begin(), randIdcs.end(), 0);
  std::shuffle(randIdcs.begin(), randIdcs.end(), gen);

  // (1) put all points of the data into buckets based on their normal direction
  size_t finite = 0;
  for (const auto randIdx : randIdcs) {

    const auto& n = normals->at(randIdx);

    if (std::isfinite(n.normal_x) && std::isfinite(n.normal_y) && std::isfinite(n.normal_z)) {

      // Theta = polar angle in [0 ; pi]
      const double theta = std::acos(n.normal_z);
      // Phi = azimuthal angle in [0 ; 2pi]
      const double phi = std::fmod(std::atan2(n.normal_y, n.normal_x) + 2. * M_PI, 2. * M_PI);

      // Catch normal space hashing errors
      /* ROS_ERROR("bucketIdx(%.2f, %.2f) = %ld (nbBucket: %ld)", theta, phi, bucketIdx(theta, phi), nbBucket); */
      idBuckets[bucketIdx(theta, phi)].push_back(randIdx);

      finite++;
    }
  }

  if (nbSample >= finite) {
    return;
  }

  // Remove empty buckets
  idBuckets.erase(std::remove_if(idBuckets.begin(), idBuckets.end(), [](const std::vector<int>& bucket) { return bucket.empty(); }), idBuckets.end());

  // (2) uniformly pick points from all the buckets until the desired number of points is selected
  /* std::vector<std::size_t> keepIndexes; */
  /* keepIndexes.reserve(nbSample); */

  for (std::size_t i = 0; i < nbSample; i++) {

    // Get a random bucket
    std::uniform_int_distribution<std::size_t> uniBucket(0, idBuckets.size() - 1);  // TODO: this has to be expensive in every loop

    const std::size_t curBucketIdx = uniBucket(gen);
    auto&             curBucket    = idBuckets[curBucketIdx];

    //(3) A point is randomly picked in a bucket that contains multiple points
    const int idToKeep = curBucket[curBucket.size() - 1];
    curBucket.pop_back();
    /* keepIndexes.push_back(static_cast<std::size_t>(idToKeep)); */

    // Remove the bucket if it is empty
    if (curBucket.empty()) {
      idBuckets.erase(idBuckets.begin() + curBucketIdx);
    }
  }

  // Invalidate points
  for (const auto& ids : idBuckets) {
    for (const int i : ids) {
      auto& p = inout_pc->at(i);
      p.x     = INVALID_VALUE;
      p.y     = INVALID_VALUE;
      p.z     = INVALID_VALUE;
    }
  }
}
/*//}*/

/*//{ bucketIdx() */
std::size_t NormSFilter::bucketIdx(double theta, double phi) const {
  // Theta = polar angle in [0 ; pi] and Phi = azimuthal angle in [0 ; 2pi]

  // Wrap Theta at Pi
  if (theta >= M_PI) {
    theta = 0.0;
  }

  // Wrap Phi at 2Pi
  if (phi >= 2.0 * M_PI) {
    phi = 0.0;
  }

  //                              block number                      block size                            element number
  return static_cast<std::size_t>(std::floor(theta / _resolution) * std::ceil(2.0 * M_PI / _resolution) + std::floor(phi / _resolution));
}
/*//}*/

/*//}*/

/*//{ class: CovSFilter */

/*//{ CovSFilter() */
CovSFilter::CovSFilter(const size_t count, const std::string& torque_norm, const bool enable_scope_timer) {
  _count              = count;
  _enable_scope_timer = enable_scope_timer;

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

  // Implementation adapted from:
  // https://github.com/norlab-ulaval/libpointmatcher/blob/master/pointmatcher/DataPointsFilters/CovarianceSampling.cpp

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("CovSFilter", nullptr, _enable_scope_timer);

  using Matrix66 = Eigen::Matrix<double, 6, 6>;
  using Vector6  = Eigen::Matrix<double, 6, 1>;
  using Vector3  = Eigen::Matrix<double, 3, 1>;

  const size_t nbPoints = inout_pc->size();
  const size_t nbSample = _count;

  if (nbSample >= nbPoints) {
    return;
  }

  // | --------------------- Compute Normals -------------------- |
  const auto& normals = estimateNormals(inout_pc, _norm_est_K);
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
