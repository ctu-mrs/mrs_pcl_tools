#pragma once

#include <random>

#include <mrs_pcl_tools/common_includes_and_typedefs.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>

/* #include <tsl/robin_map.h> */

namespace mrs_pcl_tools
{

// | ---------------------- Parent class ---------------------- |
class AbstractFilter {
public:
  virtual void filter(typename boost::shared_ptr<PC>& inout_pc) const = 0;
  AbstractFilter() {
  }
  virtual ~AbstractFilter() {
  }

  bool isValid() const;

protected:
  bool _params_valid       = true;
  bool _enable_scope_timer = false;

  const float INVALID_VALUE = std::numeric_limits<float>::quiet_NaN();

  pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const typename boost::shared_ptr<PC>& in_pc, const size_t nn_K) const;
};

// | ---------------------- Voxel Filter ---------------------- |
class VoxelFilter : public AbstractFilter {
public:
  VoxelFilter(const float resolution, const std::string& method = "centroid", const bool enable_scope_timer = true);
  void filter(typename boost::shared_ptr<PC>& inout_pc) const override;

private:
  enum METHOD
  {
    CENTROID,
    HASHMAP,
  };

  METHOD _method;
  float  _resolution;

  // TODO: hashmap method
  /* using Voxel = Eigen::Vector3i; */
  /* struct VoxelHash */
  /* { */
  /*   size_t operator()(const Voxel& voxel) const { */
  /*     const uint32_t* vec = reinterpret_cast<const uint32_t*>(voxel.data()); */
  /*     return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791); */
  /*   } */
  /* }; */
};

// | ------------------- Normal-space Filter ------------------ |
// Rusinkiewicz and Levoy, Efficient Variants of the ICP Algorithm, 2001
class NormSFilter : public AbstractFilter {
public:
  NormSFilter(const size_t count, const double resolution, const bool enable_scope_timer = true);
  void filter(typename boost::shared_ptr<PC>& inout_pc) const override;

private:
  size_t _count;
  double _resolution;

  size_t nbBucket;

  const size_t _norm_est_K = 5;

  std::size_t bucketIdx(double theta, double phi) const;
};

// | ----------------- Covariance-based Filter ---------------- |
// Gelfand et. al, Geometrically stable sampling for the ICP algorithm, 2003
class CovSFilter : public AbstractFilter {
public:
  CovSFilter(const size_t count, const std::string& torque_norm, const bool enable_scope_timer = true);
  void filter(typename boost::shared_ptr<PC>& inout_pc) const override;

private:
  enum TORQUE_NORM
  {
    L1,
    Lavg,
    Lmax  // TODO: implement this option
  };

  size_t      _count;
  TORQUE_NORM _torque_norm;

  const size_t _norm_est_K = 5;
};

// | ---------------- Wrapper class for filters --------------- |
class PointCloudFilters {
public:
  PointCloudFilters(){};
  PointCloudFilters(const std::shared_ptr<mrs_lib::ParamLoader> param_loader, const std::string& ns = "");
  void applyFilters(typename boost::shared_ptr<PC>& inout_pc_ptr, const bool remove_nans = false);

private:
  std::vector<std::shared_ptr<AbstractFilter>> _filters;
};

}  // namespace mrs_pcl_tools
