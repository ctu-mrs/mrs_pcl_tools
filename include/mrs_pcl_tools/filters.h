#pragma once

#include <mrs_pcl_tools/common_includes_and_typedefs.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>

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
  bool _params_valid = true;
};

// | ---------------------- Voxel Filter ---------------------- |
class VoxelFilter : public AbstractFilter {
public:
  VoxelFilter(const float resolution);
  void filter(typename boost::shared_ptr<PC>& inout_pc) const override;

private:
  float _resolution;
};

// | ------------------- Normal-space Filter ------------------ |
// Rusinkiewicz and Levoy, Efficient Variants of the ICP Algorithm, 2001
class NormSFilter : public AbstractFilter {
public:
  NormSFilter(const double resolution_azimuth, const double resolution_elevation);
  void filter(typename boost::shared_ptr<PC>& inout_pc) const override;

private:
  double _res_az;
  double _res_el;
};

class PointCloudFilters {
public:
  PointCloudFilters(){};
  PointCloudFilters(const std::shared_ptr<mrs_lib::ParamLoader> param_loader, const std::string& ns);
  void applyFilters(typename boost::shared_ptr<PC>& inout_pc_ptr);

private:
  std::vector<std::shared_ptr<AbstractFilter>> _filters;
};

// | ----------------- Covariance-based Filter ---------------- |
// Gelfand et. al, Geometrically stable sampling for the ICP algorithm, 2003
class CovSFilter : public AbstractFilter {
public:
  CovSFilter(const size_t count, const std::string& torque_norm);
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

  const float INVALID_VALUE = std::numeric_limits<float>::quiet_NaN();
};

}  // namespace mrs_pcl_tools
