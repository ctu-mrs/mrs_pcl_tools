#include <mrs_pcl_tools/utils/math_utils.h>

namespace mrs_pcl_tools
{

/*//{ printEigenMatrix() */
void printEigenMatrix(ILogger& logger, const Eigen::Matrix4f& mat, const std::string& prefix)
{
  const std::string st = (prefix.size() > 0) ? prefix : "Eigen matrix:";
  mrs_lib::AttitudeConverter atti = mrs_lib::AttitudeConverter(mat.block<3, 3>(0, 0).cast<double>());
  logger.info("[PCLSupportLibrary] " + st);
  logger.info(std::format("      | {:2.3f} {:2.3f} {:2.3f} {:2.3f} |", mat(0, 0), mat(0, 1), mat(0, 2), mat(0, 3)));
  logger.info(std::format("T   = | {:2.3f} {:2.3f} {:2.3f} {:2.3f} |", mat(1, 0), mat(1, 1), mat(1, 2), mat(1, 3)));
  logger.info(std::format("      | {:2.3f} {:2.3f} {:2.3f} {:2.3f} |", mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3)));
  logger.info(std::format("      | {:2.3f} {:2.3f} {:2.3f} {:2.3f} |", mat(3, 0), mat(3, 1), mat(3, 2), mat(3, 3)));
  logger.info(std::format("RPY = < {:2.3f}, {:2.3f}, {:2.3f} >", atti.getRoll(), atti.getPitch(), atti.getYaw()));
  logger.info(std::format("t   = < {:2.3f}, {:2.3f}, {:2.3f} >", mat(0, 3), mat(1, 3), mat(2, 3)));
}
/*//}*/

/*//{ getRotationMatrixAroundPoint() */
Eigen::Matrix4f getRotationMatrixAroundPoint(const Eigen::Matrix3f& rotation, const Eigen::Vector4f& point)
{
  Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T2 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T3 = Eigen::Matrix4f::Identity();

  // To the point
  T1(0, 3) = -point.x();
  T1(1, 3) = -point.y();
  T1(2, 3) = -point.z();

  // Rotate
  T2.block<3, 3>(0, 0) = rotation;

  // Back to the origin
  T3(0, 3) = point.x();
  T3(1, 3) = point.y();
  T3(2, 3) = point.z();

  return T3 * T2 * T1;
}
/*//}*/

}  // namespace mrs_pcl_tools