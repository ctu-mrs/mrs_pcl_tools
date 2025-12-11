#include <mrs_pcl_tools/utils/visualization.h>

namespace mrs_pcl_tools
{

  namespace visualization
  {

    /*//{ colorizeCloud() */
    PC_RGB::Ptr colorizeCloud(const PC::Ptr& cloud_xyz)
    {
      pt_XYZ min_xyz;
      pt_XYZ max_xyz;
      pcl::getMinMax3D(*cloud_xyz, min_xyz, max_xyz);
      const float min_z = min_xyz.z;
      const float max_z = max_xyz.z;

      const unsigned int size = cloud_xyz->points.size();
      PC_RGB::Ptr cloud_rgb = std::make_shared<PC_RGB>();
      cloud_rgb->points.resize(size);
      cloud_rgb->width = size;
      cloud_rgb->height = 1;
      cloud_rgb->is_dense = false;

      for (unsigned int i = 0; i < size; i++)
      {
        const float z = cloud_xyz->points.at(i).z;
        double height = (1.0 - std::fmin(std::fmax((z - min_z) / (max_z - min_z), 0.0f), 1.0f));
        std_msgs::msg::ColorRGBA color = heightToRGBA(height, 1.0);
        pt_XYZRGB p;
        p.x = cloud_xyz->points.at(i).x;
        p.y = cloud_xyz->points.at(i).y;
        p.z = z;
        p.r = color.r;
        p.g = color.g;
        p.b = color.b;
        p.a = 1.0;
        cloud_rgb->at(i) = p;
      }

      cloud_rgb->header = cloud_xyz->header;
      return cloud_rgb;
    }
    /*//}*/

    /*//{ heightToRGBA() */
    std_msgs::msg::ColorRGBA heightToRGBA(double& height, const double& alpha)
    {
      std_msgs::msg::ColorRGBA color;
      color.a = alpha;
      // blend over HSV-values (more colors)

      double s = 1.0;
      double v = 1.0;

      height -= floor(height);
      height *= 6;
      int i;
      double m, n, f;

      i = floor(height);
      f = height - i;
      if (!(i & 1))
        f = 1 - f;  // if i is even
      m = v * (1 - s);
      n = v * (1 - s * f);

      switch (i)
      {
        case 6:
        case 0:
          color.r = v;
          color.g = n;
          color.b = m;
          break;
        case 1:
          color.r = n;
          color.g = v;
          color.b = m;
          break;
        case 2:
          color.r = m;
          color.g = v;
          color.b = n;
          break;
        case 3:
          color.r = m;
          color.g = n;
          color.b = v;
          break;
        case 4:
          color.r = n;
          color.g = m;
          color.b = v;
          break;
        case 5:
          color.r = v;
          color.g = m;
          color.b = n;
          break;
        default:
          color.r = 1;
          color.g = 0.5;
          color.b = 0.5;
          break;
      }

      return color;
    }
    /*//}*/

  }  // namespace visualization

}  // namespace mrs_pcl_tools
