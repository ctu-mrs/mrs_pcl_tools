#include <mrs_pcl_tools/freespace_pointcloud_creator.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>

namespace mrs_pcl_tools
{

  /* onInit() //{ */

  void FreespacePointcloudCreator::onInit()
  {
    m_nh = nodelet::Nodelet::getMTPrivateNodeHandle();

    NODELET_INFO("[FreespacePointcloudCreator]: Waiting for valid time...");
    ros::Time::waitForValid();

    // Get parameters from config file
    mrs_lib::ParamLoader param_loader(m_nh, "FreespacePointcloudCreator");

    param_loader.loadParam("sensor/hfov", m_sensor_hfov);
    param_loader.loadParam("sensor/vfov", m_sensor_vfov);
    param_loader.loadParam("sensor/max_range", m_sensor_max_range);
    param_loader.loadParam("data_row_major", m_row_major);

    if (!param_loader.loadedSuccessfully()) {
      NODELET_ERROR("[FreespacePointcloudCreator]: Some compulsory parameters were not loaded successfully, ending the node");
      ros::shutdown();
      return;
    }

    mrs_lib::SubscribeHandlerOptions shopts(m_nh);
    shopts.no_message_timeout = ros::Duration(5.0);
    mrs_lib::construct_object(m_sub_pc_in, shopts, "pointcloud_in", &FreespacePointcloudCreator::pointcloud_callback, this);
    m_pub_freespace_pc_out = m_nh.advertise<sensor_msgs::PointCloud2>("freespace_pointcloud_out", 1);
  }
  //}

  void FreespacePointcloudCreator::pointcloud_callback(sensor_msgs::PointCloud2::ConstPtr pc_in)
  {
    auto pc_out = boost::make_shared<sensor_msgs::PointCloud2>();
    // copy the pc data
    *pc_out = *pc_in;

    const double yaw_halfrange = m_sensor_hfov/2;
    const double yaw_step = m_sensor_hfov/(pc_out->width-1);
    const double pitch_halfrange = m_sensor_vfov/2;
    const double pitch_step = m_sensor_vfov/(pc_out->height-1);
    size_t it = 0;
    for (sensor_msgs::PointCloud2Iterator<float> x_it(*pc_out, "x"); x_it != x_it.end(); ++x_it)
    {
      float& x = x_it[0];
      float& y = x_it[1];
      float& z = x_it[2];

      const auto [row, col] = row_col(it, pc_in->width, pc_in->height, m_row_major);
      const double yaw = yaw_halfrange - yaw_step*(double(col) - pc_in->width/2.0);
      const double pitch = pitch_halfrange - pitch_step*(double(row) - pc_in->height/2.0);

      const double calc_yaw = std::atan2(y, x);
      const double calc_pitch = std::atan2(z, std::sqrt(x*x + y*y));

      if (x == 0.0f && y == 0.0f && z == 0.0f)
      {
        x = m_sensor_max_range * std::cos(yaw) * std::cos(pitch);
        y = m_sensor_max_range * std::sin(yaw) * std::cos(pitch);
        z = m_sensor_max_range * std::sin(pitch);

        /* ROS_INFO("P: [%.2f, %.2f, %.2f]", x, y, z); */
        /* ROS_INFO("row, col: [%lu, %lu]", row, col); */
        /* ROS_INFO("yaw:\t%.2f\tcyaw:\t%.2f", yaw, calc_yaw); */
        /* ROS_INFO("pitch:\t%.2f\tcpitch:\t%.2f", pitch, calc_pitch); */

      }
      else
      {
        x = y = z = 0;
      }
      it++;
    }

    m_pub_freespace_pc_out.publish(pc_out);
  }

  std::tuple<size_t, size_t> FreespacePointcloudCreator::row_col(const size_t it, const size_t width, const size_t height, const bool row_major)
  {
    if (row_major)
      return {it / width, it % width};
    else
      return {it % height, it / height};
  }

}  // namespace mrs_pcl_tools

PLUGINLIB_EXPORT_CLASS(mrs_pcl_tools::FreespacePointcloudCreator, nodelet::Nodelet);
