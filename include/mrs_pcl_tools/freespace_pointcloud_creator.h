#pragma once

/* includes and typedefs //{ */
#include <mutex>
#include <tuple>
#include <set>

// basic ros
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/param_loader.h>

//}

namespace mrs_pcl_tools
{

  /* class FreespacePointcloudCreator //{ */
  class FreespacePointcloudCreator : public nodelet::Nodelet
  {

    public:
      virtual void onInit();

    private:
      ros::NodeHandle m_nh;

      mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> m_sub_pc_in;
      ros::Publisher m_pub_freespace_pc_out;

      void pointcloud_callback(sensor_msgs::PointCloud2::ConstPtr pc_in);
      std::tuple<size_t, size_t> row_col(const size_t it, const size_t width, const size_t height, const bool row_major);

      double m_sensor_hfov, m_sensor_vfov, m_sensor_max_range;
      bool m_row_major;
  };
  //}

}  // namespace mrs_pcl_tools
