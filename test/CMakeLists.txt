find_package(ament_cmake_gtest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

link_libraries(mrs_pcl_tools::mrs_pcl_tools)

add_subdirectory(./pointcloud_filters)
add_subdirectory(./pointcloud_io)
add_subdirectory(./pcl_filtration)
