cmake_minimum_required(VERSION 3.5)
project(mrs_pcl_tools)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_TESTS "Enable building tests" ON)


# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "" FORCE)

# disables anoying PCL warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")


set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  geometry_msgs
  mrs_lib
  mrs_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  ouster_ros
  Eigen3
  image_transport
  pcl_ros
  pcl_conversions
  # eigen_conversions
  rosidl_default_generators
  )

# find_package(PCL REQUIRED COMPONENTS
#   common
# )

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# # Include Eigen3
# find_package(Eigen3 REQUIRED)
# set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
# set(Eigen_LIBRARIES ${Eigen_LIBRARIES})


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/registration/SrvRegisterPointCloudByName.srv"
  "srv/registration/SrvRegisterPointCloudOffline.srv"
  DEPENDENCIES 
    geometry_msgs 
    std_msgs
    sensor_msgs
    visualization_msgs
    mrs_msgs
)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(LIBRARIES
  MrsPclTools_PCLSupportLib
)

## | --------------------- MrsPclTools_PCLSupportLib --------------------- |

# Change to shared later
add_library(MrsPclTools_PCLSupportLib SHARED
  src/utils/ros_logger.cpp
  src/utils/pointcloud_io.cpp
  src/utils/pointcloud_io_ros.cpp
  src/utils/math_utils.cpp
)

ament_target_dependencies(MrsPclTools_PCLSupportLib
  PUBLIC
    ${DEPENDENCIES}
)

## | -------------------- export interfaces ------------------- |
foreach(library IN LISTS LIBRARIES)
  target_include_directories(${library}
    PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
      ${PCL_INCLUDE_DIRS}
  )
endforeach()


add_library(mrs_pcl_tools_mrs_pcl_tools INTERFACE)

add_library(mrs_pcl_tools::mrs_pcl_tools ALIAS mrs_pcl_tools_mrs_pcl_tools)

set_target_properties(mrs_pcl_tools_mrs_pcl_tools
  PROPERTIES
    EXPORT_NAME mrs_pcl_tools
    OUTPUT_NAME mrs_pcl_tools
)

target_link_libraries(mrs_pcl_tools_mrs_pcl_tools
  INTERFACE
    ${LIBRARIES}
    ${PCL_LIBRARIES}
)

target_include_directories(mrs_pcl_tools_mrs_pcl_tools
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)



## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()


## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------


ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS
    mrs_pcl_tools_mrs_pcl_tools
    ${LIBRARIES}
    ${EXECUTABLES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "include/"
  DESTINATION "include/${PROJECT_NAME}"
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()