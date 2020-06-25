cmake_minimum_required(VERSION 2.8.3)
project(mrs_pcl_tools)

set(CMAKE_CXX_FLAGS "-g -O3 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDEBUG_PARAMS")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O3 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDEBUG_PARAMS")
set(CMAKE_CXX_FLAGS_RELEASE "-Werror -g -O3 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDEBUG_PARAMS")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -O3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-Wall")
add_compile_options("-Wextra")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nodelet
    message_generation
    sensor_msgs
    pcl_ros
    mrs_lib
    dynamic_reconfigure
)

find_package(PCL REQUIRED COMPONENTS)

# Include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

# Compile with distcc when available (must be after find_package(catkin))
find_file(DISTCC_FULL_PATH distcc)
find_file(DISTCPP_FULL_PATH distg++)
IF (EXISTS ${DISTCC_FULL_PATH})
 IF (EXISTS ${DISTCPP_FULL_PATH})
   message("Found distcc and distg++, using distributed compilation")
   SET(CMAKE_C_COMPILER ${DISTCC_FULL_PATH})
   SET(CMAKE_CXX_COMPILER ${DISTCPP_FULL_PATH})
 ELSE()
   message("Found only distcc, but not distg++, not using distributed compilation")
 ENDIF()
ELSE()
  message("Not found distcc, not using distributed compilation")
ENDIF()

generate_dynamic_reconfigure_options(
  cfg/pcl_filtration_dynparam.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mrs_pcl_tools 
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros mrs_lib
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# This causes adds the -march=native gcc flag that prevents ditributed compilation
add_definitions(
  ${PCL_DEFINITIONS}
  )

# Binary version of pcl on most of the linux distro is built with -march=native
# which cause a lot of compatibility problems.
# Define PCL_NO_PRECOMPILE to disable using binary version
# add_definitions(-DPCL_NO_PRECOMPILE)

## Declare a C++ Library
add_library(mrs_pcl_tools
  src/PCLFiltration.cpp
  )

add_dependencies(mrs_pcl_tools mrs_pcl_tools_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(
  mrs_pcl_tools ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES}
)

target_include_directories(mrs_pcl_tools SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
