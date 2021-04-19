#!/bin/bash

# RELATED ISSUES
# - https://github.com/PointCloudLibrary/pcl/issues/4258
# - https://stackoverflow.com/questions/64604859/how-can-i-debug-memory-alignment-problems-avx-when-they-dont-reproduce-in-a-m

# DESCRIPTION:
# This script will build PCL library from source.
# Make sure that the building flags are correctly set to the workspaces' flags of your packages (default flags correlate with mrs_workspace)
# Do not forget to require the specific PCL_VERSION in your CMakeLists.txt

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

default=n
resp=$default
[[ -t 0 ]] && { read -t 10 -n 2 -p $'\e[1;32mBuild PCL? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
response=`echo $resp | sed -r 's/(.*)$/\1=/'`
if [[ $response =~ ^(n|N)=$ ]]
then
  exit
fi

################# SETUP #################

SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

# Releases can be found at: https://github.com/PointCloudLibrary/pcl/releases
PCL_VERSION_MAJOR=1.10
PCL_VERSION_FULL=$PCL_VERSION_MAJOR.1

# IMPORTANT: These variables should match the settings of your catkin workspace
PROFILE="Release" # RelWithDebInfo, Release, Debug
BUILD_WITH_MARCH_NATIVE=true
CMAKE_STANDARD=17

# The install directory should match to where pcl-ros looks to
INSTALL_DIR=/usr

# Build with march native?
if $BUILD_WITH_MARCH_NATIVE; then
  CMAKE_MARCH_NATIVE="-march=native -msse4.2 -mfpmath=sse"
else
  CMAKE_MARCH_NATIVE=""
fi

# Defaults taken from mrs_workspace building flags
BUILD_FLAGS_GENERAL=( 
              -DPCL_VERSION=$PCL_VERSION_MAJOR
              -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR
              -DBUILD_apps=ON
              -DBUILD_examples=ON
              -DCMAKE_CXX_STANDARD=$CMAKE_STANDARD 
              -DCMAKE_BUILD_TYPE=$PROFILE
              -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
              -DCMAKE_CXX_FLAGS="-std=c++$CMAKE_STANDARD $CMAKE_MARCH_NATIVE" 
              -DCMAKE_CXX_FLAGS="$CMAKE_MARCH_NATIVE" 
              -DCMAKE_C_FLAGS="$CMAKE_MARCH_NATIVE"
            )

# Profile-dependent flags
if [[ "$PROFILE" == "RelWithDebInfo" ]]; then
  BUILD_FLAGS_PROFILE=(
                  -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O2 -g"
                  -DCMAKE_C_FLAGS_${PROFILE^^}="-O2 -g")
elif [[ "$PROFILE" == "Release" ]]; then
  BUILD_FLAGS_PROFILE=(
                  -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O3"
                  -DCMAKE_C_FLAGS_${PROFILE^^}="-O3")
else
  BUILD_FLAGS_PROFILE=(
                  -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O0 -g"
                  -DCMAKE_C_FLAGS_${PROFILE^^}="-O0 -g")
fi

################# INSTALLATION #################

# Remove precompiled PCL-ROS dependencies
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions
# sudo apt-get remove ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions libpcl-* # cross-compilation of pcl_ros and pcl_conversions

# checkout PCL_VERSION (ideally stable)
cd $GIT_PATH
[ ! -d "pcl" ] && git clone https://github.com/PointCloudLibrary/pcl.git # clone if was not cloned before
cd $GIT_PATH/pcl && git checkout pcl-$PCL_VERSION_FULL

# create the build folder
# [ -d "build" ] && rm -rf build # delete if exists
cd $GIT_PATH/pcl
[ ! -d "build" ] && mkdir build # create if does not exists

# install
cd $GIT_PATH/pcl/build
cmake "${BUILD_FLAGS_GENERAL[@]}" "${BUILD_FLAGS_PROFILE[@]}" ../
echo "Building PCL with cmake flags: ${BUILD_FLAGS_GENERAL[@]} ${BUILD_FLAGS_PROFILE[@]}"
echo "This process OFTEN FAILS due to an internal compiler error. In such case, RUN AGAIN the script: $SCRIPT_PATH/install_pcl.sh"
# echo "Building will start in 5s"
# sleep 5
make -j$[$(nproc) - 1]
sudo make install

# link shared libraries to path looked up by pcl_ros and pcl_conversions (this will also override libpcl-* binaries from debian repository)
sudo ln -sf $INSTALL_DIR/lib/libpcl*.so /usr/lib/x86_64-linux-gnu/.

# add flags to ~/.{shell}rc
bashrc_flagged=`cat ~/.bashrc | grep "PCL_CROSS_COMPILATION" | wc -l`
zshrc_flagged=`cat ~/.zshrc | grep "PCL_CROSS_COMPILATION" | wc -l`
if [ "$bashrc_flagged" -lt "1" ]; then
  echo 'export PCL_CROSS_COMPILATION="true"' >> ~/.bashrc
fi
if [ "$zshrc_flagged" -lt "1" ]; then
  echo 'export PCL_CROSS_COMPILATION="true"' >> ~/.zshrc
fi
