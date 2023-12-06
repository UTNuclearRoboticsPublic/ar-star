#! /bin/bash
echo "Building Robofleet Packages ..."

# setup
AR_STAR_ROS_PATH=$(rospack find ar_star_ros)
cd $AR_STAR_ROS_PATH/../config
RF_PKG_FILE="rf_pkgs.txt"
RF_PKG_BUILD_FILE=".rf_build_pkgs.txt"

# append _robofleet to caktin build only those files
cp "$RF_PKG_FILE" "$RF_PKG_BUILD_FILE"
sed -i 's/$/_robofleet/' "$RF_PKG_BUILD_FILE"

# generate and build
cd $AR_STAR_ROS_PATH/../..
rm -rf robofleet_pkgs
rosrun robofleet_client generate_plugin_pkg.py -o ./robofleet_pkgs -w $(cat $AR_STAR_ROS_PATH/../config/$RF_PKG_FILE)
cd ..
catkin build $(cat $AR_STAR_ROS_PATH/../config/$RF_PKG_BUILD_FILE)

# clean up
cd $AR_STAR_ROS_PATH/../config
rm $RF_PKG_BUILD_FILE
source $AR_STAR_ROS_PATH/../../../devel/setup.bash