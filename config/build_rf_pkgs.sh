#! /bin/bash
echo "Building Robofleet Packages ..."

# setup
PKG_NAME="ar_star_ros"
PATH_TO_ROS_PKG=$(rospack find $PKG_NAME)
cd $PATH_TO_ROS_PKG/../config
RF_PKG_FILE="rf_pkgs.txt"
RF_PKG_BUILD_FILE=".rf_build_pkgs.txt"

# append _robofleet to caktin build only those files
cp "$RF_PKG_FILE" "$RF_PKG_BUILD_FILE"
sed -i 's/$/_robofleet/' "$RF_PKG_BUILD_FILE"

# generate and build
cd $PATH_TO_ROS_PKG/../..
rm -rf robofleet_pkgs
rosrun robofleet_client generate_plugin_pkg.py -o ./robofleet_pkgs -w $(cat $PATH_TO_ROS_PKG/../config/$RF_PKG_FILE)
cd ..
catkin build $(cat $PATH_TO_ROS_PKG/../config/$RF_PKG_BUILD_FILE)

# clean up
cd $PATH_TO_ROS_PKG/../config
rm $RF_PKG_BUILD_FILE
source $PATH_TO_ROS_PKG/../../../devel/setup.bash