#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "ar_star_ros/polygon_utils.hpp"

void LassoPointsCallback(const std_msgs::UInt8MultiArray msg)
{
    std::cout << "I heard you" << std::endl;
}

int main(int argc, char **argv)
{
    // lasso mode node
    ros::init(argc, argv, "lasso_mode");
    ros::NodeHandle n;

    // subscribe to lasso messages from hololens
    ros::Subscriber sub = n.subscribe("/hololens/surface_repair/lasso_points", 1000, LassoPointsCallback);

    ARStar::PolygonUtils PolygonHandle;

    // standard ros spinner
    ros::spin();
    return 0;
}