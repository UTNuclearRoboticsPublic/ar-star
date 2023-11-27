#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/PolygonStamped.h"
#include "ar_star_ros/polygon_utils.hpp"

void LassoPointsCallback(const geometry_msgs::PolygonStamped msg)
{
    std::cout << "I heard you" << std::endl;
}

int main(int argc, char **argv)
{
    // lasso mode node
    ros::init(argc, argv, "lasso_listener");
    ros::NodeHandle n;

    // subscribe to lasso messages from hololens
    ros::Subscriber sub = n.subscribe("/hololens/surface_repair/polygon_points", 1000, LassoPointsCallback);

    // standard ros spinner
    ros::spin();
    return 0;
}