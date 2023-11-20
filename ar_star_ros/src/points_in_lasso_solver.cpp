#include "ros/ros.h"
#include "ar_star_ros/GetPointsInLasso.h"

bool handle_request(ar_star_ros::GetPointsInLasso::Request &req,
                    ar_star_ros::GetPointsInLasso::Response &res)
{
    //res.sum = req.a + req.b;
    //ROS_INFO("Received a request: a=%ld, b=%ld. Returning sum=%ld", req.a, req.b, res.sum);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_in_lasso_solver");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("GetPointsInLasso", handle_request);
    ROS_INFO("GetPointsInLasso service is ready.");
    ros::spin();

    return 0;
}