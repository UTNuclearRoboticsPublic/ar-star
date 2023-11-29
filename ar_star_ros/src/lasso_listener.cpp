///////////////////////////////////////////////////////////////////////////////
//      Title     : AR-STAR Lasso Mode Listener Node
//      Project   : Nuclear and Applied Robotics Group (NRG) AR-STAR Project
//      Copyright : Copyright© The University of Texas at Austin, 2023. All rights reserved.
//                
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PolygonStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "ar_star_ros/polygon_utils.hpp"
#include "ar_star_ros/GetPointsInLasso.h"

ros::ServiceClient client_;
ros::Publisher lasso_points_pub_;
bool isCloudCallbackTriggered_ = false;
bool isLassoPointsCallbackTriggered_ = false;
sensor_msgs::PointCloud2 cloud_;
geometry_msgs::PolygonStamped polygon_;
ar_star_ros::GetPointsInLasso srv_;

void ServiceCall()
{
    if (isCloudCallbackTriggered_ && isLassoPointsCallbackTriggered_)
    {
        // call service
        if (client_.call(srv_))
        {
            lasso_points_pub_.publish(srv_.response.lasso_points);
            std::cout << std::endl << "Service Complete." << std::endl;
        }
        else
        {
          ROS_ERROR("Failed to call service add_two_ints");
          return;
        }
        isCloudCallbackTriggered_ = false;
        isLassoPointsCallbackTriggered_ = false;
    }
}

void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& Msg)
{
    std::cout << std::endl << "pointcloud message received." << std::endl;
    srv_.request.cloud = *Msg;
    isCloudCallbackTriggered_ = true;
    ServiceCall();
}

void LassoPointsCallback(const geometry_msgs::PolygonStamped::ConstPtr& Msg)
{
    std::cout << std::endl << "polygon message received." << std::endl;
    srv_.request.lasso = *Msg;
    isLassoPointsCallbackTriggered_ = true;
    ServiceCall();
}

int main(int argc, char **argv)
{
    // lasso mode node
    ros::init(argc, argv, "lasso_listener");
    ros::NodeHandle n;

    // subscribe to cloud & polygon messages from hololens
    ros::Subscriber cloud_sub = n.subscribe("/vaultbot/surface_repair/surface_points", 1000, CloudCallback);
    ros::Subscriber poly_sub = n.subscribe("/hololens/surface_repair/polygon_points", 1000, LassoPointsCallback);

    // create publisher for which points are inside lasso
    lasso_points_pub_ = n.advertise<std_msgs::UInt8MultiArray>("/server/surface_repair/lasso_points", 2);

    // create service client object
    client_ = n.serviceClient<ar_star_ros::GetPointsInLasso>("get_points_in_lasso");

    // get extrusion depth
    n.param<float>("extrusion_depth", srv_.request.extrusion_depth, 0.1);

    // standard ros spinner
    ros::spin();
    return 0;
}