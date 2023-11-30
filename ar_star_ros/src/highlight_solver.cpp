///////////////////////////////////////////////////////////////////////////////
//      Title     : AR-STAR Highlight Mode Service Solver
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

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ar_star_ros/highlight_utils.hpp"
#include "ar_star_ros/GetPointsInHighlight.h"

ARStar::HighlightUtils highlight_util;

void GetPointsInHighlightPolygon(
    const sensor_msgs::PointCloud2& Cloud,
    const PointCloud<PointXYZ>::Ptr HighlightedPoints,
    const float& UniformRadius,
    std_msgs::UInt8MultiArray& Points)
{
    // init
	int i{ 0 };
    uint32_t x_offset{ 0 }; 
	uint32_t y_offset{ 0 };
	uint32_t z_offset{ 0 };
	uint32_t rgb_offset{ 0 };
	uint32_t data_index{ 0 };
	uint32_t rgb_values{ 0 };
    std::string pf_name{ "" };
    Eigen::Vector3f point;

	// get required point cloud parameters for looping
	for (auto point_field : Cloud.fields)
	{
		pf_name = point_field.name;
		if (pf_name == "x") {
			x_offset = point_field.offset;
		}
		else if (pf_name == "y") {
			y_offset = point_field.offset;
		}
		else if (pf_name == "z") {
			z_offset = point_field.offset;
		}
		else if (pf_name == "rgb") {
			rgb_offset = point_field.offset;
		}
		else {
			//std::cout << "More parameters than x, y, z, rgb were found. Ignoring." << std::endl;
		}
	}

    // iterate through the point cloud and check if each point in cloud and determine if it is inside volume
    Points.data.resize(Cloud.height * Cloud.width);
	for (uint32_t row = 0; row < Cloud.height; ++row) {
		for (uint32_t col = 0; col < Cloud.width; ++col) {
		    
            // compute the index of the current point in point cloud data array
			data_index = row * Cloud.row_step + col * Cloud.point_step;
			
            // access and assign x, y, z values to eigen vector3f (make sure points are in Right Hand and Meters)
			point(0) = *(reinterpret_cast<const float*>(Cloud.data.data() + data_index + x_offset)); // x [m]
			point(1) = *(reinterpret_cast<const float*>(Cloud.data.data() + data_index + y_offset)); // y [m]
			point(2) = *(reinterpret_cast<const float*>(Cloud.data.data() + data_index + z_offset)); // z [m]

            // check if point is inside the highligh first, if not check if inside the volume
            Points.data[i] = highlight_util.IsPointInHighlight(point, UniformRadius, HighlightedPoints) ? 1 : 0;
			i++;
		}
	}
}

bool HandleRequest(
    ar_star_ros::GetPointsInHighlight::Request &Req,
    ar_star_ros::GetPointsInHighlight::Response &Res)
{
    // init
    std::vector<int> tri_poly_indexes, side_indexes;
    std::vector<Eigen::Vector3f> upper_vertexes, lower_vertexes, interlocked_vertices;
    std::vector<Eigen::Matrix3f> tri_poly, upper_tri_poly, lower_tri_poly, side_tri, all_tri;

    // convert incoming lasso points into PCL cloud for further processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr highlight_array(
        new pcl::PointCloud<pcl::PointXYZ>);
    highlight_array->height = 1;
    for (const auto& point : Req.highlight.polygon.points)
        highlight_array->points.emplace_back(point.x, point.y, point.z);
    highlight_array->width = highlight_array->size();

    // get points inside cloud
    GetPointsInHighlightPolygon(Req.cloud, highlight_array, Req.uniform_radius, Res.tagged_points);

    return true;
}

int main(int argc, char **argv)
{
    // init
    ros::init(argc, argv, "highlight_solver");
    ros::NodeHandle n;

    // standard ros service
    ros::ServiceServer service = n.advertiseService("get_points_in_highlight", HandleRequest);
    ROS_INFO("GetPointsInHighlight service is ready.");
    ros::spin();

    return 0;
}