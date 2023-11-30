///////////////////////////////////////////////////////////////////////////////
//      Title     : AR-STAR Lasso Mode Service Solver
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
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ar_star_ros/lasso_utils.hpp"
#include "ar_star_ros/highlight_utils.hpp"
#include "ar_star_ros/GetPointsInLasso.h"

ARStar::LassoUtils lasso_util;
ARStar::HighlightUtils highlight_util;
bool PrintDebug = true;

void GetPointsInPolygon(
    const sensor_msgs::PointCloud2& Cloud,
    const std::vector<Matrix3f>& Triangles,
    const PointCloud<PointXYZ>::Ptr LassoPolyPoints,
    const float& UniformRadius,
    const float& MaxSqrDistance,
    std_msgs::UInt8MultiArray& Points
    )
{
    // init
	int i{ 0 };
    float square_dist{ 0 };
    uint32_t x_offset{ 0 }; 
	uint32_t y_offset{ 0 };
	uint32_t z_offset{ 0 };
	uint32_t rgb_offset{ 0 };
	uint32_t data_index{ 0 };
	uint32_t rgb_values{ 0 };
    std::string pf_name{ "" };
    Eigen::Vector3f point;
    Eigen::Vector3f first_lasso_poly_point;

    // set
    first_lasso_poly_point << LassoPolyPoints->points[0].x,
                              LassoPolyPoints->points[0].y,
                              LassoPolyPoints->points[0].z;

	// get required point cloud parameters for looping through point cloud
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

    // iterate through the point cloud and 
    // check if each point in cloud and determine 
    // if it is inside the volume made up of triangles
    Points.data.resize(Cloud.height * Cloud.width);
	for (size_t row = 0; row < Cloud.height; ++row) {
		for (size_t col = 0; col < Cloud.width; ++col) {
		    
            // compute the index of the current point in point cloud data array
			data_index = row * Cloud.row_step + col * Cloud.point_step;
			
            // access and assign x, y, z values to eigen vector3f (make sure points are in Right Hand and Meters)
			point(0) = *(reinterpret_cast<const float*>(Cloud.data.data() + data_index + x_offset)); // x [m]
			point(1) = *(reinterpret_cast<const float*>(Cloud.data.data() + data_index + y_offset)); // y [m]
			point(2) = *(reinterpret_cast<const float*>(Cloud.data.data() + data_index + z_offset)); // z [m]

            // only process points in cloud that are within close proximity to lasso polygon
            square_dist = (first_lasso_poly_point - point).squaredNorm();

            if(square_dist < MaxSqrDistance)
            {
                // check if point is inside the highlight first, if not check if inside the volume
                Points.data[i] = highlight_util.IsPointInHighlight(point, UniformRadius, LassoPolyPoints) ? 1 : 
                                 (lasso_util.IsPointInsideVolume(point, Triangles) ? 1 : 0);
            }

            // increment
			i++;
		}
	}
}

bool HandleRequest(
    ar_star_ros::GetPointsInLasso::Request &Req,
    ar_star_ros::GetPointsInLasso::Response &Res)
{
    // init
    float max_sqr_dist;
    std::vector<int> tri_poly_indexes, side_indexes;
    std::vector<Eigen::Vector3f> upper_vertexes, lower_vertexes, interlocked_vertices;
    std::vector<Eigen::Matrix3f> tri_poly, upper_tri_poly, lower_tri_poly, side_tri, all_tri;

    // convert incoming lasso points into PCL cloud for further processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr lasso_poly(
        new pcl::PointCloud<pcl::PointXYZ>);
    lasso_poly->height = 1;
    for (const auto& point : Req.lasso.polygon.points)
        lasso_poly->points.emplace_back(point.x, point.y, point.z);
    lasso_poly->width = lasso_poly->size();

    // create triangulated volume from lasso polygon
    lasso_util.EarClippingTriangulate( // ------------------------------------------
        lasso_poly,                                                           // in
        tri_poly, tri_poly_indexes);                                          // out
    lasso_util.ExtrudeTriangulatedPolygon( // --------------------------------------
        tri_poly, tri_poly_indexes, Req.extrusion_depth,                      // in
        upper_tri_poly, lower_tri_poly,upper_vertexes, lower_vertexes);       // out
    lasso_util.WrapPolygon( // -----------------------------------------------------
        upper_vertexes, lower_vertexes,                                       // in
        interlocked_vertices, side_indexes, side_tri);                        // out
    lasso_util.ConcatPolyMesh( // --------------------------------------------------
        upper_tri_poly, lower_tri_poly, side_tri,                             // in
        all_tri);                                                             // out
    lasso_util.GetMaxSquaredDistanceInPolygon( // ----------------------------------
        lasso_poly,                                                           // in
        max_sqr_dist);                                                        // out
   
    // find all points in the polygon
    GetPointsInPolygon( // ---------------------------------------------------------   
        Req.cloud, all_tri, lasso_poly, Req.uniform_radius, max_sqr_dist,     // in 
        Res.lasso_points);                                                    // out

    return true;
}

int main(int argc, char **argv)
{
    // init
    ros::init(argc, argv, "lasso_solver");
    ros::NodeHandle n;

    // standard ros service
    ros::ServiceServer service = n.advertiseService("get_points_in_lasso", HandleRequest);
    ROS_INFO("GetPointsInLasso service is ready.");
    ros::spin();

    return 0;
}