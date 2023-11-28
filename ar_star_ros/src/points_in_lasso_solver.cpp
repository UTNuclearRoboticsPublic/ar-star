#include "ros/ros.h"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ar_star_ros/polygon_utils.hpp"
#include "ar_star_ros/GetPointsInLasso.h"

ARStar::PolygonUtils polygon;
bool PrintDebug = true;

void GetPointsInPolygon(
    const sensor_msgs::PointCloud2& Cloud,
    const std::vector<Matrix3f>& Triangles,
    std_msgs::UInt8MultiArray& Points
    )
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

            // check if point is inside the volume
            Points.data[i] = static_cast<uint8_t>(polygon.IsPointInsideVolume(point, Triangles, false));
			i++;
		}
	}
}

bool HandleRequest(
    ar_star_ros::GetPointsInLasso::Request &Req,
    ar_star_ros::GetPointsInLasso::Response &Res)
{
    // init
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
    polygon.EarClippingTriangulate( // ------------------------------------------
        lasso_poly, PrintDebug,                                           // in
        tri_poly, tri_poly_indexes);                                       // out
    polygon.ExtrudeTriangulatedPolygon( // --------------------------------------
        tri_poly, tri_poly_indexes, Req.extrusion_depth, PrintDebug, // in
        upper_tri_poly, lower_tri_poly,upper_vertexes, lower_vertexes);    // out
    polygon.WrapPolygon( // -----------------------------------------------------
        upper_vertexes, lower_vertexes, PrintDebug,                       // in
        interlocked_vertices, side_indexes, side_tri);                     // out
    polygon.ConcatPolyMesh( // --------------------------------------------------
        upper_tri_poly, lower_tri_poly, side_tri, false,             // in
        all_tri);                                                          // out

    // check if the points in cloud are in created volume
    GetPointsInPolygon(Req.cloud, all_tri, Res.lasso_points);

    return true;
}

int main(int argc, char **argv)
{
    // init
    ros::init(argc, argv, "points_in_lasso_solver");
    ros::NodeHandle n;

    // standard ros service
    ros::ServiceServer service = n.advertiseService("get_points_in_lasso", HandleRequest);
    ROS_INFO("GetPointsInLasso service is ready.");
    ros::spin();

    return 0;
}