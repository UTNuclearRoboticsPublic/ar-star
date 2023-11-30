///////////////////////////////////////////////////////////////////////////////
//      Title     : AR-STAR Highlight Mode Utility Library
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

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <iostream>
#include <limits>

using namespace pcl;
using namespace Eigen;

namespace ARStar {

class HighlightUtils
{
public:

    bool IsPointInHighlight(
		const Eigen::Vector3f& CloudPoint,
		const float& UniformRadius,
		const PointCloud<PointXYZ>::Ptr HighlightedPoints)
    {
        // init 
		float square_dist{ 0 };
		Eigen::Vector3f highligh_point_loc;
		
		// get squared dist of uniform sphere used for highlighting
		float max_square_dist = UniformRadius* UniformRadius;

		// see if distance from sphere centroid is more than max_square distance
		for (size_t i {0}; i < HighlightedPoints->size(); i++)
		{
			highligh_point_loc << HighlightedPoints->points[i].x,
								  HighlightedPoints->points[i].y,
								  HighlightedPoints->points[i].z;

			// calculate squared distance
			square_dist = (CloudPoint - highligh_point_loc).squaredNorm();
			
			// return true if inside the uniform radius sphere
			if (square_dist < max_square_dist)
			{
				return true;
			}
		}

		return false;
    }

private:

};
} // ARStar