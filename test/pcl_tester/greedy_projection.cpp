#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

int
main ()
{
  // Load input file into a PointCloud<T> with an appropriate type
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PCLPointCloud2 cloud_blob;
//   pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
//   pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ point_0;
  point_0.x = 0.0;
  point_0.y = 0.0;
  point_0.z = 0.0;
  cloud->points.push_back(point_0);
  pcl::PointXYZ point_1;
  point_1.x = 0.0;
  point_1.y = 60.0;
  point_1.z = 0.0;
  cloud->points.push_back(point_1);
  pcl::PointXYZ point_2;
  point_2.x = 0.0;
  point_2.y = 60.0;
  point_2.z = 0.0;
  cloud->points.push_back(point_2);
  pcl::PointXYZ point_3;
  point_3.x = 60.0;
  point_3.y = 0.0;
  point_3.z = 0.0;
  cloud->points.push_back(point_3);
  pcl::PointXYZ point_4;
  point_4.x = 40.0;
  point_4.y = 0.0;
  point_4.z = 0.0;
  cloud->points.push_back(point_4);
  pcl::PointXYZ point_5;
  point_5.x = 40.0;
  point_5.y = 20.0;
  point_5.z = 0.0;
  cloud->points.push_back(point_5);
  pcl::PointXYZ point_6;
  point_6.x = 20.0;
  point_6.y = 20.0;
  point_6.z = 0.0;
  cloud->points.push_back(point_6);
  pcl::PointXYZ point_7;
  point_7.x = 20.0;
  point_7.y = 0.0;
  point_7.z = 0.0;
  cloud->points.push_back(point_7);
  pcl::PointXYZ point_8;
  point_8.x = 20.0;
  point_8.y = -20.0;
  point_8.z = 0.0;
  cloud->points.push_back(point_8);
  pcl::PointXYZ point_9;
  point_9.x = 0.0;
  point_9.y = -20.0;
  point_9.z = 0.0;
  cloud->points.push_back(point_9);

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (50);

  // Set typical values for the parameters
  gp3.setMu (25);
  gp3.setMaximumNearestNeighbors (10);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle((M_PI / 2) - 2); // 10 degrees
  gp3.setMaximumAngle((M_PI / 2) + 2); // 120 degrees
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);


  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Print vertices
    //std::cout << "Vertices:" << std::endl;
    //for (const pcl::PointXYZ& vertex : triangles.data) {
    //       std::cout << "  (" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")" << std::endl;
    //} 
// Print triangles
std::cout << "Triangles:" << std::endl;
for (const pcl::Vertices& polygon : triangles.polygons) {
        std::cout << "  Triangle with vertices:";
    for (int vertex_index : polygon.vertices) {
            std::cout << " " << vertex_index;
    }
    std::cout << std::endl;
}


  // Finish
  return (0);
}