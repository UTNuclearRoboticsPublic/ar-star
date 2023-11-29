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

class PolygonUtils
{

public:

    void EarClippingTriangulate(
        PointCloud<PointXYZ>::Ptr cloud,
        bool PrintDebug,
        std::vector<Matrix3f>& poly_mesh,
        std::vector<int>& indexes)
    {
        PrintCloudXYZ(cloud);

        Vertices vertices;
        vertices.vertices.resize (cloud->size ());

        for (int i = 0; i < static_cast<int> (vertices.vertices.size ()); ++i)
        {
          vertices.vertices[i] = i;
        }

        PolygonMesh::Ptr mesh (new PolygonMesh);
        toPCLPointCloud2 (*cloud, mesh->cloud);
        mesh->polygons.push_back (vertices);

        EarClipping clipper;
        PolygonMesh::ConstPtr mesh_aux (mesh);
        clipper.setInputMesh (mesh_aux);

        PolygonMesh triangulated_mesh;
        clipper.process (triangulated_mesh);

        // Fill triangle vector
        Matrix3f triangle;
        for (size_t i = 0; i < triangulated_mesh.polygons.size (); ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                size_t k = triangulated_mesh.polygons[i].vertices[j];
                indexes.push_back(k);
                triangle.row(j) << cloud->points[k].x,
                                   cloud->points[k].y,
                                   cloud->points[k].z;
            }
            // for (int i = 0; i < triangle.rows(); ++i) {
            //     for (int j = 0; j < triangle.cols(); ++j) {
            //     std::cout << "Element at (" << i << ", " << j << ") is: " << triangle(i, j) << std::endl;
            //     }
            // }
            poly_mesh.push_back(triangle);
        }

        if (PrintDebug)
        {
            PrintTriangleIndexes(triangulated_mesh, 1);
        }
    }

    void ExtrudeTriangulatedPolygon(
        const std::vector<Matrix3f>& PolyMesh,
        const std::vector<int>& Indexes, 
        const float& ExtrusionDist,
        bool PrintDebug,
        std::vector<Matrix3f>& UpperPolyMesh,
        std::vector<Matrix3f>& LowerPolyMesh,
        std::vector<Vector3f>& UpperVertexes,
        std::vector<Vector3f>& LowerVertexes)
    {
        // init
        Vector3f edge_one, edge_two, scaled_norm, a, b, c;
        Vector3f vertex, up_vertex, low_vertex;
        Matrix3f upper_triangle, lower_triangle;

        // init polygon mesh full of triangles
        size_t poly_mesh_size = PolyMesh.size();
        UpperPolyMesh.resize(poly_mesh_size);
        LowerPolyMesh.resize(poly_mesh_size);

        // init new set of vertices for both the upper and lower polygon
        size_t vertex_size = poly_mesh_size + 2; 
        UpperVertexes.resize(vertex_size); 
        LowerVertexes.resize(vertex_size); 

        // extrude and create top and bottom poly mesh
        int indexes_counter{ 0 };
        for (size_t i {0}; i < poly_mesh_size; i++)
        {
            // make Vector3f
            a = PolyMesh[i].row(0);
            b = PolyMesh[i].row(1);
            c = PolyMesh[i].row(2);

            // cross to get normal and scale
            edge_one << b-a;                      
            edge_two << c-a;                       
            scaled_norm << edge_one.cross(edge_two);  
            scaled_norm.normalize();
            scaled_norm *= ExtrusionDist;

            for (size_t j {0}; j < 3; j++)
            {
                // extrude
                vertex = PolyMesh[i].row(j);
                up_vertex = vertex + scaled_norm;
                low_vertex = vertex - scaled_norm;

                // save the triangles
                upper_triangle.row(j) << up_vertex(0),
                                         up_vertex(1),
                                         up_vertex(2);  
                lower_triangle.row(j) << low_vertex(0),
                                         low_vertex(1),
                                         low_vertex(2);

                // save the vertex's
                UpperVertexes[Indexes[indexes_counter]] = up_vertex;
                LowerVertexes[Indexes[indexes_counter]] = low_vertex;
                indexes_counter++;
            }

            // add
            UpperPolyMesh[i] = upper_triangle;
            LowerPolyMesh[i] = lower_triangle;

        }

        if (PrintDebug)
        {
            //PrintPolyMesh(LowerPolyMesh);
            PrintVector3(UpperVertexes, 2);
            PrintVector3(LowerVertexes, 3);
        }

    }

    void WrapPolygon(
        const std::vector<Vector3f>& UpperVertices,
        const std::vector<Vector3f>& LowerVertices,
        bool PrintDebug,
        std::vector<Vector3f>& InterlockedVertices,
        std::vector<int>& Indexes,
        std::vector<Matrix3f>& PolyMesh)
    {
        size_t up_size = UpperVertices.size();
        size_t low_size = LowerVertices.size();
        if (up_size != low_size){return;} // whole thing blown

        size_t num_vert = up_size + low_size;
        InterlockedVertices.resize(num_vert);
        
        size_t iter{ 0 };
        for (size_t i{ 0 }; i < up_size; i++)
        {
            InterlockedVertices[iter] = UpperVertices[i];
            InterlockedVertices[iter+1] = LowerVertices[i];
            iter = iter + 2;
        }

        size_t n = num_vert;

        for (size_t i = 1; i < n; i++)
        {
            size_t prev_i = i - 1;
            size_t next_i = i + 1;

             if ((i % 2) == 0) {
                Indexes.push_back(next_i);
                Indexes.push_back(i);
                Indexes.push_back(prev_i);
                Matrix3f mesh_triangle;
                mesh_triangle.row(0) = InterlockedVertices[next_i];
                mesh_triangle.row(1) = InterlockedVertices[i];
                mesh_triangle.row(2) = InterlockedVertices[prev_i];
                PolyMesh.push_back(mesh_triangle);
            } else {
                Indexes.push_back(prev_i);
                Indexes.push_back(i);
                Indexes.push_back(next_i);
                Matrix3f mesh_triangle;
                mesh_triangle.row(0) = InterlockedVertices[prev_i];
                mesh_triangle.row(1) = InterlockedVertices[i];
                mesh_triangle.row(2) = InterlockedVertices[next_i];
                PolyMesh.push_back(mesh_triangle);
            }
            
            num_vert--;
            if(num_vert < 3)
            {
                break;
            }
        }

        // deal with connecting triangles
        size_t a = n - 2;
        size_t b = n - 1;
        size_t c = 1;
        Indexes.push_back(a);
        Indexes.push_back(b);
        Indexes.push_back(c);

        Matrix3f mesh_triangle_sec_last;
        mesh_triangle_sec_last.row(0) = InterlockedVertices[a];
        mesh_triangle_sec_last.row(1) = InterlockedVertices[b];
        mesh_triangle_sec_last.row(2) = InterlockedVertices[c];
        PolyMesh.push_back(mesh_triangle_sec_last);

        
        Indexes.push_back(a);
        Indexes.push_back(1);
        Indexes.push_back(0);
        Matrix3f mesh_triangle_last;
        mesh_triangle_last.row(0) = InterlockedVertices[a];
        mesh_triangle_last.row(1) = InterlockedVertices[1];
        mesh_triangle_last.row(2) = InterlockedVertices[0];
        PolyMesh.push_back(mesh_triangle_last);  

        if(PrintDebug)
        {
            PrintVector3(InterlockedVertices, 4);
            PrintIndexes(Indexes, 2);
        } 
    }

    void ConcatPolyMesh(
        const std::vector<Matrix3f>& PolyMeshA,
        const std::vector<Matrix3f>& PolyMeshB,
        const std::vector<Matrix3f>& PolyMeshC,
        bool PrintDebug,
        std::vector<Matrix3f>& OutPolyMesh)
    {
        std::vector<Matrix3f> ConcatPolyMesh;
        OutPolyMesh.reserve(PolyMeshA.size() + PolyMeshB.size() + PolyMeshC.size());
        OutPolyMesh.insert(OutPolyMesh.end(), PolyMeshA.begin(), PolyMeshA.end());
        OutPolyMesh.insert(OutPolyMesh.end(), PolyMeshB.begin(), PolyMeshB.end());
        OutPolyMesh.insert(OutPolyMesh.end(), PolyMeshC.begin(), PolyMeshC.end()); 

        if (PrintDebug) {std::cout << "OutPolyMesh: " << OutPolyMesh.size() << std::endl;} 
    }

    // ref: https://cs.nyu.edu/~panozzo/cg/02%20-%20Ray%20Tracing,%20C++.pdf
    bool IsPointInsideVolume( 
        const Vector3f& Point,
        const std::vector<Matrix3f>& PolyMesh,
        bool PrintDebug)
    {
        Vector3f pa, pb, pc, p, N;
        Vector3f ray_dir(2000.0f, 1000.0f, 0.0f);
        int intersectionCount {0};
        for (size_t i {0}; i < PolyMesh.size(); i++)
        {
            // make Vector3f
            pa = PolyMesh[i].row(0);
            pb = PolyMesh[i].row(1);
            pc = PolyMesh[i].row(2);

            Vector3f b(Point - pa);
            Matrix3f A;
            A << pb - pa,
                 pc - pa,
                -ray_dir;

            Vector3f x = A.colPivHouseholderQr().solve(b);
            float u, v, t;
            u = x(0); // x position within triangle
            v = x(1); // y position within triangle
            t = x(2); // distance along ray

            if (u>=0 && v>=0 && u+v<=1 && t>0) 
            {
                //p = Point + (t * ray_dir); // point of intersection
                //N = (pb-pa).cross(pc-pa).normalized(); // noraml to the triangle at the intersection point
                ++intersectionCount;
                // if (PrintDebug)
                // {
                //     std::cout << "intersects: " << intersectionCount << std::endl;
                //     std::cout << "intersection point: " << p << std::endl;
                // }
            }
        }

        return (intersectionCount % 2) == 1; // if odd number of times its inside
    }



private:

    void PrintTriangleIndexes(const PolygonMesh& triangulated_mesh, const int& PrintNum)
    {
        //std::cout << "poly size: " << triangulated_mesh.polygons.size () << std::endl;
        // for (const auto &polygon : triangulated_mesh.polygons)
        // {
        //     //std::cout << "poly vert size: " << polygon.vertices.XYZ << std::endl;
        // }

        std::cout << "index" << std::to_string(PrintNum) << " = [" << std::endl;
        for (int pi = 0; pi < static_cast<int> (triangulated_mesh.polygons.size ()); ++pi)
        {
          for (int vi = 0; vi < 3; ++vi)
          {
              std::cout << " " << triangulated_mesh.polygons[pi].vertices[vi];
              //Indexes.push_back(triangulated_mesh.polygons[pi].vertices[vi]);
          }
          std::cout << std::endl;
        }
        std::cout << "];" << std::endl;
    }

    void PrintIndexes(const std::vector<int>& Index, const int& PrintNum)
    {
        //std::cout << "poly size: " << triangulated_mesh.polygons.size () << std::endl;
        // for (const auto &polygon : triangulated_mesh.polygons)
        // {
        //     //std::cout << "poly vert size: " << polygon.vertices.XYZ << std::endl;
        // }
        int count = 0;
        std::cout << "index" << std::to_string(PrintNum) << " = [" << std::endl;
        for (int i = 0; i < (Index.size() / 3); i++)
        { 
            for (int vi = 0; vi < 3; ++vi)
            {
                std::cout << " " << Index[count+vi];
                //Indexes.push_back(triangulated_mesh.polygons[pi].vertices[vi]);
            }
            std::cout << std::endl;
            count += 3;
        }
        std::cout << "];" << std::endl;

        // std::cout << "index" << std::to_string(PrintNum) << " = [" << std::endl;
        // for (int pi = 0; pi < Index.size(); pi++)
        // {
        //   for (int vi = 0; vi < 3; ++vi)
        //   {
        //       std::cout << " " << Index[pi+vi];
        //       //Indexes.push_back(triangulated_mesh.polygons[pi].vertices[vi]);
        //   }
        //   std::cout << std::endl;
        // }
        // std::cout << "];" << std::endl;
    }

    void PrintCloudXYZ(PointCloud<PointXYZ>::Ptr cloud){
        std::cout << "X = [" << std::endl;
        for (size_t i = 0; i < cloud->size(); ++i) {
            std::cout << cloud->points[i].x << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;

        std::cout << "Y = [" << std::endl;
        for (size_t i = 0; i < cloud->size(); ++i) {
            std::cout << cloud->points[i].y << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;

        std::cout << "Z = [" << std::endl;
        for (size_t i = 0; i < cloud->size(); ++i) {
            std::cout << cloud->points[i].z << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;
    }

    void PrintPolyMesh(const std::vector<Matrix3f>& Mesh)
    {
        
        for (const Matrix3f& triangle : Mesh)
        {
            std::cout << "Triangle = [" << std::endl;
            std::cout << triangle << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;
    }

    void PrintVector3(const std::vector<Vector3f>& Vector, const int& VectorNum)
    {
        std::cout << "X" << std::to_string(VectorNum) << " = [" << std::endl;
        for (size_t i = 0; i < Vector.size(); ++i) {
            std::cout << Vector[i](0) << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;

        std::cout << "Y" << std::to_string(VectorNum) << " = [" << std::endl;
        for (size_t i = 0; i < Vector.size(); ++i) {
            std::cout << Vector[i](1) << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;

        std::cout << "Z" << std::to_string(VectorNum) << " = [" << std::endl;
        for (size_t i = 0; i < Vector.size(); ++i) {
            std::cout << Vector[i](2) << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;
    }

    void PrintVector3Lower(const std::vector<Vector3f>& Vector)
    {
        std::cout << "X3 = [" << std::endl;
        for (size_t i = 0; i < Vector.size(); ++i) {
            std::cout << Vector[i](0) << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;

        std::cout << "Y3 = [" << std::endl;
        for (size_t i = 0; i < Vector.size(); ++i) {
            std::cout << Vector[i](1) << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;

        std::cout << "Z3 = [" << std::endl;
        for (size_t i = 0; i < Vector.size(); ++i) {
            std::cout << Vector[i](2) << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;
    }

    void PrintVector3Combo(const std::vector<Vector3f>& Vector)
    {
        std::cout << "X4 = [" << std::endl;
        for (size_t i = 0; i < Vector.size(); ++i) {
            std::cout << Vector[i](0) << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;

        std::cout << "Y4 = [" << std::endl;
        for (size_t i = 0; i < Vector.size(); ++i) {
            std::cout << Vector[i](1) << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;

        std::cout << "Z4 = [" << std::endl;
        for (size_t i = 0; i < Vector.size(); ++i) {
            std::cout << Vector[i](2) << std::endl;
        }
        std::cout << "];" << std::endl;
        std::cout << std::endl;
    }


};
} // ARStar