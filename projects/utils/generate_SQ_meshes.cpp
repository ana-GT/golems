
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <stdint.h>

#include <pcl/surface/poisson.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/io/ply_io.h>

#include <stdio.h>

// SQ, BB
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"

void create_mesh( pcl::PolygonMesh &_mesh,
		  Eigen::Vector3d _dim,
		  Eigen::Vector2d _e, 
		  int _N,
		  char* _mesh_name );

void fix_mesh_faces( const pcl::PointCloud<pcl::PointXYZ> &_points,
		     std::vector<pcl::Vertices> &_polygons );

int main( int argc, char* argv[] ) {
  
  // Generate meshes 
  int num_shapes = 13;
  
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > e;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > dim; 
  std::vector<std::string> names;

  // Cubic 666
  e.push_back( Eigen::Vector2d(0.1, 0.1 ) );
  dim.push_back( Eigen::Vector3d( 0.03, 0.03, 0.03 ) );
  names.push_back( std::string("cubic_6_6_6") );

  // Cubic 999
  e.push_back( Eigen::Vector2d(0.1, 0.1 ) );
  dim.push_back( Eigen::Vector3d( 0.045, 0.045, 0.045 ) );
  names.push_back( std::string("cubic_9_9_9") );
  
  // Cubic cylinder 7710
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.035, 0.035, 0.05 ) );
  names.push_back( std::string("cubic_cylinder_7_7_10") );

  // Cubic cylinder 10x10x14
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.05, 0.05, 0.07 ) );
  names.push_back( std::string("cubic_cylinder_10_10_14") );

  // Sphere 7x7x7
  e.push_back( Eigen::Vector2d(1.0, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.035, 0.035, 0.035 ) );
  names.push_back( std::string("sphere_7_7_7") );

  // Sphere 9x9x9
  e.push_back( Eigen::Vector2d(1.0, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.045, 0.045, 0.045 ) );
  names.push_back( std::string("sphere_9_9_9") );

  // Sphere 11x11x11
  e.push_back( Eigen::Vector2d(1.0, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.055, 0.055, 0.055 ) );
  names.push_back( std::string("sphere_11_11_11") );

  // Cylinder 5x5x12
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.025, 0.025, 0.06 ) );
  names.push_back( std::string("cylinder_5_5_12") );

  // Cylinder 8x8x24
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.04, 0.04, 0.12 ) );
  names.push_back( std::string("cylinder_8_8_24") );

  // Ellipsoid 
  e.push_back( Eigen::Vector2d(1.0, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.025, 0.025, 0.03 ) );
  names.push_back( std::string("ellipsoid_5_5_6") );

  // Ellipsoid 
  e.push_back( Eigen::Vector2d(0.5, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.035, 0.035, 0.04 ) );
  names.push_back( std::string("ellipsoid_7_7_8") );

  // Ellipsoid 
  e.push_back( Eigen::Vector2d(0.75, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.045, 0.045, 0.06 ) );
  names.push_back( std::string("ellipsoid_9_9_12_a") );

  // Ellipsoid - 2
  e.push_back( Eigen::Vector2d(1.0, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.045, 0.045, 0.06 ) );
  names.push_back( std::string("ellipsoid_9_9_12_b") );



  /*
  //----------- EQUANT -------------------
  // 1. Cubic
  e.push_back( Eigen::Vector2d(0.1, 0.1 ) );
  dim.push_back( Eigen::Vector3d( 0.075, 0.075, 0.075 ) );
  names.push_back( std::string("cubic") );
  // 2. Cubic Cylinder
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.075, 0.075, 0.075 ) );
  names.push_back( std::string("cubic_cylinder") );
  // 3. Sphere
  e.push_back( Eigen::Vector2d(1.0, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.075, 0.075, 0.075 ) );
  names.push_back( std::string("sphere") );
  //---------- PROLATE ---------------
  // 4. Long prism
  e.push_back( Eigen::Vector2d(0.1, 0.1 ) );
  dim.push_back( Eigen::Vector3d( 0.05, 0.04, 0.075 ) );
  names.push_back( std::string("long_prism") );
  // 5. Cylinder
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.04, 0.04, 0.075 ) );
  names.push_back( std::string("cylinder") );
  // 6. Long ellipse
  e.push_back( Eigen::Vector2d(0.5, 1.0) );
  dim.push_back( Eigen::Vector3d( 0.075, 0.02, 0.03 ) );
  names.push_back( std::string("long_ellipse") );
  // 7. Long ellipsoid 
  e.push_back( Eigen::Vector2d(1.2, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.035, 0.035, 0.075 ) );
  names.push_back( std::string("long_ellipsoid") );
  
  //---------- OBLATE ------------------
  // 8. Short prism
  e.push_back( Eigen::Vector2d(0.1, 0.1 ) );
  dim.push_back( Eigen::Vector3d( 0.04, 0.05, 0.02 ) );
  names.push_back( std::string("short_prism") );
  // 9. Short ellipse
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.02, 0.03, 0.10 ) );
  names.push_back( std::string("short_ellipse") );
  // 10. Disk
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.05, 0.05, 0.02 ) );
  names.push_back( std::string("disk") );
  // 11. Short ellipsoid
  e.push_back( Eigen::Vector2d(1.9, 1.9 ) );
  dim.push_back( Eigen::Vector3d( 0.03, 0.04, 0.075 ) );
  names.push_back( std::string("short_ellipsoid") );
  // 12. Irregular
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.05, 0.05, 0.02 ) );
  names.push_back( std::string("irregular") );

  // 13. Ellipse A
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.05, 0.05, 0.02 ) );
  names.push_back( std::string("ellipse_A") );

  // 14. Ellipse B
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.05, 0.05, 0.02 ) );
  names.push_back( std::string("ellipse_B") );

  // 15. Ellipse C
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.05, 0.05, 0.02 ) );
  names.push_back( std::string("ellipse_C") );

  // 16. Ellipsoid
  e.push_back( Eigen::Vector2d(0.1, 1.0 ) );
  dim.push_back( Eigen::Vector3d( 0.05, 0.05, 0.02 ) );
  names.push_back( std::string("ellipsoid") );
  */
  
  // Create meshes
  for( unsigned int i = 0; i < num_shapes; ++i ) {
    pcl::PolygonMesh mesh;
    char name[50]; sprintf(name,"%s.ply", names[i].c_str() );

    create_mesh( mesh, dim[i], e[i], 100, name );   

  }
  
  return 0;
}


/**
 * @function create_mesh
 */
void create_mesh( pcl::PolygonMesh &_mesh,
		  Eigen::Vector3d _dim,
		  Eigen::Vector2d _e, 
		  int _N,
		  char* _mesh_name ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr p( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_down( new pcl::PointCloud<pcl::PointXYZ>() );

  p = sampleSQ_uniform<pcl::PointXYZ>( _dim(0), _dim(1), _dim(2), _e(0), _e(1), _N );

  downsampling<pcl::PointXYZ>( p, 0.0025, p_down );

  // Convex Hull
  pcl::ConvexHull<pcl::PointXYZ> chull;
  pcl::PointCloud<pcl::PointXYZ> points;
  std::vector<pcl::Vertices> polygons;
  
  chull.setInputCloud( p_down );
  chull.reconstruct( points, polygons );
  fix_mesh_faces( points, polygons );

  // Save mesh
  pcl::PCLPointCloud2 points2;
  pcl::toPCLPointCloud2<pcl::PointXYZ>( points, points2 ); 
  _mesh.cloud = points2;
  _mesh.polygons = polygons;
  pcl::io::savePLYFile( _mesh_name, _mesh );
  
}

/**
 * @function fix_mesh_faces
 */
void fix_mesh_faces( const pcl::PointCloud<pcl::PointXYZ> &_points,
		     std::vector<pcl::Vertices> &_polygons ) {

  pcl::PointXYZ p1, p2, p3;
  Eigen::Vector3d b;
  Eigen::Vector3d p21, p31;
  Eigen::Vector3d N;
  int v2, v3;
  for( std::vector<pcl::Vertices>::iterator it = _polygons.begin();
       it != _polygons.end(); ++it ) {
    
    if( (*it).vertices.size() != 3 ) {
      continue;
    }

    p1 = _points[ (*it).vertices[0] ];
    p2 = _points[ (*it).vertices[1] ];
    p3 = _points[ (*it).vertices[2] ];

    // Find baricenter
    b << ( p1.x + p2.x + p3.x )/ 3.0, ( p1.y + p2.y + p3.y )/ 3.0, ( p1.z + p2.z + p3.z )/ 3.0;
    
    // Find normal
    p21 << p2.x - p1.x, p2.y - p1.y, p2.z - p1.z;
    p31 << p3.x - p1.x, p3.y - p1.y, p3.z - p1.z;
    N = p21.cross( p31 );

    // If
    if( b.dot(N) < 0 ) {
      v2 = (*it).vertices[1];
      v3 = (*it).vertices[2];
      (*it).vertices[1] = v3;
      (*it).vertices[2] = v2;
    }
    
  } // end for

}


