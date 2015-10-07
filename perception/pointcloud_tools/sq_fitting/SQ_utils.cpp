/**
 * @file SQ_utils.cpp
 */
#include "SQ_utils.h"
#include <iostream>

namespace SQ_utils {
  
/**
 * @function create_mesh
 */
  void create_SQ_mesh( double _a, double _b, double _c,
		       double _e1, double _e2, int _N,
		       const char* _mesh_name ) {
    pcl::PolygonMesh mesh;
    double dim[3]; double e[2];

    dim[0] = _a; dim[1] = _b; dim[2] = _c;
    e[0] = _e1; e[1] = _e2;
					  
    SQ_utils::create_SQ_mesh( mesh, dim, e, _N, _mesh_name );
  }
  
  void create_SQ_mesh( pcl::PolygonMesh &_mesh,
		       double _dim[3],
		       double _e[2], int _N,
		       const char* _mesh_name ) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr p( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_down( new pcl::PointCloud<pcl::PointXYZ>() );
    
    p = sampleSQ_uniform<pcl::PointXYZ>( _dim[0], _dim[1], _dim[2], _e[0], _e[1], _N );
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
    pcl::io::savePLYFile( _mesh_name, _mesh);
    
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

} // namespace



/**
 * @function printParamsInfo
 * @brief Print in a human-friendly form the parameters info
 */
void printParamsInfo( const SQ_parameters &_par ) {
    
    std::cout << "* Axes:("<<_par.dim[0]<<", "<<_par.dim[1]<<", "<<_par.dim[2] <<") ";
    std::cout << " E:("<<_par.e[0]<< ", "<<_par.e[1]<<") ";
    std::cout << " T:("<<_par.trans[0] << ", "<< _par.trans[1]<<", "<< _par.trans[2] <<") ";
    std::cout << " R:("<<_par.rot[0]<< ", "<<_par.rot[1]<<", "<<_par.rot[2]<<")"<<std::endl;
}

/**
 * @function dTheta
 */
double dTheta( double K,
	       double e,
	       double a1, double a2,
	       double t ) {

  double num = (cos(t)*cos(t)*sin(t)*sin(t));
  double den1 = a1*a1*pow( fabs(cos(t)),2*e)*pow( fabs(sin(t)),4);
  double den2 = a2*a2*pow( fabs(sin(t)),2*e)*pow( fabs(cos(t)),4);
  return (K/e)*sqrt( num/(den1+den2) );
}


/**
 * @function dTheta
 */
double dTheta_0( double K,
		 double e,
		 double a1, double a2,
		 double t ) {
  double factor = K /a2 - pow(t, e);
  double po = pow( fabs(factor), 1.0/e );
  double m = fabs(po - t);
  return m;
}

/**
 * @function sampleSQ_uniform_pn
 */
void sampleSQ_uniform_pn( const double &_a1, 
			  const double &_a2,
			  const double &_a3,
			  const double &_e1,
			  const double &_e2,
			  const int &_N,
			  pcl::PointCloud<pcl::PointNormal>::Ptr &_pn ) {
  // Reset, just in case
  _pn->points.resize(0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr se1( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr se2( new pcl::PointCloud<pcl::PointXYZ>() );
  std::vector<double> c1, s1, c2, s2;
  sampleSE_uniform_pcs<pcl::PointXYZ>( 1, _a3, _e1, _N, se1, c1, s1 );
  sampleSE_uniform_pcs<pcl::PointXYZ>( _a1, _a2, _e2, _N, se2, c2, s2 );

  // s1 and s2 have angles like these: [-180, -90] [0,-90], [0,90], [180,90]
  // s1: From -PI/2 o PI/2. 
  pcl::PointXYZ p1, p2;
  int n = se1->points.size()/4;
  
  for( int i = n; i < n*3; ++i ) {
    p1 = se1->points[i];
    // s2: From PI to PI
    for( int j = 0; j < se2->points.size(); ++j ) {
      p2 = se2->points[j];
      
      pcl::PointNormal p;
      p.x = p1.x*p2.x;
      p.y = p1.x*p2.y;
      p.z = p1.y;

      Eigen::Vector3d nv;
      nv(0) = (1.0/_a1)*pow( fabs(c1[i]), (2.0-_e1) )*pow( fabs(c2[j]), (2.0-_e2) );
      nv(1) = (1.0/_a2)*pow( fabs(c1[i]), (2.0-_e1) )*pow( fabs(s2[j]), (2.0-_e2) );
      nv(2) = (1.0/_a3)*pow( fabs(s1[i]), (2.0-_e1) );
      
      if( c1[i]*c2[j] < 0 ) { nv(0) = -nv(0); }
      if( c1[i]*s2[j] < 0 ) { nv(1) = -nv(1); }
      if( s1[i] < 0 ) { nv(2) = -nv(2); }
      nv.normalize();

      p.normal_x = nv(0);
      p.normal_y = nv(1);
      p.normal_z = nv(2);
      
      _pn->points.push_back(p);
    }
  }

  _pn->width = _pn->points.size();
  _pn->height = 1;
}
