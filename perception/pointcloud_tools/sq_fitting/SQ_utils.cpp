/**
 * @file SQ_utils.cpp
 */
#include "SQ_utils.h"
#include <iostream>

namespace SQ_utils {
  
/**
 * @function create_mesh
 */
  void create_SQ_mesh( SQ_parameters _p,
		       int _N,
		       const char* _mesh_name, bool _applyTransform ) {
    pcl::PolygonMesh mesh;
					  
    SQ_utils::create_SQ_mesh( mesh, _p, _N, _mesh_name, _applyTransform );
  }
  
  void create_SQ_mesh( pcl::PolygonMesh &_mesh,
		       SQ_parameters _p, int _N,
		       const char* _mesh_name,
                       bool _applyTransform ) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr p( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_down( new pcl::PointCloud<pcl::PointXYZ>() );
    
    p = sampleSQ_uniform<pcl::PointXYZ>( _p, _applyTransform ); // Don't apply rigid transform
    downsampling<pcl::PointXYZ>( p, 0.0025, p_down );
    
    // Convex Hull
    pcl::ConvexHull<pcl::PointXYZ> chull;
    pcl::PointCloud<pcl::PointXYZ> points;
    std::vector<pcl::Vertices> polygons;
    
    chull.setInputCloud( p_down );
    chull.reconstruct( points, polygons );
    
    Eigen::Vector3d center; 
    if( _applyTransform ) { center << _p.trans[0], _p.trans[1], _p.trans[2]; }
    else { center = Eigen::Vector3d(0,0,0); }
    fix_mesh_faces( points, polygons, center );
    
    // Save mesh
    pcl::PCLPointCloud2 points2;
    pcl::toPCLPointCloud2<pcl::PointXYZ>( points, points2 );  
    _mesh.cloud = points2;
    _mesh.polygons = polygons;
    pcl::io::savePLYFile( _mesh_name, _mesh);
    
  }

  /**
   * @function param2Tf
   */
  Eigen::Isometry3d param2Tf( double _trans[3], 
			      double _rot[3] ) {

    Eigen::Isometry3d Twf; 
    Twf.setIdentity(); 
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd( _rot[2], Eigen::Vector3d::UnitZ() )*
      Eigen::AngleAxisd( _rot[1], Eigen::Vector3d::UnitY() )*
      Eigen::AngleAxisd( _rot[0], Eigen::Vector3d::UnitX() );
    Twf.linear() = rot; Twf.translation() << _trans[0], _trans[1], _trans[2];    

    return Twf;
  }

  /**
   * @function Tf2param
   */
  void Tf2param( Eigen::Isometry3d _Tf, 
		 double _trans[3],
		 double _rot[3] ) {
 
    Eigen::Vector3d rpy; rpy = _Tf.linear().eulerAngles(2,1,0);
    _rot[0] = rpy(2);
    _rot[1] = rpy(1);
    _rot[2] = rpy(0);
    
    _trans[0] = _Tf.translation()(0);
    _trans[1] = _Tf.translation()(1);
    _trans[2] = _Tf.translation()(2);
  }
 
  /*****/
  void convertMeshes(std::vector<SQ_parameters> _ps,
		     std::string _mesh_output_name ) { 

    // Create meshes
    std::vector<std::string> meshes_names;
    char mesh_name[100];

    // First mesh is special, as it is the reference for the rest
    sprintf( mesh_name, "multiple_mesh_%d.ply", 0 );
    SQ_utils::create_SQ_mesh( _ps[0], 25, mesh_name, false ); // Do not apply world transform
    // Store world transform
    Eigen::Isometry3d Tw0; Tw0 = param2Tf( _ps[0].trans, _ps[0].rot );
    // Store mesh
    meshes_names.push_back( mesh_name );

    for( int i = 1; i < _ps.size(); ++i ) {
      Eigen::Isometry3d Twi; Twi = param2Tf( _ps[i].trans, _ps[i].rot );
      Eigen::Isometry3d T0i; T0i= Tw0.inverse() * Twi;
      SQ_parameters pi; pi = _ps[i];
      printf("Before tf2param: trans: %f %f %f \n", pi.trans[0], pi.trans[1], pi.trans[2]);
      Tf2param( T0i, pi.trans, pi.rot );
      printf("After tf2param: trans: %f %f %f \n", pi.trans[0], pi.trans[1], pi.trans[2]);
      sprintf( mesh_name, "multiple_mesh_%d.ply", i );
      SQ_utils::create_SQ_mesh( pi, 25, mesh_name, true );
      meshes_names.push_back( mesh_name );
    }

    // Create combined mesh
    createMixMesh( _mesh_output_name,
		   meshes_names );
  }

    /**
     *
     */
    void createMixMesh( std::string output_name,
			std::vector<std::string> mesh_part ) {
      std::cout << "MESH FINALE NAME: "<< output_name<< std::endl;
      std::string script_name("writeMixMesh.ply");
      std::ofstream of( script_name.c_str(), std::ofstream::out );
      of << "import bpy \n";
      of << "import os \n";
      of << "import sys \n";
      of << "from bpy.types import(Operator)\n";
      
      of << "# Erase Cube\n";
      of << "for ob in bpy.context.scene.objects:\n";
      of << "\t ob.select = ob.type == 'MESH' and ob.name.startswith('Cube')\n";
      of << "bpy.ops.object.delete()\n";
      
      for( int i = mesh_part.size() - 1; i >= 0; --i ) {
	of << "filename_"<<i<<"='"<<mesh_part[i]<<"'\n";
	of << "bpy.ops.import_mesh.ply(filepath=filename_"<<i<<")\n";
      }
      of << "for ob in bpy.context.scene.objects:\n";
      of << "\t if ob.type == 'MESH':\n";
      of << "\t \t ob.select = True\n";
      of << "\t \t bpy.context.scene.objects.active = ob\n";
      of << "\t else:\n";
      of << "\t \t ob.select = False\n";
      of << "bpy.ops.object.join()\n";
      
      of << "bpy.ops.export_mesh.ply(filepath='"<<output_name.c_str()<<"')\n";
      
      of.close();
      char command[100];
      sprintf( command, " blender --python %s --background \n", script_name.c_str() );
      FILE* fp = NULL; fp = popen(command, "r");
      if( fp == NULL ) { printf("Error with popen \n"); }
      usleep(0.5*1e6);
      pclose(fp);
    }
  
  /**
   * @function fix_mesh_faces
   */
  void fix_mesh_faces( const pcl::PointCloud<pcl::PointXYZ> &_points,
		       std::vector<pcl::Vertices> &_polygons,
		       Eigen::Vector3d _center ) {
    
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
      b = b - _center;

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
    
    std::cout << "\t * Dim:("<<_par.dim[0]<<", "<<_par.dim[1]<<", "<<_par.dim[2] <<") ";
    std::cout << " e:("<<_par.e[0]<< ", "<<_par.e[1]<<") ";
    std::cout << " trans:("<<_par.trans[0] << ", "<< _par.trans[1]<<", "<< _par.trans[2] <<") ";
    std::cout << " rot:("<<_par.rot[0]<< ", "<<_par.rot[1]<<", "<<_par.rot[2]<<")"<<std::endl;
    
    switch( _par.type ) {
    case REGULAR:
      {} break;
    case TAMPERED:
      { std::cout << " tamp: "<<_par.tamp << std::endl; } break;
    case BENT:
      {} break;
    }

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
pcl::PointCloud<pcl::PointNormal>::Ptr sampleSQ_uniform_pn( SQ_parameters _p, double _res ) {

    // Get pointcloud from formula
    pcl::PointCloud<pcl::PointXYZ>::Ptr input( new pcl::PointCloud<pcl::PointXYZ>() );
    input = sampleSQ_uniform<pcl::PointXYZ>( _p );
  
    // Get the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (input);
    // Create search structure
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Set viewpoint to be the centroid of the SQ (the normals will all be pointing wrong, so we just have to flip them all)
    ne.setViewPoint( _p.trans[0], _p.trans[1], _p.trans[2] );

    // Use all neighbors in a sphere of radius 1cm
    ne.setRadiusSearch (0.01);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.compute (*normals);

    // Flip them
    pcl::PointCloud<pcl::PointNormal>::Ptr output( new pcl::PointCloud<pcl::PointNormal>() );
    pcl::PointCloud<pcl::PointNormal>::Ptr output_small( new pcl::PointCloud<pcl::PointNormal>() );

    pcl::PointCloud<pcl::PointXYZ>::iterator itp;
    pcl::PointCloud<pcl::Normal>::iterator it;
    for( it = normals->begin(), itp = input->begin(); it != normals->end(), itp != input->end(); ++it, ++itp ) {
      pcl::PointNormal P;
      P.x = (*itp).x; P.y = (*itp).y; P.z = (*itp).z;
      P.normal_x = -(*it).normal_x; ; P.normal_y = -(*it).normal_y; ; P.normal_z = -(*it).normal_z;
      output->points.push_back( P );
    }
    output->height = 1; output->width = output->points.size();

    // Downsample
    downsampling<pcl::PointNormal>( output,
			  _res,
			  output_small );

    return output_small;
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
