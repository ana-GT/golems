/**
 * @file SQ_utils.cpp
 */
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include "SQ_utils.h"
#include <iostream>

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


/***
 * @function visualizeSQ
 * @brief Add sampled Super Quadric to viewer (red color by default)
 */
/*
void visualizeSQ(  boost::shared_ptr<pcl::visualization::PCLVisualizer> &_viewer,
		   const SQ_parameters &_par,
		   std::string _name,
		   int _r, int _g, int _b ) {

    // Create sample pointcloud of SQ expressed by _par
  pcl::PointCloud<pcl::PointXYZ>::Ptr sq_pcd( new pcl::PointCloud<pcl::PointXYZ>() );
  sq_pcd = sampleSQ_naive( _par );
  
    // Add it 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color( sq_pcd, _r, _g, _b );
  _viewer->addPointCloud<pcl::PointXYZ> ( sq_pcd, color, _name );	
  
}*/

/**
 * @function sampleSQ_uniform
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_uniform( const SQ_parameters &_par ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );


  // Get canonic SQ
  cloud_raw = sampleSQ_uniform( _par.dim[0], _par.dim[1], _par.dim[2],
				_par.e[0], _par.e[1] );
  // Apply transform
  Eigen::Matrix4d transf = Eigen::Matrix4d::Identity();
  transf.block(0,3,3,1) = Eigen::Vector3d( _par.trans[0], _par.trans[1], _par.trans[2] );
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd( _par.rot[2], Eigen::Vector3d::UnitZ() )*
    Eigen::AngleAxisd( _par.rot[1], Eigen::Vector3d::UnitY() )*
    Eigen::AngleAxisd( _par.rot[0], Eigen::Vector3d::UnitX() );
  transf.block(0,0,3,3) = rot;

  pcl::transformPointCloud( *cloud_raw,
			    *cloud,
			    transf );
  
  cloud->height = 1;
  cloud->width = cloud->points.size();
  
  return cloud;

}

/**
 * @function sampleSQ_uniform
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_uniform_t( const SQ_parameters &_par ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tamp( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  

  // Get canonic SQ
  cloud_raw = sampleSQ_uniform( _par.dim[0], _par.dim[1], _par.dim[2],
				_par.e[0], _par.e[1] );

  // Apply tampering
  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  pcl::PointXYZ p;
  double K = _par.tamp;
  
  for( it = cloud_raw->begin(); it != cloud_raw->end(); ++it ) {
    p = *it;
    p.y = ( 1 + (K/_par.dim[2])*p.z )*p.y;
    p.x = ( 1 + (K/_par.dim[2])*p.z )*p.x;
    cloud_tamp->points.push_back( p );
  }
  cloud_tamp->height = 1; cloud_tamp->width = cloud_tamp->points.size();

  // Apply transform
  Eigen::Matrix4d transf = Eigen::Matrix4d::Identity();
  transf.block(0,3,3,1) = Eigen::Vector3d( _par.trans[0], _par.trans[1], _par.trans[2] );
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd( _par.rot[2], Eigen::Vector3d::UnitZ() )*
    Eigen::AngleAxisd( _par.rot[1], Eigen::Vector3d::UnitY() )*
    Eigen::AngleAxisd( _par.rot[0], Eigen::Vector3d::UnitX() );
  transf.block(0,0,3,3) = rot;

  pcl::transformPointCloud( *cloud_tamp,
			    *cloud,
			    transf );
  
  cloud->height = 1;
  cloud->width = cloud->points.size();
  
  return cloud;
  
}

/**
 * @function sampleSQ_uniform_b
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_uniform_b( const SQ_parameters &_par ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bent( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  

  // Get canonic SQ
  cloud_raw = sampleSQ_uniform( _par.dim[0], _par.dim[1], _par.dim[2],
				_par.e[0], _par.e[1] );

  // Apply bending
  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  pcl::PointXYZ p;
  double R = _par.R;
  
  for( it = cloud_raw->begin(); it != cloud_raw->end(); ++it ) {
    p = *it;
    p.y = p.y - R + sqrt(R*R-p.z*p.z);
    p.z = R*sin(p.z/R);
    cloud_bent->points.push_back( p );
  }
  cloud_bent->height = 1; cloud_bent->width = cloud_bent->points.size();

  // Apply transform
  Eigen::Matrix4d transf = Eigen::Matrix4d::Identity();
  transf.block(0,3,3,1) = Eigen::Vector3d( _par.trans[0], _par.trans[1], _par.trans[2] );
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd( _par.rot[2], Eigen::Vector3d::UnitZ() )*
    Eigen::AngleAxisd( _par.rot[1], Eigen::Vector3d::UnitY() )*
    Eigen::AngleAxisd( _par.rot[0], Eigen::Vector3d::UnitX() );
  transf.block(0,0,3,3) = rot;

  pcl::transformPointCloud( *cloud_bent,
			    *cloud,
			    transf );
  
  cloud->height = 1;
  cloud->width = cloud->points.size();
  
  return cloud;
  
}

/**
 * @function sampleSQ_uniform_b2
 * @brief Sample bent shape with 2 parameters: alpha and k (from Solina)
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_uniform_b2( const SQ_parameters &_par ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bent( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  

  // Get canonic SQ
  cloud_raw = sampleSQ_uniform( _par.dim[0], _par.dim[1], _par.dim[2],
				_par.e[0], _par.e[1] );

  // Apply bending
  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  pcl::PointXYZ p;
  
  double k = _par.k;
  double alpha = _par.alpha;

  double x, y, z;
  double r,gamma, beta, R;
  printf("ALpha: %f \n", _par.alpha);
  for( it = cloud_raw->begin(); it != cloud_raw->end(); ++it ) {

    x = (*it).x; y = (*it).y; z = (*it).z;

    beta = atan2( y, x);
    r = cos(alpha - beta)*sqrt(x*x + y*y);
    gamma = z*k;
    R = (1/k) - cos(gamma)*(1.0/k-r);
    
    p.x = x + cos(alpha)*(R-r);
    p.y = y + sin(alpha)*(R-r);
    p.z = sin(gamma)*(1.0/k-1);
    
    cloud_bent->points.push_back( p );
  }
  cloud_bent->height = 1; cloud_bent->width = cloud_bent->points.size();

  // Apply transform
  Eigen::Matrix4d transf = Eigen::Matrix4d::Identity();
  transf.block(0,3,3,1) = Eigen::Vector3d( _par.trans[0], _par.trans[1], _par.trans[2] );
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd( _par.rot[2], Eigen::Vector3d::UnitZ() )*
    Eigen::AngleAxisd( _par.rot[1], Eigen::Vector3d::UnitY() )*
    Eigen::AngleAxisd( _par.rot[0], Eigen::Vector3d::UnitX() );
  transf.block(0,0,3,3) = rot;

  pcl::transformPointCloud( *cloud_bent,
			    *cloud,
			    transf );
  
  cloud->height = 1;
  cloud->width = cloud->points.size();
  
  return cloud;
  
}


/**
 * @function sampleSQ_naive
 * @brief Sample n \in [-PI/2, PI/2] and w \in [-PI,PI>
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_naive( const SQ_parameters &_par ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ> cloud_raw;
  
  double cn, sn, sw, cw;
  double n, w;
  int num_n, num_w; double dn, dw;
  
  dn = 5.0*M_PI / 180.0;
  dw = 5.0*M_PI / 180.0;
  num_n = (int)( M_PI / dn );
  num_w = (int)( 2*M_PI / dw );
  
  double a, b, c, e1, e2;
  a = _par.dim[0]; b = _par.dim[1]; c = _par.dim[2]; 
  e1 = _par.e[0]; e2 = _par.e[1];
  
  n = -M_PI / 2.0;
  for( int i = 0; i < num_n; ++i ) {
    n += dn;
    cn = cos(n); sn = sin(n);
    w = -M_PI;
    
    for( int j = 0; j < num_w; ++j ) {
      w += dw;
      cw = cos(w); sw = sin(w);
      pcl::PointXYZ p;
      p.x = a*pow( fabs(cn), e1 )*pow( fabs(cw), e2 );
      p.y = b*pow( fabs(cn), e1 )*pow( fabs(sw), e2 );
      p.z = c*pow( fabs(sn), e1 );
      
      // Assign signs, if needed
      if( cn*cw < 0 ) { p.x = -p.x; }
      if( cn*sw < 0 ) { p.y = -p.y; }
      if( sn < 0 ) { p.z = -p.z; }
      
      // Store
      cloud_raw.points.push_back(p);	    
    }
  }
  
  // Apply transform
  Eigen::Matrix4d transf = Eigen::Matrix4d::Identity();
  transf.block(0,3,3,1) = Eigen::Vector3d( _par.trans[0], _par.trans[1], _par.trans[2] );
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd( _par.rot[2], Eigen::Vector3d::UnitZ() )*
    Eigen::AngleAxisd( _par.rot[1], Eigen::Vector3d::UnitY() )*
    Eigen::AngleAxisd( _par.rot[0], Eigen::Vector3d::UnitX() );
  transf.block(0,0,3,3) = rot;
  pcl::transformPointCloud( cloud_raw,
			    *cloud,
			    transf );
  
  cloud->height = 1;
  cloud->width = cloud->points.size();
  
  return cloud;
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
 * @file sampleSQ_uniform
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSQ_uniform( const double &_a1, 
						      const double &_a2,
						      const double &_a3,
						      const double &_e1,
						      const double &_e2,
						      const int &_N ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );

  pcl::PointCloud<pcl::PointXYZ>::Ptr s1( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr s2( new pcl::PointCloud<pcl::PointXYZ>() );
  s1 = sampleSE_uniform( 1, _a3, _e1, _N );
  s2 = sampleSE_uniform( _a1, _a2, _e2, _N );


  // s1 and s2 have angles like these: [-180, -90] [0,-90], [0,90], [180,90]

  // s1: From -PI/2 o PI/2. s2: From PI to PI
  pcl::PointXYZ p1, p2;
  int n = s1->points.size() / 4; // -PI/2 to PI/2
  
  for( int i = n; i < 3*n; ++i ) {
    p1 = s1->points[i];
    for( int j = 0; j < s2->points.size(); ++j ) {
      p2 = s2->points[j];
      pcl::PointXYZ p;
      p.x = p1.x*p2.x;
      p.y = p1.x*p2.y;
      p.z = p1.y;
      cloud->points.push_back(p);
    }
  }
  
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

void sampleSQ_uniform_pn( const double &_a1, 
			  const double &_a2,
			  const double &_a3,
			  const double &_e1,
			  const double &_e2,
			  const int &_N,
			  pcl::PointCloud<pcl::PointNormal>::Ptr &_pn ) {
  // Reset, just in casei
  _pn->points.resize(0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr se1( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr se2( new pcl::PointCloud<pcl::PointXYZ>() );
  std::vector<double> c1, s1, c2, s2;
  sampleSE_uniform_pcs( 1, _a3, _e1, _N, se1, c1, s1 );
  sampleSE_uniform_pcs( _a1, _a2, _e2, _N, se2, c2, s2 );

  // s1 and s2 have angles like these: [-180, -90] [0,-90], [0,90], [180,90]
  // s1: From -PI/2 o PI/2. 
  pcl::PointXYZ p1, p2;
  
  for( int i = se1->points.size()/4; i < se1->points.size()*3/4; ++i ) {
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

/**
 * @function sampleSE_uniform
 * @brief Sample SuperEllipse with Pilu and Fischer method
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSE_uniform( const double &_a1, 
						      const  double &_a2,
						      const double &_e,
						      const int &_N ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr points( new pcl::PointCloud<pcl::PointXYZ>() );
  std::vector<double> ca, sa;

  sampleSE_uniform_pcs( _a1, _a2, _e, _N, points, ca, sa );
  return points;
} 

/**
 * @function sampleSE_uniform_pa
 */
void sampleSE_uniform_pcs( const double &_a1, 
			   const  double &_a2,
			   const double &_e,
			   const int &_N,
			   pcl::PointCloud<pcl::PointXYZ>::Ptr &_points,
			   std::vector<double> &_ca,
			   std::vector<double> &_sa ) {
  
  // Reset, just in case
  _points->points.resize(0);
  _ca.resize(0); _sa.resize(0);
  
  pcl::PointCloud<pcl::PointXYZ> cloud_base;
  std::vector<double> cos_base, sin_base;
  
  double theta;
  double thresh = 0.1;
  int numIter;
  int maxIter = 500;
  double K;

  if( _a1 < _a2 ) { K = 2*M_PI*_a1/(double)_N; } 
  else { K = 2*M_PI*_a2/(double)_N; }
 
  // theta \in [0,thresh]
  theta = 0;
  numIter = 0;
  do {
    double dt = dTheta_0(K, _e, _a1, _a2, theta );
    theta += dt;
    numIter++;

    if( dt != 0 ) {
      pcl::PointXYZ p;
      p.x = _a1*pow( fabs(cos(theta)), _e );
      p.y = _a2*pow( fabs(sin(theta)), _e );
      p.z = 0;
      cloud_base.push_back(p);
      cos_base.push_back(cos(theta));
      sin_base.push_back(sin(theta));
    }
  } while( theta < thresh   && numIter < maxIter );

  // theta \in [thresh, PI/2 - thresh]
  if( theta < thresh ) { theta = thresh; }
  numIter = 0;
  do {
    theta += dTheta( K, _e, _a1, _a2, theta ); 
    numIter++;

    pcl::PointXYZ p;
    p.x = _a1*pow( fabs(cos(theta)), _e );
    p.y = _a2*pow( fabs(sin(theta)), _e );
    p.z = 0;
    cloud_base.push_back(p);
    cos_base.push_back(cos(theta));
    sin_base.push_back(sin(theta));
  } while( theta < M_PI/2.0 - thresh  && numIter < maxIter );

 // theta \in [PI/2 - thresh, PI/2]
 double alpha = M_PI/2.0 - theta;
 numIter = 0;
 while( alpha > 0 && numIter < maxIter ) {
   alpha -= dTheta( K, _e, _a2, _a1, alpha );
   numIter++;
   
   pcl::PointXYZ p;
   p.x = _a1*pow( fabs(sin(alpha)), _e );
   p.y = _a2*pow( fabs(cos(alpha)), _e );
   p.z = 0;
   cloud_base.push_back(p);
   cos_base.push_back(sin(alpha));
   sin_base.push_back(cos(alpha));
 }
 
  
 // Store points [-PI, -PI/2], [0,-PI/2], [0,PI/2],[PI,PI/2] (notice the order, in case it is important
 double xsign[4] = {-1,1,1,-1};
 double ysign[4] = {-1,-1,1,1};
 
 for( int i = 0; i < 4; ++i ) {
   
   for( int j = 0; j < cloud_base.points.size(); ++j ) {
     pcl::PointXYZ p,b;
     b = cloud_base.points[j];
     p.x = xsign[i]*b.x;
     p.y = ysign[i]*b.y;
     p.z = b.z;
     
     _points->points.push_back(p);
     
     _ca.push_back( xsign[i]*cos_base[j] );
     _sa.push_back( ysign[i]*sin_base[j] );   
   }
 }
  
 _points->width = 1;
 _points->height = _points->points.size();

}


