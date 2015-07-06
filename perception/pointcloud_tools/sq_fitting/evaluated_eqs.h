
/**
 * @file evaluated_eqs.h
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include <SQ_utils.h>

enum SQ_FX_TYPES {
  SQ_FX_RADIAL = 0,
  SQ_FX_ICHIM = 1,
  SQ_FX_SOLINA = 2,
  SQ_FX_CHEVALIER = 3,
  SQ_FX_5 = 4,
  SQ_FX_6 = 5
};

/**
 * @class evaluated_sqs
 */
class evaluated_sqs {

 public:
  void getBoundingBox( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
		      double _dim[3],
		      double _trans[3],
		      double _rot[3] );
  bool minimize( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
		 SQ_parameters &_par, double &_e1, double &_e2, double &_e4,
		 int _type = SQ_FX_RADIAL );  

};

void error_metric( const SQ_parameters &_par,
		   const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
		   double& _e1, double &_e2, double &_e4);

void error_metric( double* p,
		   const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
		   double& _e1, double &_e2, double &_e4);

double Err_1( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &x, const double &y, const double &z );

double Err_2( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &x, const double &y, const double &z );

double Err_4( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &x, const double &y, const double &z );


double fr( const double &a, const double &b, const double &c,
	    const double &e1, const double &e2,
	    const double &px, const double &py, const double &pz,
	    const double &ra, const double &pa, const double &ya,
	    const double &x, const double &y, const double &z );

void Jr( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &_x, const double &_y, const double &_z,
	 double _J[11] );

double fi( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z );

void Ji( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &_x, const double &_y, const double &_z,
	 double _J[11] );

double fs( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z );

void Js( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &_x, const double &_y, const double &_z,
	 double _J[11] );

double fc( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z );

void Jc( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &_x, const double &_y, const double &_z,
	 double _J[11] );

double f5( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z );

void J5( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &_x, const double &_y, const double &_z,
	 double _J[11] );


double f6( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z );

void J6( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &_x, const double &_y, const double &_z,
	 double _J[11] );


/*****************/
void fr_add( double* p, double* x,
	     int m, int n, void* data );
void Jr_add( double* p, double* jac,
	     int m, int n, void* data );

void fi_add( double* p, double* x,
	     int m, int n, void* data );
void Ji_add( double* p, double* jac,
	     int m, int n, void* data );

void fs_add( double* p, double* x,
	     int m, int n, void* data );
void Js_add( double* p, double* jac,
	     int m, int n, void* data );

void fc_add( double* p, double* x,
	     int m, int n, void* data );
void Jc_add( double* p, double* jac,
	     int m, int n, void* data );

void f5_add( double* p, double* x,
	     int m, int n, void* data );
void J5_add( double* p, double* jac,
	     int m, int n, void* data );


void f6_add( double* p, double* x,
	     int m, int n, void* data );
void J6_add( double* p, double* jac,
	     int m, int n, void* data );

