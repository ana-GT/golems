
/**
 * @file evaluated_eqs.h
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"

enum SQ_FX_T_TYPES {
  SQ_FX_RADIAL_T = 0,
  SQ_FX_ICHIM_T = 1,
  SQ_FX_SOLINA_T = 2,
  SQ_FX_CHEVALIER_T = 3,
  SQ_FX_5_T = 4,
  SQ_FX_6_T = 5,
  SQ_FX_OLD_T = 7,
  SQ_FX_CHECK_T = 8
};

/**
 * @class evaluated_sqs_t
 */
class evaluated_sqs_t {

 public:
  void getBoundingBox( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
		       double _dim[3],
		       double _trans[3],
		       double _rot[3] );
  bool minimize( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
		 SQ_parameters &_par, double &_e1, double &_e2, double &_e4,
		 int _type = SQ_FX_RADIAL_T );  

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
	       const double &K,
	       const double &x, const double &y, const double &z );
 
double Err_2( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &K,
	      const double &x, const double &y, const double &z );

double Err_4( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &K,
	      const double &x, const double &y, const double &z );


/*****************/
static void fr_add( double* p, double* x,
		    int m, int n, void* data );
static void Jr_add( double* p, double* jac,
		    int m, int n, void* data );

static void fs_add( double* p, double* x,
		    int m, int n, void* data );
static void Js_add( double* p, double* jac,
		    int m, int n, void* data );

static void fc_add( double* p, double* x,
		    int m, int n, void* data );
static void Jc_add( double* p, double* jac,
		    int m, int n, void* data );

static void f5_add( double* p, double* x,
		    int m, int n, void* data );
static void J5_add( double* p, double* jac,
		    int m, int n, void* data );

};

