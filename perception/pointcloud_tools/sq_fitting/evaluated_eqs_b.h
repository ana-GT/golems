
/**
 * @file evaluated_eqs.h
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"

enum SQ_FX_B_TYPES {
  SQ_FX_RADIAL_B = 0,
  SQ_FX_ICHIM_B = 1,
  SQ_FX_SOLINA_B = 2,
  SQ_FX_CHEVALIER_B = 3,
  SQ_FX_5_B = 4,
  SQ_FX_6_B = 5,
  SQ_FX_OLD_B = 7,
  SQ_FX_CHECK_B = 8
};

/**
 * @class evaluated_sqs_b
 */
class evaluated_sqs_b {

 public:
  void getBoundingBox( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
		       double _dim[3],
		       double _trans[3],
		       double _rot[3] );
  bool minimize( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
		 SQ_parameters &_par, double &_e1, double &_e2, double &_e4,
		 int _type = SQ_FX_RADIAL_B );  

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
static void fs_add( double* p, double* x,
		    int m, int n, void* data );
static void Js_add( double* p, double* jac,
		    int m, int n, void* data );

static void fc_add( double* p, double* x,
		    int m, int n, void* data );
static void Jc_add( double* p, double* jac,
		    int m, int n, void* data );


};

