
/**
 * @file evaluated_eqs.h
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"
#include <perception/pointcloud_tools/sq_fitting/SQ_parameters.h>


template<typename PointT> 
void error_metric_t( const SQ_parameters &_par,
		       const typename pcl::PointCloud<PointT>::Ptr &_cloud,
		       double& _e_g, double &_e_r, double &_e_d);

template<typename PointT> 
void error_metric_t( double* p,
		       const typename pcl::PointCloud<PointT>::Ptr &_cloud,
		       double& _e_g, double &_e_r, double &_e_d);

template<typename PointT> 
double error_metric_g_t( const SQ_parameters &_par,
			   const typename pcl::PointCloud<PointT>::Ptr &_cloud ); 


// Radial Error
double Err_r_t( const double &a, const double &b, const double &c,
		  const double &e1, const double &e2,
		  const double &px, const double &py, const double &pz,
		  const double &ra, const double &pa, const double &ya,
		  const double &k,
		  const double &x, const double &y, const double &z );
// Error used in Duncan to stop iterations
double Err_d_t( const double &a, const double &b, const double &c,
		  const double &e1, const double &e2,
		  const double &px, const double &py, const double &pz,
		  const double &ra, const double &pa, const double &ya,
		  const double &k,
		  const double &x, const double &y, const double &z );
// Error Goodness-Of-Fit, as defined by Gupta and Bajcsy
double Err_g_t( const double &a, const double &b, const double &c,
		  const double &e1, const double &e2,
		  const double &px, const double &py, const double &pz,
		  const double &ra, const double &pa, const double &ya,
		  const double &k,
		  const double &x, const double &y, const double &z );


double fr_t( const double &a, const double &b, const double &c,
	       const double &e1, const double &e2,
	       const double &px, const double &py, const double &pz,
	       const double &ra, const double &pa, const double &ya,
	       const double &k,
	       const double &x, const double &y, const double &z );

void Jr_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &k,
	     const double &_x, const double &_y, const double &_z,
	     double _J[12] );

double fi_t( const double &a, const double &b, const double &c,
	       const double &e1, const double &e2,
	       const double &px, const double &py, const double &pz,
	       const double &ra, const double &pa, const double &ya,
	       const double &k,
	       const double &x, const double &y, const double &z );

void Ji_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &_x, const double &_y, const double &_z,
	     const double &k,
	     double _J[12] );

double fs_t( const double &a, const double &b, const double &c,
	       const double &e1, const double &e2,
	       const double &px, const double &py, const double &pz,
	       const double &ra, const double &pa, const double &ya,
	       const double &k,
	       const double &x, const double &y, const double &z );

void Js_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &_x, const double &_y, const double &_z,
	     const double &k,
	     double _J[12] );

double fc_t( const double &a, const double &b, const double &c,
	       const double &e1, const double &e2,
	       const double &px, const double &py, const double &pz,
	       const double &ra, const double &pa, const double &ya,
	       const double &k,
	       const double &x, const double &y, const double &z );

void Jc_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &k,
	     const double &_x, const double &_y, const double &_z,
	     double _J[12] );

double f5_t( const double &a, const double &b, const double &c,
	       const double &e1, const double &e2,
	       const double &px, const double &py, const double &pz,
	       const double &ra, const double &pa, const double &ya,
	       const double &k,
	       const double &x, const double &y, const double &z );

void J5_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &k,
	     const double &_x, const double &_y, const double &_z,
	     double _J[12] );


double f6_t( const double &a, const double &b, const double &c,
	       const double &e1, const double &e2,
	       const double &px, const double &py, const double &pz,
	       const double &ra, const double &pa, const double &ya,
	       const double &k,
	       const double &x, const double &y, const double &z );

void J6_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &k,
	   const double &_x, const double &_y, const double &_z,
	   double _J[12] );


/*****************/
void fr_add_t( double* p, double* x,
	     int m, int n, void* data );
void Jr_add_t( double* p, double* jac,
	     int m, int n, void* data );

void fi_add_t( double* p, double* x,
	     int m, int n, void* data );
void Ji_add_t( double* p, double* jac,
	     int m, int n, void* data );

void fs_add_t( double* p, double* x,
	     int m, int n, void* data );
void Js_add_t( double* p, double* jac,
	     int m, int n, void* data );

void fc_add_t( double* p, double* x,
	     int m, int n, void* data );
void Jc_add_t( double* p, double* jac,
	     int m, int n, void* data );

void f5_add_t( double* p, double* x,
	     int m, int n, void* data );
void J5_add_t( double* p, double* jac,
	     int m, int n, void* data );


void f6_add_t( double* p, double* x,
	     int m, int n, void* data );
void J6_add_t( double* p, double* jac,
	     int m, int n, void* data );


////////////////////////////////////////

/**
 * @function error_metric
 * @brief Errors goodness-of-fit, Radial distance, Duncan's stopping (GoF^2)
 */
template<typename PointT> 
void error_metric_t( const SQ_parameters &_par,
		     const typename pcl::PointCloud<PointT>::Ptr &_cloud,
		     double& _e_g, double &_e_r, double &_e_d ) {
  
  double* p = new double[12];
  p[0] = _par.dim[0];   p[1] = _par.dim[1];   p[2] = _par.dim[2];
  p[3] = _par.e[0];   p[4] = _par.e[1];
  p[5] = _par.trans[0]; p[6] = _par.trans[1];   p[7] = _par.trans[2];
  p[8] = _par.rot[0]; p[9] = _par.rot[1];   p[10] = _par.rot[2];
  p[11] = _par.tamp;
  return error_metric_t<PointT>( p, _cloud, _e_g, _e_r, _e_d );
}

/**
 * @brief
 */
template<typename PointT> 
void error_metric_t( double* p,
		     const typename pcl::PointCloud<PointT>::Ptr &_cloud,
		     double& _e_g, double &_e_r, double &_e_d) {

  _e_g = 0;
  _e_r = 0;
  _e_d = 0;
  
  typename pcl::PointCloud<PointT>::iterator it;
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {
    _e_g += Err_g_t( p[0], p[1], p[2],
		     p[3], p[4],
		     p[5], p[6], p[7],
		     p[8], p[9], p[10],
		     p[11], // k
		   (*it).x, (*it).y, (*it).z );

    _e_r += Err_r_t( p[0], p[1], p[2],
		     p[3], p[4],
		     p[5], p[6], p[7],
		     p[8], p[9], p[10],
		     p[11], // k
		     (*it).x, (*it).y, (*it).z );

    
    _e_d += Err_d_t( p[0], p[1], p[2],
		     p[3], p[4],
		     p[5], p[6], p[7],
		     p[8], p[9], p[10],
		     p[11], // k
		     (*it).x, (*it).y, (*it).z );
    
  }
  
  _e_g /= _cloud->points.size();
  _e_r /= _cloud->points.size();
  _e_d /= _cloud->points.size(); 
}

/**
 * @brief Get goodness-of-fit error for a pointcloud
 */
template<typename PointT> 
double error_metric_g_t( const SQ_parameters &_par,
			 const typename pcl::PointCloud<PointT>::Ptr &_cloud ) {

  double* p = new double[12];
  p[0] = _par.dim[0];   p[1] = _par.dim[1];   p[2] = _par.dim[2];
  p[3] = _par.e[0];   p[4] = _par.e[1];
  p[5] = _par.trans[0]; p[6] = _par.trans[1];   p[7] = _par.trans[2];
  p[8] = _par.rot[0]; p[9] = _par.rot[1];   p[10] = _par.rot[2];
  p[11] = _par.tamp;
  
  double er = 0;
  typename pcl::PointCloud<PointT>::iterator it;
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {

    er += Err_g_t( p[0], p[1], p[2],
		   p[3], p[4],
		   p[5], p[6], p[7],
		   p[8], p[9], p[10],
		   p[11], // k
		   (*it).x, (*it).y, (*it).z );
    
  }
 
  er /= _cloud->points.size();

  return er;
}



