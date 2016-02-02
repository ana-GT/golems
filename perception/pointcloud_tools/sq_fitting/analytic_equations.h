/**
 * @file analytic_equations.h
 * @brief Analytic solutions to SQ equations (derived using MATLAB)
 */
#pragma once

#include "perception/pointcloud_tools/sq_fitting/SQ_parameters.h"

/**
 * @structure levmar_data
 * @brief Contains the cloud's points info 
 */
struct levmar_data {
    double* x;
    double* y;
    double* z;
    int num;
};


double f_SQ( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &_x, const double &_y, const double &_z );

double f_SQ( const SQ_parameters &_par, 
	     const double &_x, const double &_y, const double &_z );

void jac_SQ( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &_x, const double &_y, const double &_z,
	     double _J[11] );


void jac_SQ( const SQ_parameters &_par, 
	     const double &_x, const double &_y, const double &_z,
	     double _J[11] );


void levmar_fx( double *p, double* x,
		int m, int n, void *data );

void levmar_jac( double* p, double* jac,
		 int m, int n, void* data );

void levmar_tampering_fx( double *p, double* x, int m, int n, void* data );
void levmar_tampering_jac( double* p, double* jac,
		 int m, int n, void* data );
double error_SQ_tampering( const SQ_parameters &_par,  
			   const double &x, 
			   const double &y, 
			   const double &z );
