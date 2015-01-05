/**
 * @file SQ_structs.h
 */
#pragma once

#include <ceres/sized_cost_function.h>
#include "analytic_equations.h"

/**
 * @struct SQBaseEquation
 * @brief Residual to be minimized
 */
struct SQBaseEquation {

  SQBaseEquation( double _x, double _y, double _z ) : 
    x_(_x), y_(_y), z_(_z) {}
  template<typename T> bool operator() ( const T* const dim, // 3
 					 const T* const e, // 2
					 const T* const trans, // 3
					 const T* const rot, // 3
					 T* residual ) const {
    
    // Get points in SQ frame
    T xe; T ye; T ze;

    T nx = cos( rot[2] )*cos( rot[1] );
    T ny = sin(rot[2])*cos(rot[1]);
    T nz = -sin(rot[1]);
    
    T ox = cos(rot[2])*sin(rot[1])*sin(rot[0]) - sin(rot[2])*cos(rot[0]);
    T oy = sin(rot[2])*sin(rot[1])*sin(rot[0]) + cos(rot[2])*cos(rot[0]);
    T oz = cos(rot[1])*sin(rot[0]);
    
    T ax = cos(rot[2])*sin(rot[1])*cos(rot[0]) + sin(rot[2])*sin(rot[0]);
    T ay = sin(rot[2])*sin(rot[1])*cos(rot[0]) - cos(rot[2])*sin(rot[0]);
    T az = cos(rot[1])*cos(rot[0]);

    xe = nx*T(x_) + ny*T(y_) + nz*T(z_) - trans[0]*nx - trans[1]*ny - trans[2]*nz;
    ye = ox*T(x_) + oy*T(y_) + oz*T(z_) - trans[0]*ox - trans[1]*oy - trans[2]*oz;
    ze = ax*T(x_) + ay*T(y_) + az*T(z_) - trans[0]*ax - trans[1]*ay - trans[2]*az;

    T F; T fxy; T fz;

    fxy = pow( ceres::abs(xe/dim[0]), T(2.0)/e[1] ) + pow( ceres::abs(ye/dim[1]), T(2.0)/e[1] );
    fz =  pow( ceres::abs(ze/dim[2]), T(2.0)/e[0] );
    F = pow( ceres::abs(fxy), T(e[1]/e[0]) ) + fz;    
    residual[0] = sqrt(dim[0]*dim[1]*dim[2])*( pow(F, e[1]) -T(1.0) );
    return true;
    
  }

private:
    double x_;
    double y_;
    double z_;
};


class SQCostFunction : public ceres::SizedCostFunction<1, 3,2,3,3> {

 public:
    SQCostFunction( double _x, double _y, double _z ) {
	x = _x; y = _y; z = _z;
    }

    virtual ~SQCostFunction() {}
    virtual bool Evaluate( double const* const* _pars,
                           double *residuals,
                           double **jacobians ) const {
	
	// dim
	double a = _pars[0][0];
	double b = _pars[0][1];
	double c = _pars[0][2];

	// e
	double e1 = _pars[1][0];
	double e2 = _pars[1][1];

	// trans
	double px = _pars[2][0];
	double py = _pars[2][1];
	double pz = _pars[2][2];

	// rot
	double ra = _pars[3][0];
	double pa = _pars[3][1];
	double ya = _pars[3][2];

	residuals[0] = f_SQ(a,b,c,
			    e1,e2,
			    px,py,pz,
			    ra,pa,ya,
			    x,y,z);

	if( jacobians != NULL && jacobians[0] != NULL ) {
	    double J[11];
	    jac_SQ( a, b, c, e1, e2,
		    px, py, pz,
		    ra, pa, ya,
		    x,y,z, J );

	    jacobians[0][0] = J[0];
	    jacobians[0][1] = J[1];
	    jacobians[0][2] = J[2];

	    jacobians[1][0] = J[3];
	    jacobians[1][1] = J[4];

	    jacobians[2][0] = J[5];
	    jacobians[2][1] = J[6];
	    jacobians[2][2] = J[7];

	    jacobians[3][0] = J[8];
	    jacobians[3][1] = J[9];
	    jacobians[3][2] = J[10];
	}


    }
    
    
    double x, y, z;
    
};
