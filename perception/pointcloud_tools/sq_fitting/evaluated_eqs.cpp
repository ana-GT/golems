

#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include "evaluated_eqs.h"
#include "levmar/levmar.h"
#include <math.h>


// COPIED FROM ANALITIC_EQUATIONS.H
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

/****/
void error_metric( double* p,
		   const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
		   double& _e1, double &_e2, double &_e4) {

  _e1 = 0;
  _e2 = 0;
  _e4 = 0;
  
  typename pcl::PointCloud<pcl::PointXYZ>::iterator it;
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {
    _e1 += Err_1( p[0], p[1], p[2],
		   p[3], p[4],
		   p[5], p[6], p[7],
		   p[8], p[9], p[10],
		   (*it).x, (*it).y, (*it).z );

    _e2 += Err_2( p[0], p[1], p[2],
		   p[3], p[4],
		   p[5], p[6], p[7],
		   p[8], p[9], p[10],
		   (*it).x, (*it).y, (*it).z );

    
    _e4 += Err_4( p[0], p[1], p[2],
		   p[3], p[4],
		   p[5], p[6], p[7],
		   p[8], p[9], p[10],
		   (*it).x, (*it).y, (*it).z );
    
  }
  
}


/****/
double Err_1( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &x, const double &y, const double &z ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23;

    t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t11 = t10-t4*t5*t6;
  t12 = cos(pa);
  t13 = px*t9-py*t11-t9*x+t11*y+pz*t4*t12-t4*t12*z;
  t14 = t4*t6;
  t15 = t14-t2*t3*t5;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t19 = px*t15-py*t18-t15*x+t18*y-pz*t3*t12+t3*t12*z;
  t20 = pz*t5-t5*z-px*t2*t12-py*t6*t12+t2*t12*x+t6*t12*y;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t23 = pow(pow(1.0/(a*a)*(t20*t20),t21)+pow(1.0/(b*b)*(t19*t19),t21),e2*t22)+pow(1.0/(c*c)*(t13*t13),t22)-1.0;
  
  return t23*t23;
}

/****/
double Err_2( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &x, const double &y, const double &z ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23;

    t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t11 = t10-t4*t5*t6;
  t12 = cos(pa);
  t13 = px*t9-py*t11-t9*x+t11*y+pz*t4*t12-t4*t12*z;
  t14 = t4*t6;
  t15 = t14-t2*t3*t5;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t19 = px*t15-py*t18-t15*x+t18*y-pz*t3*t12+t3*t12*z;
  t20 = pz*t5-t5*z-px*t2*t12-py*t6*t12+t2*t12*x+t6*t12*y;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t23 = pow(pow(pow(1.0/(a*a)*(t20*t20),t21)+pow(1.0/(b*b)*(t19*t19),t21),e2*t22)+pow(1.0/(c*c)*(t13*t13),t22),e1)-1.0;
  
  return t23*t23;
  
}

/****/
double Err_4( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &x, const double &y, const double &z ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45;

  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t24 = t4*t5*t6;
  t11 = t10-t24;
  t12 = cos(pa);
  t23 = px*t9;
  t25 = py*t11;
  t26 = t9*x;
  t27 = t11*y;
  t28 = pz*t4*t12;
  t29 = t4*t12*z;
  t13 = t23-t25-t26+t27+t28-t29;
  t14 = t4*t6;
  t31 = t2*t3*t5;
  t15 = t14-t31;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t32 = px*t15;
  t33 = py*t18;
  t34 = t15*x;
  t35 = t18*y;
  t36 = pz*t3*t12;
  t37 = t3*t12*z;
  t19 = t32-t33-t34+t35-t36+t37;
  t39 = pz*t5;
  t40 = t5*z;
  t41 = px*t2*t12;
  t42 = t2*t12*x;
  t43 = py*t6*t12;
  t44 = t6*t12*y;
  t20 = t39-t40-t41+t42-t43+t44;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t30 = t13*t13;
  t38 = t19*t19;
  t45 = t20*t20;

  return fabs(-(pow(pow(1.0/(c*c)*t30,t22)+pow(pow(1.0/(a*a)*t45,t21)+pow(1.0/(b*b)*t38,t21),e2*t22),e1*(-1.0/2.0))-1.0)*sqrt(t30+t38+t45));

}




/**
 * @function minimize
 */
bool evaluated_sqs::minimize( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
			      SQ_parameters &_par, double &_e1, double &_e2, double &_e4,
			      int _type ) {
  
  // Get parameters to start minimization
  double dim[3]; double trans[3]; double rot[3];
  this->getBoundingBox( _input, dim, trans, rot );

  // Set necessary parameters
  int n = _input->points.size();
  int m = 11; 
  double p[m]; // Parameters of SQ
  double y[n]; // Values we want to achieve
  
  double opts[LM_OPTS_SZ];
  double info[LM_INFO_SZ];
  
  opts[0] = LM_INIT_MU;
  opts[1] = 1E-15;
  opts[2] = 1E-15;
  opts[3] = 1E-20;
  opts[4] = LM_DIFF_DELTA;
  
  struct levmar_data data;
  data.x = new double[n];
  data.y = new double[n];
  data.z = new double[n];
  data.num = n;
  
  
  int i; int ret;
  typename pcl::PointCloud<pcl::PointXYZ>::iterator pit;
  for( pit = _input->begin(), i = 0; pit != _input->end(); ++pit, ++i ) {
    data.x[i] = (*pit).x;
    data.y[i] = (*pit).y;
    data.z[i] = (*pit).z;
  }
  
  // Set minimizer value to zero (could be 1, depending of what equation you are minimizing)
  for( i = 0; i < n; ++i ) { y[i] = 0.0; }
  
  // Initialize values for parameters p
  for( i = 0; i < 3; ++i ) { p[i] = dim[i];}
  for( i = 0; i < 2; ++i ) { p[i+3] = 1.0; }
  for( i = 0; i < 3; ++i ) { p[i+5] = trans[i]; }
  for( i = 0; i < 3; ++i ) { p[i+8] = rot[i]; }
  
  // Set limits
  double ub[m], lb[m];
  for( i = 0; i < 3; ++i ) { lb[i] = 0.01; ub[i] = 0.3; }
  for( i = 0; i < 2; ++i ) { lb[i+3] = 0.1; ub[i+3] = 1.9; }
  for( i = 0; i < 3; ++i ) { lb[i+5] = -1.5; ub[i+5] = 1.5; }
  for( i = 0; i < 3; ++i ) { lb[i+8] = -M_PI; ub[i+8] = M_PI; }

  switch( _type ) {

  case SQ_FX_RADIAL:
    ret = dlevmar_bc_der( fr_add,
			  Jr_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );
    break;
  case SQ_FX_ICHIM:
    ret = dlevmar_bc_der( fi_add,
			  Ji_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );
    break;

  case SQ_FX_SOLINA:
    ret = dlevmar_bc_der( fs_add,
			  Js_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );
    break;

  case SQ_FX_CHEVALIER:
    ret = dlevmar_bc_der( fc_add,
			  Jc_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );
    break;
  case SQ_FX_5:
    ret = dlevmar_bc_der( f5_add,
			  J5_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );
    break;

  case SQ_FX_6:
    ret = dlevmar_bc_der( f6_add,
			  J6_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );
    break;

    
  }
  
  // If stopped by invalid (TODO: Add other reasons)
  if( info[6] == 7 ) {
    _par.dim[0] = 0; _par.dim[1] = 0; _par.dim[2] = 0;
    _par.e[0] = 0; _par.e[1] = 0;
    _par.trans[0] = 0; _par.trans[1] = 0; _par.trans[2] = 0;
    _par.rot[0] = 0; _par.rot[1] = 0; _par.rot[2] = 0;
    _e1 = 10000; _e2 = 10000; _e4 = 10000; // A big number
    return false;
  } else {
    _par.dim[0] = p[0]; _par.dim[1] = p[1]; _par.dim[2] = p[2];
    _par.e[0] = p[3]; _par.e[1] = p[4];
    _par.trans[0] = p[5]; _par.trans[1] = p[6]; _par.trans[2] = p[7];
    _par.rot[0] = p[8]; _par.rot[1] = p[9]; _par.rot[2] = p[10];

    error_metric( p, _input, _e1, _e2, _e4 );
    
    
    return true;
  }
  
  
}

void evaluated_sqs::getBoundingBox( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
				   double _dim[3],
				   double _trans[3],
				   double _rot[3] ) {

  // 1. Compute the bounding box center
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid( *_cloud, centroid );
  _trans[0] = centroid(0);
  _trans[1] = centroid(1); 
  _trans[2] = centroid(2);

  // 2. Compute main axis orientations
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud( _cloud );
  Eigen::Vector3f eigVal = pca.getEigenValues();
  Eigen::Matrix3f eigVec = pca.getEigenVectors();
  // Make sure 3 vectors are normal w.r.t. each other
  
  eigVec.col(2) = eigVec.col(0); // Z
  Eigen::Vector3f v3 = (eigVec.col(1)).cross( eigVec.col(2) );
  eigVec.col(0) = v3; 
  
  /*
  Eigen::Vector3f v3 = (eigVec.col(0)).cross( eigVec.col(1) );
  eigVec.col(2) = v3;
  */

  Eigen::Vector3f rpy = eigVec.eulerAngles(2,1,0);
 
  _rot[0] = (double)rpy(2);
  _rot[1] = (double)rpy(1);
  _rot[2] = (double)rpy(0);

  // Transform _cloud
  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  transf.block(0,3,3,1) << (float)centroid(0), (float)centroid(1), (float)centroid(2);
  transf.block(0,0,3,3) = eigVec;

  Eigen::Matrix4f tinv; tinv = transf.inverse();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::transformPointCloud( *_cloud, *cloud_temp, tinv );

  // Get maximum and minimum
  pcl::PointXYZ minPt; pcl::PointXYZ maxPt;
  pcl::getMinMax3D( *cloud_temp, minPt, maxPt );
  
  _dim[0] = ( maxPt.x - minPt.x ) / 2.0;
  _dim[1] = ( maxPt.y - minPt.y ) / 2.0;
  _dim[2] = ( maxPt.z - minPt.z ) / 2.0;

}


/**
 * @function fr_add
 */
void fr_add( double* p, double* x,
	     int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fr( p[0], p[1], p[2],
	       p[3], p[4],
	       p[5], p[6], p[7],
	       p[8], p[9], p[10],
	       xi, yi, zi );
    
  }
}

/**
 * @function fs_add
 */
void fs_add( double* p, double* x,
	     int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fs( p[0], p[1], p[2],
	       p[3], p[4],
	       p[5], p[6], p[7],
	       p[8], p[9], p[10],
	       xi, yi, zi );
    
  }
}

/**
 * @function fi_add
 */
void fi_add( double* p, double* x,
	     int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fi( p[0], p[1], p[2],
	       p[3], p[4],
	       p[5], p[6], p[7],
	       p[8], p[9], p[10],
	       xi, yi, zi );
    
  }
}

/**
 * @function fc_add
 */
void fc_add( double* p, double* x,
	     int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fc( p[0], p[1], p[2],
	       p[3], p[4],
	       p[5], p[6], p[7],
	       p[8], p[9], p[10],
	       xi, yi, zi );
    
  }
}

/**
 * @function f5_add
 */
void f5_add( double* p, double* x,
	     int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = f5( p[0], p[1], p[2],
	       p[3], p[4],
	       p[5], p[6], p[7],
	       p[8], p[9], p[10],
	       xi, yi, zi );
    
  }
}

/**
 * @function f6_add
 */
void f6_add( double* p, double* x,
	     int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = f6( p[0], p[1], p[2],
	       p[3], p[4],
	       p[5], p[6], p[7],
	       p[8], p[9], p[10],
	       xi, yi, zi );
    
  }
}

/**
 * @function Jr_add
 */
void Jr_add( double* p, double* jac,
	     int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[11];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Jr( p[0], p[1], p[2],
	p[3], p[4],
	p[5], p[6], p[7],
	p[8], p[9], p[10],
	xi, yi, zi,
	J );

    for( int k = 0; k < 11; k++ ) {
      jac[11*i+k] = J[k];
    }
  }

}

/**
 * @function Js_add
 */
void Js_add( double* p, double* jac,
	     int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[11];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Js( p[0], p[1], p[2],
	p[3], p[4],
	p[5], p[6], p[7],
	p[8], p[9], p[10],
	xi, yi, zi,
	J );

    for( int k = 0; k < 11; k++ ) {
      jac[11*i+k] = J[k];
    }
  }

}

/**
 * @function Ji_add
 */
void Ji_add( double* p, double* jac,
	     int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[11];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Ji( p[0], p[1], p[2],
	p[3], p[4],
	p[5], p[6], p[7],
	p[8], p[9], p[10],
	xi, yi, zi,
	J );

    for( int k = 0; k < 11; k++ ) {
      jac[11*i+k] = J[k];
    }
  }

}

/**
 * @function Jc_add
 */
void Jc_add( double* p, double* jac,
	     int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[11];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Jc( p[0], p[1], p[2],
	p[3], p[4],
	p[5], p[6], p[7],
	p[8], p[9], p[10],
	xi, yi, zi,
	J );

    for( int k = 0; k < 11; k++ ) {
      jac[11*i+k] = J[k];
    }
  }

}

/**
 * @function J5_add
 */
void J5_add( double* p, double* jac,
	     int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[11];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    J5( p[0], p[1], p[2],
	p[3], p[4],
	p[5], p[6], p[7],
	p[8], p[9], p[10],
	xi, yi, zi,
	J );

    for( int k = 0; k < 11; k++ ) {
      jac[11*i+k] = J[k];
    }
  }

}

/**
 * @function J6_add
 */
void J6_add( double* p, double* jac,
	     int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[11];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    J6( p[0], p[1], p[2],
	p[3], p[4],
	p[5], p[6], p[7],
	p[8], p[9], p[10],
	xi, yi, zi,
	J );

    for( int k = 0; k < 11; k++ ) {
      jac[11*i+k] = J[k];
    }
  }

}




/**
 * @function fr
 */
double fr( const double &a, const double &b, const double &c,
	    const double &e1, const double &e2,
	    const double &px, const double &py, const double &pz,
	    const double &ra, const double &pa, const double &ya,
	    const double &x, const double &y, const double &z ) {
  
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46;

  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t25 = t4*t5*t6;
  t11 = t10-t25;
  t12 = cos(pa);
  t24 = px*t9;
  t26 = py*t11;
  t27 = t9*x;
  t28 = t11*y;
  t29 = pz*t4*t12;
  t30 = t4*t12*z;
  t13 = t24-t26-t27+t28+t29-t30;
  t14 = t4*t6;
  t32 = t2*t3*t5;
  t15 = t14-t32;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t33 = px*t15;
  t34 = py*t18;
  t35 = t15*x;
  t36 = t18*y;
  t37 = pz*t3*t12;
  t38 = t3*t12*z;
  t19 = t33-t34-t35+t36-t37+t38;
  t40 = pz*t5;
  t41 = t5*z;
  t42 = px*t2*t12;
  t43 = t2*t12*x;
  t44 = py*t6*t12;
  t45 = t6*t12*y;
  t20 = t40-t41-t42+t43-t44+t45;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t31 = t13*t13;
  t39 = t19*t19;
  t46 = t20*t20;
  t23 = pow(pow(1.0/(c*c)*t31,t22)+pow(pow(1.0/(a*a)*t46,t21)+pow(1.0/(b*b)*t39,t21),e2*t22),e1*(-1.0/2.0))-1.0;


  return (t23*t23)*(t31+t39+t46);  
}


/**
 * @function Jr
 */
void Jr( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &x, const double &y, const double &z,
	 double _J[11] ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79;
  double t80, t81, t82, t83, t84, t85, t86, t87, t88, t89;
  double t90, t91, t92, t93, t94, t95, t96;

  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t48 = t4*t5*t6;
  t11 = t10-t48;
  t12 = cos(pa);
  t47 = px*t9;
  t49 = py*t11;
  t50 = t9*x;
  t51 = t11*y;
  t52 = pz*t4*t12;
  t53 = t4*t12*z;
  t13 = t47-t49-t50+t51+t52-t53;
  t14 = t4*t6;
  t33 = t2*t3*t5;
  t15 = t14-t33;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t34 = px*t15;
  t35 = py*t18;
  t36 = t15*x;
  t37 = t18*y;
  t38 = pz*t3*t12;
  t39 = t3*t12*z;
  t19 = t34-t35-t36+t37-t38+t39;
  t24 = pz*t5;
  t25 = t5*z;
  t26 = px*t2*t12;
  t27 = t2*t12*x;
  t28 = py*t6*t12;
  t29 = t6*t12*y;
  t20 = t24-t25-t26+t27-t28+t29;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t23 = 1.0/(a*a);
  t30 = t20*t20;
  t31 = t23*t30;
  t32 = 1.0/(b*b);
  t40 = t19*t19;
  t41 = t32*t40;
  t42 = pow(t41,t21);
  t43 = pow(t31,t21);
  t44 = t42+t43;
  t45 = e2*t22;
  t46 = 1.0/(c*c);
  t54 = t13*t13;
  t55 = t46*t54;
  t56 = pow(t55,t22);
  t57 = pow(t44,t45);
  t58 = t56+t57;
  t64 = e1*(1.0/2.0);
  t59 = -t64-1.0;
  t60 = pow(t58,t59);
  t61 = t21-1.0;
  t62 = t45-1.0;
  t63 = pow(t44,t62);
  t65 = t30+t40+t54;
  t66 = pow(t58,-t64);
  t67 = t66-1.0;
  t68 = 1.0/(e1*e1);
  t69 = log(t44);
  t70 = 1.0/(e2*e2);
  t71 = pow(t41,t61);
  t72 = pow(t31,t61);
  t73 = t22-1.0;
  t74 = pow(t55,t73);
  t75 = t67*t67;
  t76 = pz*t12;
  t77 = px*t2*t5;
  t78 = py*t5*t6;
  t79 = t76+t77+t78-t12*z-t2*t5*x-t5*t6*y;
  t80 = pz*t3*t5;
  t81 = t2*t3*t12*x;
  t82 = t3*t6*t12*y;
  t83 = t80+t81+t82-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12;
  t84 = pz*t4*t5;
  t85 = t2*t4*t12*x;
  t86 = t4*t6*t12*y;
  t87 = t84+t85+t86-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12;
  t88 = t2*t12*y;
  t89 = px*t6*t12;
  t90 = t88+t89-py*t2*t12-t6*t12*x;
  t91 = px*t18;
  t92 = py*t15;
  t93 = t91+t92-t18*x-t15*y;
  t94 = px*t11;
  t95 = py*t9;
  t96 = t94+t95-t11*x-t9*y;
  
  _J[0] = 1.0/(a*a*a)*t30*t60*t63*t65*t72*(pow(t58,e1*(-1.0/2.0))-1.0)*2.0;
  _J[1] = 1.0/(b*b*b)*t40*t60*t63*t65*t67*t71*2.0;
  _J[2] = 1.0/(c*c*c)*t54*t60*t65*t67*t74*2.0;
  _J[3] = t65*t67*(t66*log(t58)*(1.0/2.0)-e1*t60*(t56*t68*log(t55)+e2*t57*t68*t69)*(1.0/2.0))*-2.0;
  _J[4] = -e1*t60*t65*t67*(t22*t57*t69-e2*t22*t63*(t43*t70*log(t31)+t42*t70*log(t41)));
  _J[5] = t75*(t9*t13*2.0+t15*t19*2.0-t2*t12*t20*2.0)-e1*t60*t65*t67*(e2*t22*t63*(t15*t19*t21*t32*t71*2.0-t2*t12*t20*t21*t23*t72*2.0)+t9*t13*t22*t46*t74*2.0);
  _J[6] = -t75*(t11*t13*2.0+t18*t19*2.0+t6*t12*t20*2.0)+e1*t60*t65*t67*(e2*t22*t63*(t18*t19*t21*t32*t71*2.0+t6*t12*t20*t21*t23*t72*2.0)+t11*t13*t22*t46*t74*2.0);
  _J[7] = t75*(t5*t20*2.0+t4*t12*t13*2.0-t3*t12*t19*2.0)-e1*t60*t65*t67*(e2*t22*t63*(t5*t20*t21*t23*t72*2.0-t3*t12*t19*t21*t32*t71*2.0)+t4*t12*t13*t22*t46*t74*2.0);
  _J[8] = -e1*t60*t65*t67*(t13*t19*t22*t46*t74*2.0-t13*t19*t22*t32*t63*t71*2.0);
  _J[9] = t75*(t20*t79*2.0-t13*t87*2.0+t19*t83*2.0)-e1*t60*t65*t67*(e2*t22*t63*(t20*t21*t23*t72*t79*2.0+t19*t21*t32*t71*t83*2.0)-t13*t22*t46*t74*t87*2.0);
  _J[10] = t75*(t13*t96*2.0+t20*t90*2.0+t19*t93*2.0)-e1*t60*t65*t67*(e2*t22*t63*(t20*t21*t23*t72*t90*2.0+t19*t21*t32*t71*t93*2.0)+t13*t22*t46*t74*t96*2.0); 

}


/**
 * @function fs
 */
double fs( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23;

  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t11 = t10-t4*t5*t6;
  t12 = cos(pa);
  t13 = px*t9-py*t11-t9*x+t11*y+pz*t4*t12-t4*t12*z;
  t14 = t4*t6;
  t15 = t14-t2*t3*t5;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t19 = px*t15-py*t18-t15*x+t18*y-pz*t3*t12+t3*t12*z;
  t20 = pz*t5-t5*z-px*t2*t12-py*t6*t12+t2*t12*x+t6*t12*y;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t23 = pow(pow(pow(1.0/(a*a)*(t20*t20),t21)+pow(1.0/(b*b)*(t19*t19),t21),e2*t22)+pow(1.0/(c*c)*(t13*t13),t22),e1)-1.0;
  
  return a*b*c*(t23*t23);
  
}


/**
 * @function Jr
 */
void Js( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &x, const double &y, const double &z,
	 double _J[11] ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  double t70, t71, t72, t73, t74, t75;

    t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t27 = t4*t5*t6;
  t11 = t10-t27;
  t12 = cos(pa);
  t26 = px*t9;
  t28 = py*t11;
  t29 = t9*x;
  t30 = t11*y;
  t31 = pz*t4*t12;
  t32 = t4*t12*z;
  t13 = t26-t28-t29+t30+t31-t32;
  t14 = t4*t6;
  t37 = t2*t3*t5;
  t15 = t14-t37;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t38 = px*t15;
  t39 = py*t18;
  t40 = t15*x;
  t41 = t18*y;
  t42 = pz*t3*t12;
  t43 = t3*t12*z;
  t19 = t38-t39-t40+t41-t42+t43;
  t47 = pz*t5;
  t48 = t5*z;
  t49 = px*t2*t12;
  t50 = t2*t12*x;
  t51 = py*t6*t12;
  t52 = t6*t12*y;
  t20 = t47-t48-t49+t50-t51+t52;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t24 = 1.0/(a*a);
  t25 = 1.0/(c*c);
  t33 = t13*t13;
  t34 = t25*t33;
  t35 = pow(t34,t22);
  t36 = 1.0/(b*b);
  t44 = t19*t19;
  t45 = t36*t44;
  t46 = pow(t45,t21);
  t53 = t20*t20;
  t54 = t24*t53;
  t55 = pow(t54,t21);
  t56 = t46+t55;
  t57 = e2*t22;
  t58 = pow(t56,t57);
  t59 = t35+t58;
  t60 = pow(t59,e1);
  t23 = t60-1.0;
  t61 = t23*t23;
  t62 = e1-1.0;
  t63 = pow(t59,t62);
  t64 = t21-1.0;
  t65 = t57-1.0;
  t66 = pow(t56,t65);
  t67 = 1.0/(e1*e1);
  t68 = log(t56);
  t69 = 1.0/(e2*e2);
  t70 = pow(t45,t64);
  t71 = pow(t54,t64);
  t72 = t22-1.0;
  t73 = pow(t34,t72);
  
  _J[0] = b*c*t61-b*c*t23*t24*t53*t63*t66*t71*4.0;
  _J[1] = a*c*t61-a*c*t23*t36*t44*t63*t66*t70*4.0;
  _J[2] = a*b*t61-a*b*t23*t25*t33*t63*t73*4.0;
  _J[3] = a*b*c*t23*(t60*log(t59)-e1*t63*(t35*t67*log(t34)+e2*t58*t67*t68))*2.0;
  _J[4] = a*b*c*e1*t23*t63*(t22*t58*t68-e2*t22*t66*(t46*t69*log(t45)+t55*t69*log(t54)))*2.0;
  _J[5] = a*b*c*e1*t23*t63*(e2*t22*t66*(t15*t19*t21*t36*t70*2.0-t2*t12*t20*t21*t24*t71*2.0)+t9*t13*t22*t25*t73*2.0)*2.0;
  _J[6] = a*b*c*e1*t23*t63*(e2*t22*t66*(t18*t19*t21*t36*t70*2.0+t6*t12*t20*t21*t24*t71*2.0)+t11*t13*t22*t25*t73*2.0)*-2.0;
  _J[7] = a*b*c*e1*t23*t63*(e2*t22*t66*(t5*t20*t21*t24*t71*2.0-t3*t12*t19*t21*t36*t70*2.0)+t4*t12*t13*t22*t25*t73*2.0)*2.0;
  _J[8] = a*b*c*e1*t23*t63*(t13*t19*t22*t25*t73*2.0-t13*t19*t22*t36*t66*t70*2.0)*2.0;
  _J[9] = a*b*c*e1*t23*t63*(e2*t22*t66*(t20*t21*t24*t71*(pz*t12-t12*z+px*t2*t5+py*t5*t6-t2*t5*x-t5*t6*y)*2.0+t19*t21*t36*t70*(pz*t3*t5-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12+t2*t3*t12*x+t3*t6*t12*y)*2.0)-t13*t22*t25*t73*(pz*t4*t5-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12+t2*t4*t12*x+t4*t6*t12*y)*2.0)*2.0;
  _J[10] = a*b*c*e1*t23*t63*(e2*t22*t66*(t20*t21*t24*t71*(px*t6*t12-py*t2*t12-t6*t12*x+t2*t12*y)*2.0+t19*t21*t36*t70*(px*t18+py*t15-t18*x-t15*y)*2.0)+t13*t22*t25*t73*(px*t11+py*t9-t11*x-t9*y)*2.0)*2.0;
  
}


/**
 * @function fi
 */
double fi( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46;

    t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t25 = t4*t5*t6;
  t11 = t10-t25;
  t12 = cos(pa);
  t24 = px*t9;
  t26 = py*t11;
  t27 = t9*x;
  t28 = t11*y;
  t29 = pz*t4*t12;
  t30 = t4*t12*z;
  t13 = t24-t26-t27+t28+t29-t30;
  t14 = t4*t6;
  t32 = t2*t3*t5;
  t15 = t14-t32;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t33 = px*t15;
  t34 = py*t18;
  t35 = t15*x;
  t36 = t18*y;
  t37 = pz*t3*t12;
  t38 = t3*t12*z;
  t19 = t33-t34-t35+t36-t37+t38;
  t40 = pz*t5;
  t41 = t5*z;
  t42 = px*t2*t12;
  t43 = t2*t12*x;
  t44 = py*t6*t12;
  t45 = t6*t12*y;
  t20 = t40-t41-t42+t43-t44+t45;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t31 = t13*t13;
  t39 = t19*t19;
  t46 = t20*t20;
  t23 = pow(pow(1.0/(c*c)*t31,t22)+pow(pow(1.0/(a*a)*t46,t21)+pow(1.0/(b*b)*t39,t21),e2*t22),e1)-1.0;


  return a*b*c*(t23*t23)*(t31+t39+t46);
  
}


/**
 * @function Jr
 */
void Ji( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &x, const double &y, const double &z,
	 double _J[11] ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79;
  double t80, t81, t82, t83, t84, t85, t86, t87, t88, t89;
  double t90, t91, t92, t93, t94, t95;

  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t25 = t4*t5*t6;
  t11 = t10-t25;
  t12 = cos(pa);
  t24 = px*t9;
  t26 = py*t11;
  t27 = t9*x;
  t28 = t11*y;
  t29 = pz*t4*t12;
  t30 = t4*t12*z;
  t13 = t24-t26-t27+t28+t29-t30;
  t14 = t4*t6;
  t32 = t2*t3*t5;
  t15 = t14-t32;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t33 = px*t15;
  t34 = py*t18;
  t35 = t15*x;
  t36 = t18*y;
  t37 = pz*t3*t12;
  t38 = t3*t12*z;
  t19 = t33-t34-t35+t36-t37+t38;
  t40 = pz*t5;
  t41 = t5*z;
  t42 = px*t2*t12;
  t43 = t2*t12*x;
  t44 = py*t6*t12;
  t45 = t6*t12*y;
  t20 = t40-t41-t42+t43-t44+t45;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t31 = t13*t13;
  t39 = t19*t19;
  t46 = t20*t20;
  t47 = 1.0/(a*a);
  t48 = 1.0/(c*c);
  t49 = t31*t48;
  t50 = pow(t49,t22);
  t51 = 1.0/(b*b);
  t52 = t39*t51;
  t53 = pow(t52,t21);
  t54 = t46*t47;
  t55 = pow(t54,t21);
  t56 = t53+t55;
  t57 = e2*t22;
  t58 = pow(t56,t57);
  t59 = t50+t58;
  t60 = pow(t59,e1);
  t23 = t60-1.0;
  t61 = t31+t39+t46;
  t62 = t23*t23;
  t63 = e1-1.0;
  t64 = pow(t59,t63);
  t65 = t21-1.0;
  t66 = t57-1.0;
  t67 = pow(t56,t66);
  t68 = 1.0/(e1*e1);
  t69 = log(t56);
  t70 = 1.0/(e2*e2);
  t71 = pow(t52,t65);
  t72 = pow(t54,t65);
  t73 = t22-1.0;
  t74 = pow(t49,t73);
  t75 = pz*t12;
  t76 = px*t2*t5;
  t77 = py*t5*t6;
  t78 = t75+t76+t77-t12*z-t2*t5*x-t5*t6*y;
  t79 = pz*t3*t5;
  t80 = t2*t3*t12*x;
  t81 = t3*t6*t12*y;
  t82 = t79+t80+t81-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12;
  t83 = pz*t4*t5;
  t84 = t2*t4*t12*x;
  t85 = t4*t6*t12*y;
  t86 = t83+t84+t85-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12;
  t87 = t2*t12*y;
  t88 = px*t6*t12;
  t89 = t87+t88-py*t2*t12-t6*t12*x;
  t90 = px*t18;
  t91 = py*t15;
  t92 = t90+t91-t18*x-t15*y;
  t93 = px*t11;
  t94 = py*t9;
  t95 = t93+t94-t11*x-t9*y;
  
  _J[0] = b*c*t61*t62-b*c*t23*t46*t47*t61*t64*t67*t72*4.0;
  _J[1] = a*c*t61*t62-a*c*t23*t39*t51*t61*t64*t67*t71*4.0;
  _J[2] = a*b*t61*t62-a*b*t23*t31*t48*t61*t64*t74*4.0;
  _J[3] = a*b*c*t23*t61*(t60*log(t59)-e1*t64*(t50*t68*log(t49)+e2*t58*t68*t69))*2.0;
  _J[4] = a*b*c*e1*t23*t61*t64*(t22*t58*t69-e2*t22*t67*(t53*t70*log(t52)+t55*t70*log(t54)))*2.0;
  _J[5] = a*b*c*t62*(t9*t13*2.0+t15*t19*2.0-t2*t12*t20*2.0)+a*b*c*e1*t23*t61*t64*(e2*t22*t67*(t15*t19*t21*t51*t71*2.0-t2*t12*t20*t21*t47*t72*2.0)+t9*t13*t22*t48*t74*2.0)*2.0;
  _J[6] = -a*b*c*t62*(t11*t13*2.0+t18*t19*2.0+t6*t12*t20*2.0)-a*b*c*e1*t23*t61*t64*(e2*t22*t67*(t18*t19*t21*t51*t71*2.0+t6*t12*t20*t21*t47*t72*2.0)+t11*t13*t22*t48*t74*2.0)*2.0;
  _J[7] = a*b*c*t62*(t5*t20*2.0+t4*t12*t13*2.0-t3*t12*t19*2.0)+a*b*c*e1*t23*t61*t64*(e2*t22*t67*(t5*t20*t21*t47*t72*2.0-t3*t12*t19*t21*t51*t71*2.0)+t4*t12*t13*t22*t48*t74*2.0)*2.0;
  _J[8] = a*b*c*e1*t23*t61*t64*(t13*t19*t22*t48*t74*2.0-t13*t19*t22*t51*t67*t71*2.0)*2.0;
  _J[9] = a*b*c*t62*(t20*t78*2.0-t13*t86*2.0+t19*t82*2.0)+a*b*c*e1*t23*t61*t64*(e2*t22*t67*(t20*t21*t47*t72*t78*2.0+t19*t21*t51*t71*t82*2.0)-t13*t22*t48*t74*t86*2.0)*2.0;
  _J[10] = a*b*c*t62*(t13*t95*2.0+t20*t89*2.0+t19*t92*2.0)+a*b*c*e1*t23*t61*t64*(e2*t22*t67*(t20*t21*t47*t72*t89*2.0+t19*t21*t51*t71*t92*2.0)+t13*t22*t48*t74*t95*2.0)*2.0;
  
}


/**
 * @function fc
 */
double fc( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z ) {
  
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46;


  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t25 = t4*t5*t6;
  t11 = t10-t25;
  t12 = cos(pa);
  t24 = px*t9;
  t26 = py*t11;
  t27 = t9*x;
  t28 = t11*y;
  t29 = pz*t4*t12;
  t30 = t4*t12*z;
  t13 = t24-t26-t27+t28+t29-t30;
  t14 = t4*t6;
  t32 = t2*t3*t5;
  t15 = t14-t32;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t33 = px*t15;
  t34 = py*t18;
  t35 = t15*x;
  t36 = t18*y;
  t37 = pz*t3*t12;
  t38 = t3*t12*z;
  t19 = t33-t34-t35+t36-t37+t38;
  t40 = pz*t5;
  t41 = t5*z;
  t42 = px*t2*t12;
  t43 = t2*t12*x;
  t44 = py*t6*t12;
  t45 = t6*t12*y;
  t20 = t40-t41-t42+t43-t44+t45;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t31 = t13*t13;
  t39 = t19*t19;
  t46 = t20*t20;
  t23 = pow(pow(1.0/(c*c)*t31,t22)+pow(pow(1.0/(a*a)*t46,t21)+pow(1.0/(b*b)*t39,t21),e2*t22),e1*(1.0/2.0))-1.0;

  return (t23*t23)*(t31+t39+t46);
}



/**
 * @function Jc
 */
void Jc( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &x, const double &y, const double &z,
	 double _J[11] ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79;
  double t80, t81, t82, t83, t84, t85, t86, t87, t88, t89;
  double t90, t91, t92, t93, t94, t95, t96;

    t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t48 = t4*t5*t6;
  t11 = t10-t48;
  t12 = cos(pa);
  t47 = px*t9;
  t49 = py*t11;
  t50 = t9*x;
  t51 = t11*y;
  t52 = pz*t4*t12;
  t53 = t4*t12*z;
  t13 = t47-t49-t50+t51+t52-t53;
  t14 = t4*t6;
  t33 = t2*t3*t5;
  t15 = t14-t33;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t34 = px*t15;
  t35 = py*t18;
  t36 = t15*x;
  t37 = t18*y;
  t38 = pz*t3*t12;
  t39 = t3*t12*z;
  t19 = t34-t35-t36+t37-t38+t39;
  t24 = pz*t5;
  t25 = t5*z;
  t26 = px*t2*t12;
  t27 = t2*t12*x;
  t28 = py*t6*t12;
  t29 = t6*t12*y;
  t20 = t24-t25-t26+t27-t28+t29;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t23 = 1.0/(a*a);
  t30 = t20*t20;
  t31 = t23*t30;
  t32 = 1.0/(b*b);
  t40 = t19*t19;
  t41 = t32*t40;
  t42 = pow(t41,t21);
  t43 = pow(t31,t21);
  t44 = t42+t43;
  t45 = e2*t22;
  t46 = 1.0/(c*c);
  t54 = t13*t13;
  t55 = t46*t54;
  t56 = pow(t55,t22);
  t57 = pow(t44,t45);
  t58 = t56+t57;
  t59 = e1*(1.0/2.0);
  t60 = t59-1.0;
  t61 = pow(t58,t60);
  t62 = t21-1.0;
  t63 = t45-1.0;
  t64 = pow(t44,t63);
  t65 = pow(t58,t59);
  t66 = t65-1.0;
  t67 = t30+t40+t54;
  t68 = 1.0/(e1*e1);
  t69 = log(t44);
  t70 = 1.0/(e2*e2);
  t71 = pow(t41,t62);
  t72 = pow(t31,t62);
  t73 = t22-1.0;
  t74 = pow(t55,t73);
  t75 = t66*t66;
  t76 = pz*t12;
  t77 = px*t2*t5;
  t78 = py*t5*t6;
  t79 = t76+t77+t78-t12*z-t2*t5*x-t5*t6*y;
  t80 = pz*t3*t5;
  t81 = t2*t3*t12*x;
  t82 = t3*t6*t12*y;
  t83 = t80+t81+t82-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12;
  t84 = pz*t4*t5;
  t85 = t2*t4*t12*x;
  t86 = t4*t6*t12*y;
  t87 = t84+t85+t86-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12;
  t88 = t2*t12*y;
  t89 = px*t6*t12;
  t90 = t88+t89-py*t2*t12-t6*t12*x;
  t91 = px*t18;
  t92 = py*t15;
  t93 = t91+t92-t18*x-t15*y;
  t94 = px*t11;
  t95 = py*t9;
  t96 = t94+t95-t11*x-t9*y;
  
  _J[0] = 1.0/(a*a*a)*t30*t61*t64*t66*t67*t72*-2.0;
  _J[1] = 1.0/(b*b*b)*t40*t61*t64*t66*t67*t71*-2.0;
  _J[2] = 1.0/(c*c*c)*t54*t61*t66*t67*t74*-2.0;
  _J[3] = t66*t67*(t65*log(t58)*(1.0/2.0)-e1*t61*(t56*t68*log(t55)+e2*t57*t68*t69)*(1.0/2.0))*2.0;
  _J[4] = e1*t61*t66*t67*(t22*t57*t69-e2*t22*t64*(t43*t70*log(t31)+t42*t70*log(t41)));
  _J[5] = t75*(t9*t13*2.0+t15*t19*2.0-t2*t12*t20*2.0)+e1*t61*t66*t67*(e2*t22*t64*(t15*t19*t21*t32*t71*2.0-t2*t12*t20*t21*t23*t72*2.0)+t9*t13*t22*t46*t74*2.0);
  _J[6] = -t75*(t11*t13*2.0+t18*t19*2.0+t6*t12*t20*2.0)-e1*t61*t66*t67*(e2*t22*t64*(t18*t19*t21*t32*t71*2.0+t6*t12*t20*t21*t23*t72*2.0)+t11*t13*t22*t46*t74*2.0);
  _J[7] = t75*(t5*t20*2.0+t4*t12*t13*2.0-t3*t12*t19*2.0)+e1*t61*t66*t67*(e2*t22*t64*(t5*t20*t21*t23*t72*2.0-t3*t12*t19*t21*t32*t71*2.0)+t4*t12*t13*t22*t46*t74*2.0);
  _J[8] = e1*t61*t66*t67*(t13*t19*t22*t46*t74*2.0-t13*t19*t22*t32*t64*t71*2.0);
  _J[9] = t75*(t20*t79*2.0-t13*t87*2.0+t19*t83*2.0)+e1*t61*t66*t67*(e2*t22*t64*(t20*t21*t23*t72*t79*2.0+t19*t21*t32*t71*t83*2.0)-t13*t22*t46*t74*t87*2.0);
  _J[10] = t75*(t13*t96*2.0+t20*t90*2.0+t19*t93*2.0)+e1*t61*t66*t67*(e2*t22*t64*(t20*t21*t23*t72*t90*2.0+t19*t21*t32*t71*t93*2.0)+t13*t22*t46*t74*t96*2.0);
  
}



/**
 * @function f5
 */
double f5( const double &a, const double &b, const double &c,
	    const double &e1, const double &e2,
	    const double &px, const double &py, const double &pz,
	    const double &ra, const double &pa, const double &ya,
	    const double &x, const double &y, const double &z ) {
  
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46;

  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t25 = t4*t5*t6;
  t11 = t10-t25;
  t12 = cos(pa);
  t24 = px*t9;
  t26 = py*t11;
  t27 = t9*x;
  t28 = t11*y;
  t29 = pz*t4*t12;
  t30 = t4*t12*z;
  t13 = t24-t26-t27+t28+t29-t30;
  t14 = t4*t6;
  t32 = t2*t3*t5;
  t15 = t14-t32;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t33 = px*t15;
  t34 = py*t18;
  t35 = t15*x;
  t36 = t18*y;
  t37 = pz*t3*t12;
  t38 = t3*t12*z;
  t19 = t33-t34-t35+t36-t37+t38;
  t40 = pz*t5;
  t41 = t5*z;
  t42 = px*t2*t12;
  t43 = t2*t12*x;
  t44 = py*t6*t12;
  t45 = t6*t12*y;
  t20 = t40-t41-t42+t43-t44+t45;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t31 = t13*t13;
  t39 = t19*t19;
  t46 = t20*t20;
  t23 = pow(pow(1.0/(c*c)*t31,t22)+pow(pow(1.0/(a*a)*t46,t21)+pow(1.0/(b*b)*t39,t21),e2*t22),e1)-1.0;

  return (t23*t23)*(t31+t39+t46);
  
}



/**
 * @function J5
 */
void J5( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &x, const double &y, const double &z,
	 double _J[11] ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79;
  double t80, t81, t82, t83, t84, t85, t86, t87, t88, t89;
  double t90, t91, t92, t93, t94, t95;

    t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t25 = t4*t5*t6;
  t11 = t10-t25;
  t12 = cos(pa);
  t24 = px*t9;
  t26 = py*t11;
  t27 = t9*x;
  t28 = t11*y;
  t29 = pz*t4*t12;
  t30 = t4*t12*z;
  t13 = t24-t26-t27+t28+t29-t30;
  t14 = t4*t6;
  t35 = t2*t3*t5;
  t15 = t14-t35;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t36 = px*t15;
  t37 = py*t18;
  t38 = t15*x;
  t39 = t18*y;
  t40 = pz*t3*t12;
  t41 = t3*t12*z;
  t19 = t36-t37-t38+t39-t40+t41;
  t46 = pz*t5;
  t47 = t5*z;
  t48 = px*t2*t12;
  t49 = t2*t12*x;
  t50 = py*t6*t12;
  t51 = t6*t12*y;
  t20 = t46-t47-t48+t49-t50+t51;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t23 = 1.0/(c*c);
  t31 = t13*t13;
  t32 = t23*t31;
  t33 = pow(t32,t22);
  t34 = 1.0/(b*b);
  t42 = t19*t19;
  t43 = t34*t42;
  t44 = pow(t43,t21);
  t45 = 1.0/(a*a);
  t52 = t20*t20;
  t53 = t45*t52;
  t54 = pow(t53,t21);
  t55 = t44+t54;
  t56 = e2*t22;
  t57 = pow(t55,t56);
  t58 = t33+t57;
  t59 = e1-1.0;
  t60 = pow(t58,t59);
  t61 = t21-1.0;
  t62 = pow(t58,e1);
  t63 = t62-1.0;
  t64 = t56-1.0;
  t65 = pow(t55,t64);
  t66 = t31+t42+t52;
  t67 = 1.0/(e1*e1);
  t68 = log(t55);
  t69 = 1.0/(e2*e2);
  t70 = pow(t43,t61);
  t71 = pow(t53,t61);
  t72 = t22-1.0;
  t73 = pow(t32,t72);
  t74 = t63*t63;
  t75 = pz*t12;
  t76 = px*t2*t5;
  t77 = py*t5*t6;
  t78 = t75+t76+t77-t12*z-t2*t5*x-t5*t6*y;
  t79 = pz*t3*t5;
  t80 = t2*t3*t12*x;
  t81 = t3*t6*t12*y;
  t82 = t79+t80+t81-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12;
  t83 = pz*t4*t5;
  t84 = t2*t4*t12*x;
  t85 = t4*t6*t12*y;
  t86 = t83+t84+t85-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12;
  t87 = t2*t12*y;
  t88 = px*t6*t12;
  t89 = t87+t88-py*t2*t12-t6*t12*x;
  t90 = px*t18;
  t91 = py*t15;
  t92 = t90+t91-t18*x-t15*y;
  t93 = px*t11;
  t94 = py*t9;
  t95 = t93+t94-t11*x-t9*y;
  
  _J[0] = 1.0/(a*a*a)*t52*t60*t63*t65*t66*t71*-4.0;
  _J[1] = 1.0/(b*b*b)*t42*t60*t63*t65*t66*t70*-4.0;
  _J[2] = 1.0/(c*c*c)*t31*t60*t63*t66*t73*-4.0;
  _J[3] = t63*t66*(t62*log(t58)-e1*t60*(t33*t67*log(t32)+e2*t57*t67*t68))*2.0;
  _J[4] = e1*t60*t63*t66*(t22*t57*t68-e2*t22*t65*(t44*t69*log(t43)+t54*t69*log(t53)))*2.0;
  _J[5] = t74*(t9*t13*2.0+t15*t19*2.0-t2*t12*t20*2.0)+e1*t60*t63*t66*(e2*t22*t65*(t15*t19*t21*t34*t70*2.0-t2*t12*t20*t21*t45*t71*2.0)+t9*t13*t22*t23*t73*2.0)*2.0;
  _J[6] = -t74*(t11*t13*2.0+t18*t19*2.0+t6*t12*t20*2.0)-e1*t60*t63*t66*(e2*t22*t65*(t18*t19*t21*t34*t70*2.0+t6*t12*t20*t21*t45*t71*2.0)+t11*t13*t22*t23*t73*2.0)*2.0;
  _J[7] = t74*(t5*t20*2.0+t4*t12*t13*2.0-t3*t12*t19*2.0)+e1*t60*t63*t66*(e2*t22*t65*(t5*t20*t21*t45*t71*2.0-t3*t12*t19*t21*t34*t70*2.0)+t4*t12*t13*t22*t23*t73*2.0)*2.0;
  _J[8] = e1*t60*t63*t66*(t13*t19*t22*t23*t73*2.0-t13*t19*t22*t34*t65*t70*2.0)*2.0;
  _J[9] = t74*(t20*t78*2.0-t13*t86*2.0+t19*t82*2.0)+e1*t60*t63*t66*(e2*t22*t65*(t19*t21*t34*t70*t82*2.0+t20*t21*t45*t71*t78*2.0)-t13*t22*t23*t73*t86*2.0)*2.0;
  _J[10] = t74*(t13*t95*2.0+t20*t89*2.0+t19*t92*2.0)+e1*t60*t63*t66*(e2*t22*t65*(t19*t21*t34*t70*t92*2.0+t20*t21*t45*t71*t89*2.0)+t13*t22*t23*t73*t95*2.0)*2.0;
  
}



/**
 * @function f6
 */
double f6( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z ) {
  
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60;
  
  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t26 = t4*t5*t6;
  t11 = t10-t26;
  t12 = cos(pa);
  t25 = px*t9;
  t27 = py*t11;
  t28 = t9*x;
  t29 = t11*y;
  t30 = pz*t4*t12;
  t31 = t4*t12*z;
  t13 = t25-t27-t28+t29+t30-t31;
  t14 = t4*t6;
  t36 = t2*t3*t5;
  t15 = t14-t36;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t37 = px*t15;
  t38 = py*t18;
  t39 = t15*x;
  t40 = t18*y;
  t41 = pz*t3*t12;
  t42 = t3*t12*z;
  t19 = t37-t38-t39+t40-t41+t42;
  t47 = pz*t5;
  t48 = t5*z;
  t49 = px*t2*t12;
  t50 = t2*t12*x;
  t51 = py*t6*t12;
  t52 = t6*t12*y;
  t20 = t47-t48-t49+t50-t51+t52;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t24 = 1.0/(c*c);
  t32 = t13*t13;
  t33 = t24*t32;
  t34 = pow(t33,t22);
  t35 = 1.0/(b*b);
  t43 = t19*t19;
  t44 = t35*t43;
  t45 = pow(t44,t21);
  t46 = 1.0/(a*a);
  t53 = t20*t20;
  t54 = t46*t53;
  t55 = pow(t54,t21);
  t56 = t45+t55;
  t57 = e2*t22;
  t58 = pow(t56,t57);
  t59 = t34+t58;
  t23 = pow(t59,e1)-1.0;
  t60 = pow(t59,e1*(-1.0/2.0))-1.0;
  
  return (t23*t23)*(t60*t60)*(t32+t43+t53); 
}



/**
 * @function J
 */
void J6( const double &a, const double &b, const double &c,
	 const double &e1, const double &e2,
	 const double &px, const double &py, const double &pz,
	 const double &ra, const double &pa, const double &ya,
	 const double &x, const double &y, const double &z,
	 double _J[11] ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79;
  double t80, t81, t82, t83, t84, t85, t86, t87, t88, t89;
  double t90, t91, t92, t93, t94, t95, t96, t97, t98, t99;
  double t100, t101, t102, t103, t104, t105, t106, t107, t108, t109;  
  double t110, t111, t112, t113, t114, t115, t116, t117, t118, t119;
  double t120, t121, t122, t123, t124, t125, t126, t127, t128, t129;
  double t130, t131, t132, t133, t134, t135, t136, t137, t138, t139;
  double t140, t141, t142, t143, t144, t145, t146, t147, t148, t149;
  double t150, t151, t152, t153, t154, t155, t156, t157, t158, t159;
  double t160;
  

    t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t25 = t4*t5*t6;
  t11 = t10-t25;
  t12 = cos(pa);
  t24 = px*t9;
  t26 = py*t11;
  t27 = t9*x;
  t28 = t11*y;
  t29 = pz*t4*t12;
  t30 = t4*t12*z;
  t13 = t24-t26-t27+t28+t29-t30;
  t14 = t4*t6;
  t35 = t2*t3*t5;
  t15 = t14-t35;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t36 = px*t15;
  t37 = py*t18;
  t38 = t15*x;
  t39 = t18*y;
  t40 = pz*t3*t12;
  t41 = t3*t12*z;
  t19 = t36-t37-t38+t39-t40+t41;
  t46 = pz*t5;
  t47 = t5*z;
  t48 = px*t2*t12;
  t49 = t2*t12*x;
  t50 = py*t6*t12;
  t51 = t6*t12*y;
  t20 = t46-t47-t48+t49-t50+t51;
  t21 = 1.0/e2;
  t22 = 1.0/e1;
  t23 = 1.0/(c*c);
  t31 = t13*t13;
  t32 = t23*t31;
  t33 = pow(t32,t22);
  t34 = 1.0/(b*b);
  t42 = t19*t19;
  t43 = t34*t42;
  t44 = pow(t43,t21);
  t45 = 1.0/(a*a);
  t52 = t20*t20;
  t53 = t45*t52;
  t54 = pow(t53,t21);
  t55 = t44+t54;
  t56 = e2*t22;
  t57 = pow(t55,t56);
  t58 = t33+t57;
  t61 = e1*(1.0/2.0);
  t68 = pow(t58,-t61);
  t59 = t68-1.0;
  t60 = 1.0/(a*a*a);
  t62 = pow(t58,e1);
  t63 = t62-1.0;
  t64 = t21-1.0;
  t65 = pow(t53,t64);
  t66 = t56-1.0;
  t67 = pow(t55,t66);
  t69 = t31+t42+t52;
  t70 = e1-1.0;
  t71 = pow(t58,t70);
  t72 = t59*t59;
  t73 = 1.0/(b*b*b);
  t74 = -t61-1.0;
  t75 = pow(t58,t74);
  t76 = pow(t43,t64);
  t77 = t63*t63;
  t78 = 1.0/(c*c*c);
  t79 = t22-1.0;
  t80 = pow(t32,t79);
  t81 = 1.0/(e1*e1);
  t82 = log(t58);
  t83 = log(t32);
  t84 = t33*t81*t83;
  t85 = log(t55);
  t86 = e2*t57*t81*t85;
  t87 = t84+t86;
  t88 = 1.0/(e2*e2);
  t89 = t22*t57*t85;
  t90 = log(t43);
  t91 = t44*t88*t90;
  t92 = log(t53);
  t93 = t54*t88*t92;
  t94 = t91+t93;
  t95 = t89-e2*t22*t67*t94;
  t96 = t15*t19*t21*t34*t76*2.0;
  t97 = t96-t2*t12*t20*t21*t45*t65*2.0;
  t98 = e2*t22*t67*t97;
  t99 = t9*t13*t22*t23*t80*2.0;
  t100 = t98+t99;
  t101 = t18*t19*t21*t34*t76*2.0;
  t102 = t6*t12*t20*t21*t45*t65*2.0;
  t103 = t101+t102;
  t104 = e2*t22*t67*t103;
  t105 = t11*t13*t22*t23*t80*2.0;
  t106 = t104+t105;
  t107 = t5*t20*t21*t45*t65*2.0;
  t108 = t107-t3*t12*t19*t21*t34*t76*2.0;
  t109 = e2*t22*t67*t108;
  t110 = t4*t12*t13*t22*t23*t80*2.0;
  t111 = t109+t110;
  t112 = t13*t19*t22*t23*t80*2.0;
  t113 = t112-t13*t19*t22*t34*t67*t76*2.0;
  t114 = pz*t12;
  t115 = px*t2*t5;
  t116 = py*t5*t6;
  t126 = t12*z;
  t127 = t2*t5*x;
  t128 = t5*t6*y;
  t117 = t114+t115+t116-t126-t127-t128;
  t118 = pz*t3*t5;
  t119 = t2*t3*t12*x;
  t120 = t3*t6*t12*y;
  t130 = t3*t5*z;
  t131 = px*t2*t3*t12;
  t132 = py*t3*t6*t12;
  t121 = t118+t119+t120-t130-t131-t132;
  t122 = pz*t4*t5;
  t123 = t2*t4*t12*x;
  t124 = t4*t6*t12*y;
  t136 = t4*t5*z;
  t137 = px*t2*t4*t12;
  t138 = py*t4*t6*t12;
  t125 = t122+t123+t124-t136-t137-t138;
  t129 = t20*t21*t45*t65*t117*2.0;
  t133 = t19*t21*t34*t76*t121*2.0;
  t134 = t129+t133;
  t135 = e2*t22*t67*t134;
  t139 = t135-t13*t22*t23*t80*t125*2.0;
  t140 = t2*t12*y;
  t141 = px*t6*t12;
  t149 = py*t2*t12;
  t150 = t6*t12*x;
  t142 = t140+t141-t149-t150;
  t143 = px*t18;
  t144 = py*t15;
  t152 = t18*x;
  t153 = t15*y;
  t145 = t143+t144-t152-t153;
  t146 = px*t11;
  t147 = py*t9;
  t157 = t11*x;
  t158 = t9*y;
  t148 = t146+t147-t157-t158;
  t151 = t20*t21*t45*t65*t142*2.0;
  t154 = t19*t21*t34*t76*t145*2.0;
  t155 = t151+t154;
  t156 = e2*t22*t67*t155;
  t159 = t13*t22*t23*t80*t148*2.0;
  t160 = t156+t159;
  
  _J[0] = t52*t60*t63*t65*t67*t69*t71*t72*-4.0+t52*t59*t60*t65*t67*t69*t75*t77*2.0;
  _J[1] = t42*t63*t67*t69*t71*t72*t73*t76*-4.0+t42*t59*t67*t69*t73*t75*t76*t77*2.0;
  _J[2] = t31*t63*t69*t71*t72*t78*t80*-4.0+t31*t59*t69*t75*t77*t78*t80*2.0;
  _J[3] = t63*t69*t72*(t62*t82-e1*t71*t87)*2.0-t59*t69*t77*(t68*t82*(1.0/2.0)-e1*t75*t87*(1.0/2.0))*2.0;
  _J[4] = e1*t63*t69*t71*t72*t95*2.0-e1*t59*t69*t75*t77*t95;
  _J[5] = t72*t77*(t9*t13*2.0+t15*t19*2.0-t2*t12*t20*2.0)+e1*t63*t69*t71*t72*t100*2.0-e1*t59*t69*t75*t77*t100;
  _J[6] = -t72*t77*(t11*t13*2.0+t18*t19*2.0+t6*t12*t20*2.0)-e1*t63*t69*t71*t72*t106*2.0+e1*t59*t69*t75*t77*t106;
  _J[7] = t72*t77*(t5*t20*2.0+t4*t12*t13*2.0-t3*t12*t19*2.0)+e1*t63*t69*t71*t72*t111*2.0-e1*t59*t69*t75*t77*t111;
  _J[8] = e1*t63*t69*t71*t72*t113*2.0-e1*t59*t69*t75*t77*t113;
  _J[9] = t72*t77*(t20*t117*2.0-t13*t125*2.0+t19*t121*2.0)+e1*t63*t69*t71*t72*t139*2.0-e1*t59*t69*t75*t77*t139;
  _J[10] = t72*t77*(t13*t148*2.0+t20*t142*2.0+t19*t145*2.0)+e1*t63*t69*t71*t72*t160*2.0-e1*t59*t69*t75*t77*t160;
  
}
