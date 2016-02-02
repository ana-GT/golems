/**
 * @file evaluated_eqs_t.cpp 
 * @brief Calculate with tampering
 */
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include "analytic_equations.h"
#include "evaluated_eqs_b.h"
extern "C" {
#include "levmar/levmar.h"
}
#include <math.h>
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter.h"



/**
 * @function error_metric
 */
void evaluated_sqs_b::error_metric( const SQ_parameters &_par,
				    const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
				    double& _e1, double &_e2, double &_e4) {

  double* p = new double[12];

  p[0] = _par.dim[0];   p[1] = _par.dim[1];   p[2] = _par.dim[2];
  p[3] = _par.e[0];   p[4] = _par.e[1];
  p[5] = _par.trans[0]; p[6] = _par.trans[1];   p[7] = _par.trans[2];
  p[8] = _par.rot[0]; p[9] = _par.rot[1];   p[10] = _par.rot[2];
  p[11] = _par.tamp;
  
  return error_metric( p, _cloud, _e1, _e2, _e4 );
}

/**
 * @function error_metric
 */
void evaluated_sqs_b::error_metric( double* p,
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
		  p[11],
		  (*it).x, (*it).y, (*it).z );

    _e2 += Err_2( p[0], p[1], p[2],
		  p[3], p[4],
		  p[5], p[6], p[7],
		  p[8], p[9], p[10],
		  p[11],
		  (*it).x, (*it).y, (*it).z );

    
    _e4 += Err_4( p[0], p[1], p[2],
		  p[3], p[4],
		  p[5], p[6], p[7],
		  p[8], p[9], p[10],
		  p[11],
		  (*it).x, (*it).y, (*it).z );    
  }
  
}


/****/
double evaluated_sqs_b::Err_1( const double &a, const double &b, const double &c,
			       const double &e1, const double &e2,
			       const double &px, const double &py, const double &pz,
			       const double &ra, const double &pa, const double &ya,
			       const double &K,
			       const double &x, const double &y, const double &z ) {
  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27;

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
  t20 = 1.0/c;
  t21 = K*t20*z;
  t22 = t21+1.0;
  t23 = 1.0/(t22*t22);
  t24 = pz*t5-t5*z-px*t2*t12-py*t6*t12+t2*t12*x+t6*t12*y;
  t25 = 1.0/e2;
  t26 = 1.0/e1;
  t27 = pow(pow(1.0/(a*a)*t23*(t24*t24),t25)+pow(1.0/(b*b)*(t19*t19)*t23,t25),e2*t26)+pow(1.0/(c*c)*(t13*t13),t26)-1.0;
  t0 = t27*t27;


  return t0;
}

/****/
double evaluated_sqs_b::Err_2( const double &a, const double &b, const double &c,
			       const double &e1, const double &e2,
			       const double &px, const double &py, const double &pz,
			       const double &ra, const double &pa, const double &ya,
			       const double &K,
			       const double &x, const double &y, const double &z ) {
  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27;

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
  t20 = 1.0/c;
  t21 = K*t20*z;
  t22 = t21+1.0;
  t23 = 1.0/(t22*t22);
  t24 = pz*t5-t5*z-px*t2*t12-py*t6*t12+t2*t12*x+t6*t12*y;
  t25 = 1.0/e2;
  t26 = 1.0/e1;
  t27 = pow(pow(pow(1.0/(a*a)*t23*(t24*t24),t25)+pow(1.0/(b*b)*(t19*t19)*t23,t25),e2*t26)+pow(1.0/(c*c)*(t13*t13),t26),e1)-1.0;
  t0 = t27*t27;

  return t0;
}

/****/
double evaluated_sqs_b::Err_4( const double &a, const double &b, const double &c,
			       const double &e1, const double &e2,
			       const double &px, const double &py, const double &pz,
			       const double &ra, const double &pa, const double &ya,
			       const double &K,
			       const double &x, const double &y, const double &z ) {
  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;

  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t28 = t4*t5*t6;
  t11 = t10-t28;
  t12 = cos(pa);
  t27 = px*t9;
  t29 = py*t11;
  t30 = t9*x;
  t31 = t11*y;
  t32 = pz*t4*t12;
  t33 = t4*t12*z;
  t13 = t27-t29-t30+t31+t32-t33;
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
  t20 = 1.0/c;
  t21 = K*t20*z;
  t22 = t21+1.0;
  t23 = 1.0/(t22*t22);
  t43 = pz*t5;
  t44 = t5*z;
  t45 = px*t2*t12;
  t46 = t2*t12*x;
  t47 = py*t6*t12;
  t48 = t6*t12*y;
  t24 = t43-t44-t45+t46-t47+t48;
  t25 = 1.0/e2;
  t26 = 1.0/e1;
  t34 = t13*t13;
  t42 = t19*t19;
  t49 = t24*t24;
  t0 = fabs(pow(pow(1.0/(c*c)*t34,t26)+pow(pow(1.0/(a*a)*t23*t49,t25)+pow(1.0/(b*b)*t23*t42,t25),e2*t26),e1*(-1.0/2.0))-1.0)*sqrt(t34+t23*t42+t23*t49);

  return t0;
}


/**
 * @function minimize
 */
bool evaluated_sqs_b::minimize( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
				SQ_parameters &_par, double &_e1, double &_e2, double &_e4,
				int _type ) {
  
  // Get parameters to start minimization
  double dim[3]; double trans[3]; double rot[3];
  this->getBoundingBox( _input, dim, trans, rot );

  // Set necessary parameters
  int n = _input->points.size();
  int m = 12; 
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
  p[3] = 0.5; p[4] = 1.0;
  for( i = 0; i < 3; ++i ) { p[i+5] = trans[i]; }
  for( i = 0; i < 3; ++i ) { p[i+8] = rot[i]; }
  p[11] = 1.4;
  
  // Set limits
  double ub[m], lb[m];
  for( i = 0; i < 3; ++i ) { lb[i] = 0.01; ub[i] = 0.3; }
  for( i = 0; i < 2; ++i ) { lb[i+3] = 0.1; ub[i+3] = 1.9; }
  for( i = 0; i < 3; ++i ) { lb[i+5] = -1.5; ub[i+5] = 1.5; }
  for( i = 0; i < 3; ++i ) { lb[i+8] = -M_PI; ub[i+8] = M_PI; }
  lb[11] = 0.01; ub[11] = 2000;
  
  switch( _type ) {

  case SQ_FX_SOLINA_B: {
    
    ret = dlevmar_bc_der( evaluated_sqs_b::fs_add,
			  evaluated_sqs_b::Js_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );

    
    printf("Ret: %d \n", ret);
  }  break;    

  case SQ_FX_CHEVALIER_B: {
    
    ret = dlevmar_bc_der( evaluated_sqs_b::fc_add,
			  evaluated_sqs_b::Jc_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );

    
    printf("Ret: %d \n", ret);
  }  break;    



    
  }

  // If stopped by invalid (TODO: Add other reasons)
  if( info[6] == 7 ) {
    _par.dim[0] = 0; _par.dim[1] = 0; _par.dim[2] = 0;
    _par.e[0] = 0; _par.e[1] = 0;
    _par.trans[0] = 0; _par.trans[1] = 0; _par.trans[2] = 0;
    _par.rot[0] = 0; _par.rot[1] = 0; _par.rot[2] = 0;
    _par.R = 1000;
    _e1 = 10000; _e2 = 10000; _e4 = 10000; // A big number
    return false;
  } else {
    _par.dim[0] = p[0]; _par.dim[1] = p[1]; _par.dim[2] = p[2];
    _par.e[0] = p[3]; _par.e[1] = p[4];
    _par.trans[0] = p[5]; _par.trans[1] = p[6]; _par.trans[2] = p[7];
    _par.rot[0] = p[8]; _par.rot[1] = p[9]; _par.rot[2] = p[10];
    _par.R = p[11];
    error_metric( p, _input, _e1, _e2, _e4 );
   
    return true;
  }
}


void evaluated_sqs_b::getBoundingBox( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
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
 * @function fs_add
 */
void evaluated_sqs_b::fs_add( double* p, double* _x,
			      int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double x, y, z;


  double a,b,c,e1,e2,px,py,pz,ra,pa,ya,r;
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  r = p[11];

  register double t0, t2, t3, t4, t5, t6, t7, t8, t9;
  register double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  register double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29; 
  register double t30, t31, t32, t33; 
  
  for( int i = 0; i < n; ++i ) {
    
    x = dptr->x[i];
    y = dptr->y[i];
    z = dptr->z[i];
  
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
  t15 = t14-t2*t3*t5;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t22 = r*r;
  t19 = r+px*t15-py*t18-t15*x+t18*y-sqrt(t22-t13*t13)-pz*t3*t12+t3*t12*z;
  t20 = pz*t5-t5*z-px*t2*t12-py*t6*t12+t2*t12*x+t6*t12*y;
  t21 = 1.0/e2;
  t30 = asin(t13/r);
  t31 = 1.0/e1;
  t0 = (pow(pow(1.0/(c*c)*t22*(t30*t30),t31)+pow(pow(1.0/(a*a)*(t20*t20),t21)+pow(1.0/(b*b)*(t19*t19),t21),e2*t31),e1)-1.0)*sqrt(a*b*c);

   _x[i] = t0;
  }
}


/**
 * @function Js_add
 */
void evaluated_sqs_b::Js_add( double* p, double* jac,
			      int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;

  double x, y, z;
  double a, b, c, e1, e2, ra, pa, ya, px,py,pz,r;

  
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  r = p[11];

  register double t2, t3, t4, t5, t6, t7, t8, t9;
  register double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  register double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  register double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  register double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  register double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  register double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  register double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79;
  register double t80, t81, t82, t83, t84, t85, t86, t87, t88, t89;
  register double t90, t91, t92, t93, t94, t95, t96, t97, t98, t99;


  for( int i = 0; i < n; ++i ) {
    
    x = dptr->x[i];
    y = dptr->y[i];
    z = dptr->z[i];

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
  t36 = t2*t3*t5;
  t15 = t14-t36;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t22 = r*r;
  t33 = t13*t13;
  t34 = t22-t33;
  t35 = sqrt(t34);
  t37 = px*t15;
  t38 = py*t18;
  t39 = t15*x;
  t40 = t18*y;
  t41 = pz*t3*t12;
  t42 = t3*t12*z;
  t19 = r-t35+t37-t38-t39+t40-t41+t42;
  t47 = pz*t5;
  t48 = t5*z;
  t49 = px*t2*t12;
  t50 = t2*t12*x;
  t51 = py*t6*t12;
  t52 = t6*t12*y;
  t20 = t47-t48-t49+t50-t51+t52;
  t21 = 1.0/e2;
  t60 = 1.0/r;
  t61 = t13*t60;
  t30 = asin(t61);
  t31 = 1.0/e1;
  t32 = 1.0/(b*b);
  t43 = t19*t19;
  t44 = t32*t43;
  t45 = pow(t44,t21);
  t46 = 1.0/(a*a);
  t53 = t20*t20;
  t54 = t46*t53;
  t55 = pow(t54,t21);
  t56 = t45+t55;
  t57 = e2*t31;
  t58 = pow(t56,t57);
  t59 = 1.0/(c*c);
  t62 = t30*t30;
  t63 = t22*t59*t62;
  t64 = pow(t63,t31);
  t65 = t58+t64;
  t66 = a*b*c;
  t67 = pow(t65,e1);
  t68 = t67-1.0;
  t69 = 1.0/sqrt(t66);
  t70 = t21-1.0;
  t71 = t57-1.0;
  t72 = pow(t56,t71);
  t73 = e1-1.0;
  t74 = pow(t65,t73);
  t75 = sqrt(t66);
  t76 = 1.0/(e1*e1);
  t77 = log(t56);
  t78 = 1.0/(e2*e2);
  t79 = pow(t44,t70);
  t80 = pow(t54,t70);
  t81 = t31-1.0;
  t82 = pow(t63,t81);
  t83 = 1.0/sqrt(t34);
  t84 = 1.0/(r*r);
  t87 = t33*t84;
  t85 = -t87+1.0;
  t86 = 1.0/sqrt(t85);
  t88 = t37-t38-t39+t40-t41+t42;
  t89 = pz*t4*t5;
  t90 = t2*t4*t12*x;
  t91 = t4*t6*t12*y;
  t92 = t89+t90+t91-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12;
  t93 = px*t11;
  t94 = py*t9;
  t95 = t93+t94-t11*x-t9*y;
  jac[12*i+0] = b*c*t68*t69*(1.0/2.0)-1.0/(a*a*a)*t53*t72*t74*t75*t80*2.0;
  jac[12*i+1] = a*c*t68*t69*(1.0/2.0)-1.0/(b*b*b)*t43*t72*t74*t75*t79*2.0;
  jac[12*i+2] = a*b*t68*t69*(1.0/2.0)-1.0/(c*c*c)*t22*t62*t74*t75*t82*2.0;
  jac[12*i+3] = t75*(t67*log(t65)-e1*t74*(t64*t76*log(t63)+e2*t58*t76*t77));
  jac[12*i+4] = e1*t74*t75*(t31*t58*t77-e2*t31*t72*(t45*t78*log(t44)+t55*t78*log(t54)));
  jac[12*i+5] = e1*t74*t75*(e2*t31*t72*(t19*t21*t32*t79*(t14-t36+t9*t13*t83)*2.0-t2*t12*t20*t21*t46*t80*2.0)+r*t9*t30*t31*t59*t82*t86*2.0);
  jac[12*i+6] = -e1*t74*t75*(e2*t31*t72*(t19*t21*t32*t79*(t16+t17+t11*t13*t83)*2.0+t6*t12*t20*t21*t46*t80*2.0)+r*t11*t30*t31*t59*t82*t86*2.0);
  jac[12*i+7] = -e1*t74*t75*(e2*t31*t72*(t19*t21*t32*t79*(t3*t12-t4*t12*t13*t83)*2.0-t5*t20*t21*t46*t80*2.0)-r*t4*t12*t30*t31*t59*t82*t86*2.0);
  jac[12*i+8] = e1*t74*t75*(t19*t31*t32*t72*t79*(-t23+t25+t26-t27-t28+t29+t13*t83*t88)*2.0+r*t30*t31*t59*t82*t86*t88*2.0);
  jac[12*i+9] = e1*t74*t75*(e2*t31*t72*(t20*t21*t46*t80*(pz*t12-t12*z+px*t2*t5+py*t5*t6-t2*t5*x-t5*t6*y)*2.0-t19*t21*t32*t79*(-pz*t3*t5+t13*t83*t92+t3*t5*z+px*t2*t3*t12+py*t3*t6*t12-t2*t3*t12*x-t3*t6*t12*y)*2.0)-r*t30*t31*t59*t82*t86*t92*2.0);
  jac[12*i+10] = e1*t74*t75*(e2*t31*t72*(t19*t21*t32*t79*(px*t18+py*t15-t18*x-t15*y+t13*t83*t95)*2.0+t20*t21*t46*t80*(px*t6*t12-py*t2*t12-t6*t12*x+t2*t12*y)*2.0)+r*t30*t31*t59*t82*t86*t95*2.0);
  jac[12*i+11] = e1*t74*t75*(t31*t82*(r*t59*t62*2.0-t13*t30*t59*t86*2.0)-t19*t31*t32*t72*t79*(r*t83-1.0)*2.0);


  }
}




/**
 * @function fc_add
 */
void evaluated_sqs_b::fc_add( double* p, double* _x,
			      int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double x, y, z;


  double a,b,c,e1,e2,px,py,pz,ra,pa,ya,r;
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  r = p[11];

  register double t0, t2, t3, t4, t5, t6, t7, t8, t9;
  register double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  register double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29; 
  register double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39; 
  register double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49; 
  register double t50, t51, t52;
  
  for( int i = 0; i < n; ++i ) {
    
    x = dptr->x[i];
    y = dptr->y[i];
    z = dptr->z[i];

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
  t35 = t2*t3*t5;
  t15 = t14-t35;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t22 = r*r;
  t32 = t13*t13;
  t33 = t22-t32;
  t34 = sqrt(t33);
  t36 = px*t15;
  t37 = py*t18;
  t38 = t15*x;
  t39 = t18*y;
  t40 = pz*t3*t12;
  t41 = t3*t12*z;
  t19 = r-t34+t36-t37-t38+t39-t40+t41;
  t43 = pz*t5;
  t44 = t5*z;
  t45 = px*t2*t12;
  t46 = t2*t12*x;
  t47 = py*t6*t12;
  t48 = t6*t12*y;
  t20 = t43-t44-t45+t46-t47+t48;
  t21 = 1.0/e2;
  t50 = 1.0/r;
  t51 = t13*t50;
  t30 = asin(t51);
  t31 = 1.0/e1;
  t42 = t19*t19;
  t49 = t20*t20;
  t52 = t30*t30;
  t0 = (pow(pow(pow(1.0/(a*a)*t49,t21)+pow(1.0/(b*b)*t42,t21),e2*t31)+pow(1.0/(c*c)*t22*t52,t31),e1*(1.0/2.0))-1.0)*sqrt(t42+t49+t22*t52);

   _x[i] = t0;
  }
}


/**
 * @function Jc_add
 */
void evaluated_sqs_b::Jc_add( double* p, double* jac,
			      int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;

  double x, y, z;
  double a, b, c, e1, e2, ra, pa, ya, px,py,pz,r;

  
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  r = p[11];

  register double t2, t3, t4, t5, t6, t7, t8, t9;
  register double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  register double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  register double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  register double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  register double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  register double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  register double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79;
  register double t80, t81, t82, t83, t84, t85, t86, t87, t88, t89;
  register double t90, t91, t92, t93, t94, t95, t96, t97, t98, t99;
  register double t100, t101, t102, t103, t104, t105, t106, t107, t108, t109;
  register double t110, t111, t112, t113, t114, t115, t116, t117, t118, t119;
  register double t120, t121, t122, t123, t124, t125, t126, t127, t128, t129;

  for( int i = 0; i < n; ++i ) {
    
    x = dptr->x[i];
    y = dptr->y[i];
    z = dptr->z[i];

  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t33 = t4*t5*t6;
  t11 = t10-t33;
  t12 = cos(pa);
  t32 = px*t9;
  t34 = py*t11;
  t35 = t9*x;
  t36 = t11*y;
  t37 = pz*t4*t12;
  t38 = t4*t12*z;
  t13 = t32-t34-t35+t36+t37-t38;
  t14 = t4*t6;
  t43 = t2*t3*t5;
  t15 = t14-t43;
  t16 = t2*t4;
  t17 = t3*t5*t6;
  t18 = t16+t17;
  t39 = t13*t13;
  t40 = r*r;
  t41 = -t39+t40;
  t42 = sqrt(t41);
  t44 = px*t15;
  t45 = py*t18;
  t46 = t15*x;
  t47 = t18*y;
  t48 = pz*t3*t12;
  t49 = t3*t12*z;
  t19 = r-t42+t44-t45-t46+t47-t48+t49;
  t23 = pz*t5;
  t24 = t5*z;
  t25 = px*t2*t12;
  t26 = t2*t12*x;
  t27 = py*t6*t12;
  t28 = t6*t12*y;
  t20 = t23-t24-t25+t26-t27+t28;
  t21 = 1.0/e2;
  t22 = 1.0/(a*a);
  t29 = t20*t20;
  t30 = t22*t29;
  t31 = 1.0/(b*b);
  t50 = t19*t19;
  t51 = t31*t50;
  t52 = pow(t51,t21);
  t53 = pow(t30,t21);
  t54 = t52+t53;
  t55 = 1.0/e1;
  t56 = e2*t55;
  t58 = 1.0/r;
  t59 = t13*t58;
  t57 = asin(t59);
  t60 = t57*t57;
  t61 = t21-1.0;
  t62 = t56-1.0;
  t63 = pow(t54,t62);
  t64 = pow(t54,t56);
  t65 = 1.0/(c*c);
  t66 = t40*t60*t65;
  t67 = pow(t66,t55);
  t68 = t64+t67;
  t69 = e1*(1.0/2.0);
  t70 = t69-1.0;
  t71 = pow(t68,t70);
  t72 = t40*t60;
  t73 = t29+t50+t72;
  t74 = sqrt(t73);
  t75 = 1.0/(e1*e1);
  t76 = log(t54);
  t77 = 1.0/(e2*e2);
  t78 = pow(t68,t69);
  t79 = pow(t51,t61);
  t80 = 1.0/sqrt(t41);
  t81 = t9*t13*t80;
  t82 = t14-t43+t81;
  t83 = pow(t30,t61);
  t84 = 1.0/(r*r);
  t90 = t39*t84;
  t85 = -t90+1.0;
  t86 = 1.0/sqrt(t85);
  t87 = t55-1.0;
  t88 = pow(t66,t87);
  t89 = t78-1.0;
  t91 = 1.0/sqrt(t73);
  t92 = t11*t13*t80;
  t93 = t16+t17+t92;
  t94 = t3*t12;
  t95 = t94-t4*t12*t13*t80;
  t96 = t44-t45-t46+t47-t48+t49;
  t97 = t13*t80*t96;
  t98 = -t32+t34+t35-t36-t37+t38+t97;
  t99 = pz*t4*t5;
  t100 = t2*t4*t12*x;
  t101 = t4*t6*t12*y;
  t107 = t4*t5*z;
  t108 = px*t2*t4*t12;
  t109 = py*t4*t6*t12;
  t102 = t99+t100+t101-t107-t108-t109;
  t103 = pz*t12;
  t104 = px*t2*t5;
  t105 = py*t5*t6;
  t106 = t103+t104+t105-t12*z-t2*t5*x-t5*t6*y;
  t110 = t13*t80*t102;
  t111 = t3*t5*z;
  t112 = px*t2*t3*t12;
  t113 = py*t3*t6*t12;
  t114 = t110+t111+t112+t113-pz*t3*t5-t2*t3*t12*x-t3*t6*t12*y;
  t115 = px*t11;
  t116 = py*t9;
  t120 = t11*x;
  t121 = t9*y;
  t117 = t115+t116-t120-t121;
  t118 = px*t18;
  t119 = py*t15;
  t122 = t13*t80*t117;
  t123 = t118+t119+t122-t18*x-t15*y;
  t124 = t2*t12*y;
  t125 = px*t6*t12;
  t126 = r*t80;
  t127 = t126-1.0;
  jac[12*i+0] = -1.0/(a*a*a)*t29*t63*t71*t74*t83;
  jac[12*i+1] = -1.0/(b*b*b)*t50*t63*t71*t74*t79;
  jac[12*i+2] = -1.0/(c*c*c)*t40*t60*t71*t74*t88;
  jac[12*i+3] = t74*(t78*log(t68)*(1.0/2.0)-e1*t71*(t67*t75*log(t66)+e2*t64*t75*t76)*(1.0/2.0));
  jac[12*i+4] = e1*t71*t74*(t55*t64*t76-e2*t55*t63*(t53*t77*log(t30)+t52*t77*log(t51)))*(1.0/2.0);
  jac[12*i+5] = t89*t91*(t19*t82*2.0-t2*t12*t20*2.0+r*t9*t57*t86*2.0)*(1.0/2.0)+e1*t71*t74*(e2*t55*t63*(t19*t21*t31*t79*t82*2.0-t2*t12*t20*t21*t22*t83*2.0)+r*t9*t55*t57*t65*t86*t88*2.0)*(1.0/2.0);
  jac[12*i+6] = t89*t91*(t19*t93*2.0+t6*t12*t20*2.0+r*t11*t57*t86*2.0)*(-1.0/2.0)-e1*t71*t74*(e2*t55*t63*(t19*t21*t31*t79*t93*2.0+t6*t12*t20*t21*t22*t83*2.0)+r*t11*t55*t57*t65*t86*t88*2.0)*(1.0/2.0);
  jac[12*i+7] = t89*t91*(t5*t20*2.0-t19*t95*2.0+r*t4*t12*t57*t86*2.0)*(1.0/2.0)+e1*t71*t74*(e2*t55*t63*(t5*t20*t21*t22*t83*2.0-t19*t21*t31*t79*t95*2.0)+r*t4*t12*t55*t57*t65*t86*t88*2.0)*(1.0/2.0);
  jac[12*i+8] = t89*t91*(t19*t98*2.0+r*t57*t86*t96*2.0)*(1.0/2.0)+e1*t71*t74*(t19*t31*t55*t63*t79*t98*2.0+r*t55*t57*t65*t86*t88*t96*2.0)*(1.0/2.0);
  jac[12*i+9] = t89*t91*(t20*t106*-2.0+t19*t114*2.0+r*t57*t86*t102*2.0)*(-1.0/2.0)+e1*t71*t74*(e2*t55*t63*(t20*t21*t22*t83*t106*2.0-t19*t21*t31*t79*t114*2.0)-r*t55*t57*t65*t86*t88*t102*2.0)*(1.0/2.0);
  jac[12*i+10] = t89*t91*(t19*t123*2.0+t20*(t124+t125-py*t2*t12-t6*t12*x)*2.0+r*t57*t86*t117*2.0)*(1.0/2.0)+e1*t71*t74*(e2*t55*t63*(t19*t21*t31*t79*t123*2.0+t20*t21*t22*t83*(t124+t125-py*t2*t12-t6*t12*x)*2.0)+r*t55*t57*t65*t86*t88*t117*2.0)*(1.0/2.0);
  jac[12*i+11] = t89*t91*(r*t60*-2.0+t19*t127*2.0+t13*t57*t86*2.0)*(-1.0/2.0)+e1*t71*t74*(t55*t88*(r*t60*t65*2.0-t13*t57*t65*t86*2.0)-t19*t31*t55*t63*t79*t127*2.0)*(1.0/2.0);



  }
}

