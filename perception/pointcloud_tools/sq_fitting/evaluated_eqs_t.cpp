/**
 * @file evaluated_eqs_t.cpp 
 * @brief Calculate with tampering
 */
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include "analytic_equations.h"
#include "evaluated_eqs_t.h"
extern "C" {
#include "levmar/levmar.h"
}
#include <math.h>
#include <SQ_fitter.h>



/**
 * @function error_metric
 */
void evaluated_sqs_t::error_metric( const SQ_parameters &_par,
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
void evaluated_sqs_t::error_metric( double* p,
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
double evaluated_sqs_t::Err_1( const double &a, const double &b, const double &c,
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
double evaluated_sqs_t::Err_2( const double &a, const double &b, const double &c,
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
double evaluated_sqs_t::Err_4( const double &a, const double &b, const double &c,
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
bool evaluated_sqs_t::minimize( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
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
  p[11] = 0.0;
  
  // Set limits
  double ub[m], lb[m];
  for( i = 0; i < 3; ++i ) { lb[i] = 0.01; ub[i] = 0.35; }
  for( i = 0; i < 2; ++i ) { lb[i+3] = 0.1; ub[i+3] = 1.9; }
  for( i = 0; i < 3; ++i ) { lb[i+5] = -1.5; ub[i+5] = 1.5; }
  for( i = 0; i < 3; ++i ) { lb[i+8] = -M_PI; ub[i+8] = M_PI; }
  lb[11] = -1.0; ub[11] = 1.0;
  
  switch( _type ) {

  case SQ_FX_RADIAL_T: {
        
    ret = dlevmar_bc_der( evaluated_sqs_t::fr_add,
			  evaluated_sqs_t::Jr_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );

    
    printf("Ret: %d \n", ret);
  }  break;    


  case SQ_FX_SOLINA_T: {
    
    ret = dlevmar_bc_der( evaluated_sqs_t::fs_add,
			  evaluated_sqs_t::Js_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );

    
    printf("Ret: %d \n", ret);
  }  break;    

  case SQ_FX_OLD_T: {

    double* xt = new double[n];
    levmar_tampering_fx( p, xt, m, n, (void*)&data );
    std::ofstream out( "old_t_fx.txt", std::ofstream::out );
    for( int k = 0; k < n; ++k ) {
      out << xt[k] << std::endl;
    }
    out.close();

    double* jt = new double[m*n];
    levmar_tampering_jac( p, jt, m, n, (void*)&data );
    std::ofstream outJ( "old_t_jc.txt", std::ofstream::out );
    for( int k = 0; k < n; ++k ) {
      for( int l = 0; l < m; ++l ) {
	outJ << jt[m*k + l] << " ";
      }
      outJ << std::endl;
    }
    outJ.close();

    
    ret = dlevmar_bc_der( levmar_tampering_fx,
			  levmar_tampering_jac,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );


    printf("Ret: %d \n", ret);
    
  }    break;    


  case SQ_FX_CHEVALIER_T: {
    
    ret = dlevmar_bc_der( evaluated_sqs_t::fc_add,
			  evaluated_sqs_t::Jc_add,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );

    
    printf("Ret: %d \n", ret);
  }  break;        

  case SQ_FX_5_T: {
    
    ret = dlevmar_bc_der( evaluated_sqs_t::f5_add,
			  evaluated_sqs_t::J5_add,
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
    _par.tamp = 1000;
    _e1 = 10000; _e2 = 10000; _e4 = 10000; // A big number
    return false;
  } else {
    _par.dim[0] = p[0]; _par.dim[1] = p[1]; _par.dim[2] = p[2];
    _par.e[0] = p[3]; _par.e[1] = p[4];
    _par.trans[0] = p[5]; _par.trans[1] = p[6]; _par.trans[2] = p[7];
    _par.rot[0] = p[8]; _par.rot[1] = p[9]; _par.rot[2] = p[10];
    _par.tamp = p[11];
    error_metric( p, _input, _e1, _e2, _e4 );
   
    return true;
  }
}


void evaluated_sqs_t::getBoundingBox( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
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
 * @brief Radial function
 */
void evaluated_sqs_t::fr_add( double* p, double* _x,
			      int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double x, y, z;


  double a,b,c,e1,e2,px,py,pz,ra,pa,ya,k;
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  k = p[11];

  register double t0, t2, t3, t4, t5, t6, t7, t8, t9;
  register double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  register double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29; 
  register double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  register double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49; 
  
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
  t15 = t4*t5*t6;
  t11 = t10-t15;
  t12 = cos(pa);
  t14 = px*t9;
  t16 = py*t11;
  t17 = t9*x;
  t18 = t11*y;
  t19 = pz*t4*t12;
  t20 = t4*t12*z;
  t13 = t14-t16-t17+t18+t19-t20;
  t21 = t4*t6;
  t35 = t2*t3*t5;
  t22 = t21-t35;
  t23 = t2*t4;
  t24 = t3*t5*t6;
  t25 = t23+t24;
  t36 = px*t22;
  t37 = py*t25;
  t38 = t22*x;
  t39 = t25*y;
  t40 = pz*t3*t12;
  t41 = t3*t12*z;
  t26 = t36-t37-t38+t39-t40+t41;
  t27 = 1.0/c;
  t28 = k*t13*t27;
  t29 = t28-1.0;
  t30 = 1.0/(t29*t29);
  t43 = pz*t5;
  t44 = t5*z;
  t45 = px*t2*t12;
  t46 = t2*t12*x;
  t47 = py*t6*t12;
  t48 = t6*t12*y;
  t31 = t43-t44-t45+t46-t47+t48;
  t32 = 1.0/e2;
  t33 = 1.0/e1;
  t34 = t13*t13;
  t42 = t26*t26;
  t49 = t31*t31;
  t0 = fabs(pow(pow(1.0/(c*c)*t34,t33)+pow(pow(1.0/(a*a)*t30*t49,t32)+pow(1.0/(b*b)*t30*t42,t32),e2*t33),e1*(-1.0/2.0))-1.0)*sqrt(t34+t30*t42+t30*t49);

  _x[i] = t0;
  }
}



/**
 * @function Jr_add
 */
void evaluated_sqs_t::Jr_add( double* p, double* jac,
			      int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;

  double x, y, z;
  double a, b, c, e1, e2, ra, pa, ya, px,py,pz,k;

  
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  k = p[11];

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
  register double t110, t111;

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
  t15 = t4*t5*t6;
  t11 = t10-t15;
  t12 = cos(pa);
  t14 = px*t9;
  t16 = py*t11;
  t17 = t9*x;
  t18 = t11*y;
  t19 = pz*t4*t12;
  t20 = t4*t12*z;
  t13 = t14-t16-t17+t18+t19-t20;
  t21 = t4*t6;
  t39 = t2*t3*t5;
  t22 = t21-t39;
  t23 = t2*t4;
  t24 = t3*t5*t6;
  t25 = t23+t24;
  t40 = px*t22;
  t41 = py*t25;
  t42 = t22*x;
  t43 = t25*y;
  t44 = pz*t3*t12;
  t45 = t3*t12*z;
  t26 = t40-t41-t42+t43-t44+t45;
  t27 = 1.0/c;
  t28 = k*t13*t27;
  t29 = t28-1.0;
  t30 = 1.0/(t29*t29);
  t50 = pz*t5;
  t51 = t5*z;
  t52 = px*t2*t12;
  t53 = t2*t12*x;
  t54 = py*t6*t12;
  t55 = t6*t12*y;
  t31 = t50-t51-t52+t53-t54+t55;
  t32 = 1.0/e2;
  t33 = 1.0/e1;
  t34 = 1.0/(c*c);
  t35 = t13*t13;
  t36 = t34*t35;
  t37 = pow(t36,t33);
  t38 = 1.0/(b*b);
  t46 = t26*t26;
  t47 = t30*t38*t46;
  t48 = pow(t47,t32);
  t49 = 1.0/(a*a);
  t56 = t31*t31;
  t57 = t30*t49*t56;
  t58 = pow(t57,t32);
  t59 = t48+t58;
  t60 = e2*t33;
  t61 = pow(t59,t60);
  t62 = t37+t61;
  t66 = e1*(1.0/2.0);
  t63 = pow(t62,-t66);
  t64 = t63-1.0;
  t65 = (t64/fabs(t64));
  t67 = t60-1.0;
  t68 = pow(t59,t67);
  t69 = t30*t46;
  t70 = t30*t56;
  t71 = t35+t69+t70;
  t72 = sqrt(t71);
  t73 = t32-1.0;
  t74 = 1.0/(t29*t29*t29);
  t75 = -t66-1.0;
  t76 = pow(t62,t75);
  t77 = pow(t47,t73);
  t78 = pow(t57,t73);
  t79 = 1.0/(e1*e1);
  t80 = log(t59);
  t81 = 1.0/(e2*e2);
  t82 = fabs(t64);
  t83 = 1.0/sqrt(t71);
  t84 = t33-1.0;
  t85 = pow(t36,t84);
  t86 = pz*t4*t5;
  t87 = t2*t4*t12*x;
  t88 = t4*t6*t12*y;
  t90 = t4*t5*z;
  t91 = px*t2*t4*t12;
  t92 = py*t4*t6*t12;
  t89 = t86+t87+t88-t90-t91-t92;
  t93 = pz*t3*t5;
  t94 = t2*t3*t12*x;
  t95 = t3*t6*t12*y;
  t96 = t93+t94+t95-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12;
  t97 = pz*t12;
  t98 = px*t2*t5;
  t99 = py*t5*t6;
  t100 = t97+t98+t99-t12*z-t2*t5*x-t5*t6*y;
  t101 = px*t11;
  t102 = py*t9;
  t104 = t11*x;
  t105 = t9*y;
  t103 = t101+t102-t104-t105;
  t106 = px*t25;
  t107 = py*t22;
  t108 = t106+t107-t25*x-t22*y;
  t109 = t2*t12*y;
  t110 = px*t6*t12;
  t111 = t109+t110-py*t2*t12-t6*t12*x;
  jac[12*i+0] = 1.0/(a*a*a)*t30*t56*pow(t62,e1*(-1.0/2.0)-1.0)*t65*t68*t72*t78;
  jac[12*i+1] = 1.0/(b*b*b)*t30*t46*t65*t68*t72*t76*t77;
  jac[12*i+2] = t82*t83*(k*t13*t34*t46*t74*2.0+k*t13*t34*t56*t74*2.0)*(1.0/2.0)+e1*t65*t72*t76*(1.0/(c*c*c)*t33*t35*t85*2.0-e2*t33*t68*(k*t13*t32*t34*t38*t46*t74*t77*2.0+k*t13*t32*t34*t49*t56*t74*t78*2.0))*(1.0/2.0);
  jac[12*i+3] = -t65*t72*(t63*log(t62)*(1.0/2.0)-e1*t76*(t37*t79*log(t36)+e2*t61*t79*t80)*(1.0/2.0));
  jac[12*i+4] = e1*t65*t72*t76*(t33*t61*t80-e2*t33*t68*(t48*t81*log(t47)+t58*t81*log(t57)))*(-1.0/2.0);
  jac[12*i+5] = t82*t83*(t9*t13*-2.0-t22*t26*t30*2.0+t2*t12*t30*t31*2.0+k*t9*t27*t46*t74*2.0+k*t9*t27*t56*t74*2.0)*(-1.0/2.0)+e1*t65*t72*t76*(e2*t33*t68*(t32*t78*(t2*t12*t30*t31*t49*2.0+k*t9*t27*t49*t56*t74*2.0)-t32*t77*(t22*t26*t30*t38*2.0-k*t9*t27*t38*t46*t74*2.0))-t9*t13*t33*t34*t85*2.0)*(1.0/2.0);
  jac[12*i+6] = t82*t83*(t11*t13*2.0+t25*t26*t30*2.0+t6*t12*t30*t31*2.0-k*t11*t27*t46*t74*2.0-k*t11*t27*t56*t74*2.0)*(-1.0/2.0)+e1*t65*t72*t76*(e2*t33*t68*(t32*t78*(t6*t12*t30*t31*t49*2.0-k*t11*t27*t49*t56*t74*2.0)+t32*t77*(t25*t26*t30*t38*2.0-k*t11*t27*t38*t46*t74*2.0))+t11*t13*t33*t34*t85*2.0)*(1.0/2.0);
  jac[12*i+7] = t82*t83*(t4*t12*t13*-2.0-t5*t30*t31*2.0+t3*t12*t26*t30*2.0+k*t4*t12*t27*t46*t74*2.0+k*t4*t12*t27*t56*t74*2.0)*(-1.0/2.0)-e1*t65*t72*t76*(e2*t33*t68*(t32*t78*(t5*t30*t31*t49*2.0-k*t4*t12*t27*t49*t56*t74*2.0)-t32*t77*(t3*t12*t26*t30*t38*2.0+k*t4*t12*t27*t38*t46*t74*2.0))+t4*t12*t13*t33*t34*t85*2.0)*(1.0/2.0);
  jac[12*i+8] = t82*t83*(t13*t26*-2.0+t13*t26*t30*2.0+k*t26*t27*t46*t74*2.0+k*t26*t27*t56*t74*2.0)*(-1.0/2.0)+e1*t65*t72*t76*(e2*t33*t68*(t32*t77*(t13*t26*t30*t38*2.0+k*t26*t27*t38*t46*t74*2.0)+k*t26*t27*t32*t49*t56*t74*t78*2.0)-t13*t26*t33*t34*t85*2.0)*(1.0/2.0);
  jac[12*i+9] = t82*t83*(t13*t89*-2.0+t26*t30*t96*2.0+t30*t31*t100*2.0+k*t27*t46*t74*t89*2.0+k*t27*t56*t74*t89*2.0)*(1.0/2.0)-e1*t65*t72*t76*(e2*t33*t68*(t32*t77*(t26*t30*t38*t96*2.0+k*t27*t38*t46*t74*t89*2.0)+t32*t78*(t30*t31*t49*t100*2.0+k*t27*t49*t56*t74*t89*2.0))-t13*t33*t34*t85*t89*2.0)*(1.0/2.0);
  jac[12*i+10] = t82*t83*(t13*t103*2.0+t26*t30*t108*2.0+t30*t31*t111*2.0-k*t27*t46*t74*t103*2.0-k*t27*t56*t74*t103*2.0)*(1.0/2.0)-e1*t65*t72*t76*(e2*t33*t68*(t32*t77*(t26*t30*t38*t108*2.0-k*t27*t38*t46*t74*t103*2.0)+t32*t78*(t30*t31*t49*t111*2.0-k*t27*t49*t56*t74*t103*2.0))+t13*t33*t34*t85*t103*2.0)*(1.0/2.0);
  jac[12*i+11] = t82*t83*(t13*t27*t46*t74*2.0+t13*t27*t56*t74*2.0)*(-1.0/2.0)+e2*t65*t68*t72*t76*(t13*t27*t32*t38*t46*t74*t77*2.0+t13*t27*t32*t49*t56*t74*t78*2.0)*(1.0/2.0);

  }

}

/**
 * @function fs_add
 */
void evaluated_sqs_t::fs_add( double* p, double* _x,
			      int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double x, y, z;


  double a,b,c,e1,e2,px,py,pz,ra,pa,ya,k;
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  k = p[11];

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
  t15 = t4*t5*t6;
  t11 = t10-t15;
  t12 = cos(pa);
  t14 = px*t9;
  t16 = py*t11;
  t17 = t9*x;
  t18 = t11*y;
  t19 = pz*t4*t12;
  t20 = t4*t12*z;
  t13 = t14-t16-t17+t18+t19-t20;
  t21 = t4*t6;
  t22 = t21-t2*t3*t5;
  t23 = t2*t4;
  t24 = t3*t5*t6;
  t25 = t23+t24;
  t26 = px*t22-py*t25-t22*x+t25*y-pz*t3*t12+t3*t12*z;
  t27 = 1.0/c;
  t28 = k*t13*t27;
  t29 = t28-1.0;
  t30 = 1.0/(t29*t29);
  t31 = pz*t5-t5*z-px*t2*t12-py*t6*t12+t2*t12*x+t6*t12*y;
  t32 = 1.0/e2;
  t33 = 1.0/e1;
  t0 = (pow(pow(pow(1.0/(a*a)*t30*(t31*t31),t32)+pow(1.0/(b*b)*(t26*t26)*t30,t32),e2*t33)+pow(1.0/(c*c)*(t13*t13),t33),e1)-1.0)*sqrt(a*b*c);
  
   _x[i] = t0;
  }
}


/**
 * @function Js_add
 */
void evaluated_sqs_t::Js_add( double* p, double* jac,
			      int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;

  double x, y, z;
  double a, b, c, e1, e2, ra, pa, ya, px,py,pz,k;

  
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  k = p[11];

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
  t15 = t4*t5*t6;
  t11 = t10-t15;
  t12 = cos(pa);
  t14 = px*t9;
  t16 = py*t11;
  t17 = t9*x;
  t18 = t11*y;
  t19 = pz*t4*t12;
  t20 = t4*t12*z;
  t13 = t14-t16-t17+t18+t19-t20;
  t21 = t4*t6;
  t39 = t2*t3*t5;
  t22 = t21-t39;
  t23 = t2*t4;
  t24 = t3*t5*t6;
  t25 = t23+t24;
  t40 = px*t22;
  t41 = py*t25;
  t42 = t22*x;
  t43 = t25*y;
  t44 = pz*t3*t12;
  t45 = t3*t12*z;
  t26 = t40-t41-t42+t43-t44+t45;
  t27 = 1.0/c;
  t28 = k*t13*t27;
  t29 = t28-1.0;
  t30 = 1.0/(t29*t29);
  t50 = pz*t5;
  t51 = t5*z;
  t52 = px*t2*t12;
  t53 = t2*t12*x;
  t54 = py*t6*t12;
  t55 = t6*t12*y;
  t31 = t50-t51-t52+t53-t54+t55;
  t32 = 1.0/e2;
  t33 = 1.0/e1;
  t34 = 1.0/(c*c);
  t35 = t13*t13;
  t36 = t34*t35;
  t37 = pow(t36,t33);
  t38 = 1.0/(b*b);
  t46 = t26*t26;
  t47 = t30*t38*t46;
  t48 = pow(t47,t32);
  t49 = 1.0/(a*a);
  t56 = t31*t31;
  t57 = t30*t49*t56;
  t58 = pow(t57,t32);
  t59 = t48+t58;
  t60 = e2*t33;
  t61 = pow(t59,t60);
  t62 = t37+t61;
  t63 = a*b*c;
  t64 = pow(t62,e1);
  t65 = t64-1.0;
  t66 = 1.0/sqrt(t63);
  t67 = e1-1.0;
  t68 = pow(t62,t67);
  t69 = t60-1.0;
  t70 = pow(t59,t69);
  t71 = t32-1.0;
  t72 = sqrt(t63);
  t73 = pow(t47,t71);
  t74 = 1.0/(t29*t29*t29);
  t75 = pow(t57,t71);
  t76 = 1.0/(e1*e1);
  t77 = log(t59);
  t78 = 1.0/(e2*e2);
  t79 = t33-1.0;
  t80 = pow(t36,t79);
  t81 = pz*t4*t5;
  t82 = t2*t4*t12*x;
  t83 = t4*t6*t12*y;
  t85 = t4*t5*z;
  t86 = px*t2*t4*t12;
  t87 = py*t4*t6*t12;
  t84 = t81+t82+t83-t85-t86-t87;
  t88 = px*t11;
  t89 = py*t9;
  t91 = t11*x;
  t92 = t9*y;
  t90 = t88+t89-t91-t92;
  jac[12*i+0] = b*c*t65*t66*(1.0/2.0)-1.0/(a*a*a)*t30*t56*t68*t70*t72*t75*2.0;
  jac[12*i+1] = a*c*t65*t66*(1.0/2.0)-1.0/(b*b*b)*t30*t46*t68*t70*t72*t73*2.0;
  jac[12*i+2] = a*b*t65*t66*(1.0/2.0)-e1*t68*t72*(1.0/(c*c*c)*t33*t35*t80*2.0-e2*t33*t70*(k*t13*t32*t34*t38*t46*t73*t74*2.0+k*t13*t32*t34*t49*t56*t74*t75*2.0));
  jac[12*i+3] = t72*(t64*log(t62)-e1*t68*(t37*t76*log(t36)+e2*t61*t76*t77));
  jac[12*i+4] = e1*t68*t72*(t33*t61*t77-e2*t33*t70*(t48*t78*log(t47)+t58*t78*log(t57)));
  jac[12*i+5] = -e1*t68*t72*(e2*t33*t70*(t32*t75*(t2*t12*t30*t31*t49*2.0+k*t9*t27*t49*t56*t74*2.0)-t32*t73*(t22*t26*t30*t38*2.0-k*t9*t27*t38*t46*t74*2.0))-t9*t13*t33*t34*t80*2.0);
  jac[12*i+6] = -e1*t68*t72*(e2*t33*t70*(t32*t75*(t6*t12*t30*t31*t49*2.0-k*t11*t27*t49*t56*t74*2.0)+t32*t73*(t25*t26*t30*t38*2.0-k*t11*t27*t38*t46*t74*2.0))+t11*t13*t33*t34*t80*2.0);
  jac[12*i+7] = e1*t68*t72*(e2*t33*t70*(t32*t75*(t5*t30*t31*t49*2.0-k*t4*t12*t27*t49*t56*t74*2.0)-t32*t73*(t3*t12*t26*t30*t38*2.0+k*t4*t12*t27*t38*t46*t74*2.0))+t4*t12*t13*t33*t34*t80*2.0);
  jac[12*i+8] = -e1*t68*t72*(e2*t33*t70*(t32*t73*(t13*t26*t30*t38*2.0+k*t26*t27*t38*t46*t74*2.0)+k*t26*t27*t32*t49*t56*t74*t75*2.0)-t13*t26*t33*t34*t80*2.0);
  jac[12*i+9] = e1*t68*t72*(e2*t33*t70*(t32*t73*(t26*t30*t38*(pz*t3*t5-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12+t2*t3*t12*x+t3*t6*t12*y)*2.0+k*t27*t38*t46*t74*t84*2.0)+t32*t75*(t30*t31*t49*(pz*t12-t12*z+px*t2*t5+py*t5*t6-t2*t5*x-t5*t6*y)*2.0+k*t27*t49*t56*t74*t84*2.0))-t13*t33*t34*t80*t84*2.0);
  jac[12*i+10] = e1*t68*t72*(e2*t33*t70*(t32*t75*(t30*t31*t49*(px*t6*t12-py*t2*t12-t6*t12*x+t2*t12*y)*2.0-k*t27*t49*t56*t74*t90*2.0)+t32*t73*(t26*t30*t38*(px*t25+py*t22-t25*x-t22*y)*2.0-k*t27*t38*t46*t74*t90*2.0))+t13*t33*t34*t80*t90*2.0);
  jac[12*i+11] = -e2*t68*t70*t72*(t13*t27*t32*t38*t46*t73*t74*2.0+t13*t27*t32*t49*t56*t74*t75*2.0);


  }
}


/**
 * @function fc_add
 */
void evaluated_sqs_t::fc_add( double* p, double* _x,
			      int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double x, y, z;


  double a,b,c,e1,e2,px,py,pz,ra,pa,ya,k;
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  k = p[11];

  register double t0, t2, t3, t4, t5, t6, t7, t8, t9;
  register double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  register double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29; 
  register double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39; 
  register double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49; 
  
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
  t15 = t4*t5*t6;
  t11 = t10-t15;
  t12 = cos(pa);
  t14 = px*t9;
  t16 = py*t11;
  t17 = t9*x;
  t18 = t11*y;
  t19 = pz*t4*t12;
  t20 = t4*t12*z;
  t13 = t14-t16-t17+t18+t19-t20;
  t21 = t4*t6;
  t35 = t2*t3*t5;
  t22 = t21-t35;
  t23 = t2*t4;
  t24 = t3*t5*t6;
  t25 = t23+t24;
  t36 = px*t22;
  t37 = py*t25;
  t38 = t22*x;
  t39 = t25*y;
  t40 = pz*t3*t12;
  t41 = t3*t12*z;
  t26 = t36-t37-t38+t39-t40+t41;
  t27 = 1.0/c;
  t28 = k*t13*t27;
  t29 = t28-1.0;
  t30 = 1.0/(t29*t29);
  t43 = pz*t5;
  t44 = t5*z;
  t45 = px*t2*t12;
  t46 = t2*t12*x;
  t47 = py*t6*t12;
  t48 = t6*t12*y;
  t31 = t43-t44-t45+t46-t47+t48;
  t32 = 1.0/e2;
  t33 = 1.0/e1;
  t34 = t13*t13;
  t42 = t26*t26;
  t49 = t31*t31;
  t0 = (pow(pow(1.0/(c*c)*t34,t33)+pow(pow(1.0/(a*a)*t30*t49,t32)+pow(1.0/(b*b)*t30*t42,t32),e2*t33),e1*(1.0/2.0))-1.0)*sqrt(t34+t30*t42+t30*t49);
 
   _x[i] = t0;
  }

}


/**
 * @function Jc_add
 * @brief Chevalier
 */
void evaluated_sqs_t::Jc_add( double* p, double* jac,
			      int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;

  double x, y, z;
  double a, b, c, e1, e2, ra, pa, ya, px,py,pz,k;

  
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  k = p[11];

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
  t15 = t4*t5*t6;
  t11 = t10-t15;
  t12 = cos(pa);
  t14 = px*t9;
  t16 = py*t11;
  t17 = t9*x;
  t18 = t11*y;
  t19 = pz*t4*t12;
  t20 = t4*t12*z;
  t13 = t14-t16-t17+t18+t19-t20;
  t21 = t4*t6;
  t35 = t2*t3*t5;
  t22 = t21-t35;
  t23 = t2*t4;
  t24 = t3*t5*t6;
  t25 = t23+t24;
  t36 = px*t22;
  t37 = py*t25;
  t38 = t22*x;
  t39 = t25*y;
  t40 = pz*t3*t12;
  t41 = t3*t12*z;
  t26 = t36-t37-t38+t39-t40+t41;
  t27 = 1.0/c;
  t28 = k*t13*t27;
  t29 = t28-1.0;
  t30 = 1.0/(t29*t29);
  t46 = pz*t5;
  t47 = t5*z;
  t48 = px*t2*t12;
  t49 = t2*t12*x;
  t50 = py*t6*t12;
  t51 = t6*t12*y;
  t31 = t46-t47-t48+t49-t50+t51;
  t32 = 1.0/e2;
  t33 = 1.0/e1;
  t34 = 1.0/(b*b);
  t42 = t26*t26;
  t43 = t30*t34*t42;
  t44 = pow(t43,t32);
  t45 = 1.0/(a*a);
  t52 = t31*t31;
  t53 = t30*t45*t52;
  t54 = pow(t53,t32);
  t55 = t44+t54;
  t56 = e2*t33;
  t57 = t13*t13;
  t58 = 1.0/(c*c);
  t59 = t57*t58;
  t60 = pow(t59,t33);
  t61 = pow(t55,t56);
  t62 = t60+t61;
  t63 = e1*(1.0/2.0);
  t64 = t63-1.0;
  t65 = pow(t62,t64);
  t66 = t56-1.0;
  t67 = pow(t55,t66);
  t68 = t30*t42;
  t69 = t30*t52;
  t70 = t57+t68+t69;
  t71 = sqrt(t70);
  t72 = t32-1.0;
  t73 = 1.0/(t29*t29*t29);
  t74 = pow(t43,t72);
  t75 = pow(t53,t72);
  t76 = pow(t62,t63);
  t77 = 1.0/(e1*e1);
  t78 = log(t55);
  t79 = 1.0/(e2*e2);
  t80 = t76-1.0;
  t81 = 1.0/sqrt(t70);
  t82 = t33-1.0;
  t83 = pow(t59,t82);
  t84 = pz*t4*t5;
  t85 = t2*t4*t12*x;
  t86 = t4*t6*t12*y;
  t88 = t4*t5*z;
  t89 = px*t2*t4*t12;
  t90 = py*t4*t6*t12;
  t87 = t84+t85+t86-t88-t89-t90;
  t91 = pz*t3*t5;
  t92 = t2*t3*t12*x;
  t93 = t3*t6*t12*y;
  t94 = t91+t92+t93-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12;
  t95 = pz*t12;
  t96 = px*t2*t5;
  t97 = py*t5*t6;
  t98 = t95+t96+t97-t12*z-t2*t5*x-t5*t6*y;
  t99 = px*t11;
  t100 = py*t9;
  t102 = t11*x;
  t103 = t9*y;
  t101 = t99+t100-t102-t103;
  t104 = px*t25;
  t105 = py*t22;
  t106 = t104+t105-t25*x-t22*y;
  t107 = t2*t12*y;
  t108 = px*t6*t12;
  t109 = t107+t108-py*t2*t12-t6*t12*x;
  jac[12*i+0] = -1.0/(a*a*a)*t30*t52*t65*t67*t71*t75;
  jac[12*i+1] = -1.0/(b*b*b)*t30*t42*t65*t67*t71*t74;
  jac[12*i+2] = t80*t81*(k*t13*t42*t58*t73*2.0+k*t13*t52*t58*t73*2.0)*(1.0/2.0)-e1*t65*t71*(1.0/(c*c*c)*t33*t57*t83*2.0-e2*t33*t67*(k*t13*t32*t34*t42*t58*t73*t74*2.0+k*t13*t32*t45*t52*t58*t73*t75*2.0))*(1.0/2.0);
  jac[12*i+3] = t71*(t76*log(t62)*(1.0/2.0)-e1*t65*(t60*t77*log(t59)+e2*t61*t77*t78)*(1.0/2.0));
  jac[12*i+4] = e1*t65*t71*(t33*t61*t78-e2*t33*t67*(t44*t79*log(t43)+t54*t79*log(t53)))*(1.0/2.0);
  jac[12*i+5] = t80*t81*(t9*t13*-2.0-t22*t26*t30*2.0+t2*t12*t30*t31*2.0+k*t9*t27*t42*t73*2.0+k*t9*t27*t52*t73*2.0)*(-1.0/2.0)-e1*t65*t71*(e2*t33*t67*(t32*t75*(t2*t12*t30*t31*t45*2.0+k*t9*t27*t45*t52*t73*2.0)-t32*t74*(t22*t26*t30*t34*2.0-k*t9*t27*t34*t42*t73*2.0))-t9*t13*t33*t58*t83*2.0)*(1.0/2.0);
  jac[12*i+6] = t80*t81*(t11*t13*2.0+t25*t26*t30*2.0+t6*t12*t30*t31*2.0-k*t11*t27*t42*t73*2.0-k*t11*t27*t52*t73*2.0)*(-1.0/2.0)-e1*t65*t71*(e2*t33*t67*(t32*t75*(t6*t12*t30*t31*t45*2.0-k*t11*t27*t45*t52*t73*2.0)+t32*t74*(t25*t26*t30*t34*2.0-k*t11*t27*t34*t42*t73*2.0))+t11*t13*t33*t58*t83*2.0)*(1.0/2.0);
  jac[12*i+7] = t80*t81*(t4*t12*t13*-2.0-t5*t30*t31*2.0+t3*t12*t26*t30*2.0+k*t4*t12*t27*t42*t73*2.0+k*t4*t12*t27*t52*t73*2.0)*(-1.0/2.0)+e1*t65*t71*(e2*t33*t67*(t32*t75*(t5*t30*t31*t45*2.0-k*t4*t12*t27*t45*t52*t73*2.0)-t32*t74*(t3*t12*t26*t30*t34*2.0+k*t4*t12*t27*t34*t42*t73*2.0))+t4*t12*t13*t33*t58*t83*2.0)*(1.0/2.0);
  jac[12*i+8] = t80*t81*(t13*t26*-2.0+t13*t26*t30*2.0+k*t26*t27*t42*t73*2.0+k*t26*t27*t52*t73*2.0)*(-1.0/2.0)-e1*t65*t71*(e2*t33*t67*(t32*t74*(t13*t26*t30*t34*2.0+k*t26*t27*t34*t42*t73*2.0)+k*t26*t27*t32*t45*t52*t73*t75*2.0)-t13*t26*t33*t58*t83*2.0)*(1.0/2.0);
  jac[12*i+9] = t80*t81*(t13*t87*-2.0+t26*t30*t94*2.0+t30*t31*t98*2.0+k*t27*t42*t73*t87*2.0+k*t27*t52*t73*t87*2.0)*(1.0/2.0)+e1*t65*t71*(e2*t33*t67*(t32*t74*(t26*t30*t34*t94*2.0+k*t27*t34*t42*t73*t87*2.0)+t32*t75*(t30*t31*t45*t98*2.0+k*t27*t45*t52*t73*t87*2.0))-t13*t33*t58*t83*t87*2.0)*(1.0/2.0);
  jac[12*i+10] = t80*t81*(t13*t101*2.0+t26*t30*t106*2.0+t30*t31*t109*2.0-k*t27*t42*t73*t101*2.0-k*t27*t52*t73*t101*2.0)*(1.0/2.0)+e1*t65*t71*(e2*t33*t67*(t32*t74*(t26*t30*t34*t106*2.0-k*t27*t34*t42*t73*t101*2.0)+t32*t75*(t30*t31*t45*t109*2.0-k*t27*t45*t52*t73*t101*2.0))+t13*t33*t58*t83*t101*2.0)*(1.0/2.0);
  jac[12*i+11] = t80*t81*(t13*t27*t42*t73*2.0+t13*t27*t52*t73*2.0)*(-1.0/2.0)-e2*t65*t67*t71*(t13*t27*t32*t34*t42*t73*t74*2.0+t13*t27*t32*t45*t52*t73*t75*2.0)*(1.0/2.0);


  }

}


/**
 * @function f5_add
 */
void evaluated_sqs_t::f5_add( double* p, double* _x,
			      int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double x, y, z;


  double a,b,c,e1,e2,px,py,pz,ra,pa,ya,k;
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  k = p[11];

  register double t0, t2, t3, t4, t5, t6, t7, t8, t9;
  register double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  register double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29; 
  register double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39; 
  register double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49; 
  
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
  t15 = t4*t5*t6;
  t11 = t10-t15;
  t12 = cos(pa);
  t14 = px*t9;
  t16 = py*t11;
  t17 = t9*x;
  t18 = t11*y;
  t19 = pz*t4*t12;
  t20 = t4*t12*z;
  t13 = t14-t16-t17+t18+t19-t20;
  t21 = t4*t6;
  t35 = t2*t3*t5;
  t22 = t21-t35;
  t23 = t2*t4;
  t24 = t3*t5*t6;
  t25 = t23+t24;
  t36 = px*t22;
  t37 = py*t25;
  t38 = t22*x;
  t39 = t25*y;
  t40 = pz*t3*t12;
  t41 = t3*t12*z;
  t26 = t36-t37-t38+t39-t40+t41;
  t27 = 1.0/c;
  t28 = k*t13*t27;
  t29 = t28-1.0;
  t30 = 1.0/(t29*t29);
  t43 = pz*t5;
  t44 = t5*z;
  t45 = px*t2*t12;
  t46 = t2*t12*x;
  t47 = py*t6*t12;
  t48 = t6*t12*y;
  t31 = t43-t44-t45+t46-t47+t48;
  t32 = 1.0/e2;
  t33 = 1.0/e1;
  t34 = t13*t13;
  t42 = t26*t26;
  t49 = t31*t31;
  t0 = (pow(pow(1.0/(c*c)*t34,t33)+pow(pow(1.0/(a*a)*t30*t49,t32)+pow(1.0/(b*b)*t30*t42,t32),e2*t33),e1)-1.0)*sqrt(t34+t30*t42+t30*t49);

    _x[i] = t0;
  }
}


/**
 * @function J5_add
 * @brief Trial 5
 */
void evaluated_sqs_t::J5_add( double* p, double* jac,
			      int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;

  double x, y, z;
  double a, b, c, e1, e2, ra, pa, ya, px,py,pz,k;

  
  a = p[0]; b = p[1]; c = p[2];
  e1 = p[3]; e2 = p[4];
  px = p[5]; py = p[6]; pz = p[7];
  ra = p[8]; pa = p[9]; ya = p[10];
  k = p[11];

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
  t15 = t4*t5*t6;
  t11 = t10-t15;
  t12 = cos(pa);
  t14 = px*t9;
  t16 = py*t11;
  t17 = t9*x;
  t18 = t11*y;
  t19 = pz*t4*t12;
  t20 = t4*t12*z;
  t13 = t14-t16-t17+t18+t19-t20;
  t21 = t4*t6;
  t35 = t2*t3*t5;
  t22 = t21-t35;
  t23 = t2*t4;
  t24 = t3*t5*t6;
  t25 = t23+t24;
  t36 = px*t22;
  t37 = py*t25;
  t38 = t22*x;
  t39 = t25*y;
  t40 = pz*t3*t12;
  t41 = t3*t12*z;
  t26 = t36-t37-t38+t39-t40+t41;
  t27 = 1.0/c;
  t28 = k*t13*t27;
  t29 = t28-1.0;
  t30 = 1.0/(t29*t29);
  t46 = pz*t5;
  t47 = t5*z;
  t48 = px*t2*t12;
  t49 = t2*t12*x;
  t50 = py*t6*t12;
  t51 = t6*t12*y;
  t31 = t46-t47-t48+t49-t50+t51;
  t32 = 1.0/e2;
  t33 = 1.0/e1;
  t34 = 1.0/(b*b);
  t42 = t26*t26;
  t43 = t30*t34*t42;
  t44 = pow(t43,t32);
  t45 = 1.0/(a*a);
  t52 = t31*t31;
  t53 = t30*t45*t52;
  t54 = pow(t53,t32);
  t55 = t44+t54;
  t56 = e2*t33;
  t57 = t13*t13;
  t58 = 1.0/(c*c);
  t59 = t57*t58;
  t60 = pow(t59,t33);
  t61 = pow(t55,t56);
  t62 = t60+t61;
  t63 = e1-1.0;
  t64 = pow(t62,t63);
  t65 = t56-1.0;
  t66 = pow(t55,t65);
  t67 = t30*t42;
  t68 = t30*t52;
  t69 = t57+t67+t68;
  t70 = sqrt(t69);
  t71 = t32-1.0;
  t72 = 1.0/(t29*t29*t29);
  t73 = pow(t43,t71);
  t74 = pow(t53,t71);
  t75 = pow(t62,e1);
  t76 = 1.0/(e1*e1);
  t77 = log(t55);
  t78 = 1.0/(e2*e2);
  t79 = t75-1.0;
  t80 = 1.0/sqrt(t69);
  t81 = t33-1.0;
  t82 = pow(t59,t81);
  t83 = pz*t4*t5;
  t84 = t2*t4*t12*x;
  t85 = t4*t6*t12*y;
  t87 = t4*t5*z;
  t88 = px*t2*t4*t12;
  t89 = py*t4*t6*t12;
  t86 = t83+t84+t85-t87-t88-t89;
  t90 = pz*t3*t5;
  t91 = t2*t3*t12*x;
  t92 = t3*t6*t12*y;
  t93 = t90+t91+t92-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12;
  t94 = pz*t12;
  t95 = px*t2*t5;
  t96 = py*t5*t6;
  t97 = t94+t95+t96-t12*z-t2*t5*x-t5*t6*y;
  t98 = px*t11;
  t99 = py*t9;
  t101 = t11*x;
  t102 = t9*y;
  t100 = t98+t99-t101-t102;
  t103 = px*t25;
  t104 = py*t22;
  t105 = t103+t104-t25*x-t22*y;
  t106 = t2*t12*y;
  t107 = px*t6*t12;
  t108 = t106+t107-py*t2*t12-t6*t12*x;
  jac[12*i+0] = 1.0/(a*a*a)*t30*t52*t64*t66*t70*t74*-2.0;
  jac[12*i+1] = 1.0/(b*b*b)*t30*t42*t64*t66*t70*t73*-2.0;
  jac[12*i+2] = t79*t80*(k*t13*t42*t58*t72*2.0+k*t13*t52*t58*t72*2.0)*(1.0/2.0)-e1*t64*t70*(1.0/(c*c*c)*t33*t57*t82*2.0-e2*t33*t66*(k*t13*t32*t34*t42*t58*t72*t73*2.0+k*t13*t32*t45*t52*t58*t72*t74*2.0));
  jac[12*i+3] = t70*(t75*log(t62)-e1*t64*(t60*t76*log(t59)+e2*t61*t76*t77));
  jac[12*i+4] = e1*t64*t70*(t33*t61*t77-e2*t33*t66*(t44*t78*log(t43)+t54*t78*log(t53)));
  jac[12*i+5] = t79*t80*(t9*t13*-2.0-t22*t26*t30*2.0+t2*t12*t30*t31*2.0+k*t9*t27*t42*t72*2.0+k*t9*t27*t52*t72*2.0)*(-1.0/2.0)-e1*t64*t70*(e2*t33*t66*(t32*t74*(t2*t12*t30*t31*t45*2.0+k*t9*t27*t45*t52*t72*2.0)-t32*t73*(t22*t26*t30*t34*2.0-k*t9*t27*t34*t42*t72*2.0))-t9*t13*t33*t58*t82*2.0);
  jac[12*i+6] = t79*t80*(t11*t13*2.0+t25*t26*t30*2.0+t6*t12*t30*t31*2.0-k*t11*t27*t42*t72*2.0-k*t11*t27*t52*t72*2.0)*(-1.0/2.0)-e1*t64*t70*(e2*t33*t66*(t32*t74*(t6*t12*t30*t31*t45*2.0-k*t11*t27*t45*t52*t72*2.0)+t32*t73*(t25*t26*t30*t34*2.0-k*t11*t27*t34*t42*t72*2.0))+t11*t13*t33*t58*t82*2.0);
  jac[12*i+7] = t79*t80*(t4*t12*t13*-2.0-t5*t30*t31*2.0+t3*t12*t26*t30*2.0+k*t4*t12*t27*t42*t72*2.0+k*t4*t12*t27*t52*t72*2.0)*(-1.0/2.0)+e1*t64*t70*(e2*t33*t66*(t32*t74*(t5*t30*t31*t45*2.0-k*t4*t12*t27*t45*t52*t72*2.0)-t32*t73*(t3*t12*t26*t30*t34*2.0+k*t4*t12*t27*t34*t42*t72*2.0))+t4*t12*t13*t33*t58*t82*2.0);
  jac[12*i+8] = t79*t80*(t13*t26*-2.0+t13*t26*t30*2.0+k*t26*t27*t42*t72*2.0+k*t26*t27*t52*t72*2.0)*(-1.0/2.0)-e1*t64*t70*(e2*t33*t66*(t32*t73*(t13*t26*t30*t34*2.0+k*t26*t27*t34*t42*t72*2.0)+k*t26*t27*t32*t45*t52*t72*t74*2.0)-t13*t26*t33*t58*t82*2.0);
  jac[12*i+9] = t79*t80*(t13*t86*-2.0+t26*t30*t93*2.0+t30*t31*t97*2.0+k*t27*t42*t72*t86*2.0+k*t27*t52*t72*t86*2.0)*(1.0/2.0)+e1*t64*t70*(e2*t33*t66*(t32*t73*(t26*t30*t34*t93*2.0+k*t27*t34*t42*t72*t86*2.0)+t32*t74*(t30*t31*t45*t97*2.0+k*t27*t45*t52*t72*t86*2.0))-t13*t33*t58*t82*t86*2.0);
  jac[12*i+10] = t79*t80*(t13*t100*2.0+t26*t30*t105*2.0+t30*t31*t108*2.0-k*t27*t42*t72*t100*2.0-k*t27*t52*t72*t100*2.0)*(1.0/2.0)+e1*t64*t70*(e2*t33*t66*(t32*t73*(t26*t30*t34*t105*2.0-k*t27*t34*t42*t72*t100*2.0)+t32*t74*(t30*t31*t45*t108*2.0-k*t27*t45*t52*t72*t100*2.0))+t13*t33*t58*t82*t100*2.0);
  jac[12*i+11] = t79*t80*(t13*t27*t42*t72*2.0+t13*t27*t52*t72*2.0)*(-1.0/2.0)-e2*t64*t66*t70*(t13*t27*t32*t34*t42*t72*t73*2.0+t13*t27*t32*t45*t52*t72*t74*2.0);


  }

}




