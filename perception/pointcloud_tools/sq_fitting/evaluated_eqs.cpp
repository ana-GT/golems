

#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include "perception/pointcloud_tools/sq_fitting/evaluated_eqs.h"
extern "C" {
#include "levmar/levmar.h"
}
#include <math.h>
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter.h"




/**
 * @brief Radial Distance Error per point
 */
double Err_r( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &x, const double &y, const double &z ) {
  double t0;
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
  t0 = fabs(pow(pow(1.0/(c*c)*t30,t22)+pow(pow(1.0/(a*a)*t45,t21)+pow(1.0/(b*b)*t38,t21),e2*t22),e1*(-1.0/2.0))-1.0)*sqrt(t30+t38+t45);

  return t0;  
}

/**
 * @brief Goodness-of-fit per point
 */
double Err_g( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &x, const double &y, const double &z ) {

  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22;

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
  t0 = fabs(pow(pow(1.0/(a*a)*(t20*t20),t21)+pow(1.0/(b*b)*(t19*t19),t21),e2*t22)+pow(1.0/(c*c)*(t13*t13),t22)-1.0);
  
  return t0;
}

/**
 * @brief Using Duncan for stopping criterion (per point)
 */
double Err_d( const double &a, const double &b, const double &c,
	      const double &e1, const double &e2,
	      const double &px, const double &py, const double &pz,
	      const double &ra, const double &pa, const double &ya,
	      const double &x, const double &y, const double &z ) {

  double t0;
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
  t0 = t23*t23;

  return t0;
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

  double t0;
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
  t0 = fabs(pow(pow(1.0/(c*c)*t30,t22)+pow(pow(1.0/(a*a)*t45,t21)+pow(1.0/(b*b)*t38,t21),e2*t22),e1*(-1.0/2.0))-1.0)*sqrt(t30+t38+t45);

  return t0;
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
  double t90, t91, t92, t93, t94, t95, t96, t97, t98, t99;
  
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
  t62 = e1*(1.0/2.0);
  t59 = pow(t58,-t62);
  t60 = t59-1.0;
  t61 = (t60/fabs(t60));
  t63 = t21-1.0;
  t64 = t56-1.0;
  t65 = pow(t55,t64);
  t66 = t31+t42+t52;
  t67 = sqrt(t66);
  t68 = -t62-1.0;
  t69 = pow(t58,t68);
  t70 = 1.0/(e1*e1);
  t71 = log(t55);
  t72 = 1.0/(e2*e2);
  t73 = pow(t43,t63);
  t74 = pow(t53,t63);
  t75 = t22-1.0;
  t76 = pow(t32,t75);
  t77 = fabs(t60);
  t78 = 1.0/sqrt(t66);
  t79 = pz*t12;
  t80 = px*t2*t5;
  t81 = py*t5*t6;
  t82 = t79+t80+t81-t12*z-t2*t5*x-t5*t6*y;
  t83 = pz*t3*t5;
  t84 = t2*t3*t12*x;
  t85 = t3*t6*t12*y;
  t86 = t83+t84+t85-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12;
  t87 = pz*t4*t5;
  t88 = t2*t4*t12*x;
  t89 = t4*t6*t12*y;
  t90 = t87+t88+t89-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12;
  t91 = t2*t12*y;
  t92 = px*t6*t12;
  t93 = t91+t92-py*t2*t12-t6*t12*x;
  t94 = px*t18;
  t95 = py*t15;
  t96 = t94+t95-t18*x-t15*y;
  t97 = px*t11;
  t98 = py*t9;
  t99 = t97+t98-t11*x-t9*y;
  
  _J[0] = 1.0/(a*a*a)*t52*pow(t58,e1*(-1.0/2.0)-1.0)*t61*t65*t67*t74;
  _J[1] = 1.0/(b*b*b)*t42*t61*t65*t67*t69*t73;
  _J[2] = 1.0/(c*c*c)*t31*t61*t67*t69*t76;
  _J[3] = -t61*t67*(t59*log(t58)*(1.0/2.0)-e1*t69*(t33*t70*log(t32)+e2*t57*t70*t71)*(1.0/2.0));
  _J[4] = e1*t61*t67*t69*(t22*t57*t71-e2*t22*t65*(t44*t72*log(t43)+t54*t72*log(t53)))*(-1.0/2.0);
  _J[5] = t77*t78*(t9*t13*2.0+t15*t19*2.0-t2*t12*t20*2.0)*(1.0/2.0)-e1*t61*t67*t69*(e2*t22*t65*(t15*t19*t21*t34*t73*2.0-t2*t12*t20*t21*t45*t74*2.0)+t9*t13*t22*t23*t76*2.0)*(1.0/2.0);
  _J[6] = t77*t78*(t11*t13*2.0+t18*t19*2.0+t6*t12*t20*2.0)*(-1.0/2.0)+e1*t61*t67*t69*(e2*t22*t65*(t18*t19*t21*t34*t73*2.0+t6*t12*t20*t21*t45*t74*2.0)+t11*t13*t22*t23*t76*2.0)*(1.0/2.0);
  _J[7] = t77*t78*(t5*t20*2.0+t4*t12*t13*2.0-t3*t12*t19*2.0)*(1.0/2.0)-e1*t61*t67*t69*(e2*t22*t65*(t5*t20*t21*t45*t74*2.0-t3*t12*t19*t21*t34*t73*2.0)+t4*t12*t13*t22*t23*t76*2.0)*(1.0/2.0);
  _J[8] = e1*t61*t67*t69*(t13*t19*t22*t23*t76*2.0-t13*t19*t22*t34*t65*t73*2.0)*(-1.0/2.0);
  _J[9] = t77*t78*(t20*t82*2.0-t13*t90*2.0+t19*t86*2.0)*(1.0/2.0)-e1*t61*t67*t69*(e2*t22*t65*(t19*t21*t34*t73*t86*2.0+t20*t21*t45*t74*t82*2.0)-t13*t22*t23*t76*t90*2.0)*(1.0/2.0);
  _J[10] = t77*t78*(t13*t99*2.0+t20*t93*2.0+t19*t96*2.0)*(1.0/2.0)-e1*t61*t67*t69*(e2*t22*t65*(t19*t21*t34*t73*t96*2.0+t20*t21*t45*t74*t93*2.0)+t13*t22*t23*t76*t99*2.0)*(1.0/2.0);


}


/**
 * @function fs
 */
double fs( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z ) {

  double t0;
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

  t0 = (pow(pow(pow(1.0/(a*a)*(t20*t20),t21)+pow(1.0/(b*b)*(t19*t19),t21),e2*t22)+pow(1.0/(c*c)*(t13*t13),t22),e1)-1.0)*sqrt(a*b*c);
  
  return t0;
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
  t59 = a*b*c;
  t60 = pow(t58,e1);
  t61 = t60-1.0;
  t62 = 1.0/sqrt(t59);
  t63 = e1-1.0;
  t64 = pow(t58,t63);
  t65 = t21-1.0;
  t66 = t56-1.0;
  t67 = pow(t55,t66);
  t68 = sqrt(t59);
  t69 = 1.0/(e1*e1);
  t70 = log(t55);
  t71 = 1.0/(e2*e2);
  t72 = pow(t43,t65);
  t73 = pow(t53,t65);
  t74 = t22-1.0;
  t75 = pow(t32,t74);
  
  _J[0] = b*c*t61*t62*(1.0/2.0)-1.0/(a*a*a)*t52*t64*t67*t68*t73*2.0;
  _J[1] = a*c*t61*t62*(1.0/2.0)-1.0/(b*b*b)*t42*t64*t67*t68*t72*2.0;
  _J[2] = a*b*t61*t62*(1.0/2.0)-1.0/(c*c*c)*t31*t64*t68*t75*2.0;
  _J[3] = t68*(t60*log(t58)-e1*t64*(t33*t69*log(t32)+e2*t57*t69*t70));
  _J[4] = e1*t64*t68*(t22*t57*t70-e2*t22*t67*(t44*t71*log(t43)+t54*t71*log(t53)));
  _J[5] = e1*t64*t68*(e2*t22*t67*(t15*t19*t21*t34*t72*2.0-t2*t12*t20*t21*t45*t73*2.0)+t9*t13*t22*t23*t75*2.0);
  _J[6] = -e1*t64*t68*(e2*t22*t67*(t18*t19*t21*t34*t72*2.0+t6*t12*t20*t21*t45*t73*2.0)+t11*t13*t22*t23*t75*2.0);
  _J[7] = e1*t64*t68*(e2*t22*t67*(t5*t20*t21*t45*t73*2.0-t3*t12*t19*t21*t34*t72*2.0)+t4*t12*t13*t22*t23*t75*2.0);
  _J[8] = e1*t64*t68*(t13*t19*t22*t23*t75*2.0-t13*t19*t22*t34*t67*t72*2.0);
  _J[9] = e1*t64*t68*(e2*t22*t67*(t20*t21*t45*t73*(pz*t12-t12*z+px*t2*t5+py*t5*t6-t2*t5*x-t5*t6*y)*2.0+t19*t21*t34*t72*(pz*t3*t5-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12+t2*t3*t12*x+t3*t6*t12*y)*2.0)-t13*t22*t23*t75*(pz*t4*t5-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12+t2*t4*t12*x+t4*t6*t12*y)*2.0);
  _J[10] = e1*t64*t68*(e2*t22*t67*(t20*t21*t45*t73*(px*t6*t12-py*t2*t12-t6*t12*x+t2*t12*y)*2.0+t19*t21*t34*t72*(px*t18+py*t15-t18*x-t15*y)*2.0)+t13*t22*t23*t75*(px*t11+py*t9-t11*x-t9*y)*2.0);
 
}


/**
 * @function fi
 */
double fi( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z ) {

  double t0;
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
  t0 = (pow(pow(1.0/(c*c)*t30,t22)+pow(pow(1.0/(a*a)*t45,t21)+pow(1.0/(b*b)*t38,t21),e2*t22),e1)-1.0)*sqrt(a*b*c)*sqrt(t30+t38+t45);

  return t0;  
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
  double t90, t91, t92, t93, t94, t95, t96, t97, t98, t99;

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
  t46 = 1.0/(c*c);
  t47 = t30*t46;
  t48 = pow(t47,t22);
  t49 = 1.0/(b*b);
  t50 = t38*t49;
  t51 = pow(t50,t21);
  t52 = 1.0/(a*a);
  t53 = t45*t52;
  t54 = pow(t53,t21);
  t55 = t51+t54;
  t56 = e2*t22;
  t57 = pow(t55,t56);
  t58 = t48+t57;
  t59 = t30+t38+t45;
  t60 = sqrt(t59);
  t61 = a*b*c;
  t62 = pow(t58,e1);
  t63 = t62-1.0;
  t64 = 1.0/sqrt(t61);
  t65 = e1-1.0;
  t66 = pow(t58,t65);
  t67 = t21-1.0;
  t68 = t56-1.0;
  t69 = pow(t55,t68);
  t70 = sqrt(t61);
  t71 = 1.0/(e1*e1);
  t72 = log(t55);
  t73 = 1.0/(e2*e2);
  t74 = pow(t50,t67);
  t75 = pow(t53,t67);
  t76 = t22-1.0;
  t77 = pow(t47,t76);
  t78 = 1.0/sqrt(t59);
  t79 = pz*t12;
  t80 = px*t2*t5;
  t81 = py*t5*t6;
  t82 = t79+t80+t81-t12*z-t2*t5*x-t5*t6*y;
  t83 = pz*t3*t5;
  t84 = t2*t3*t12*x;
  t85 = t3*t6*t12*y;
  t86 = t83+t84+t85-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12;
  t87 = pz*t4*t5;
  t88 = t2*t4*t12*x;
  t89 = t4*t6*t12*y;
  t90 = t87+t88+t89-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12;
  t91 = t2*t12*y;
  t92 = px*t6*t12;
  t93 = t91+t92-py*t2*t12-t6*t12*x;
  t94 = px*t18;
  t95 = py*t15;
  t96 = t94+t95-t18*x-t15*y;
  t97 = px*t11;
  t98 = py*t9;
  t99 = t97+t98-t11*x-t9*y;
  
  _J[0] = b*c*t60*t63*t64*(1.0/2.0)-1.0/(a*a*a)*t45*t60*t66*t69*t70*t75*2.0;
  _J[1] = a*c*t60*t63*t64*(1.0/2.0)-1.0/(b*b*b)*t38*t60*t66*t69*t70*t74*2.0;
  _J[2] = a*b*t60*t63*t64*(1.0/2.0)-1.0/(c*c*c)*t30*t60*t66*t70*t77*2.0;
  _J[3] = t60*t70*(t62*log(t58)-e1*t66*(t48*t71*log(t47)+e2*t57*t71*t72));
  _J[4] = e1*t60*t66*t70*(t22*t57*t72-e2*t22*t69*(t51*t73*log(t50)+t54*t73*log(t53)));
  _J[5] = t63*t70*t78*(t9*t13*2.0+t15*t19*2.0-t2*t12*t20*2.0)*(1.0/2.0)+e1*t60*t66*t70*(e2*t22*t69*(t15*t19*t21*t49*t74*2.0-t2*t12*t20*t21*t52*t75*2.0)+t9*t13*t22*t46*t77*2.0);
  _J[6] = t63*t70*t78*(t11*t13*2.0+t18*t19*2.0+t6*t12*t20*2.0)*(-1.0/2.0)-e1*t60*t66*t70*(e2*t22*t69*(t18*t19*t21*t49*t74*2.0+t6*t12*t20*t21*t52*t75*2.0)+t11*t13*t22*t46*t77*2.0);
  _J[7] = t63*t70*t78*(t5*t20*2.0+t4*t12*t13*2.0-t3*t12*t19*2.0)*(1.0/2.0)+e1*t60*t66*t70*(e2*t22*t69*(t5*t20*t21*t52*t75*2.0-t3*t12*t19*t21*t49*t74*2.0)+t4*t12*t13*t22*t46*t77*2.0);
  _J[8] = e1*t60*t66*t70*(t13*t19*t22*t46*t77*2.0-t13*t19*t22*t49*t69*t74*2.0);
  _J[9] = t63*t70*t78*(t20*t82*2.0-t13*t90*2.0+t19*t86*2.0)*(1.0/2.0)+e1*t60*t66*t70*(e2*t22*t69*(t19*t21*t49*t74*t86*2.0+t20*t21*t52*t75*t82*2.0)-t13*t22*t46*t77*t90*2.0);
  _J[10] = t63*t70*t78*(t13*t99*2.0+t20*t93*2.0+t19*t96*2.0)*(1.0/2.0)+e1*t60*t66*t70*(e2*t22*t69*(t19*t21*t49*t74*t96*2.0+t20*t21*t52*t75*t93*2.0)+t13*t22*t46*t77*t99*2.0);
  
}


/**
 * @function fc
 */
double fc( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z ) {

  double t0;
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
  t0 = (pow(pow(1.0/(c*c)*t30,t22)+pow(pow(1.0/(a*a)*t45,t21)+pow(1.0/(b*b)*t38,t21),e2*t22),e1*(1.0/2.0))-1.0)*sqrt(t30+t38+t45);

  return t0;
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
  double t90, t91, t92, t93, t94, t95, t96, t97;

  t2 = cos(ya);
  t3 = sin(ra);
  t4 = cos(ra);
  t5 = sin(pa);
  t6 = sin(ya);
  t7 = t3*t6;
  t8 = t2*t4*t5;
  t9 = t7+t8;
  t10 = t2*t3;
  t47 = t4*t5*t6;
  t11 = t10-t47;
  t12 = cos(pa);
  t46 = px*t9;
  t48 = py*t11;
  t49 = t9*x;
  t50 = t11*y;
  t51 = pz*t4*t12;
  t52 = t4*t12*z;
  t13 = t46-t48-t49+t50+t51-t52;
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
  t53 = t13*t13;
  t54 = 1.0/(c*c);
  t55 = t53*t54;
  t56 = pow(t55,t22);
  t57 = pow(t44,t45);
  t58 = t56+t57;
  t59 = e1*(1.0/2.0);
  t60 = t59-1.0;
  t61 = pow(t58,t60);
  t62 = t21-1.0;
  t63 = t45-1.0;
  t64 = pow(t44,t63);
  t65 = t30+t40+t53;
  t66 = sqrt(t65);
  t67 = 1.0/(e1*e1);
  t68 = log(t44);
  t69 = 1.0/(e2*e2);
  t70 = pow(t58,t59);
  t71 = pow(t41,t62);
  t72 = pow(t31,t62);
  t73 = t22-1.0;
  t74 = pow(t55,t73);
  t75 = t70-1.0;
  t76 = 1.0/sqrt(t65);
  t77 = pz*t12;
  t78 = px*t2*t5;
  t79 = py*t5*t6;
  t80 = t77+t78+t79-t12*z-t2*t5*x-t5*t6*y;
  t81 = pz*t3*t5;
  t82 = t2*t3*t12*x;
  t83 = t3*t6*t12*y;
  t84 = t81+t82+t83-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12;
  t85 = pz*t4*t5;
  t86 = t2*t4*t12*x;
  t87 = t4*t6*t12*y;
  t88 = t85+t86+t87-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12;
  t89 = t2*t12*y;
  t90 = px*t6*t12;
  t91 = t89+t90-py*t2*t12-t6*t12*x;
  t92 = px*t18;
  t93 = py*t15;
  t94 = t92+t93-t18*x-t15*y;
  t95 = px*t11;
  t96 = py*t9;
  t97 = t95+t96-t11*x-t9*y;
  _J[0] = -1.0/(a*a*a)*t30*t61*t64*t66*t72;
  _J[1] = -1.0/(b*b*b)*t40*t61*t64*t66*t71;
  _J[2] = -1.0/(c*c*c)*t53*t61*t66*t74;
  _J[3] = t66*(t70*log(t58)*(1.0/2.0)-e1*t61*(t56*t67*log(t55)+e2*t57*t67*t68)*(1.0/2.0));
  _J[4] = e1*t61*t66*(t22*t57*t68-e2*t22*t64*(t43*t69*log(t31)+t42*t69*log(t41)))*(1.0/2.0);
  _J[5] = t75*t76*(t9*t13*2.0+t15*t19*2.0-t2*t12*t20*2.0)*(1.0/2.0)+e1*t61*t66*(e2*t22*t64*(t15*t19*t21*t32*t71*2.0-t2*t12*t20*t21*t23*t72*2.0)+t9*t13*t22*t54*t74*2.0)*(1.0/2.0);
  _J[6] = t75*t76*(t11*t13*2.0+t18*t19*2.0+t6*t12*t20*2.0)*(-1.0/2.0)-e1*t61*t66*(e2*t22*t64*(t18*t19*t21*t32*t71*2.0+t6*t12*t20*t21*t23*t72*2.0)+t11*t13*t22*t54*t74*2.0)*(1.0/2.0);
  _J[7] = t75*t76*(t5*t20*2.0+t4*t12*t13*2.0-t3*t12*t19*2.0)*(1.0/2.0)+e1*t61*t66*(e2*t22*t64*(t5*t20*t21*t23*t72*2.0-t3*t12*t19*t21*t32*t71*2.0)+t4*t12*t13*t22*t54*t74*2.0)*(1.0/2.0);
  _J[8] = e1*t61*t66*(t13*t19*t22*t54*t74*2.0-t13*t19*t22*t32*t64*t71*2.0)*(1.0/2.0);
  _J[9] = t75*t76*(t20*t80*2.0-t13*t88*2.0+t19*t84*2.0)*(1.0/2.0)+e1*t61*t66*(e2*t22*t64*(t20*t21*t23*t72*t80*2.0+t19*t21*t32*t71*t84*2.0)-t13*t22*t54*t74*t88*2.0)*(1.0/2.0);
  _J[10] = t75*t76*(t13*t97*2.0+t20*t91*2.0+t19*t94*2.0)*(1.0/2.0)+e1*t61*t66*(e2*t22*t64*(t20*t21*t23*t72*t91*2.0+t19*t21*t32*t71*t94*2.0)+t13*t22*t54*t74*t97*2.0)*(1.0/2.0);  
  
}



/**
 * @function f5
 */
double f5( const double &a, const double &b, const double &c,
	    const double &e1, const double &e2,
	    const double &px, const double &py, const double &pz,
	    const double &ra, const double &pa, const double &ya,
	    const double &x, const double &y, const double &z ) {

  double t0;
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
  t0 = (pow(pow(1.0/(c*c)*t30,t22)+pow(pow(1.0/(a*a)*t45,t21)+pow(1.0/(b*b)*t38,t21),e2*t22),e1)-1.0)*sqrt(t30+t38+t45);
  
  return t0;
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
  t47 = t4*t5*t6;
  t11 = t10-t47;
  t12 = cos(pa);
  t46 = px*t9;
  t48 = py*t11;
  t49 = t9*x;
  t50 = t11*y;
  t51 = pz*t4*t12;
  t52 = t4*t12*z;
  t13 = t46-t48-t49+t50+t51-t52;
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
  t53 = t13*t13;
  t54 = 1.0/(c*c);
  t55 = t53*t54;
  t56 = pow(t55,t22);
  t57 = pow(t44,t45);
  t58 = t56+t57;
  t59 = e1-1.0;
  t60 = pow(t58,t59);
  t61 = t21-1.0;
  t62 = t45-1.0;
  t63 = pow(t44,t62);
  t64 = t30+t40+t53;
  t65 = sqrt(t64);
  t66 = 1.0/(e1*e1);
  t67 = log(t44);
  t68 = 1.0/(e2*e2);
  t69 = pow(t58,e1);
  t70 = pow(t41,t61);
  t71 = pow(t31,t61);
  t72 = t22-1.0;
  t73 = pow(t55,t72);
  t74 = t69-1.0;
  t75 = 1.0/sqrt(t64);
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
  
  _J[0] = 1.0/(a*a*a)*t30*t60*t63*t65*t71*-2.0;
  _J[1] = 1.0/(b*b*b)*t40*t60*t63*t65*t70*-2.0;
  _J[2] = 1.0/(c*c*c)*t53*t60*t65*t73*-2.0;
  _J[3] = t65*(t69*log(t58)-e1*t60*(t56*t66*log(t55)+e2*t57*t66*t67));
  _J[4] = e1*t60*t65*(t22*t57*t67-e2*t22*t63*(t43*t68*log(t31)+t42*t68*log(t41)));
  _J[5] = t74*t75*(t9*t13*2.0+t15*t19*2.0-t2*t12*t20*2.0)*(1.0/2.0)+e1*t60*t65*(e2*t22*t63*(t15*t19*t21*t32*t70*2.0-t2*t12*t20*t21*t23*t71*2.0)+t9*t13*t22*t54*t73*2.0);
  _J[6] = t74*t75*(t11*t13*2.0+t18*t19*2.0+t6*t12*t20*2.0)*(-1.0/2.0)-e1*t60*t65*(e2*t22*t63*(t18*t19*t21*t32*t70*2.0+t6*t12*t20*t21*t23*t71*2.0)+t11*t13*t22*t54*t73*2.0);
  _J[7] = t74*t75*(t5*t20*2.0+t4*t12*t13*2.0-t3*t12*t19*2.0)*(1.0/2.0)+e1*t60*t65*(e2*t22*t63*(t5*t20*t21*t23*t71*2.0-t3*t12*t19*t21*t32*t70*2.0)+t4*t12*t13*t22*t54*t73*2.0);
  _J[8] = e1*t60*t65*(t13*t19*t22*t54*t73*2.0-t13*t19*t22*t32*t63*t70*2.0);
  _J[9] = t74*t75*(t20*t79*2.0-t13*t87*2.0+t19*t83*2.0)*(1.0/2.0)+e1*t60*t65*(e2*t22*t63*(t20*t21*t23*t71*t79*2.0+t19*t21*t32*t70*t83*2.0)-t13*t22*t54*t73*t87*2.0);
  _J[10] = t74*t75*(t13*t96*2.0+t20*t90*2.0+t19*t93*2.0)*(1.0/2.0)+e1*t60*t65*(e2*t22*t63*(t20*t21*t23*t71*t90*2.0+t19*t21*t32*t70*t93*2.0)+t13*t22*t54*t73*t96*2.0);
}



/**
 * @function f6
 */
double f6( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &x, const double &y, const double &z ) {

  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58;
  
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
  
  t0 = -(pow(t58,e1)-1.0)*(pow(t58,e1*(-1.0/2.0))-1.0)*sqrt(t31+t42+t52);

  return t0;
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
  t59 = 1.0/(a*a*a);
  t60 = t21-1.0;
  t61 = pow(t31,t60);
  t62 = t45-1.0;
  t63 = pow(t44,t62);
  t64 = t30+t40+t54;
  t65 = sqrt(t64);
  t66 = e1-1.0;
  t67 = pow(t58,t66);
  t71 = e1*(1.0/2.0);
  t68 = pow(t58,-t71);
  t69 = t68-1.0;
  t70 = 1.0/(b*b*b);
  t72 = pow(t41,t60);
  t73 = pow(t58,e1);
  t74 = t73-1.0;
  t75 = 1.0/(c*c*c);
  t76 = -t71-1.0;
  t77 = pow(t58,t76);
  t78 = t22-1.0;
  t79 = pow(t55,t78);
  t80 = 1.0/(e1*e1);
  t81 = log(t58);
  t82 = log(t55);
  t83 = t56*t80*t82;
  t84 = log(t44);
  t85 = e2*t57*t80*t84;
  t86 = t83+t85;
  t87 = 1.0/(e2*e2);
  t88 = t22*t57*t84;
  t89 = log(t41);
  t90 = t42*t87*t89;
  t91 = log(t31);
  t92 = t43*t87*t91;
  t93 = t90+t92;
  t94 = t88-e2*t22*t63*t93;
  t95 = t15*t19*t21*t32*t72*2.0;
  t96 = t95-t2*t12*t20*t21*t23*t61*2.0;
  t97 = e2*t22*t63*t96;
  t98 = t9*t13*t22*t46*t79*2.0;
  t99 = t97+t98;
  t100 = 1.0/sqrt(t64);
  t101 = t18*t19*t21*t32*t72*2.0;
  t102 = t6*t12*t20*t21*t23*t61*2.0;
  t103 = t101+t102;
  t104 = e2*t22*t63*t103;
  t105 = t11*t13*t22*t46*t79*2.0;
  t106 = t104+t105;
  t107 = t5*t20*t21*t23*t61*2.0;
  t108 = t107-t3*t12*t19*t21*t32*t72*2.0;
  t109 = e2*t22*t63*t108;
  t110 = t4*t12*t13*t22*t46*t79*2.0;
  t111 = t109+t110;
  t112 = t13*t19*t22*t46*t79*2.0;
  t113 = t112-t13*t19*t22*t32*t63*t72*2.0;
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
  t129 = t20*t21*t23*t61*t117*2.0;
  t133 = t19*t21*t32*t72*t121*2.0;
  t134 = t129+t133;
  t135 = e2*t22*t63*t134;
  t139 = t135-t13*t22*t46*t79*t125*2.0;
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
  t151 = t20*t21*t23*t61*t142*2.0;
  t154 = t19*t21*t32*t72*t145*2.0;
  t155 = t151+t154;
  t156 = e2*t22*t63*t155;
  t159 = t13*t22*t46*t79*t148*2.0;
  
  _J[0] = -t30*pow(t58,e1*(-1.0/2.0)-1.0)*t59*t61*t63*t65*t74+t30*t59*t61*t63*t65*t67*t69*2.0;
  _J[1] = t40*t63*t65*t67*t69*t70*t72*2.0-t40*t63*t65*t70*t72*t74*t77;
  _J[2] = t54*t65*t67*t69*t75*t79*2.0-t54*t65*t74*t75*t77*t79;
  _J[3] = -t65*t69*(t73*t81-e1*t67*t86)+t65*t74*(t68*t81*(1.0/2.0)-e1*t77*t86*(1.0/2.0));
  _J[4] = -e1*t65*t67*t69*t94+e1*t65*t74*t77*t94*(1.0/2.0);
  _J[5] = t69*t74*t100*(t9*t13*2.0+t15*t19*2.0-t2*t12*t20*2.0)*(-1.0/2.0)-e1*t65*t67*t69*t99+e1*t65*t74*t77*t99*(1.0/2.0);
  _J[6] = t69*t74*t100*(t11*t13*2.0+t18*t19*2.0+t6*t12*t20*2.0)*(1.0/2.0)+e1*t65*t67*t69*t106-e1*t65*t74*t77*t106*(1.0/2.0);
  _J[7] = t69*t74*t100*(t5*t20*2.0+t4*t12*t13*2.0-t3*t12*t19*2.0)*(-1.0/2.0)-e1*t65*t67*t69*t111+e1*t65*t74*t77*t111*(1.0/2.0);
  _J[8] = -e1*t65*t67*t69*t113+e1*t65*t74*t77*t113*(1.0/2.0);
  _J[9] = t69*t74*t100*(t20*t117*2.0-t13*t125*2.0+t19*t121*2.0)*(-1.0/2.0)-e1*t65*t67*t69*t139+e1*t65*t74*t77*t139*(1.0/2.0);
  _J[10] = t69*t74*t100*(t13*t148*2.0+t20*t142*2.0+t19*t145*2.0)*(-1.0/2.0)-e1*t65*t67*t69*(t156+t159)+e1*t65*t74*t77*(t156+t159)*(1.0/2.0);

}

