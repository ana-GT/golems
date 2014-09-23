/**
 * @function levmar_eqs.cpp
 */
#include "levmar_eqs.h"

/**
 * @brief Function to minimize (error between projected points)
 */
void levmar_fx( double *p, 
		double* x, 
		int m, 
		int n, 
		void *data ) {

  double xr, yr, zr, xk, yk, zk;
  double t2, t3, t4, t5, t6, t7, t8, t9, t10;

  struct levmar_data *dptr;
  dptr = (struct levmar_data*) data;

  for( int i = 0; i < n; ++i ) {
    
    xr = dptr->xr[i];
    yr = dptr->yr[i];
    zr = dptr->zr[i];
    
    xk = dptr->xk[i];
    yk = dptr->yk[i];
    zk = dptr->zk[i];
    
    // ra = p[0] pa = p[1] ya = p[2]
    // tx = p[3] ty = p[4] pz = p[5]
    t2 = sin(p[0]);
    t3 = sin(p[2]);
    t4 = cos(p[0]);
    t5 = cos(p[2]);
    t6 = sin(p[1]);
    t8 = cos(p[1]);
    t7 = p[3]-xr-yk*(t3*t4-t2*t5*t6)+zk*(t2*t3+t4*t5*t6)+t5*t8*xk;
    t9 = p[4]-yr+yk*(t4*t5+t2*t3*t6)-zk*(t2*t5-t3*t4*t6)+t3*t8*xk;
    t10 = p[5]-zr-t6*xk+t2*t8*yk+t4*t8*zk;
    
    x[i] = t7*t7+t9*t9+t10*t10;
  }
}

/**
 * @function levmar_jac
 */
void levmar_jac( double* p, double* jac,
		 int m, int n, void* data ) {

  double xr, yr, zr, xk, yk, zk;
  double t12, t13, t14, t15, t16, t17, t18, t19, t20;
  double t21, t22, t23, t24, t25, t26, t27, t28, t29, t30;
  double t31, t32, t33, t34, t35, t36, t37, t38, t39, t40;

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;


  // ra = p[0] pa = p[1] ya = p[2]
  // tx = p[3] ty = p[4] pz = p[5] 
  for( int i = 0; i < n; ++i ) {

    xr = dptr->xr[i];
    yr = dptr->yr[i];
    zr = dptr->zr[i];
    
    xk = dptr->xk[i];
    yk = dptr->yk[i];
    zk = dptr->zk[i];


    t12 = cos(p[1]);
    t13 = cos(p[0]);
    t14 = sin(p[0]);
    t15 = sin(p[1]);
    t16 = sin(p[2]);
    t17 = cos(p[2]);
    t18 = t13*t16;
    t28 = t14*t15*t17;
    t19 = t18-t28;
    t20 = t14*t16;
    t21 = t13*t15*t17;
    t22 = t20+t21;
    t23 = t13*t17;
    t24 = t14*t15*t16;
    t25 = t23+t24;
    t26 = t14*t17;
    t36 = t13*t15*t16;
    t27 = t26-t36;
    t29 = t22*zk;
    t30 = t12*t17*xk;
    t40 = t19*yk;
    t31 = t29+t30-t40+p[3]-xr;
    t32 = t12*t13*zk;
    t33 = t12*t14*yk;
    t34 = t32+t33+p[5]-zr-t15*xk;
    t35 = t25*yk;
    t37 = t12*t16*xk;
    t39 = t27*zk;
    t38 = t35+t37-t39+p[4]-yr;
    
    jac[6*i+0] = t34*(t12*t13*yk-t12*t14*zk)*2.0+t31*(t22*yk+t19*zk)*2.0-t38*(t27*yk+t25*zk)*2.0;
    jac[6*i+1] = t34*(t12*xk+t14*t15*yk+t13*t15*zk)*-2.0+t31*(-t15*t17*xk+t12*t14*t17*yk+t12*t13*t17*zk)*2.0+t38*(-t15*t16*xk+t12*t14*t16*yk+t12*t13*t16*zk)*2.0;
    jac[6*i+2] = t38*(t29+t30-t40)*2.0-t31*(t35+t37-t39)*2.0;
    jac[6*i+3] = p[3]*2.0-xr*2.0-t19*yk*2.0+t22*zk*2.0+t12*t17*xk*2.0;
    jac[6*i+4] = p[4]*2.0-yr*2.0+t25*yk*2.0-t27*zk*2.0+t12*t16*xk*2.0;
    jac[6*i+5] = p[5]*2.0-zr*2.0-t15*xk*2.0+t12*t14*yk*2.0+t12*t13*zk*2.0;
  } // end for

}
