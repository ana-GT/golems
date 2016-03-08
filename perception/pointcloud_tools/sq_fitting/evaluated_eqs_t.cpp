

#include "perception/pointcloud_tools/sq_fitting/evaluated_eqs_t.h"

/**
 * @brief Radial Distance Error per point in tampered SQ
 */
double Err_r_t( const double &a, const double &b, const double &c,
		const double &e1, const double &e2,
		const double &px, const double &py, const double &pz,
		const double &ra, const double &pa, const double &ya,
		const double &k,
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

  return t0;
}



/**
 * @brief Goodness-of-fit per point
 */
double Err_g_t( const double &a, const double &b, const double &c,
		const double &e1, const double &e2,
		const double &px, const double &py, const double &pz,
		const double &ra, const double &pa, const double &ya,
		const double &k,
		const double &x, const double &y, const double &z ) {

  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33;

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
  t0 = fabs(pow(pow(1.0/(a*a)*t30*(t31*t31),t32)+pow(1.0/(b*b)*(t26*t26)*t30,t32),e2*t33)+pow(1.0/(c*c)*(t13*t13),t33)-1.0);

  return t0;
}



/**
 * @brief Using Duncan for stopping criterion (per point)
 */
double Err_d_t( const double &a, const double &b, const double &c,
		const double &e1, const double &e2,
		const double &px, const double &py, const double &pz,
		const double &ra, const double &pa, const double &ya,
		const double &k,
		const double &x, const double &y, const double &z ) {

  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34;

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
  t34 = pow(pow(pow(1.0/(a*a)*t30*(t31*t31),t32)+pow(1.0/(b*b)*(t26*t26)*t30,t32),e2*t33)+pow(1.0/(c*c)*(t13*t13),t33),e1)-1.0;
  t0 = t34*t34;

  return t0;
}


/**
 * @function fr_add_t
 */
void fr_add_t( double* p, double* x,
	       int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fr_t( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], // k
		 xi, yi, zi );
    
  }
}

/**
 * @function fs_add
 */
void fs_add_t( double* p, double* x,
	       int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fs_t( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], // k
		 xi, yi, zi );
    
  }
}

/**
 * @function fi_add_t
 */
void fi_add_t( double* p, double* x,
	     int m, int n, void* data ) {
  
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fi_t( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], // k
		 xi, yi, zi );
    
  }
}

/**
 * @function fc_add_t
 */
void fc_add_t( double* p, double* x,
	       int m, int n, void* data ) {
  
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fc_t( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], // k
		 xi, yi, zi );
    
  }
}

/**
 * @function f5_add_t
 */
void f5_add_t( double* p, double* x,
	       int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = f5_t( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], // k
		 xi, yi, zi );
    
  }
}

/**
 * @function f6_add_t
 */
void f6_add_t( double* p, double* x,
	       int m, int n, void* data ) {
  
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = f6_t( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], // k
		 xi, yi, zi );
    
  }
}

/**
 * @function Jr_add
 */
void Jr_add_t( double* p, double* jac,
	       int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Jr_t( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], // k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

}

/**
 * @function Js_add_t
 */
void Js_add_t( double* p, double* jac,
	       int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;

  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Js_t( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], // k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

  
}

/**
 * @function Ji_add_t
 */
void Ji_add_t( double* p, double* jac,
	       int m, int n, void* data ) {
  
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Ji_t( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], // k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

}

/**
 * @function Jc_add_t
 */
void Jc_add_t( double* p, double* jac,
	       int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Jc_t( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], // k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

}

/**
 * @function J5_add_t
 */
void J5_add_t( double* p, double* jac,
	       int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    J5_t( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], //k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

}

/**
 * @function J6_add_t
 */
void J6_add_t( double* p, double* jac,
	       int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    J6_t( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], // k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

}




/**
 * @function fr
 */
double fr_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &k, // k
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

  return t0;
}

/**
 * @function Jr_t
 */
void Jr_t( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[12] ) {

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
  double t110, t111;

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
  _J[0] = 1.0/(a*a*a)*t30*t56*pow(t62,e1*(-1.0/2.0)-1.0)*t65*t68*t72*t78;
  _J[1] = 1.0/(b*b*b)*t30*t46*t65*t68*t72*t76*t77;
  _J[2] = t82*t83*(k*t13*t34*t46*t74*2.0+k*t13*t34*t56*t74*2.0)*(1.0/2.0)+e1*t65*t72*t76*(1.0/(c*c*c)*t33*t35*t85*2.0-e2*t33*t68*(k*t13*t32*t34*t38*t46*t74*t77*2.0+k*t13*t32*t34*t49*t56*t74*t78*2.0))*(1.0/2.0);
  _J[3] = -t65*t72*(t63*log(t62)*(1.0/2.0)-e1*t76*(t37*t79*log(t36)+e2*t61*t79*t80)*(1.0/2.0));
  _J[4] = e1*t65*t72*t76*(t33*t61*t80-e2*t33*t68*(t48*t81*log(t47)+t58*t81*log(t57)))*(-1.0/2.0);
  _J[5] = t82*t83*(t9*t13*-2.0-t22*t26*t30*2.0+t2*t12*t30*t31*2.0+k*t9*t27*t46*t74*2.0+k*t9*t27*t56*t74*2.0)*(-1.0/2.0)+e1*t65*t72*t76*(e2*t33*t68*(t32*t78*(t2*t12*t30*t31*t49*2.0+k*t9*t27*t49*t56*t74*2.0)-t32*t77*(t22*t26*t30*t38*2.0-k*t9*t27*t38*t46*t74*2.0))-t9*t13*t33*t34*t85*2.0)*(1.0/2.0);
  _J[6] = t82*t83*(t11*t13*2.0+t25*t26*t30*2.0+t6*t12*t30*t31*2.0-k*t11*t27*t46*t74*2.0-k*t11*t27*t56*t74*2.0)*(-1.0/2.0)+e1*t65*t72*t76*(e2*t33*t68*(t32*t78*(t6*t12*t30*t31*t49*2.0-k*t11*t27*t49*t56*t74*2.0)+t32*t77*(t25*t26*t30*t38*2.0-k*t11*t27*t38*t46*t74*2.0))+t11*t13*t33*t34*t85*2.0)*(1.0/2.0);
  _J[7] = t82*t83*(t4*t12*t13*-2.0-t5*t30*t31*2.0+t3*t12*t26*t30*2.0+k*t4*t12*t27*t46*t74*2.0+k*t4*t12*t27*t56*t74*2.0)*(-1.0/2.0)-e1*t65*t72*t76*(e2*t33*t68*(t32*t78*(t5*t30*t31*t49*2.0-k*t4*t12*t27*t49*t56*t74*2.0)-t32*t77*(t3*t12*t26*t30*t38*2.0+k*t4*t12*t27*t38*t46*t74*2.0))+t4*t12*t13*t33*t34*t85*2.0)*(1.0/2.0);
  _J[8] = t82*t83*(t13*t26*-2.0+t13*t26*t30*2.0+k*t26*t27*t46*t74*2.0+k*t26*t27*t56*t74*2.0)*(-1.0/2.0)+e1*t65*t72*t76*(e2*t33*t68*(t32*t77*(t13*t26*t30*t38*2.0+k*t26*t27*t38*t46*t74*2.0)+k*t26*t27*t32*t49*t56*t74*t78*2.0)-t13*t26*t33*t34*t85*2.0)*(1.0/2.0);
  _J[9] = t82*t83*(t13*t89*-2.0+t26*t30*t96*2.0+t30*t31*t100*2.0+k*t27*t46*t74*t89*2.0+k*t27*t56*t74*t89*2.0)*(1.0/2.0)-e1*t65*t72*t76*(e2*t33*t68*(t32*t77*(t26*t30*t38*t96*2.0+k*t27*t38*t46*t74*t89*2.0)+t32*t78*(t30*t31*t49*t100*2.0+k*t27*t49*t56*t74*t89*2.0))-t13*t33*t34*t85*t89*2.0)*(1.0/2.0);
  _J[10] = t82*t83*(t13*t103*2.0+t26*t30*t108*2.0+t30*t31*t111*2.0-k*t27*t46*t74*t103*2.0-k*t27*t56*t74*t103*2.0)*(1.0/2.0)-e1*t65*t72*t76*(e2*t33*t68*(t32*t77*(t26*t30*t38*t108*2.0-k*t27*t38*t46*t74*t103*2.0)+t32*t78*(t30*t31*t49*t111*2.0-k*t27*t49*t56*t74*t103*2.0))+t13*t33*t34*t85*t103*2.0)*(1.0/2.0);
  _J[11] = t82*t83*(t13*t27*t46*t74*2.0+t13*t27*t56*t74*2.0)*(-1.0/2.0)+e2*t65*t68*t72*t76*(t13*t27*t32*t38*t46*t74*t77*2.0+t13*t27*t32*t49*t56*t74*t78*2.0)*(1.0/2.0);

}


/**
 * @function fs
 */
double fs_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &k,
	     const double &x, const double &y, const double &z ) {

  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33;

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

  return t0;
}

/**
 * @function Js_t
 */
void Js_t( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[12] ) {

  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79;
  double t80, t81, t82, t83, t84, t85, t86, t87, t88, t89;
  double t90, t91, t92;

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
  _J[0] = b*c*t65*t66*(1.0/2.0)-1.0/(a*a*a)*t30*t56*t68*t70*t72*t75*2.0;
  _J[1] = a*c*t65*t66*(1.0/2.0)-1.0/(b*b*b)*t30*t46*t68*t70*t72*t73*2.0;
  _J[2] = a*b*t65*t66*(1.0/2.0)-e1*t68*t72*(1.0/(c*c*c)*t33*t35*t80*2.0-e2*t33*t70*(k*t13*t32*t34*t38*t46*t73*t74*2.0+k*t13*t32*t34*t49*t56*t74*t75*2.0));
  _J[3] = t72*(t64*log(t62)-e1*t68*(t37*t76*log(t36)+e2*t61*t76*t77));
  _J[4] = e1*t68*t72*(t33*t61*t77-e2*t33*t70*(t48*t78*log(t47)+t58*t78*log(t57)));
  _J[5] = -e1*t68*t72*(e2*t33*t70*(t32*t75*(t2*t12*t30*t31*t49*2.0+k*t9*t27*t49*t56*t74*2.0)-t32*t73*(t22*t26*t30*t38*2.0-k*t9*t27*t38*t46*t74*2.0))-t9*t13*t33*t34*t80*2.0);
  _J[6] = -e1*t68*t72*(e2*t33*t70*(t32*t75*(t6*t12*t30*t31*t49*2.0-k*t11*t27*t49*t56*t74*2.0)+t32*t73*(t25*t26*t30*t38*2.0-k*t11*t27*t38*t46*t74*2.0))+t11*t13*t33*t34*t80*2.0);
  _J[7] = e1*t68*t72*(e2*t33*t70*(t32*t75*(t5*t30*t31*t49*2.0-k*t4*t12*t27*t49*t56*t74*2.0)-t32*t73*(t3*t12*t26*t30*t38*2.0+k*t4*t12*t27*t38*t46*t74*2.0))+t4*t12*t13*t33*t34*t80*2.0);
  _J[8] = -e1*t68*t72*(e2*t33*t70*(t32*t73*(t13*t26*t30*t38*2.0+k*t26*t27*t38*t46*t74*2.0)+k*t26*t27*t32*t49*t56*t74*t75*2.0)-t13*t26*t33*t34*t80*2.0);
  _J[9] = e1*t68*t72*(e2*t33*t70*(t32*t73*(t26*t30*t38*(pz*t3*t5-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12+t2*t3*t12*x+t3*t6*t12*y)*2.0+k*t27*t38*t46*t74*t84*2.0)+t32*t75*(t30*t31*t49*(pz*t12-t12*z+px*t2*t5+py*t5*t6-t2*t5*x-t5*t6*y)*2.0+k*t27*t49*t56*t74*t84*2.0))-t13*t33*t34*t80*t84*2.0);
  _J[10] = e1*t68*t72*(e2*t33*t70*(t32*t75*(t30*t31*t49*(px*t6*t12-py*t2*t12-t6*t12*x+t2*t12*y)*2.0-k*t27*t49*t56*t74*t90*2.0)+t32*t73*(t26*t30*t38*(px*t25+py*t22-t25*x-t22*y)*2.0-k*t27*t38*t46*t74*t90*2.0))+t13*t33*t34*t80*t90*2.0);
  _J[11] = -e2*t68*t70*t72*(t13*t27*t32*t38*t46*t73*t74*2.0+t13*t27*t32*t49*t56*t74*t75*2.0);
}



/**
 * @function fi_t
 */
double fi_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &k,
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
  t0 = (pow(pow(1.0/(c*c)*t34,t33)+pow(pow(1.0/(a*a)*t30*t49,t32)+pow(1.0/(b*b)*t30*t42,t32),e2*t33),e1)-1.0)*sqrt(t34+t30*t42+t30*t49)*sqrt(a*b*c);

  return t0;

}


/**
 * @function Ji_t
 */
void Ji_t( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[12] ) {

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
  double t110, t111;


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
  t50 = 1.0/(c*c);
  t51 = t34*t50;
  t52 = pow(t51,t33);
  t53 = 1.0/(b*b);
  t54 = t30*t42*t53;
  t55 = pow(t54,t32);
  t56 = 1.0/(a*a);
  t57 = t30*t49*t56;
  t58 = pow(t57,t32);
  t59 = t55+t58;
  t60 = e2*t33;
  t61 = pow(t59,t60);
  t62 = t52+t61;
  t63 = t30*t42;
  t64 = t30*t49;
  t65 = t34+t63+t64;
  t66 = sqrt(t65);
  t67 = a*b*c;
  t68 = pow(t62,e1);
  t69 = t68-1.0;
  t70 = 1.0/sqrt(t67);
  t71 = e1-1.0;
  t72 = pow(t62,t71);
  t73 = t60-1.0;
  t74 = pow(t59,t73);
  t75 = t32-1.0;
  t76 = sqrt(t67);
  t77 = 1.0/(t29*t29*t29);
  t78 = pow(t54,t75);
  t79 = pow(t57,t75);
  t80 = 1.0/(e1*e1);
  t81 = log(t59);
  t82 = 1.0/(e2*e2);
  t83 = 1.0/sqrt(t65);
  t84 = t33-1.0;
  t85 = pow(t51,t84);
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
  _J[0] = b*c*t66*t69*t70*(1.0/2.0)-1.0/(a*a*a)*t30*t49*t66*t72*t74*t76*t79*2.0;
  _J[1] = a*c*t66*t69*t70*(1.0/2.0)-1.0/(b*b*b)*t30*t42*t66*t72*t74*t76*t78*2.0;
  _J[2] = t69*t76*t83*(k*t13*t42*t50*t77*2.0+k*t13*t49*t50*t77*2.0)*(1.0/2.0)+a*b*t66*t69*t70*(1.0/2.0)-e1*t66*t72*t76*(1.0/(c*c*c)*t33*t34*t85*2.0-e2*t33*t74*(k*t13*t32*t42*t50*t53*t77*t78*2.0+k*t13*t32*t49*t50*t56*t77*t79*2.0));
  _J[3] = t66*t76*(t68*log(t62)-e1*t72*(t52*t80*log(t51)+e2*t61*t80*t81));
  _J[4] = e1*t66*t72*t76*(t33*t61*t81-e2*t33*t74*(t55*t82*log(t54)+t58*t82*log(t57)));
  _J[5] = t69*t76*t83*(t9*t13*-2.0-t22*t26*t30*2.0+t2*t12*t30*t31*2.0+k*t9*t27*t42*t77*2.0+k*t9*t27*t49*t77*2.0)*(-1.0/2.0)-e1*t66*t72*t76*(e2*t33*t74*(t32*t79*(t2*t12*t30*t31*t56*2.0+k*t9*t27*t49*t56*t77*2.0)-t32*t78*(t22*t26*t30*t53*2.0-k*t9*t27*t42*t53*t77*2.0))-t9*t13*t33*t50*t85*2.0);
  _J[6] = t69*t76*t83*(t11*t13*2.0+t25*t26*t30*2.0+t6*t12*t30*t31*2.0-k*t11*t27*t42*t77*2.0-k*t11*t27*t49*t77*2.0)*(-1.0/2.0)-e1*t66*t72*t76*(e2*t33*t74*(t32*t79*(t6*t12*t30*t31*t56*2.0-k*t11*t27*t49*t56*t77*2.0)+t32*t78*(t25*t26*t30*t53*2.0-k*t11*t27*t42*t53*t77*2.0))+t11*t13*t33*t50*t85*2.0);
  _J[7] = t69*t76*t83*(t4*t12*t13*-2.0-t5*t30*t31*2.0+t3*t12*t26*t30*2.0+k*t4*t12*t27*t42*t77*2.0+k*t4*t12*t27*t49*t77*2.0)*(-1.0/2.0)+e1*t66*t72*t76*(e2*t33*t74*(t32*t79*(t5*t30*t31*t56*2.0-k*t4*t12*t27*t49*t56*t77*2.0)-t32*t78*(t3*t12*t26*t30*t53*2.0+k*t4*t12*t27*t42*t53*t77*2.0))+t4*t12*t13*t33*t50*t85*2.0);
  _J[8] = t69*t76*t83*(t13*t26*-2.0+t13*t26*t30*2.0+k*t26*t27*t42*t77*2.0+k*t26*t27*t49*t77*2.0)*(-1.0/2.0)-e1*t66*t72*t76*(e2*t33*t74*(t32*t78*(t13*t26*t30*t53*2.0+k*t26*t27*t42*t53*t77*2.0)+k*t26*t27*t32*t49*t56*t77*t79*2.0)-t13*t26*t33*t50*t85*2.0);
  _J[9] = t69*t76*t83*(t13*t89*-2.0+t26*t30*t96*2.0+t30*t31*t100*2.0+k*t27*t42*t77*t89*2.0+k*t27*t49*t77*t89*2.0)*(1.0/2.0)+e1*t66*t72*t76*(e2*t33*t74*(t32*t78*(t26*t30*t53*t96*2.0+k*t27*t42*t53*t77*t89*2.0)+t32*t79*(t30*t31*t56*t100*2.0+k*t27*t49*t56*t77*t89*2.0))-t13*t33*t50*t85*t89*2.0);
  _J[10] = t69*t76*t83*(t13*t103*2.0+t26*t30*t108*2.0+t30*t31*t111*2.0-k*t27*t42*t77*t103*2.0-k*t27*t49*t77*t103*2.0)*(1.0/2.0)+e1*t66*t72*t76*(e2*t33*t74*(t32*t78*(t26*t30*t53*t108*2.0-k*t27*t42*t53*t77*t103*2.0)+t32*t79*(t30*t31*t56*t111*2.0-k*t27*t49*t56*t77*t103*2.0))+t13*t33*t50*t85*t103*2.0);
  _J[11] = t69*t76*t83*(t13*t27*t42*t77*2.0+t13*t27*t49*t77*2.0)*(-1.0/2.0)-e2*t66*t72*t74*t76*(t13*t27*t32*t42*t53*t77*t78*2.0+t13*t27*t32*t49*t56*t77*t79*2.0);

}


/**
 * @function fc_t
 */
double fc_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &k,
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

  return t0;
}

/**
 * @function Jc_t
 */
void Jc_t( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[12] ) {

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

  _J[0] = -1.0/(a*a*a)*t30*t52*t65*t67*t71*t75;
  _J[1] = -1.0/(b*b*b)*t30*t42*t65*t67*t71*t74;
  _J[2] = t80*t81*(k*t13*t42*t58*t73*2.0+k*t13*t52*t58*t73*2.0)*(1.0/2.0)-e1*t65*t71*(1.0/(c*c*c)*t33*t57*t83*2.0-e2*t33*t67*(k*t13*t32*t34*t42*t58*t73*t74*2.0+k*t13*t32*t45*t52*t58*t73*t75*2.0))*(1.0/2.0);
  _J[3] = t71*(t76*log(t62)*(1.0/2.0)-e1*t65*(t60*t77*log(t59)+e2*t61*t77*t78)*(1.0/2.0));
  _J[4] = e1*t65*t71*(t33*t61*t78-e2*t33*t67*(t44*t79*log(t43)+t54*t79*log(t53)))*(1.0/2.0);
  _J[5] = t80*t81*(t9*t13*-2.0-t22*t26*t30*2.0+t2*t12*t30*t31*2.0+k*t9*t27*t42*t73*2.0+k*t9*t27*t52*t73*2.0)*(-1.0/2.0)-e1*t65*t71*(e2*t33*t67*(t32*t75*(t2*t12*t30*t31*t45*2.0+k*t9*t27*t45*t52*t73*2.0)-t32*t74*(t22*t26*t30*t34*2.0-k*t9*t27*t34*t42*t73*2.0))-t9*t13*t33*t58*t83*2.0)*(1.0/2.0);
  _J[6] = t80*t81*(t11*t13*2.0+t25*t26*t30*2.0+t6*t12*t30*t31*2.0-k*t11*t27*t42*t73*2.0-k*t11*t27*t52*t73*2.0)*(-1.0/2.0)-e1*t65*t71*(e2*t33*t67*(t32*t75*(t6*t12*t30*t31*t45*2.0-k*t11*t27*t45*t52*t73*2.0)+t32*t74*(t25*t26*t30*t34*2.0-k*t11*t27*t34*t42*t73*2.0))+t11*t13*t33*t58*t83*2.0)*(1.0/2.0);
  _J[7] = t80*t81*(t4*t12*t13*-2.0-t5*t30*t31*2.0+t3*t12*t26*t30*2.0+k*t4*t12*t27*t42*t73*2.0+k*t4*t12*t27*t52*t73*2.0)*(-1.0/2.0)+e1*t65*t71*(e2*t33*t67*(t32*t75*(t5*t30*t31*t45*2.0-k*t4*t12*t27*t45*t52*t73*2.0)-t32*t74*(t3*t12*t26*t30*t34*2.0+k*t4*t12*t27*t34*t42*t73*2.0))+t4*t12*t13*t33*t58*t83*2.0)*(1.0/2.0);
  _J[8] = t80*t81*(t13*t26*-2.0+t13*t26*t30*2.0+k*t26*t27*t42*t73*2.0+k*t26*t27*t52*t73*2.0)*(-1.0/2.0)-e1*t65*t71*(e2*t33*t67*(t32*t74*(t13*t26*t30*t34*2.0+k*t26*t27*t34*t42*t73*2.0)+k*t26*t27*t32*t45*t52*t73*t75*2.0)-t13*t26*t33*t58*t83*2.0)*(1.0/2.0);
  _J[9] = t80*t81*(t13*t87*-2.0+t26*t30*t94*2.0+t30*t31*t98*2.0+k*t27*t42*t73*t87*2.0+k*t27*t52*t73*t87*2.0)*(1.0/2.0)+e1*t65*t71*(e2*t33*t67*(t32*t74*(t26*t30*t34*t94*2.0+k*t27*t34*t42*t73*t87*2.0)+t32*t75*(t30*t31*t45*t98*2.0+k*t27*t45*t52*t73*t87*2.0))-t13*t33*t58*t83*t87*2.0)*(1.0/2.0);
  _J[10] = t80*t81*(t13*t101*2.0+t26*t30*t106*2.0+t30*t31*t109*2.0-k*t27*t42*t73*t101*2.0-k*t27*t52*t73*t101*2.0)*(1.0/2.0)+e1*t65*t71*(e2*t33*t67*(t32*t74*(t26*t30*t34*t106*2.0-k*t27*t34*t42*t73*t101*2.0)+t32*t75*(t30*t31*t45*t109*2.0-k*t27*t45*t52*t73*t101*2.0))+t13*t33*t58*t83*t101*2.0)*(1.0/2.0);
  _J[11] = t80*t81*(t13*t27*t42*t73*2.0+t13*t27*t52*t73*2.0)*(-1.0/2.0)-e2*t65*t67*t71*(t13*t27*t32*t34*t42*t73*t74*2.0+t13*t27*t32*t45*t52*t73*t75*2.0)*(1.0/2.0);
}



/**
 * @function f5_t
 */
double f5_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &k,
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

  return t0;
}


/**
 * @function J5_t
 */
void J5_t( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[12] ) {

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
  double t100, t101, t102, t103, t104, t105, t106, t107, t108;


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

  _J[0] = 1.0/(a*a*a)*t30*t52*t64*t66*t70*t74*-2.0;
  _J[1] = 1.0/(b*b*b)*t30*t42*t64*t66*t70*t73*-2.0;
  _J[2] = t79*t80*(k*t13*t42*t58*t72*2.0+k*t13*t52*t58*t72*2.0)*(1.0/2.0)-e1*t64*t70*(1.0/(c*c*c)*t33*t57*t82*2.0-e2*t33*t66*(k*t13*t32*t34*t42*t58*t72*t73*2.0+k*t13*t32*t45*t52*t58*t72*t74*2.0));
  _J[3] = t70*(t75*log(t62)-e1*t64*(t60*t76*log(t59)+e2*t61*t76*t77));
  _J[4] = e1*t64*t70*(t33*t61*t77-e2*t33*t66*(t44*t78*log(t43)+t54*t78*log(t53)));
  _J[5] = t79*t80*(t9*t13*-2.0-t22*t26*t30*2.0+t2*t12*t30*t31*2.0+k*t9*t27*t42*t72*2.0+k*t9*t27*t52*t72*2.0)*(-1.0/2.0)-e1*t64*t70*(e2*t33*t66*(t32*t74*(t2*t12*t30*t31*t45*2.0+k*t9*t27*t45*t52*t72*2.0)-t32*t73*(t22*t26*t30*t34*2.0-k*t9*t27*t34*t42*t72*2.0))-t9*t13*t33*t58*t82*2.0);
  _J[6] = t79*t80*(t11*t13*2.0+t25*t26*t30*2.0+t6*t12*t30*t31*2.0-k*t11*t27*t42*t72*2.0-k*t11*t27*t52*t72*2.0)*(-1.0/2.0)-e1*t64*t70*(e2*t33*t66*(t32*t74*(t6*t12*t30*t31*t45*2.0-k*t11*t27*t45*t52*t72*2.0)+t32*t73*(t25*t26*t30*t34*2.0-k*t11*t27*t34*t42*t72*2.0))+t11*t13*t33*t58*t82*2.0);
  _J[7] = t79*t80*(t4*t12*t13*-2.0-t5*t30*t31*2.0+t3*t12*t26*t30*2.0+k*t4*t12*t27*t42*t72*2.0+k*t4*t12*t27*t52*t72*2.0)*(-1.0/2.0)+e1*t64*t70*(e2*t33*t66*(t32*t74*(t5*t30*t31*t45*2.0-k*t4*t12*t27*t45*t52*t72*2.0)-t32*t73*(t3*t12*t26*t30*t34*2.0+k*t4*t12*t27*t34*t42*t72*2.0))+t4*t12*t13*t33*t58*t82*2.0);
  _J[8] = t79*t80*(t13*t26*-2.0+t13*t26*t30*2.0+k*t26*t27*t42*t72*2.0+k*t26*t27*t52*t72*2.0)*(-1.0/2.0)-e1*t64*t70*(e2*t33*t66*(t32*t73*(t13*t26*t30*t34*2.0+k*t26*t27*t34*t42*t72*2.0)+k*t26*t27*t32*t45*t52*t72*t74*2.0)-t13*t26*t33*t58*t82*2.0);
  _J[9] = t79*t80*(t13*t86*-2.0+t26*t30*t93*2.0+t30*t31*t97*2.0+k*t27*t42*t72*t86*2.0+k*t27*t52*t72*t86*2.0)*(1.0/2.0)+e1*t64*t70*(e2*t33*t66*(t32*t73*(t26*t30*t34*t93*2.0+k*t27*t34*t42*t72*t86*2.0)+t32*t74*(t30*t31*t45*t97*2.0+k*t27*t45*t52*t72*t86*2.0))-t13*t33*t58*t82*t86*2.0);
  _J[10] = t79*t80*(t13*t100*2.0+t26*t30*t105*2.0+t30*t31*t108*2.0-k*t27*t42*t72*t100*2.0-k*t27*t52*t72*t100*2.0)*(1.0/2.0)+e1*t64*t70*(e2*t33*t66*(t32*t73*(t26*t30*t34*t105*2.0-k*t27*t34*t42*t72*t100*2.0)+t32*t74*(t30*t31*t45*t108*2.0-k*t27*t45*t52*t72*t100*2.0))+t13*t33*t58*t82*t100*2.0);
  _J[11] = t79*t80*(t13*t27*t42*t72*2.0+t13*t27*t52*t72*2.0)*(-1.0/2.0)-e2*t64*t66*t70*(t13*t27*t32*t34*t42*t72*t73*2.0+t13*t27*t32*t45*t52*t72*t74*2.0);

}


/**
 * @function f6_t
 */
double f6_t( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &k,
	     const double &x, const double &y, const double &z ) {
  
  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62;

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
  t0 = -(pow(t62,e1)-1.0)*(pow(t62,e1*(-1.0/2.0))-1.0)*sqrt(t35+t30*t46+t30*t56);

  return t0;

}


/**
 * @function J6_t
 */
void J6_t( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[12] ) {

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
  double t160, t161, t162, t163, t164, t165, t166, t167, t168, t169;
  double t170, t171, t172, t173, t174, t175, t176, t177, t178, t179;
  double t180, t181, t182, t183, t184, t185, t186, t187, t188, t189;
  double t190, t191, t192, t193, t194, t195, t196, t197, t198, t199;
  double t200, t201, t202, t203;

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
  t57 = 1.0/(c*c);
  t58 = t13*t13;
  t59 = t57*t58;
  t60 = pow(t59,t33);
  t61 = pow(t55,t56);
  t62 = t60+t61;
  t63 = 1.0/(a*a*a);
  t64 = t56-1.0;
  t65 = pow(t55,t64);
  t66 = t30*t42;
  t67 = t30*t52;
  t68 = t58+t66+t67;
  t69 = sqrt(t68);
  t70 = t32-1.0;
  t71 = pow(t53,t70);
  t72 = e1-1.0;
  t73 = pow(t62,t72);
  t77 = e1*(1.0/2.0);
  t74 = pow(t62,-t77);
  t75 = t74-1.0;
  t76 = 1.0/(b*b*b);
  t78 = pow(t62,e1);
  t79 = t78-1.0;
  t80 = pow(t43,t70);
  t81 = 1.0/(t29*t29*t29);
  t82 = -t77-1.0;
  t83 = pow(t62,t82);
  t84 = k*t13*t32*t34*t42*t57*t80*t81*2.0;
  t85 = k*t13*t32*t45*t52*t57*t71*t81*2.0;
  t86 = t84+t85;
  t87 = e2*t33*t65*t86;
  t88 = 1.0/(c*c*c);
  t89 = t33-1.0;
  t90 = pow(t59,t89);
  t91 = t87-t33*t58*t88*t90*2.0;
  t92 = 1.0/(e1*e1);
  t93 = log(t62);
  t94 = log(t59);
  t95 = t60*t92*t94;
  t96 = log(t55);
  t97 = e2*t61*t92*t96;
  t98 = t95+t97;
  t99 = 1.0/(e2*e2);
  t100 = t33*t61*t96;
  t101 = log(t43);
  t102 = t44*t99*t101;
  t103 = log(t53);
  t104 = t54*t99*t103;
  t105 = t102+t104;
  t106 = t100-e2*t33*t65*t105;
  t107 = 1.0/sqrt(t68);
  t108 = t22*t26*t30*t34*2.0;
  t109 = t108-k*t9*t27*t34*t42*t81*2.0;
  t110 = t2*t12*t30*t31*t45*2.0;
  t111 = k*t9*t27*t45*t52*t81*2.0;
  t112 = t110+t111;
  t113 = t32*t71*t112;
  t114 = t113-t32*t80*t109;
  t115 = e2*t33*t65*t114;
  t116 = t115-t9*t13*t33*t57*t90*2.0;
  t117 = t25*t26*t30*t34*2.0;
  t118 = t117-k*t11*t27*t34*t42*t81*2.0;
  t119 = t32*t80*t118;
  t120 = t6*t12*t30*t31*t45*2.0;
  t121 = t120-k*t11*t27*t45*t52*t81*2.0;
  t122 = t32*t71*t121;
  t123 = t119+t122;
  t124 = e2*t33*t65*t123;
  t125 = t11*t13*t33*t57*t90*2.0;
  t126 = t124+t125;
  t127 = t5*t30*t31*t45*2.0;
  t128 = t127-k*t4*t12*t27*t45*t52*t81*2.0;
  t129 = t32*t71*t128;
  t130 = t3*t12*t26*t30*t34*2.0;
  t131 = k*t4*t12*t27*t34*t42*t81*2.0;
  t132 = t130+t131;
  t133 = t129-t32*t80*t132;
  t134 = e2*t33*t65*t133;
  t135 = t4*t12*t13*t33*t57*t90*2.0;
  t136 = t134+t135;
  t137 = t13*t26*t30*t34*2.0;
  t138 = k*t26*t27*t34*t42*t81*2.0;
  t139 = t137+t138;
  t140 = t32*t80*t139;
  t141 = k*t26*t27*t32*t45*t52*t71*t81*2.0;
  t142 = t140+t141;
  t143 = e2*t33*t65*t142;
  t144 = t143-t13*t26*t33*t57*t90*2.0;
  t145 = pz*t4*t5;
  t146 = t2*t4*t12*x;
  t147 = t4*t6*t12*y;
  t149 = t4*t5*z;
  t150 = px*t2*t4*t12;
  t151 = py*t4*t6*t12;
  t148 = t145+t146+t147-t149-t150-t151;
  t152 = pz*t3*t5;
  t153 = t2*t3*t12*x;
  t154 = t3*t6*t12*y;
  t160 = t3*t5*z;
  t161 = px*t2*t3*t12;
  t162 = py*t3*t6*t12;
  t155 = t152+t153+t154-t160-t161-t162;
  t156 = pz*t12;
  t157 = px*t2*t5;
  t158 = py*t5*t6;
  t167 = t12*z;
  t168 = t2*t5*x;
  t169 = t5*t6*y;
  t159 = t156+t157+t158-t167-t168-t169;
  t163 = t26*t30*t34*t155*2.0;
  t164 = k*t27*t34*t42*t81*t148*2.0;
  t165 = t163+t164;
  t166 = t32*t80*t165;
  t170 = t30*t31*t45*t159*2.0;
  t171 = k*t27*t45*t52*t81*t148*2.0;
  t172 = t170+t171;
  t173 = t32*t71*t172;
  t174 = t166+t173;
  t175 = e2*t33*t65*t174;
  t176 = t175-t13*t33*t57*t90*t148*2.0;
  t177 = px*t11;
  t178 = py*t9;
  t180 = t11*x;
  t181 = t9*y;
  t179 = t177+t178-t180-t181;
  t182 = px*t25;
  t183 = py*t22;
  t188 = t25*x;
  t189 = t22*y;
  t184 = t182+t183-t188-t189;
  t185 = t2*t12*y;
  t186 = px*t6*t12;
  t193 = py*t2*t12;
  t194 = t6*t12*x;
  t187 = t185+t186-t193-t194;
  t190 = t26*t30*t34*t184*2.0;
  t191 = t190-k*t27*t34*t42*t81*t179*2.0;
  t192 = t32*t80*t191;
  t195 = t30*t31*t45*t187*2.0;
  t196 = t32*t71*(t195-k*t27*t45*t52*t81*t179*2.0);
  t197 = t192+t196;
  t198 = e2*t33*t65*t197;
  t199 = t13*t33*t57*t90*t179*2.0;
  t200 = t198+t199;
  t201 = t13*t27*t32*t34*t42*t80*t81*2.0;
  t202 = t13*t27*t32*t45*t52*t71*t81*2.0;
  t203 = t201+t202;
  _J[0] = -t30*t52*pow(t62,e1*(-1.0/2.0)-1.0)*t63*t65*t69*t71*t79+t30*t52*t63*t65*t69*t71*t73*t75*2.0;
  _J[1] = t30*t42*t65*t69*t73*t75*t76*t80*2.0-t30*t42*t65*t69*t76*t79*t80*t83;
  _J[2] = t75*t79*t107*(k*t13*t42*t57*t81*2.0+k*t13*t52*t57*t81*2.0)*(-1.0/2.0)-e1*t69*t73*t75*t91+e1*t69*t79*t83*t91*(1.0/2.0);
  _J[3] = -t69*t75*(t78*t93-e1*t73*t98)+t69*t79*(t74*t93*(1.0/2.0)-e1*t83*t98*(1.0/2.0));
  _J[4] = -e1*t69*t73*t75*t106+e1*t69*t79*t83*t106*(1.0/2.0);
  _J[5] = t75*t79*t107*(t9*t13*-2.0-t22*t26*t30*2.0+t2*t12*t30*t31*2.0+k*t9*t27*t42*t81*2.0+k*t9*t27*t52*t81*2.0)*(1.0/2.0)+e1*t69*t73*t75*t116-e1*t69*t79*t83*t116*(1.0/2.0);
  _J[6] = t75*t79*t107*(t11*t13*2.0+t25*t26*t30*2.0+t6*t12*t30*t31*2.0-k*t11*t27*t42*t81*2.0-k*t11*t27*t52*t81*2.0)*(1.0/2.0)+e1*t69*t73*t75*t126-e1*t69*t79*t83*t126*(1.0/2.0);
  _J[7] = t75*t79*t107*(t4*t12*t13*-2.0-t5*t30*t31*2.0+t3*t12*t26*t30*2.0+k*t4*t12*t27*t42*t81*2.0+k*t4*t12*t27*t52*t81*2.0)*(1.0/2.0)-e1*t69*t73*t75*t136+e1*t69*t79*t83*t136*(1.0/2.0);
  _J[8] = t75*t79*t107*(t13*t26*-2.0+t13*t26*t30*2.0+k*t26*t27*t42*t81*2.0+k*t26*t27*t52*t81*2.0)*(1.0/2.0)+e1*t69*t73*t75*t144-e1*t69*t79*t83*t144*(1.0/2.0);
  _J[9] = t75*t79*t107*(t13*t148*-2.0+t26*t30*t155*2.0+t30*t31*t159*2.0+k*t27*t42*t81*t148*2.0+k*t27*t52*t81*t148*2.0)*(-1.0/2.0)-e1*t69*t73*t75*t176+e1*t69*t79*t83*t176*(1.0/2.0);
  _J[10] = t75*t79*t107*(t13*t179*2.0+t26*t30*t184*2.0+t30*t31*t187*2.0-k*t27*t42*t81*t179*2.0-k*t27*t52*t81*t179*2.0)*(-1.0/2.0)-e1*t69*t73*t75*t200+e1*t69*t79*t83*t200*(1.0/2.0);
  _J[11] = t75*t79*t107*(t13*t27*t42*t81*2.0+t13*t27*t52*t81*2.0)*(1.0/2.0)+e2*t65*t69*t73*t75*t203-e2*t65*t69*t79*t83*t203*(1.0/2.0);
}
