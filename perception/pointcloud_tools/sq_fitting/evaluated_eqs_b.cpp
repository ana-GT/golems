/**
 * @file evaluated_eqs_b.cpp 
 * @brief Calculate with simpler bending (1 parameter, no alpha)
 */

#include "perception/pointcloud_tools/sq_fitting/evaluated_eqs_b.h"


/**
 * @brief Radial Distance Error per point in bent SQ
 */
double Err_r_b( const double &a, const double &b, const double &c,
		const double &e1, const double &e2,
		const double &px, const double &py, const double &pz,
		const double &ra, const double &pa, const double &ya,
		const double &alpha, const double &k,
		const double &x, const double &y, const double &z ) {

  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60;


  return t0;
}



/**
 * @brief Goodness-of-fit per point
 */
double Err_g_b( const double &a, const double &b, const double &c,
		const double &e1, const double &e2,
		const double &px, const double &py, const double &pz,
		const double &ra, const double &pa, const double &ya,
		const double &alpha, const double &k,
		const double &x, const double &y, const double &z ) {


  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32;


  return t0;
}



/**
 * @brief Using Duncan for stopping criterion (per point)
 */
double Err_d_b( const double &a, const double &b, const double &c,
		const double &e1, const double &e2,
		const double &px, const double &py, const double &pz,
		const double &ra, const double &pa, const double &ya,
		const double &alpha, const double &k,
		const double &x, const double &y, const double &z ) {

  double t1;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62, t63, t64, t65;


  t2 = cos(ra);
  t3 = cos(ya);
  t4 = sin(pa);
  t5 = sin(ra);
  t6 = sin(ya);
  t7 = t2*t6;
  t13 = t3*t4*t5;
  t8 = t7-t13;
  t9 = t2*t3;
  t10 = t4*t5*t6;
  t11 = t9+t10;
  t12 = cos(pa);
  t35 = px*t8;
  t36 = py*t11;
  t37 = t8*x;
  t38 = t11*y;
  t39 = pz*t5*t12;
  t40 = t5*t12*z;
  t14 = t35-t36-t37+t38-t39+t40;
  t15 = pz*t4;
  t16 = t3*t12*x;
  t17 = t6*t12*y;
  t30 = t4*z;
  t31 = px*t3*t12;
  t33 = py*t6*t12;
  t18 = t15+t16+t17-t30-t31-t33;
  
  // Imaginary
  t26 = px*t8;
  t27 = -py*t11;
  t28 = -t8*x;
  t29 = t11*y;
  t32 = -pz*t5*t12;
  t34 = t5*t12*z;
  // End imaginary

  t41 = t14*t14;
  t42 = t18*t18;
  t43 = t41+t42;
  t44 = sqrt(t43);
  t45 = 1.0/k;

  t19 = -t45+t44*cos(alpha-atan2(t26+t27+t28+t29+t32+t34, t15+t16+t17-t4*z-px*t3*t12-py*t6*t12) );
  t20 = t5*t6;
  t21 = t2*t3*t4;
  t22 = t20+t21;
  t23 = t3*t5;
  t54 = t2*t4*t6;
  t24 = t23-t54;
  t53 = px*t22;
  t55 = py*t24;
  t56 = t22*x;
  t57 = t24*y;
  t58 = pz*t2*t12;
  t59 = t2*t12*z;
  t25 = t53-t55-t56+t57+t58-t59;
  t47 = t15+t16+t17-t30-t31-t33;
  t48 = atan2(t26+t27+t28+t29+t32+t34, t47);
  t49 = alpha-t48;
  t50 = cos(t49);
  t51 = t44*t50;
  t60 = t25*t25;
  t46 = -t35+t36+t37-t38+t39-t40+sin(alpha)*(-t45+t51+sqrt(t60+t19*t19));
  t52 = t45-t51;
  t61 = -t15-t16-t17+t30+t31+t33+cos(alpha)*(-t45+t51+sqrt(t60+t52*t52));
  t62 = 1.0/e2;

  t63 = atan2(py*t24 - px*t22 + t22*x - t24*y - pz*t2*t12 + t2*t12*z, t45 - t51);

  t64 = 1.0/e1;
  t65 = pow(pow(pow(1.0/(a*a)*(t61*t61),t62)+pow(1.0/(b*b)*(t46*t46),t62),e2*t64)+pow(1.0/(c*c)*1.0/(k*k)*(t63*t63),t64),e1)-1.0;
  t1 = t65*t65;


  return t1;
}


/**
 * @function fr_add_b
 */
void fr_add_b( double* p, double* x,
	       int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fr_b( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], p[12], // alpha, k
		 xi, yi, zi );
    
  }
}

/**
 * @function fs_add
 */
void fs_add_b( double* p, double* x,
	       int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fs_b( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], p[12], // alpha, k
		 xi, yi, zi );
    
  }
}

/**
 * @function fi_add_b
 */
void fi_add_b( double* p, double* x,
	     int m, int n, void* data ) {
  
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fi_b( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], p[12], // alpha, k
		 xi, yi, zi );
    
  }
}

/**
 * @function fc_add_b
 */
void fc_add_b( double* p, double* x,
	       int m, int n, void* data ) {
  
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = fc_b( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], p[12], // alpha, k
		 xi, yi, zi );
    
  }
}

/**
 * @function f5_add_b
 */
void f5_add_b( double* p, double* x,
	       int m, int n, void* data ) {
    
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = f5_b( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], p[12], // alpha, k
		 xi, yi, zi );
    
  }
}

/**
 * @function f6_add_b
 */
void f6_add_b( double* p, double* x,
	       int m, int n, void* data ) {
  
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
    

  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];

    x[i] = f6_b( p[0], p[1], p[2],
		 p[3], p[4],
		 p[5], p[6], p[7],
		 p[8], p[9], p[10],
		 p[11], p[12], // alpha, k
		 xi, yi, zi );
    
  }
}

/**
 * @function Jr_add
 */
void Jr_add_b( double* p, double* jac,
	       int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Jr_b( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], p[12], // k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

}

/**
 * @function Js_add_b
 */
void Js_add_b( double* p, double* jac,
	       int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;

  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Js_b( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], p[12], // alpha, k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

  
}

/**
 * @function Ji_add_b
 */
void Ji_add_b( double* p, double* jac,
	       int m, int n, void* data ) {
  
  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Ji_b( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], p[12],// alpha, k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

}

/**
 * @function Jc_add_b
 */
void Jc_add_b( double* p, double* jac,
	       int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    Jc_b( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], p[12], // alpha, k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

}

/**
 * @function J5_add_b
 */
void J5_add_b( double* p, double* jac,
	       int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    J5_b( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], p[12], // alpha, k
	  xi, yi, zi,
	  J );

    for( int k = 0; k < m; k++ ) {
      jac[m*i+k] = J[k];
    }
  }

}

/**
 * @function J6_add_b
 */
void J6_add_b( double* p, double* jac,
	       int m, int n, void* data ) {

  struct levmar_data* dptr;
  dptr = (struct levmar_data*) data;
  double xi, yi, zi;
  double J[m];
  
  for( int i = 0; i < n; ++i ) {

    xi = dptr->x[i];
    yi = dptr->y[i];
    zi = dptr->z[i];
    
    J6_b( p[0], p[1], p[2],
	  p[3], p[4],
	  p[5], p[6], p[7],
	  p[8], p[9], p[10],
	  p[11], p[12], // alpha, k
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
double fr_b( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &alpha, const double &k, // alpha, k
	     const double &x, const double &y, const double &z ) {

  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79;
  double t80, t81, t82, t83, t84, t85, t86, t87, t88, t89;
 

  return t0;
}


/**
 * @function Jr_b
 */
void Jr_b( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &alpha, const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[13] ) {

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
  double t200, t201, t202, t203, t204, t205, t206, t207, t208, t209;
  double t210, t211, t212, t213, t214, t215, t216, t217, t218, t219;
  double t220, t221, t222, t223, t224, t225, t226, t227, t228, t229;
  double t230, t231, t232, t233, t234, t235, t236, t237, t238, t239;
  double t240, t241, t242, t243, t244, t245, t246, t247, t248, t249;
  double t250, t251, t252, t253, t254, t255, t256, t257, t258, t259;
  double t260, t261, t262, t263, t264, t265, t266, t267, t268, t269;
  double t270, t271, t272, t273, t274, t275, t276, t277, t278, t279;
  double t280, t281, t282, t283, t284, t285, t286, t287, t288, t289;
  double t290, t291, t292, t293, t294, t295, t296, t297, t298, t299;


  


}

/**
 * @function fs
 */
double fs_b( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &alpha, const double &k,
	     const double &x, const double &y, const double &z ) {

  double t0;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32;



  return t0;
}


/**
 * @function Js_b
 */
void Js_b( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &alpha, const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[13] ) {

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
  double t130, t131, t132, t133, t134, t135, t136, t137;



}




/**
 * @function fi_t
 */
double fi_b( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &alpha, const double &k,
	     const double &x, const double &y, const double &z ) {

  double t1;
  double t2, t3, t4, t5, t6, t7, t8, t9;
  double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19;
  double t20, t21, t22, t23, t24, t25, t26, t27, t28, t29;
  double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
  double t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
  double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59;
  double t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
  double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79;

  t2 = cos(ra);
  t3 = cos(ya);
  t4 = sin(pa);
  t5 = sin(ra);
  t6 = sin(ya);
  t7 = t2*t6;
  t13 = t3*t4*t5;
  t8 = t7-t13;
  t9 = t2*t3;
  t10 = t4*t5*t6;
  t11 = t9+t10;
  t12 = cos(pa);
  t35 = px*t8;
  t36 = py*t11;
  t37 = t8*x;
  t38 = t11*y;
  t39 = pz*t5*t12;
  t40 = t5*t12*z;
  t14 = t35-t36-t37+t38-t39+t40;
  t15 = pz*t4;
  t16 = t3*t12*x;
  t17 = t6*t12*y;
  t30 = t4*z;
  t31 = px*t3*t12;
  t33 = py*t6*t12;
  t18 = t15+t16+t17-t30-t31-t33;

  // Imaginary
  t26 = px*t8;
  t27 = -py*t11;
  t28 = -t8*x;
  t29 = t11*y;
  t32 = -pz*t5*t12;
  t34 = t5*t12*z;
  // End imaginary

  t41 = t14*t14;
  t42 = t18*t18;
  t43 = t41+t42;
  t44 = sqrt(t43);
  t45 = 1.0/k;

  t19 = t44*cos(alpha - atan2(t26 + t27 + t28 + t29 + t32 + t34, t15 + t16 + t17 - t4*z - px*t3*t12 - py*t6*t12)) - t45;

  t20 = t5*t6;
  t21 = t2*t3*t4;
  t22 = t20+t21;
  t23 = t3*t5;
  t54 = t2*t4*t6;
  t24 = t23-t54;
  t53 = px*t22;
  t55 = py*t24;
  t56 = t22*x;
  t57 = t24*y;
  t58 = pz*t2*t12;
  t59 = t2*t12*z;
  t25 = t53-t55-t56+t57+t58-t59;
  t47 = t15+t16+t17 -t30-t31 -t33;
  t48 = atan2(t26+t27+t28+t29+t32+t34, t47);

  t49 = alpha-t48;
  t50 = cos(t49);
  t51 = t44*t50;
  t60 = t25*t25;
  t65 = sin(alpha);
  t46 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t19*t19));
  t52 = t45-t51;
  t68 = cos(alpha);
  t61 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t52*t52));
  t62 = 1.0/e2;

  // Imaginary
  t72 = -px*t22;
  t73 = py*t24;
  t74 = t22*x;
  t75 = -t24*y;
  t76 = -pz*t2*t12;
  t77 = t2*t12*z;
  // End imaginary

  t78 = t45-t51;
  t63 = atan2(t72+t73+t74+t75+t76+t77, t78);
  t64 = 1.0/e1;
  t66 = t45-t51;
  t67 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t66*t66));
  t69 = t45-t51;
  t70 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t69*t69));
  t71 = 1.0/(k*k);
  t79 = t63*t63;
  t1 = (pow(pow(pow(1.0/(a*a)*(t61*t61),t62)+pow(1.0/(b*b)*(t46*t46),t62),e2*t64)+pow(1.0/(c*c)*t71*t79,t64),e1)-1.0)*sqrt(t71*t79+t67*t67+t70*t70)*sqrt(a*b*c);


  return t1;

}


/**
 * @function Ji_t
 */
void Ji_b( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &alpha, const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[13] ) {

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
  double t200, t201, t202, t203, t204, t205, t206, t207, t208, t209;
  double t210, t211, t212, t213, t214, t215, t216, t217, t218, t219;
  double t220, t221, t222, t223, t224, t225, t226, t227, t228, t229;
  double t230, t231, t232, t233, t234, t235, t236, t237, t238, t239;
  double t240, t241, t242, t243, t244, t245, t246, t247, t248, t249;
  double t250, t251, t252, t253, t254, t255, t256, t257, t258, t259;
  double t260, t261, t262, t263, t264, t265, t266, t267, t268, t269;
  double t270, t271, t272, t273, t274, t275, t276, t277, t278, t279;
  double t280, t281, t282, t283, t284, t285, t286, t287, t288, t289;
  double t290, t291, t292, t293, t294, t295, t296, t297, t298, t299;

  double t300, t301, t302, t303, t304, t305, t306, t307, t308, t309;
  double t310, t311, t312, t313, t314, t315, t316, t317, t318, t319;
  double t320, t321, t322, t323, t324, t325, t326, t327, t328, t329;
  double t330, t331, t332, t333, t334, t335, t336, t337, t338, t339;
  double t340, t341, t342, t343, t344, t345, t346, t347, t348, t349;
  double t350, t351, t352, t353, t354, t355, t356, t357, t358, t359;
  double t360, t361, t362, t363, t364, t365, t366, t367, t368, t369;
  double t370, t371, t372, t373, t374, t375, t376, t377, t378, t379;
  double t380, t381, t382, t383, t384, t385, t386, t387, t388, t389;
  double t390, t391, t392, t393, t394, t395, t396, t397, t398, t399;
  double t400, t401, t402, t403, t404, t405, t406, t407, t408, t409;
  double t410, t411, t412, t413, t414, t415, t416, t417, t418, t419;
  double t420, t421, t422, t423, t424, t425, t426, t427, t428, t429;
  double t430, t431, t432, t433, t434, t435, t436, t437, t438, t439;
  double t440, t441, t442, t443, t444, t445, t446, t447, t448, t449;
  double t450, t451, t452, t453, t454, t455, t456, t457, t458, t459;
  double t460, t461, t462, t463, t464, t465, t466, t467, t468, t469;
  double t470, t471, t472, t473, t474, t475, t476, t477, t478, t479;
  double t480, t481, t482, t483, t484, t485, t486, t487, t488, t489;
  double t490, t491, t492, t493, t494, t495, t496, t497, t498, t499;
  double t500, t501, t502, t503, t504, t505, t506, t507, t508, t509;
  double t510, t511, t512, t513, t514, t515, t516, t517, t518, t519;
  double t520, t521, t522, t523, t524, t525, t526, t527, t528, t529;
  double t530, t531, t532, t533, t534, t535, t536, t537, t538, t539;
  double t540, t541, t542, t543, t544, t545, t546, t547, t548, t549;
  double t550, t551, t552, t553, t554, t555, t556, t557, t558, t559;
  double t560, t561, t562, t563, t564, t565, t566, t567, t568, t569;
  double t570, t571, t572, t573, t574, t575, t576, t577, t578, t579;
  double t580, t581, t582, t583, t584, t585, t586, t587, t588, t589;
  double t590, t591, t592, t593, t594, t595, t596, t597, t598, t599;
  double t600, t601, t602, t603, t604, t605, t606, t607, t608, t609;
  double t610, t611, t612, t613, t614, t615, t616, t617, t618, t619;
  double t620, t621, t622;

  t2 = cos(ra);
  t3 = cos(ya);
  t4 = sin(pa);
  t5 = sin(ra);
  t6 = sin(ya);
  t7 = t2*t6;
  t13 = t3*t4*t5;
  t8 = t7-t13;
  t9 = t2*t3;
  t10 = t4*t5*t6;
  t11 = t9+t10;
  t12 = cos(pa);
  t35 = px*t8;
  t36 = py*t11;
  t37 = t8*x;
  t38 = t11*y;
  t39 = pz*t5*t12;
  t40 = t5*t12*z;
  t14 = t35-t36-t37+t38-t39+t40;
  t15 = pz*t4;
  t16 = t3*t12*x;
  t17 = t6*t12*y;
  t30 = t4*z;
  t31 = px*t3*t12;
  t33 = py*t6*t12;
  t18 = t15+t16+t17-t30-t31-t33;

  // Imaginary
  t26 = px*t8;
  t27 = -py*t11;
  t28 = -t8*x;
  t29 = t11*y;
  t32 = -pz*t5*t12;
  t34 = t5*t12*z;
  // End imaginary

  t41 = t14*t14;
  t42 = t18*t18;
  t43 = t41+t42;
  t44 = sqrt(t43);
  t45 = 1.0/k;

  // Using imaginary
  t19 = -t45+t44*cos(alpha-atan2(t26+t27+t28+t29+t32+t34, t15+t16+t17-t4*z-px*t3*t12-py*t6*t12)  );

  t20 = t5*t6;
  t21 = t2*t3*t4;
  t22 = t20+t21;
  t23 = t3*t5;
  t54 = t2*t4*t6;
  t24 = t23-t54;
  t53 = px*t22;
  t55 = py*t24;
  t56 = t22*x;
  t57 = t24*y;
  t58 = pz*t2*t12;
  t59 = t2*t12*z;
  t25 = t53-t55-t56+t57+t58-t59;

  t47 = t15+t16+t17-t30-t31-t33;
  // Using imaginary
  t48 = atan2(t26+t27+t28+t29+t32+t34, t47);

  t49 = alpha-t48;
  t50 = cos(t49);
  t51 = t44*t50;
  t60 = t25*t25;
  t65 = sin(alpha);
  t46 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t19*t19));
  t52 = t45-t51;
  t68 = cos(alpha);
  t61 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t52*t52));
  t62 = 1.0/e2;

  // Imaginary
  t72 = -px*t22;
  t73 = py*t24;
  t74 = t22*x;
  t75 = -t24*y;
  t76 = -pz*t2*t12;
  t77 = t2*t12*z;
  // End imaginary

  // Using imaginary
  t78 = t45-t51;
  t63 = atan2(+t72+t73+t74+t75+t76+t77, t78);
  t64 = 1.0/e1;
  t66 = t45-t51;
  t67 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t66*t66));
  t69 = t45-t51;
  t70 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t69*t69));
  t71 = 1.0/(k*k);
  t79 = t63*t63;
  t80 = 1.0/(b*b);
  t81 = t45-t51;
  t82 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t81*t81));
  t83 = 1.0/(a*a);
  t84 = t45-t51;
  t85 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t84*t84));
  t86 = e2*t64;
  t87 = 1.0/(c*c);
  t88 = t71*t79*t87;
  t89 = pow(t88,t64);
  t90 = t45-t51;
  t91 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t90*t90));
  t92 = t45-t51;
  t93 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t92*t92));
  t94 = t45-t51;
  t95 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t94*t94));
  t96 = a*b*c;
  t97 = t45-t51;
  t98 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t97*t97));
  t99 = t45-t51;
  t100 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t99*t99));
  t101 = t71*t79;
  t102 = t45-t51;
  t103 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t102*t102));
  t104 = t45-t51;
  t105 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t104*t104));
  t106 = t45-t51;
  t107 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t106*t106));
  t108 = 1.0/sqrt(t96);
  t109 = t45-t51;
  t110 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t109*t109));
  t111 = t45-t51;
  t112 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t111*t111));
  t113 = t45-t51;
  t114 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t113*t113));
  t115 = t45-t51;
  t116 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t115*t115));
  t117 = e1-1.0;
  t118 = t45-t51;
  t119 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t118*t118));
  t120 = t62-1.0;
  t121 = t45-t51;
  t122 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t121*t121));
  t123 = t45-t51;
  t124 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t123*t123));
  t125 = t86-1.0;
  t126 = sqrt(t96);
  t127 = t45-t51;
  t128 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t127*t127));
  t129 = t45-t51;
  t130 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t129*t129));
  t131 = t45-t51;
  t132 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t131*t131));
  t133 = t45-t51;
  t134 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t133*t133));
  t135 = t45-t51;
  t136 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t135*t135));
  t137 = t45-t51;
  t138 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t137*t137));
  t139 = t45-t51;
  t140 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t139*t139));
  t141 = t45-t51;
  t142 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t141*t141));
  t143 = t45-t51;
  t144 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t143*t143));
  t145 = t45-t51;
  t146 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t145*t145));
  t147 = t45-t51;
  t148 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t147*t147));
  t149 = t45-t51;
  t150 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t149*t149));
  t151 = t45-t51;
  t152 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t151*t151));
  t153 = t45-t51;
  t154 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t153*t153));
  t155 = t45-t51;
  t156 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t155*t155));
  t157 = t45-t51;
  t158 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t157*t157));
  t159 = t45-t51;
  t160 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t159*t159));
  t161 = 1.0/(e1*e1);
  t162 = t45-t51;
  t163 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t162*t162));
  t164 = t45-t51;
  t165 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t164*t164));
  t166 = t45-t51;
  t167 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t166*t166));
  t168 = t45-t51;
  t169 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t168*t168));
  t170 = t45-t51;
  t171 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t170*t170));
  t172 = t45-t51;
  t173 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t172*t172));
  t174 = t45-t51;
  t175 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t174*t174));
  t176 = t45-t51;
  t177 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t176*t176));
  t178 = t45-t51;
  t179 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t178*t178));
  t180 = t45-t51;
  t181 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t180*t180));
  t182 = t45-t51;
  t183 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t182*t182));
  t184 = t45-t51;
  t185 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t184*t184));
  t186 = t45-t51;
  t187 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t186*t186));
  t188 = t45-t51;
  t189 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t188*t188));
  t190 = 1.0/(e2*e2);
  t191 = t45-t51;
  t192 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t191*t191));
  t193 = t45-t51;
  t194 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t193*t193));
  t195 = t45-t51;
  t196 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t195*t195));
  t197 = t45-t51;
  t198 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t197*t197));
  t199 = t45-t51;
  t200 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t199*t199));
  t201 = t45-t51;
  t202 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t201*t201));
  t203 = t45-t51;
  t204 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t203*t203));
  t205 = t45-t51;
  t206 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t205*t205));
  t207 = t45-t51;
  t208 = 1.0/sqrt(t43);
  t209 = t8*t14*2.0;
  t221 = t3*t12*t18*2.0;
  t210 = t209-t221;
  t211 = t50*t208*t210*(1.0/2.0);
  t212 = sin(t49);
  t213 = 1.0/t18;
  t214 = t8*t213;
  t215 = 1.0/(t18*t18);
  t216 = t3*t12*t14*t215;
  t217 = t214+t216;
  t218 = t42*t208*t212*t217;
  t219 = t45-t51;
  t220 = t45-t51;
  t222 = t211+t218;
  t223 = t22*t25*2.0;
  t242 = t52*t222*2.0;
  t224 = t223-t242;
  t225 = t45-t51;
  t226 = t45-t51;
  t227 = t45-t51;
  t228 = t45-t51;
  t229 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t228*t228));
  t230 = t45-t51;
  t231 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t230*t230));
  t232 = t45-t51;
  t233 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t232*t232));
  t234 = t45-t51;
  t235 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t234*t234));
  t236 = t45-t51;
  t237 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t236*t236));
  t238 = t45-t51;
  t239 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t238*t238));
  t240 = t3*t12;
  t241 = t45-t51;
  t243 = t45-t51;
  t244 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t243*t243));
  t245 = t45-t51;
  t246 = t45-t51;
  t247 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t246*t246));
  t248 = t45-t51;
  t249 = t45-t51;
  t250 = 1.0/t52;
  t251 = t22*t250;
  t252 = 1.0/(t52*t52);
  t253 = t25*t222*t252;
  t254 = t45-t51;
  t255 = t45-t51;
  t256 = t64-1.0;
  t257 = pow(t88,t256);
  t258 = t251+t253;
  t259 = t45-t51;
  t260 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t259*t259));
  t261 = t45-t51;
  t262 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t261*t261));
  t263 = t45-t51;
  t264 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t263*t263));
  t265 = t45-t51;
  t266 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t265*t265));
  t267 = t45-t51;
  t268 = t11*t14*2.0;
  t269 = t6*t12*t18*2.0;
  t270 = t268+t269;
  t271 = t50*t208*t270*(1.0/2.0);
  t272 = t11*t213;
  t277 = t6*t12*t14*t215;
  t273 = t272-t277;
  t274 = t42*t208*t212*t273;
  t275 = t45-t51;
  t276 = t45-t51;
  t278 = t271+t274;
  t290 = t24*t25*2.0;
  t291 = t52*t278*2.0;
  t279 = t290-t291;
  t280 = t45-t51;
  t281 = t45-t51;
  t282 = t45-t51;
  t283 = t45-t51;
  t284 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t283*t283));
  t285 = t45-t51;
  t286 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t285*t285));
  t287 = t45-t51;
  t288 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t287*t287));
  t289 = t45-t51;
  t292 = t6*t12;
  t293 = t45-t51;
  t294 = t45-t51;
  t295 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t294*t294));
  t296 = t45-t51;
  t297 = t45-t51;
  t298 = t45-t51;
  t299 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t298*t298));
  t300 = t45-t51;
  t301 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t300*t300));
  t302 = t24*t250;
  t303 = t25*t252*t278;
  t304 = t45-t51;
  t305 = t45-t51;
  t306 = t302+t303;
  t307 = t45-t51;
  t308 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t307*t307));
  t309 = t45-t51;
  t310 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t309*t309));
  t311 = t45-t51;
  t312 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t311*t311));
  t313 = t45-t51;
  t314 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t313*t313));
  t315 = t45-t51;
  t316 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t315*t315));
  t317 = t45-t51;
  t318 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t317*t317));
  t319 = t45-t51;
  t320 = t4*t18*2.0;
  t327 = t5*t12*t14*2.0;
  t321 = t320-t327;
  t322 = t50*t208*t321*(1.0/2.0);
  t323 = t5*t12*t213;
  t324 = t4*t14*t215;
  t325 = t323+t324;
  t326 = t45-t51;
  t328 = t42*t208*t212*t325;
  t329 = t45-t51;
  t330 = t45-t51;
  t331 = t322-t328;
  t332 = t45-t51;
  t333 = t45-t51;
  t334 = t45-t51;
  t335 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t334*t334));
  t336 = t45-t51;
  t337 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t336*t336));
  t338 = t52*t331*2.0;
  t350 = t2*t12*t25*2.0;
  t339 = t338-t350;
  t340 = t45-t51;
  t341 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t340*t340));
  t342 = t45-t51;
  t343 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t342*t342));
  t344 = t45-t51;
  t345 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t344*t344));
  t346 = t45-t51;
  t347 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t346*t346));
  t348 = t45-t51;
  t349 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t348*t348));
  t351 = t45-t51;
  t352 = t45-t51;
  t353 = t5*t12;
  t354 = t45-t51;
  t355 = t45-t51;
  t356 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t355*t355));
  t357 = t45-t51;
  t358 = t2*t12*t250;
  t359 = t25*t252*t331;
  t360 = t45-t51;
  t361 = t45-t51;
  t362 = t45-t51;
  t363 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t362*t362));
  t364 = t45-t51;
  t365 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t364*t364));
  t366 = t358+t359;
  t367 = t45-t51;
  t368 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t367*t367));
  t369 = t45-t51;
  t370 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t369*t369));
  t371 = t45-t51;
  t372 = t18*t25*t208*t212;
  t373 = t14*t25*t50*t208;
  t374 = t45-t51;
  t375 = t45-t51;
  t376 = t14*t25*2.0;
  t377 = t372+t373;
  t378 = t52*t377*2.0;
  t379 = t376+t378;
  t380 = t45-t51;
  t381 = t45-t51;
  t382 = t45-t51;
  t383 = t45-t51;
  t384 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t383*t383));
  t385 = t45-t51;
  t386 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t385*t385));
  t387 = t45-t51;
  t388 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t387*t387));
  t389 = t45-t51;
  t390 = t45-t51;
  t391 = t45-t51;
  t392 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t391*t391));
  t393 = t45-t51;
  t394 = t45-t51;
  t395 = t45-t51;
  t396 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t395*t395));
  t397 = t45-t51;
  t398 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t397*t397));
  t399 = t14*t250;
  t400 = t45-t51;
  t401 = t45-t51;
  t402 = t45-t51;
  t403 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t402*t402));
  t404 = t45-t51;
  t405 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t404*t404));
  t406 = t45-t51;
  t407 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t406*t406));
  t408 = t45-t51;
  t409 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t408*t408));
  t410 = t399-t25*t252*t377;
  t411 = t45-t51;
  t412 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t411*t411));
  t413 = t45-t51;
  t414 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t413*t413));
  t415 = t45-t51;
  t416 = pz*t4*t5;
  t417 = t3*t5*t12*x;
  t418 = t5*t6*t12*y;
  t428 = t4*t5*z;
  t429 = px*t3*t5*t12;
  t430 = py*t5*t6*t12;
  t419 = t416+t417+t418-t428-t429-t430;
  t420 = pz*t12;
  t421 = px*t3*t4;
  t422 = py*t4*t6;
  t424 = t12*z;
  t425 = t3*t4*x;
  t426 = t4*t6*y;
  t423 = t420+t421+t422-t424-t425-t426;
  t427 = t18*t423*2.0;
  t431 = t14*t419*2.0;
  t432 = t427+t431;
  t433 = t50*t208*t432*(1.0/2.0);
  t434 = t213*t419;
  t444 = t14*t215*t423;
  t435 = t434-t444;
  t436 = t42*t208*t212*t435;
  t437 = t45-t51;
  t438 = t45-t51;
  t439 = pz*t2*t4;
  t440 = t2*t3*t12*x;
  t441 = t2*t6*t12*y;
  t449 = t2*t4*z;
  t450 = px*t2*t3*t12;
  t451 = py*t2*t6*t12;
  t442 = t439+t440+t441-t449-t450-t451;
  t443 = t25*t442*2.0;
  t445 = t433+t436;
  t446 = t52*t445*2.0;
  t447 = t443+t446;
  t448 = t45-t51;
  t452 = t45-t51;
  t453 = t45-t51;
  t454 = t45-t51;
  t455 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t454*t454));
  t456 = t45-t51;
  t457 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t456*t456));
  t458 = t45-t51;
  t459 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t458*t458));
  t460 = t45-t51;
  t461 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t460*t460));
  t462 = t45-t51;
  t463 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t462*t462));
  t464 = t45-t51;
  t465 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t464*t464));
  t466 = t45-t51;
  t467 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t466*t466));
  t468 = t45-t51;
  t469 = t45-t51;
  t470 = t45-t51;
  t471 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t470*t470));
  t472 = t45-t51;
  t473 = t45-t51;
  t474 = t250*t442;
  t475 = 1.0/pow(t45-t51,2.0);
  t476 = t45-t51;
  t477 = t45-t51;
  t478 = t45-t51;
  t479 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t478*t478));
  t480 = t45-t51;
  t481 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t480*t480));
  t482 = t45-t51;
  t483 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t482*t482));
  t484 = t45-t51;
  t485 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t484*t484));
  t486 = px*t11;
  t487 = py*t8;
  t493 = t11*x;
  t494 = t8*y;
  t488 = t486+t487-t493-t494;
  t489 = t3*t12*y;
  t490 = px*t6*t12;
  t496 = py*t3*t12;
  t497 = t6*t12*x;
  t491 = t489+t490-t496-t497;
  t492 = t45-t51;
  t495 = t14*t488*2.0;
  t498 = t213*t488;
  t508 = t14*t215*t491;
  t499 = t498-t508;
  t500 = t42*t208*t212*t499;
  t501 = t45-t51;
  t502 = px*t24;
  t503 = py*t22;
  t513 = t24*x;
  t514 = t22*y;
  t504 = t502+t503-t513-t514;
  t505 = t18*(t489+t490-t496-t497)*2.0;
  t506 = t495+t505;
  t507 = t50*t208*t506*(1.0/2.0);
  t509 = t45-t51;
  t510 = t45-t51;
  t511 = t45-t51;
  t512 = t45-t51;
  t515 = t500+t507;
  t516 = t45-t51;
  t517 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t516*t516));
  t518 = t45-t51;
  t519 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t518*t518));
  t520 = t45-t51;
  t521 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t520*t520));
  t522 = t45-t51;
  t523 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t522*t522));
  t524 = t45-t51;
  t525 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t524*t524));
  t526 = t25*t504*2.0;
  t532 = t52*t515*2.0;
  t527 = t526-t532;
  t528 = t45-t51;
  t529 = t45-t51;
  t530 = t45-t51;
  t531 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t530*t530));
  t533 = t45-t51;
  t534 = t45-t51;
  t535 = t45-t51;
  t536 = t45-t51;
  t537 = t250*t504;
  t538 = t25*t475*t515;
  t539 = t537+t538;
  t540 = t45-t51;
  t541 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t540*t540));
  t542 = t45-t51;
  t543 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t542*t542));
  t544 = t45-t51;
  t545 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t544*t544));
  t546 = t45-t51;
  t547 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t546*t546));
  t548 = t45-t51;
  t549 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t548*t548));
  t550 = t45-t51;
  t551 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t550*t550));
  t552 = t45-t51;
  t553 = t45-t51;
  t554 = t45-t51;
  t555 = t45-t51;
  t556 = t44*t212;
  t557 = t45-t51;
  t558 = t45-t51;
  t559 = t45-t51;
  t560 = t45-t51;
  t561 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t560*t560));
  t562 = t45-t51;
  t563 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t562*t562));
  t564 = t45-t51;
  t565 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t564*t564));
  t566 = t45-t51;
  t567 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t566*t566));
  t568 = t45-t51;
  t569 = t45-t51;
  t570 = t45-t51;
  t571 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t570*t570));
  t572 = t45-t51;
  t573 = t45-t51;
  t574 = t45-t51;
  t575 = t45-t51;
  t576 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t575*t575));
  t577 = t45-t51;
  t578 = t45-t51;
  t579 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t578*t578));
  t580 = t45-t51;
  t581 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t580*t580));
  t582 = t45-t51;
  t583 = t45-t51;
  t584 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t583*t583));
  t585 = t45-t51;
  t586 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t585*t585));
  t587 = t45-t51;
  t588 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t587*t587));
  t589 = t45-t51;
  t590 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t589*t589));
  t591 = t45-t51;
  t592 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t591*t591));
  t593 = t45-t51;
  t594 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t593*t593));
  t595 = t45-t51;
  t596 = t45-t51;
  t597 = t45-t51;
  t598 = t45-t51;
  t599 = t45-t51;
  t600 = 1.0/(k*k*k);
  t601 = 1.0/(k*k*k*k);
  t602 = t45-t51;
  t603 = t45-t51;
  t604 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t603*t603));
  t605 = t45-t51;
  t606 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t605*t605));
  t607 = t45-t51;
  t608 = t45-t51;
  t609 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t608*t608));
  t610 = t45-t51;
  t611 = t45-t51;
  t612 = t45-t51;
  t613 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t612*t612));
  t614 = t45-t51;
  t615 = t45-t51;
  t616 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t615*t615));
  t617 = t45-t51;
  t618 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t617*t617));
  t619 = t45-t51;
  t620 = -t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t619*t619));
  t621 = t45-t51;
  t622 = -t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t621*t621));
  _J[0] = b*c*t108*(pow(t89+pow(pow((t46*t46)*t80,t62)+pow((t61*t61)*t83,t62),t86),e1)-1.0)*sqrt(t101+t67*t67+t70*t70)*(1.0/2.0)-1.0/(a*a*a)*(t103*t103)*t126*pow(pow(t80*(t93*t93),t62)+pow(t83*(t95*t95),t62),t125)*pow(t83*(t91*t91),t120)*pow(t89+pow(pow(t80*(t82*t82),t62)+pow(t83*(t85*t85),t62),t86),t117)*sqrt(t101+t98*t98+t100*t100)*2.0;
  _J[1] = a*c*t108*(pow(t89+pow(pow(t80*(t105*t105),t62)+pow(t83*(t107*t107),t62),t86),e1)-1.0)*sqrt(t101+t110*t110+t112*t112)*(1.0/2.0)-1.0/(b*b*b)*t126*(t132*t132)*pow(pow(t80*(t122*t122),t62)+pow(t83*(t124*t124),t62),t125)*pow(t80*(t119*t119),t120)*pow(t89+pow(pow(t80*(t114*t114),t62)+pow(t83*(t116*t116),t62),t86),t117)*sqrt(t101+t128*t128+t130*t130)*2.0;
  _J[2] = a*b*t108*(pow(t89+pow(pow(t80*(t134*t134),t62)+pow(t83*(t136*t136),t62),t86),e1)-1.0)*sqrt(t101+t138*t138+t140*t140)*(1.0/2.0)-1.0/(c*c*c)*t71*t79*t126*t257*pow(t89+pow(pow(t80*(t142*t142),t62)+pow(t83*(t144*t144),t62),t86),t117)*sqrt(t101+t146*t146+t148*t148)*2.0;
  _J[3] = t126*(log(t89+pow(pow(t80*(t150*t150),t62)+pow(t83*(t152*t152),t62),t86))*pow(t89+pow(pow(t80*(t154*t154),t62)+pow(t83*(t156*t156),t62),t86),e1)-e1*(t89*t161*log(t88)+e2*t161*log(pow(t80*(t163*t163),t62)+pow(t83*(t165*t165),t62))*pow(pow(t80*(t167*t167),t62)+pow(t83*(t169*t169),t62),t86))*pow(t89+pow(pow(t80*(t158*t158),t62)+pow(t83*(t160*t160),t62),t86),t117))*sqrt(t101+t171*t171+t173*t173);
  _J[4] = e1*t126*(t64*log(pow(t80*(t179*t179),t62)+pow(t83*(t181*t181),t62))*pow(pow(t80*(t183*t183),t62)+pow(t83*(t185*t185),t62),t86)-e2*t64*(t190*log(t80*(t187*t187))*pow(t80*(t189*t189),t62)+t190*log(t83*(t192*t192))*pow(t83*(t194*t194),t62))*pow(pow(t80*(t196*t196),t62)+pow(t83*(t198*t198),t62),t125))*pow(t89+pow(pow(t80*(t175*t175),t62)+pow(t83*(t177*t177),t62),t86),t117)*sqrt(t101+t200*t200+t202*t202);
  _J[5] = t126*(pow(t89+pow(pow(t80*(t204*t204),t62)+pow(t83*(t206*t206),t62),t86),e1)-1.0)*1.0/sqrt(t101+t229*t229+t231*t231)*((-t7+t13+t65*(t211+t218+t224*1.0/sqrt(t60+t220*t220)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t225*t225)))*2.0+(t240+t68*(t211+t218+t224*1.0/sqrt(t60+t207*t207)*(1.0/2.0)))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t219*t219)))*2.0-(t63*t71*(t227*t227)*t258*2.0)/(t60+t226*t226))*(1.0/2.0)+e1*t126*(e2*t64*pow(pow(t80*(t237*t237),t62)+pow(t83*(t239*t239),t62),t125)*(t62*t80*pow(t80*(t247*t247),t120)*(-t7+t13+t65*(t211+t218+t224*1.0/sqrt(t60+t248*t248)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t249*t249)))*2.0+t62*t83*pow(t83*(t244*t244),t120)*(t240+t68*(t211+t218+t224*1.0/sqrt(t60+t241*t241)*(1.0/2.0)))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t245*t245)))*2.0)-(t63*t64*t71*t87*(t255*t255)*t257*t258*2.0)/(t60+t254*t254))*pow(t89+pow(pow(t80*(t233*t233),t62)+pow(t83*(t235*t235),t62),t86),t117)*sqrt(t101+t260*t260+t262*t262);
  _J[6] = t126*(pow(t89+pow(pow(t80*(t264*t264),t62)+pow(t83*(t266*t266),t62),t86),e1)-1.0)*((t9+t10-t65*(t271+t274+t279*1.0/sqrt(t60+t276*t276)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t280*t280)))*2.0+(t292-t68*(t271+t274+t279*1.0/sqrt(t60+t267*t267)*(1.0/2.0)))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t275*t275)))*2.0+(t63*t71*(t282*t282)*t306*2.0)/(t60+t281*t281))*1.0/sqrt(t101+t284*t284+t286*t286)*(1.0/2.0)+e1*t126*(e2*t64*(t62*t83*pow(t83*(t288*t288),t120)*(t292-t68*(t271+t274+t279*1.0/sqrt(t60+t289*t289)*(1.0/2.0)))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t293*t293)))*2.0+t62*t80*pow(t80*(t295*t295),t120)*(t9+t10-t65*(t271+t274+t279*1.0/sqrt(t60+t296*t296)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t297*t297)))*2.0)*pow(pow(t80*(t299*t299),t62)+pow(t83*(t301*t301),t62),t125)+(t63*t64*t71*t87*t257*(t305*t305)*t306*2.0)/(t60+t304*t304))*pow(t89+pow(pow(t80*(t308*t308),t62)+pow(t83*(t310*t310),t62),t86),t117)*sqrt(t101+t312*t312+t314*t314);
  _J[7] = t126*(pow(t89+pow(pow(t80*(t316*t316),t62)+pow(t83*(t318*t318),t62),t86),e1)-1.0)*((t4+t68*(-t322+t328+t339*1.0/sqrt(t60+t319*t319)*(1.0/2.0)))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t326*t326)))*2.0-(t353-t65*(-t322+t328+t339*1.0/sqrt(t60+t329*t329)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t330*t330)))*2.0+(t63*t71*(t333*t333)*t366*2.0)/(t60+t332*t332))*1.0/sqrt(t101+t335*t335+t337*t337)*(-1.0/2.0)-e1*t126*(e2*t64*pow(pow(t80*(t345*t345),t62)+pow(t83*(t347*t347),t62),t125)*(t62*t83*(t4+t68*(-t322+t328+t339*1.0/sqrt(t60+t351*t351)*(1.0/2.0)))*pow(t83*(t349*t349),t120)*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t352*t352)))*2.0-t62*t80*(t353-t65*(-t322+t328+t339*1.0/sqrt(t60+t354*t354)*(1.0/2.0)))*pow(t80*(t356*t356),t120)*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t357*t357)))*2.0)+(t63*t64*t71*t87*t257*(t361*t361)*t366*2.0)/(t60+t360*t360))*pow(t89+pow(pow(t80*(t341*t341),t62)+pow(t83*(t343*t343),t62),t86),t117)*sqrt(t101+t363*t363+t365*t365);
  _J[8] = t126*(pow(t89+pow(pow(t80*(t368*t368),t62)+pow(t83*(t370*t370),t62),t86),e1)-1.0)*1.0/sqrt(t101+t384*t384+t386*t386)*((-t53+t55+t56-t57-t58+t59+t65*(t372+t373-t379*1.0/sqrt(t60+t371*t371)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t374*t374)))*2.0+t68*(t372+t373-t379*1.0/sqrt(t60+t375*t375)*(1.0/2.0))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t380*t380)))*2.0+(t63*t71*(t382*t382)*t410*2.0)/(t60+t381*t381))*(-1.0/2.0)-e1*t126*(e2*t64*(t62*t80*pow(t80*(t388*t388),t120)*(-t53+t55+t56-t57-t58+t59+t65*(t372+t373-t379*1.0/sqrt(t60+t389*t389)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t390*t390)))*2.0+t62*t68*t83*pow(t83*(t392*t392),t120)*(t372+t373-t379*1.0/sqrt(t60+t393*t393)*(1.0/2.0))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t394*t394)))*2.0)*pow(pow(t80*(t396*t396),t62)+pow(t83*(t398*t398),t62),t125)+(t63*t64*t71*t87*t257*(t401*t401)*t410*2.0)/(t60+t400*t400))*pow(t89+pow(pow(t80*(t403*t403),t62)+pow(t83*(t405*t405),t62),t86),t117)*sqrt(t101+t407*t407+t409*t409);
  _J[9] = t126*(pow(t89+pow(pow(t80*(t412*t412),t62)+pow(t83*(t414*t414),t62),t86),e1)-1.0)*1.0/sqrt(t101+t455*t455+t457*t457)*((-t420-t421-t422+t424+t425+t426+t68*(t433+t436-t447*1.0/sqrt(t60+t415*t415)*(1.0/2.0)))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t437*t437)))*2.0+(-t416-t417-t418+t428+t429+t430+t65*(t433+t436-t447*1.0/sqrt(t60+t438*t438)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t448*t448)))*2.0+(t63*t71*(t453*t453)*(t474-t25*t445*t475)*2.0)/(t60+t452*t452))*(1.0/2.0)+e1*t126*(e2*t64*pow(pow(t80*(t463*t463),t62)+pow(t83*(t465*t465),t62),t125)*(t62*t83*pow(t83*(t467*t467),t120)*(-t420-t421-t422+t424+t425+t426+t68*(t433+t436-t447*1.0/sqrt(t60+t468*t468)*(1.0/2.0)))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t469*t469)))*2.0+t62*t80*pow(t80*(t471*t471),t120)*(-t416-t417-t418+t428+t429+t430+t65*(t433+t436-t447*1.0/sqrt(t60+t472*t472)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t473*t473)))*2.0)+(t63*t64*t71*t87*t257*(t477*t477)*(t474-t25*t445*t475)*2.0)/(t60+t476*t476))*pow(t89+pow(pow(t80*(t459*t459),t62)+pow(t83*(t461*t461),t62),t86),t117)*sqrt(t101+t479*t479+t481*t481);
  _J[10] = t126*(pow(t89+pow(pow(t80*(t483*t483),t62)+pow(t83*(t485*t485),t62),t86),e1)-1.0)*((-t489-t490+t496+t497+t68*(t500+t507-1.0/sqrt(t60+t492*t492)*(t52*(t500+t50*t208*(t495+t18*t491*2.0)*(1.0/2.0))*2.0-t25*t504*2.0)*(1.0/2.0)))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t501*t501)))*2.0+(-t486-t487+t493+t494+t65*(t500+t507+t527*1.0/sqrt(t60+t509*t509)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t510*t510)))*2.0-(t63*t71*(t512*t512)*t539*2.0)/(t60+t511*t511))*1.0/sqrt(t101+t517*t517+t519*t519)*(1.0/2.0)+e1*t126*(e2*t64*(t62*t83*pow(t83*(t525*t525),t120)*(-t489-t490+t496+t497+t68*(t500+t507+t527*1.0/sqrt(t60+t528*t528)*(1.0/2.0)))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t529*t529)))*2.0+t62*t80*pow(t80*(t531*t531),t120)*(-t486-t487+t493+t494+t65*(t500+t507+t527*1.0/sqrt(t60+t533*t533)*(1.0/2.0)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t534*t534)))*2.0)*pow(pow(t80*(t521*t521),t62)+pow(t83*(t523*t523),t62),t125)-(t63*t64*t71*t87*t257*(t536*t536)*t539*2.0)/(t60+t535*t535))*pow(t89+pow(pow(t80*(t541*t541),t62)+pow(t83*(t543*t543),t62),t86),t117)*sqrt(t101+t545*t545+t547*t547);
  _J[11] = t126*(pow(t89+pow(pow(t80*(t549*t549),t62)+pow(t83*(t551*t551),t62),t86),e1)-1.0)*1.0/sqrt(t101+t561*t561+t563*t563)*((t68*(t556-t44*t52*t212*1.0/sqrt(t60+t557*t557))+t65*(-t45+t51+sqrt(t60+t555*t555)))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t558*t558)))*2.0+(t65*(t556-t44*t52*t212*1.0/sqrt(t60+t553*t553))-t68*(-t45+t51+sqrt(t60+t552*t552)))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t554*t554)))*2.0-(t25*t44*t63*t71*t212*2.0)/(t60+t559*t559))*(-1.0/2.0)-e1*t126*(e2*t64*pow(pow(t80*(t579*t579),t62)+pow(t83*(t581*t581),t62),t125)*(t62*t83*(t68*(t556-t44*t52*t212*1.0/sqrt(t60+t569*t569))+t65*(-t45+t51+sqrt(t60+t568*t568)))*pow(t83*(t571*t571),t120)*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t572*t572)))*2.0+t62*t80*(t65*(t556-t44*t52*t212*1.0/sqrt(t60+t574*t574))-t68*(-t45+t51+sqrt(t60+t573*t573)))*pow(t80*(t576*t576),t120)*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t577*t577)))*2.0)-(t25*t44*t63*t64*t71*t87*t212*t257*2.0)/(t60+t582*t582))*pow(t89+pow(pow(t80*(t565*t565),t62)+pow(t83*(t567*t567),t62),t86),t117)*sqrt(t101+t584*t584+t586*t586);
  _J[12] = t126*(pow(t89+pow(pow(t80*(t588*t588),t62)+pow(t83*(t590*t590),t62),t86),e1)-1.0)*1.0/sqrt(t101+t592*t592+t594*t594)*(t79*t600*2.0-t68*(t71-t52*t71*1.0/sqrt(t60+t595*t595))*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t596*t596)))*2.0-t65*(t71-t52*t71*1.0/sqrt(t60+t597*t597))*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t598*t598)))*2.0+(t25*t63*t601*2.0)/(t60+t599*t599))*(-1.0/2.0)-e1*t126*(t64*t257*(t79*t87*t600*2.0+(t25*t63*t87*t601*2.0)/(t60+t602*t602))-e2*t64*pow(pow(t80*(t604*t604),t62)+pow(t83*(t606*t606),t62),t125)*(t62*t68*t83*(t71-t52*t71*1.0/sqrt(t60+t611*t611))*pow(t83*(t613*t613),t120)*(-t15-t16-t17+t30+t31+t33+t68*(-t45+t51+sqrt(t60+t614*t614)))*2.0+t62*t65*t80*(t71-t52*t71*1.0/sqrt(t60+t607*t607))*pow(t80*(t609*t609),t120)*(-t35+t36+t37-t38+t39-t40+t65*(-t45+t51+sqrt(t60+t610*t610)))*2.0))*pow(t89+pow(pow(t80*(t616*t616),t62)+pow(t83*(t618*t618),t62),t86),t117)*sqrt(t101+t620*t620+t622*t622);

}



/**
 * @function fc
 */
double fc_b( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &alpha, const double &k,
	     const double &x, const double &y, const double &z ) {

  double t0;


  return t0;
}


/**
 * @function Jc_b
 */
void Jc_b( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &alpha, const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[13] ) {
}

/**
 * @function f5
 */
double f5_b( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &alpha, const double &k,
	     const double &x, const double &y, const double &z ) {

  double t0;


  return t0;
}


/**
 * @function J5_b
 */
void J5_b( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &alpha, const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[13] ) {
}

/**
 * @function f6
 */
double f6_b( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &alpha, const double &k,
	     const double &x, const double &y, const double &z ) {

  double t0;


  return t0;
}


/**
 * @function J6_b
 */
void J6_b( const double &a, const double &b, const double &c,
	   const double &e1, const double &e2,
	   const double &px, const double &py, const double &pz,
	   const double &ra, const double &pa, const double &ya,
	   const double &alpha, const double &k,
	   const double &x, const double &y, const double &z,
	   double _J[13] ) {
}
