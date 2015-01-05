/**
 * @file analytic_equations.cpp
 */

#include "analytic_equations.h"
#include <cmath>


double f_SQ( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &x, const double &y, const double &z ) {

  double t2 = cos(ya);
  double t3 = sin(ra);
  double t4 = cos(ra);
  double t5 = sin(pa);
  double t6 = sin(ya);
  double t7 = t3*t6;
  double t8 = t2*t4*t5;
  double t9 = t7+t8;
  double t10 = t2*t3;
  double t11 = t10-t4*t5*t6;
  double t12 = cos(pa);
  double t13 = px*t9-py*t11-t9*x+t11*y+pz*t4*t12-t4*t12*z;
  double t14 = t4*t6;
  double t15 = t14-t2*t3*t5;
  double t16 = t2*t4;
  double t17 = t3*t5*t6;
  double t18 = t16+t17;
  double t19 = px*t15-py*t18-t15*x+t18*y-pz*t3*t12+t3*t12*z;
  double t20 = pz*t5-t5*z-px*t2*t12-py*t6*t12+t2*t12*x+t6*t12*y;
  double t21 = 1.0/e2;
  double t22 = 1.0/e1;
  double t23 = pow(pow(pow(1.0/(a*a)*(t20*t20),t21)+pow(1.0/(b*b)*(t19*t19),t21),e2*t22)+pow(1.0/(c*c)*(t13*t13),t22),e1)-1.0;
  return (t23*t23)*sqrt(a*b*c);

}


/**
 * @function f_SQ
 * @brief Equation (1) from Duncan's paper
 * @brief [ (x/a)^(2/e2) + (y/b)^(2/e2) ]^(e2/e1) + (z/c)^(2/e1)
 * @brief While x,y,z are the transformed points (Rot, trans)
 */
double f_SQ( const SQ_parameters &_par, 
	     const double &x, const double &y, const double &z ) {
  
  double a = _par.dim[0];
  double b = _par.dim[1];
  double c = _par.dim[2];
  double e1 = _par.e[0];
  double e2 = _par.dim[1];
  double px = _par.trans[0];
  double py = _par.trans[1]; 
  double pz = _par.trans[2]; 
  double ra = _par.rot[0]; 
  double pa = _par.rot[1]; 
  double ya = _par.rot[2];
  
  return f_SQ(a,b,c, e1,e2, px,py,pz, ra,pa,ya, x,y,z );

}

/**
 * @function error_SQ
 * @brief Eq.(4) from Duncan's paper (F^e1 -1)^2
 */
double error_SQ( const SQ_parameters &_par,  
		 const double &x, const double &y, const double &z ) {

  double a = _par.dim[0];
  double b = _par.dim[1];
  double c = _par.dim[2];
  double e1 = _par.e[0];
  double e2 = _par.dim[1];
  double px = _par.trans[0];
  double py = _par.trans[1]; 
  double pz = _par.trans[2]; 
  double ra = _par.rot[0]; 
  double pa = _par.rot[1]; 
  double ya = _par.rot[2];

  double t2 = cos(ya);
    double t3 = sin(ra);
    double t4 = cos(ra);
    double t5 = sin(pa);
    double t6 = sin(ya);
    double t7 = t3*t6;
    double t8 = t2*t4*t5;
    double t9 = t7+t8;
    double t10 = t2*t3;
    double t11 = t10-t4*t5*t6;
    double t12 = cos(pa);
    double t13 = px*t9-py*t11-t9*x+t11*y+pz*t4*t12-t4*t12*z;
    double t14 = t4*t6;
    double t15 = t14-t2*t3*t5;
    double t16 = t2*t4;
    double t17 = t3*t5*t6;
    double t18 = t16+t17;
    double t19 = px*t15-py*t18-t15*x+t18*y-pz*t3*t12+t3*t12*z;
    double t20 = pz*t5-t5*z-px*t2*t12-py*t6*t12+t2*t12*x+t6*t12*y;
    double t21 = 1.0/e2;
    double t22 = 1.0/e1;
    double t23 = pow(pow(pow(1.0/(a*a)*(t20*t20),t21)+pow(1.0/(b*b)*(t19*t19),t21),e2*t22)+pow(1.0/(c*c)*(t13*t13),t22),e1)-1.0;
    
    return t23*t23;
}

/**
 * @function jac_SQ
 * @brief
 */
void jac_SQ( const SQ_parameters &_par, 
	     const double &x, const double &y, const double &z,
	     double Jac[11] ) {

  double a = _par.dim[0];
  double b = _par.dim[1];
  double c = _par.dim[2];
  double e1 = _par.e[0];
  double e2 = _par.dim[1];
  double px = _par.trans[0];
  double py = _par.trans[1]; 
  double pz = _par.trans[2]; 
  double ra = _par.rot[0]; 
  double pa = _par.rot[1]; 
  double ya = _par.rot[2];

  return jac_SQ( a, b, c,
		 e1, e2,
		 px, py, pz,
		 ra, pa, ya,
		 x, y, z,
		 Jac );
}

/**
 * @function jac_SQ
 * @brief
 */

void jac_SQ( const double &a, const double &b, const double &c,
	     const double &e1, const double &e2,
	     const double &px, const double &py, const double &pz,
	     const double &ra, const double &pa, const double &ya,
	     const double &x, const double &y, const double &z,
	     double Jac[11] ) {

    double t2 = cos(ya);
    double  t3 = sin(ra);
    double  t4 = cos(ra);
    double  t5 = sin(pa);
    double  t6 = sin(ya);
    double  t7 = t3*t6;
    double  t8 = t2*t4*t5;
    double  t9 = t7+t8;
    double  t10 = t2*t3;
    double  t26 = t4*t5*t6;
    double  t11 = t10-t26;
    double  t12 = cos(pa);
    double  t25 = px*t9;
    double  t27 = py*t11;
    double  t28 = t9*x;
    double  t29 = t11*y;
    double  t30 = pz*t4*t12;
    double  t31 = t4*t12*z;
    double  t13 = t25-t27-t28+t29+t30-t31;
    double  t14 = t4*t6;
    double  t36 = t2*t3*t5;
    double  t15 = t14-t36;
    double  t16 = t2*t4;
    double  t17 = t3*t5*t6;
    double  t18 = t16+t17;
    double  t37 = px*t15;
    double  t38 = py*t18;
double  t39 = t15*x;
double  t40 = t18*y;
double  t41 = pz*t3*t12;
double  t42 = t3*t12*z;
double  t19 = t37-t38-t39+t40-t41+t42;
double  t47 = pz*t5;
double  t48 = t5*z;
double  t49 = px*t2*t12;
double  t50 = t2*t12*x;
double  t51 = py*t6*t12;
double  t52 = t6*t12*y;
double  t20 = t47-t48-t49+t50-t51+t52;
double  t21 = 1.0/e2;
double  t22 = 1.0/e1;
double  t24 = 1.0/(c*c);
double  t32 = t13*t13;
double  t33 = t24*t32;
double  t34 = pow(t33,t22);
double  t35 = 1.0/(b*b);
double  t43 = t19*t19;
double  t44 = t35*t43;
double  t45 = pow(t44,t21);
double  t46 = 1.0/(a*a);
double  t53 = t20*t20;
double  t54 = t46*t53;
double  t55 = pow(t54,t21);
double  t56 = t45+t55;
double  t57 = e2*t22;
double  t58 = pow(t56,t57);
double  t59 = t34+t58;
double  t60 = pow(t59,e1);
double  t23 = t60-1.0;
double  t61 = a*b*c;
double  t62 = t23*t23;
double  t63 = 1.0/sqrt(t61);
double  t64 = e1-1.0;
double  t65 = pow(t59,t64);
double  t66 = t21-1.0;
double  t67 = t57-1.0;
double  t68 = pow(t56,t67);
double  t69 = sqrt(t61);
double  t70 = 1.0/(e1*e1);
double  t71 = log(t56);
double  t72 = 1.0/(e2*e2);
double  t73 = pow(t44,t66);
double  t74 = pow(t54,t66);
double  t75 = t22-1.0;
double  t76 = pow(t33,t75);
  
 Jac[0] = b*c*t62*t63*(1.0/2.0)-1.0/(a*a*a)*t23*t53*t65*t68*t69*t74*4.0;
 Jac[1] = a*c*t62*t63*(1.0/2.0)-1.0/(b*b*b)*t23*t43*t65*t68*t69*t73*4.0;
 Jac[2] = a*b*t62*t63*(1.0/2.0)-1.0/(c*c*c)*t23*t32*t65*t69*t76*4.0;
 Jac[3] = t23*t69*(t60*log(t59)-e1*t65*(t34*t70*log(t33)+e2*t58*t70*t71))*2.0;
 Jac[4] = e1*t23*t65*t69*(t22*t58*t71-e2*t22*t68*(t45*t72*log(t44)+t55*t72*log(t54)))*2.0;
 Jac[5] = e1*t23*t65*t69*(e2*t22*t68*(t15*t19*t21*t35*t73*2.0-t2*t12*t20*t21*t46*t74*2.0)+t9*t13*t22*t24*t76*2.0)*2.0;
 Jac[6] = e1*t23*t65*t69*(e2*t22*t68*(t18*t19*t21*t35*t73*2.0+t6*t12*t20*t21*t46*t74*2.0)+t11*t13*t22*t24*t76*2.0)*-2.0;
 Jac[7] = e1*t23*t65*t69*(e2*t22*t68*(t5*t20*t21*t46*t74*2.0-t3*t12*t19*t21*t35*t73*2.0)+t4*t12*t13*t22*t24*t76*2.0)*2.0;
 Jac[8] = e1*t23*t65*t69*(t13*t19*t22*t24*t76*2.0-t13*t19*t22*t35*t68*t73*2.0)*2.0;
 Jac[9] = e1*t23*t65*t69*(e2*t22*t68*(t20*t21*t46*t74*(pz*t12-t12*z+px*t2*t5+py*t5*t6-t2*t5*x-t5*t6*y)*2.0+t19*t21*t35*t73*(pz*t3*t5-t3*t5*z-px*t2*t3*t12-py*t3*t6*t12+t2*t3*t12*x+t3*t6*t12*y)*2.0)-t13*t22*t24*t76*(pz*t4*t5-t4*t5*z-px*t2*t4*t12-py*t4*t6*t12+t2*t4*t12*x+t4*t6*t12*y)*2.0)*2.0;
  Jac[10] = e1*t23*t65*t69*(e2*t22*t68*(t20*t21*t46*t74*(px*t6*t12-py*t2*t12-t6*t12*x+t2*t12*y)*2.0+t19*t21*t35*t73*(px*t18+py*t15-t18*x-t15*y)*2.0)+t13*t22*t24*t76*(px*t11+py*t9-t11*x-t9*y)*2.0)*2.0;

}

/**
 * @function hess_SQ
 * @brief
 */
void hess_SQ( const SQ_parameters &_par, 
	      const double &x, const double &y, const double &z,
	      double _H[][11] ) {

  double a = _par.dim[0];
  double b = _par.dim[1];
  double c = _par.dim[2];
  double e1 = _par.e[0];
  double e2 = _par.dim[1];
  double px = _par.trans[0];
  double py = _par.trans[1]; 
  double pz = _par.trans[2]; 
  double ra = _par.rot[0]; 
  double pa = _par.rot[1]; 
  double ya = _par.rot[2];

double t2 = cos(ya);
double t3 = sin(ra);
double t4 = cos(ra);
double t5 = sin(pa);
double t6 = sin(ya);
double t7 = t3*t6;
double t8 = t2*t4*t5;
double t9 = t7+t8;
double t10 = t2*t3;
double t26 = t4*t5*t6;
double t11 = t10-t26;
double t12 = cos(pa);
double t25 = px*t9;
double t27 = py*t11;
double t28 = t9*x;
double t29 = t11*y;
double t30 = pz*t4*t12;
double t31 = t4*t12*z;
double t13 = t25-t27-t28+t29+t30-t31;
double t14 = t4*t6;
double t36 = t2*t3*t5;
double t15 = t14-t36;
double t16 = t2*t4;
double t17 = t3*t5*t6;
double t18 = t16+t17;
double t37 = px*t15;
double t38 = py*t18;
double t39 = t15*x;
double t40 = t18*y;
double t41 = pz*t3*t12;
double t42 = t3*t12*z;
double t19 = t37-t38-t39+t40-t41+t42;
double t47 = pz*t5;
double t48 = t5*z;
double t49 = px*t2*t12;
double t50 = t2*t12*x;
double t51 = py*t6*t12;
double t52 = t6*t12*y;
double t20 = t47-t48-t49+t50-t51+t52;
double t21 = 1.0/e2;
double t22 = 1.0/e1;
double t24 = 1.0/(c*c);
double t32 = t13*t13;
double t33 = t24*t32;
double t34 = pow(t33,t22);
double t35 = 1.0/(b*b);
double t43 = t19*t19;
double t44 = t35*t43;
double t45 = pow(t44,t21);
double t46 = 1.0/(a*a);
double t53 = t20*t20;
double t54 = t46*t53;
double t55 = pow(t54,t21);
double t56 = t45+t55;
double t57 = e2*t22;
double t58 = pow(t56,t57);
double t59 = t34+t58;
double t61 = pow(t59,e1);
double t23 = t61-1.0;
double t60 = a*b*c;
double t62 = sqrt(t60);
double t63 = 1.0/(a*a*a*a*a*a);
double t64 = t21-1.0;
double t65 = e1-1.0;
double t66 = pow(t59,t65);
double t67 = t57-1.0;
double t68 = pow(t56,t67);
double t69 = t53*t53;
double t70 = t21*2.0;
double t71 = t70-2.0;
double t72 = pow(t54,t71);
double t73 = e2*t22*2.0;
double t74 = t73-2.0;
double t75 = pow(t56,t74);
double t76 = pow(t54,t64);
double t77 = t23*t23;
double t78 = 1.0/sqrt(t60);
double t79 = c*c;
double t80 = 1.0/pow(t60,3.0/2.0);
double t81 = 1.0/(a*a*a);
double t82 = e1*2.0;
double t83 = t82-2.0;
double t84 = pow(t59,t83);
double t85 = pow(t44,t64);
double t86 = 1.0/(b*b*b);
double t87 = e1-2.0;
double t88 = pow(t59,t87);
double t89 = t57-2.0;
double t90 = pow(t56,t89);
double t91 = b*b;
double t92 = t22-1.0;
double t93 = pow(t33,t92);
double t94 = 1.0/(c*c*c);
double t95 = 1.0/(e1*e1);
double t96 = log(t56);
double t97 = log(t59);
double t98 = log(t33);
double t99 = t34*t95*t98;
double t100 = e2*t58*t95*t96;
double t101 = t99+t100;
double t102 = t61*t97;
double t197 = e1*t66*t101;
double t103 = t102-t197;
double t104 = 1.0/t56;
double t105 = 1.0/(e2*e2);
double t106 = log(t54);
double t107 = log(t44);
double t108 = t45*t105*t107;
double t109 = t55*t105*t106;
double t110 = t108+t109;
double t111 = t22*t58*t96;
double t113 = e2*t22*t68*t110;
double t112 = t111-t113;
double t114 = t21-2.0;
double t115 = pow(t54,t114);
double t116 = t15*t19*t21*t35*t85*2.0;
double t118 = t2*t12*t20*t21*t46*t76*2.0;
double t117 = t116-t118;
double t119 = e2*t22*t68*t117;
double t120 = t9*t13*t22*t24*t93*2.0;
double t121 = t119+t120;
double t122 = 1.0/(a*a*a*a*a);
double t123 = t18*t19*t21*t35*t85*2.0;
double t124 = t6*t12*t20*t21*t46*t76*2.0;
double t125 = t123+t124;
double t126 = e2*t22*t68*t125;
double t127 = t11*t13*t22*t24*t93*2.0;
double t128 = t126+t127;
double t129 = t5*t20*t21*t46*t76*2.0;
double t131 = t3*t12*t19*t21*t35*t85*2.0;
double t130 = t129-t131;
double t132 = e2*t22*t68*t130;
double t133 = t4*t12*t13*t22*t24*t93*2.0;
double t134 = t132+t133;
double t135 = t13*t19*t22*t24*t93*2.0;
double t137 = t13*t19*t22*t35*t68*t85*2.0;
double t136 = t135-t137;
double t138 = pz*t12;
double t139 = px*t2*t5;
double t140 = py*t5*t6;
double t142 = t12*z;
double t143 = t2*t5*x;
double t144 = t5*t6*y;
double t141 = t138+t139+t140-t142-t143-t144;
double t145 = t20*t21*t46*t76*t141*2.0;
double t146 = pz*t3*t5;
double t147 = t2*t3*t12*x;
double t148 = t3*t6*t12*y;
double t152 = t3*t5*z;
double t153 = px*t2*t3*t12;
double t154 = py*t3*t6*t12;
double t149 = t146+t147+t148-t152-t153-t154;
double t150 = t19*t21*t35*t85*t149*2.0;
double t151 = t145+t150;
double t155 = e2*t22*t68*t151;
double t156 = pz*t4*t5;
double t157 = t2*t4*t12*x;
double t158 = t4*t6*t12*y;
double t161 = t4*t5*z;
double t162 = px*t2*t4*t12;
double t163 = py*t4*t6*t12;
double t159 = t156+t157+t158-t161-t162-t163;
double t164 = t13*t22*t24*t93*t159*2.0;
double t160 = t155-t164;
double t165 = t2*t12*y;
double t166 = px*t6*t12;
double t168 = py*t2*t12;
double t169 = t6*t12*x;
double t167 = t165+t166-t168-t169;
double t170 = t20*t21*t46*t76*t167*2.0;
double t171 = px*t18;
double t172 = py*t15;
double t176 = t18*x;
double t177 = t15*y;
double t173 = t171+t172-t176-t177;
double t174 = t19*t21*t35*t85*t173*2.0;
double t175 = t170+t174;
double t178 = e2*t22*t68*t175;
double t179 = px*t11;
double t180 = py*t9;
double t184 = t11*x;
double t185 = t9*y;
double t181 = t179+t180-t184-t185;
double t182 = t13*t22*t24*t93*t181*2.0;
double t183 = t178+t182;
double t186 = c*t77*t78*(1.0/2.0);
double t187 = t43*t53*t62*t75*t76*t81*t84*t85*t86*8.0;
double t188 = t22*t23*t43*t53*t62*t65*t75*t76*t81*t85*t86*t88*8.0;
double t189 = t21*t23*t43*t53*t62*t66*t67*t76*t81*t85*t86*t90*8.0;
double t190 = t186+t187+t188+t189-a*b*t77*t79*t80*(1.0/4.0)-c*t23*t35*t43*t66*t68*t78*t85*2.0-c*t23*t46*t53*t66*t68*t76*t78*2.0;
double t191 = 1.0/(b*b*b*b*b*b);
double t192 = t43*t43;
double t193 = pow(t44,t71);
double t194 = a*a;
double t195 = 1.0/(e1*e1*e1);
double t196 = 1.0/t59;
double t198 = 1.0/(e2*e2*e2);
double t199 = pow(t44,t114);
double t200 = 1.0/(b*b*b*b*b);
double t201 = b*t77*t78*(1.0/2.0);
double t202 = t32*t53*t62*t68*t76*t81*t84*t93*t94*8.0;
double t203 = t22*t23*t32*t53*t62*t65*t68*t76*t81*t88*t93*t94*8.0;
double t204 = t201+t202+t203-a*c*t77*t80*t91*(1.0/4.0)-b*t23*t24*t32*t66*t78*t93*2.0-b*t23*t46*t53*t66*t68*t76*t78*2.0;
double t205 = a*t77*t78*(1.0/2.0);
double t206 = t32*t43*t62*t68*t84*t85*t86*t93*t94*8.0;
double t207 = t22*t23*t32*t43*t62*t65*t68*t85*t86*t88*t93*t94*8.0;
double t208 = t205+t206+t207-b*c*t77*t80*t194*(1.0/4.0)-a*t23*t24*t32*t66*t78*t93*2.0-a*t23*t35*t43*t66*t68*t78*t85*2.0;
double t209 = 1.0/(c*c*c*c*c*c);
double t210 = t32*t32;
double t211 = t22*2.0;
double t212 = t211-2.0;
double t213 = pow(t33,t212);
double t214 = t22-2.0;
double t215 = pow(t33,t214);
double t216 = 1.0/(c*c*c*c*c);
double t217 = b*c*t23*t78*t103;
double t218 = a*c*t23*t78*t103;
double t219 = t66*t97;
double t222 = t65*t88*t101;
double t220 = t219-t222;
double t221 = a*b*t23*t78*t103;
double t223 = 1.0/(e1*e1*e1*e1);
double t224 = t96*t96;
double t225 = e2*e2;
double t226 = b*c*e1*t23*t66*t78*t112;
double t227 = t22*t68*t96;
double t234 = t67*t90*t110;
double t228 = t227-t234;
double t229 = a*c*e1*t23*t66*t78*t112;
double t230 = a*b*e1*t23*t66*t78*t112;
double t231 = t230-e1*t32*t62*t84*t93*t94*t112*4.0-t23*t32*t62*t65*t88*t93*t94*t112*4.0;
double t232 = t58*t95*t96;
double t233 = e1*t62*t66*t103*t112*2.0;
double t235 = 1.0/(e2*e2*e2*e2);
double t236 = t112*t112;
double t237 = e1*e1;
double t238 = b*c*e1*t23*t66*t78*t121;
double t239 = a*c*e1*t23*t66*t78*t121;
double t240 = a*b*e1*t23*t66*t78*t121;
double t241 = t68*t96*t117*t195*t225;
double t242 = t9*t13*t24*t93*t98*t195*2.0;
double t243 = e1*t62*t66*t103*t121*2.0;
double t244 = t15*t19*t35*t85*t107*t198*2.0;
double t245 = t62*t84*t112*t121*t237*2.0;
double t246 = e1*t23*t62*t65*t88*t112*t121*2.0;
double t247 = 1.0/(b*b*b*b);
double t248 = t15*t15;
double t249 = 1.0/(a*a*a*a);
double t250 = t12*t12;
double t251 = t2*t2;
double t252 = 1.0/(c*c*c*c);
double t253 = t9*t9;
double t254 = t121*t121;
double t255 = e1*t53*t62*t68*t76*t81*t84*t128*4.0;
double t256 = t23*t53*t62*t65*t68*t76*t81*t88*t128*4.0;
double t257 = e1*t43*t62*t68*t84*t85*t86*t128*4.0;
double t258 = t23*t43*t62*t65*t68*t85*t86*t88*t128*4.0;
double t259 = e1*t32*t62*t84*t93*t94*t128*4.0;
double t260 = t23*t32*t62*t65*t88*t93*t94*t128*4.0;
double t261 = 1.0/t13;
double t262 = t68*t96*t125*t195*t225;
double t263 = t11*t13*t24*t93*t98*t195*2.0;
double t264 = 1.0/t19;
double t265 = 1.0/t20;
double t266 = t18*t19*t35*t85*t107*t198*2.0;
double t267 = t6*t12*t20*t46*t76*t106*t198*2.0;
double t268 = t15*t18*t21*t35*t85*2.0;
double t269 = t15*t18*t21*t43*t64*t199*t247*4.0;
double t270 = t268+t269-t2*t6*t21*t46*t76*t250*2.0-t2*t6*t21*t53*t64*t115*t249*t250*4.0;
double t271 = e2*t22*t68*t270;
double t272 = t9*t11*t22*t24*t93*2.0;
double t273 = e2*t22*t67*t90*t117*t125;
double t274 = t9*t11*t22*t32*t92*t215*t252*4.0;
double t275 = t271+t272+t273+t274;
double t276 = e1*t23*t62*t66*t275*-2.0-t62*t84*t121*t128*t237*2.0-e1*t23*t62*t65*t88*t121*t128*2.0;
double t277 = t18*t18;
double t278 = t6*t6;
double t279 = t11*t11;
double t280 = t128*t128;
double t281 = b*c*e1*t23*t66*t78*t134;
double t282 = a*c*e1*t23*t66*t78*t134;
double t283 = a*b*e1*t23*t66*t78*t134;
double t284 = t68*t96*t130*t195*t225;
double t285 = t4*t12*t13*t24*t93*t98*t195*2.0;
double t286 = e1*t62*t66*t103*t134*2.0;
double t287 = t5*t20*t46*t76*t106*t198*2.0;
double t288 = t62*t84*t112*t134*t237*2.0;
double t289 = e1*t23*t62*t65*t88*t112*t134*2.0;
double t290 = t62*t84*t121*t134*t237*2.0;
double t291 = t3*t12*t15*t21*t35*t85*2.0;
double t292 = t2*t5*t12*t21*t46*t76*2.0;
double t293 = t2*t5*t12*t21*t53*t64*t115*t249*4.0;
double t294 = t3*t12*t15*t21*t43*t64*t199*t247*4.0;
double t295 = t291+t292+t293+t294;
double t296 = e2*t22*t67*t90*t117*t130;
double t297 = t4*t9*t12*t22*t24*t93*2.0;
double t298 = t4*t9*t12*t22*t32*t92*t215*t252*4.0;
double t299 = t296+t297+t298-e2*t22*t68*t295;
double t300 = e1*t23*t62*t66*t299*2.0;
double t301 = e1*t23*t62*t65*t88*t121*t134*2.0;
double t302 = t290+t300+t301;
double t303 = t5*t6*t12*t21*t46*t76*2.0;
double t304 = t5*t6*t12*t21*t53*t64*t115*t249*4.0;
double t305 = e2*t22*t68*(t303+t304-t3*t12*t18*t21*t35*t85*2.0-t3*t12*t18*t21*t43*t64*t199*t247*4.0);
double t306 = e2*t22*t67*t90*t125*t130;
double t307 = t4*t11*t12*t22*t24*t93*2.0;
double t308 = t4*t11*t12*t22*t32*t92*t215*t252*4.0;
double t309 = t305+t306+t307+t308;
double t310 = e1*t23*t62*t66*t309*-2.0-t62*t84*t128*t134*t237*2.0-e1*t23*t62*t65*t88*t128*t134*2.0;
double t311 = t5*t5;
double t312 = t3*t3;
double t313 = t4*t4;
double t314 = t134*t134;
double t315 = b*c*e1*t23*t66*t78*t136;
double t316 = t13*t19*t21*t23*t35*t53*t62*t66*t67*t76*t81*t85*t90*8.0;
double t317 = t315+t316-e1*t53*t62*t68*t76*t81*t84*t136*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t136*4.0;
double t318 = a*c*e1*t23*t66*t78*t136;
double t319 = a*b*e1*t23*t66*t78*t136;
double t320 = t13*t19*t24*t93*t98*t195*2.0;
double t321 = e1*t62*t66*t103*t136*2.0;
double t322 = t62*t84*t112*t136*t237*2.0;
double t323 = e1*t23*t62*t65*t88*t112*t136*2.0;
double t324 = t13*t19*t22*t35*t67*t85*t90*t117*2.0;
double t325 = t62*t84*t121*t136*t237*2.0;
double t326 = e1*t23*t62*t65*t88*t121*t136*2.0;
double t327 = t13*t19*t22*t35*t67*t85*t90*t125*2.0;
double t328 = t4*t12*t19*t22*t24*t93*2.0;
double t329 = t4*t12*t19*t22*t32*t92*t215*t252*4.0;
double t330 = t62*t84*t134*t136*t237*2.0;
double t331 = e1*t23*t62*t65*t88*t134*t136*2.0;
double t332 = t136*t136;
double t333 = b*c*e1*t23*t66*t78*t160;
double t334 = a*c*e1*t23*t66*t78*t160;
double t335 = a*b*e1*t23*t66*t78*t160;
double t336 = t68*t96*t151*t195*t225;
double t337 = e1*t62*t66*t103*t160*2.0;
double t338 = t62*t84*t112*t160*t237*2.0;
double t339 = t19*t35*t85*t107*t149*t198*2.0;
double t340 = t20*t46*t76*t106*t141*t198*2.0;
double t341 = e1*t23*t62*t65*t88*t112*t160*2.0;
double t342 = t15*t21*t35*t85*t149*2.0;
double t343 = t2*t5*t20*t21*t46*t76*2.0;
double t344 = t15*t21*t43*t64*t149*t199*t247*4.0;
double t345 = t342+t343+t344-t2*t12*t21*t46*t76*t141*2.0-t2*t3*t12*t19*t21*t35*t85*2.0-t2*t12*t21*t53*t64*t115*t141*t249*4.0;
double t346 = e2*t22*t68*t345;
double t347 = e2*t22*t67*t90*t117*t151;
double t348 = t2*t4*t12*t13*t22*t24*t93*2.0;
double t349 = t346+t347+t348-t9*t22*t24*t93*t159*2.0-t9*t22*t32*t92*t159*t215*t252*4.0;
double t350 = e1*t23*t62*t66*t349*2.0;
double t351 = t62*t84*t121*t160*t237*2.0;
double t352 = e1*t23*t62*t65*t88*t121*t160*2.0;
double t353 = t350+t351+t352;
double t354 = t18*t21*t35*t85*t149*2.0;
double t355 = t6*t12*t21*t46*t76*t141*2.0;
double t356 = t18*t21*t43*t64*t149*t199*t247*4.0;
double t357 = t3*t6*t12*t19*t21*t35*t85*2.0;
double t358 = t6*t12*t21*t53*t64*t115*t141*t249*4.0;
double t359 = t354+t355+t356+t357+t358-t5*t6*t20*t21*t46*t76*2.0;
double t360 = t11*t22*t24*t93*t159*2.0;
double t361 = t4*t6*t12*t13*t22*t24*t93*2.0;
double t362 = t11*t22*t32*t92*t159*t215*t252*4.0;
double t363 = t360+t361+t362-e2*t22*t68*t359-e2*t22*t67*t90*t125*t151;
double t364 = e1*t23*t62*t66*t363*2.0;
double t365 = t364-t62*t84*t128*t160*t237*2.0-e1*t23*t62*t65*t88*t128*t160*2.0;
double t366 = t62*t84*t134*t160*t237*2.0;
double t367 = t12*t20*t21*t46*t76*2.0;
double t368 = t5*t21*t46*t76*t141*2.0;
double t369 = t3*t5*t19*t21*t35*t85*2.0;
double t370 = t5*t21*t53*t64*t115*t141*t249*4.0;
double t371 = t367+t368+t369+t370-t3*t12*t21*t35*t85*t149*2.0-t3*t12*t21*t43*t64*t149*t199*t247*4.0;
double t372 = t4*t5*t13*t22*t24*t93*2.0;
double t373 = t4*t12*t22*t24*t93*t159*2.0;
double t374 = t4*t12*t22*t32*t92*t159*t215*t252*4.0;
double t375 = t372+t373+t374-e2*t22*t68*t371-e2*t22*t67*t90*t130*t151;
double t376 = e1*t23*t62*t65*t88*t134*t160*2.0;
double t377 = t366+t376-e1*t23*t62*t66*t375*2.0;
double t378 = t62*t84*t136*t160*t237*2.0;
double t379 = t19*t22*t24*t93*t159*2.0;
double t380 = t19*t22*t32*t92*t159*t215*t252*4.0;
double t381 = t13*t19*t22*t35*t67*t85*t90*t151*2.0;
double t382 = e1*t23*t62*t65*t88*t136*t160*2.0;
double t383 = t149*t149;
double t384 = t141*t141;
double t385 = t159*t159;
double t386 = t160*t160;
double t387 = a*c*e1*t23*t66*t78*(t178+t182);
double t388 = a*b*e1*t23*t66*t78*(t178+t182);
double t389 = t13*t24*t93*t98*t181*t195*2.0;
double t390 = t68*t96*t175*t195*t225;
double t391 = e1*t62*t66*t103*(t178+t182)*2.0;
double t392 = t62*t84*t112*t237*(t178+t182)*2.0;
double t393 = t19*t35*t85*t107*t173*t198*2.0;
double t394 = t20*t46*t76*t106*t167*t198*2.0;
double t395 = e1*t23*t62*t65*t88*t112*(t178+t182)*2.0;
double t396 = t62*t84*t121*t237*(t178+t182)*2.0;
double t397 = t15*t21*t35*t85*t173*2.0;
double t398 = t15*t21*t43*t64*t173*t199*t247*4.0;
double t399 = t123+t124+t397+t398-t2*t12*t21*t46*t76*t167*2.0-t2*t12*t21*t53*t64*t115*t167*t249*4.0;
double t400 = e2*t22*t68*t399;
double t401 = t9*t22*t24*t93*t181*2.0;
double t402 = e2*t22*t67*t90*t117*(t170+t174);
double t403 = t9*t22*t32*t92*t181*t215*t252*4.0;
double t404 = t127+t400+t401+t402+t403;
double t405 = e1*t23*t62*t66*t404*2.0;
double t406 = e1*t23*t62*t65*t88*t121*(t178+t182)*2.0;
double t407 = t396+t405+t406;
double t408 = t18*t21*t35*t85*t173*2.0;
double t409 = t18*t21*t43*t64*t173*t199*t247*4.0;
double t410 = t11*t22*t24*t93*t181*2.0;
double t411 = t11*t22*t32*t92*t181*t215*t252*4.0;
double t412 = t6*t12*t21*t46*t76*t167*2.0;
double t413 = t6*t12*t21*t53*t64*t115*t167*t249*4.0;
double t414 = -t116+t118+t408+t409+t412+t413;
double t415 = e2*t22*t68*t414;
double t416 = e2*t22*t67*t90*t125*t175;
double t417 = -t120+t410+t411+t415+t416;
double t418 = e1*t23*t62*t66*t417*-2.0-t62*t84*t128*t183*t237*2.0-e1*t23*t62*t65*t88*t128*t183*2.0;
double t419 = t62*t84*t134*t237*(t178+t182)*2.0;
double t420 = t5*t21*t46*t76*t167*2.0;
double t421 = t5*t21*t53*t64*t115*t167*t249*4.0;
double t422 = t420+t421-t3*t12*t21*t35*t85*t173*2.0-t3*t12*t21*t43*t64*t173*t199*t247*4.0;
double t423 = e2*t22*t68*t422;
double t424 = t4*t12*t22*t24*t93*t181*2.0;
double t425 = e2*t22*t67*t90*t130*t175;
double t426 = t4*t12*t22*t32*t92*t181*t215*t252*4.0;
double t427 = e1*t23*t62*t66*(t423+t424+t425+t426)*2.0;
double t428 = e1*t23*t62*t65*t88*t134*(t178+t182)*2.0;
double t429 = t419+t427+t428;
double t430 = t13*t22*t24*t93*t173*2.0;
double t431 = t19*t22*t24*t93*t181*2.0;
double t432 = t19*t22*t32*t92*t181*t215*t252*4.0;
double t433 = t62*t84*t136*t237*(t178+t182)*2.0;
double t434 = e1*t23*t62*t65*t88*t136*(t178+t182)*2.0;
double t435 = t62*t84*t160*t237*(t178+t182)*2.0;
double t436 = t21*t35*t85*t149*t173*2.0;
double t437 = t2*t5*y;
double t438 = px*t5*t6;
double t439 = t437+t438-py*t2*t5-t5*t6*x;
double t440 = t2*t3*t12*y;
double t441 = px*t3*t6*t12;
double t442 = t19*t21*t35*t85*(t440+t441-py*t2*t3*t12-t3*t6*t12*x)*2.0;
double t443 = t21*t43*t64*t149*t173*t199*t247*4.0;
double t444 = t21*t46*t76*t141*t167*2.0;
double t445 = t21*t53*t64*t115*t141*t167*t249*4.0;
double t446 = t436+t442+t443+t444+t445-t20*t21*t46*t76*t439*2.0;
double t447 = t2*t4*t12*y;
double t448 = px*t4*t6*t12;
double t449 = t13*t22*t24*t93*(t447+t448-py*t2*t4*t12-t4*t6*t12*x)*2.0;
double t450 = t22*t24*t93*t159*t181*2.0;
double t451 = t22*t32*t92*t159*t181*t215*t252*4.0;
double t452 = t449+t450+t451-e2*t22*t68*t446-e2*t22*t67*t90*t151*t175;
double t453 = e1*t23*t62*t65*t88*t160*(t178+t182)*2.0;
double t454 = t435+t453-e1*t23*t62*t66*t452*2.0;
double t455 = t178+t182;
double t456 = t165+t166-t168-t169;
double t457 = t165+t166-t168-t169;
double t458 = t173*t173;
double t459 = t170+t174;
double t460 = t181*t181;
double t461 = t178+t182;
  _H[0][0] = t77*t79*t80*t91*(-1.0/4.0)+t62*t63*t69*t72*t75*t84*8.0+t23*t53*t62*t66*t68*t76*t249*1.2E1+t23*t62*t63*t64*t66*t68*t69*t115*8.0-b*c*t23*t53*t66*t68*t76*t78*t81*4.0+t21*t23*t62*t63*t66*t67*t69*t72*t90*8.0+t22*t23*t62*t63*t65*t69*t72*t75*t88*8.0;
  _H[0][1] = t190;
  _H[0][2] = t204;
  _H[0][3] = t217+t23*t62*(e1*t66*(t53*t58*t76*t81*t95*t104*2.0+e2*t53*t68*t76*t81*t96*t195*2.0)-t53*t66*t68*t76*t81*t97*2.0+t53*t65*t68*t76*t81*t88*t101*2.0-t22*t53*t61*t68*t76*t81*t196*2.0)*2.0-t53*t62*t66*t68*t76*t81*t103*4.0;
  _H[0][4] = t226+e1*t23*t62*t66*(e2*t22*t68*((t55*t105*2.0)/a+t53*t76*t81*t106*t198*2.0)-t53*t68*t76*t81*t95*t96*2.0-t21*t22*t53*t58*t76*t81*t104*2.0+t22*t53*t67*t76*t81*t90*t110*2.0)*2.0-e1*t53*t62*t68*t76*t81*t84*t112*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t112*4.0;
  _H[0][5] = t238+e1*t23*t62*t66*(e2*t22*t68*(t2*t12*t20*t21*t76*t81*4.0+t2*t12*t20*t21*t53*t64*t115*t122*4.0)-t22*t53*t67*t76*t81*t90*t117*2.0)*2.0-e1*t53*t62*t68*t76*t81*t84*t121*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t121*4.0;
  _H[0][6] = t255+t256+e1*t23*t62*t66*(e2*t22*t68*(t6*t12*t20*t21*t76*t81*4.0+t6*t12*t20*t21*t53*t64*t115*t122*4.0)+t22*t53*t67*t76*t81*t90*t125*2.0)*2.0-b*c*e1*t23*t66*t78*t128;
  _H[0][7] = t281-e1*t23*t62*t66*(e2*t22*t68*(t5*t20*t21*t76*t81*4.0+t5*t20*t21*t53*t64*t115*t122*4.0)+t22*t53*t67*t76*t81*t90*t130*2.0)*2.0-e1*t53*t62*t68*t76*t81*t84*t134*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t134*4.0;
  _H[0][8] = t317;
  _H[0][9] = t333-e1*t23*t62*t66*(e2*t22*t68*(t20*t21*t76*t81*t141*4.0+t20*t21*t53*t64*t115*t122*t141*4.0)+t22*t53*t67*t76*t81*t90*t151*2.0)*2.0-e1*t53*t62*t68*t76*t81*t84*t160*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t160*4.0;
  _H[0][10] = e1*t23*t62*t66*(e2*t22*t68*(t20*t21*t76*t81*t167*4.0+t20*t21*t53*t64*t115*t122*t167*4.0)+t22*t53*t67*t76*t81*t90*t175*2.0)*-2.0+b*c*e1*t23*t66*t78*t183-e1*t53*t62*t68*t76*t81*t84*t183*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t183*4.0;
  _H[1][0] = t190;
  _H[1][1] = t77*t79*t80*t194*(-1.0/4.0)+t62*t75*t84*t191*t192*t193*8.0+t23*t43*t62*t66*t68*t85*t247*1.2E1+t23*t62*t64*t66*t68*t191*t192*t199*8.0-a*c*t23*t43*t66*t68*t78*t85*t86*4.0+t21*t23*t62*t66*t67*t90*t191*t192*t193*8.0+t22*t23*t62*t65*t75*t88*t191*t192*t193*8.0;
  _H[1][2] = t208;
  _H[1][3] = t218+t23*t62*(e1*t66*(t43*t58*t85*t86*t95*t104*2.0+e2*t43*t68*t85*t86*t96*t195*2.0)-t43*t66*t68*t85*t86*t97*2.0+t43*t65*t68*t85*t86*t88*t101*2.0-t22*t43*t61*t68*t85*t86*t196*2.0)*2.0-t43*t62*t66*t68*t85*t86*t103*4.0;
  _H[1][4] = t229+e1*t23*t62*t66*(e2*t22*t68*((t45*t105*2.0)/b+t43*t85*t86*t107*t198*2.0)-t43*t68*t85*t86*t95*t96*2.0-t21*t22*t43*t58*t85*t86*t104*2.0+t22*t43*t67*t85*t86*t90*t110*2.0)*2.0-e1*t43*t62*t68*t84*t85*t86*t112*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t112*4.0;
  _H[1][5] = t239-e1*t23*t62*t66*(e2*t22*t68*(t15*t19*t21*t85*t86*4.0+t15*t19*t21*t43*t64*t199*t200*4.0)+t22*t43*t67*t85*t86*t90*t117*2.0)*2.0-e1*t43*t62*t68*t84*t85*t86*t121*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t121*4.0;
  _H[1][6] = t257+t258+e1*t23*t62*t66*(e2*t22*t68*(t18*t19*t21*t85*t86*4.0+t18*t19*t21*t43*t64*t199*t200*4.0)+t22*t43*t67*t85*t86*t90*t125*2.0)*2.0-a*c*e1*t23*t66*t78*t128;
  _H[1][7] = t282+e1*t23*t62*t66*(e2*t22*t68*(t3*t12*t19*t21*t85*t86*4.0+t3*t12*t19*t21*t43*t64*t199*t200*4.0)-t22*t43*t67*t85*t86*t90*t130*2.0)*2.0-e1*t43*t62*t68*t84*t85*t86*t134*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t134*4.0;
  _H[1][8] = t318+e1*t23*t62*t66*(t13*t19*t22*t68*t85*t86*4.0+t13*t19*t22*t43*t64*t68*t199*t200*4.0+t13*t19*t21*t22*t43*t67*t90*t193*t200*4.0)*2.0-e1*t43*t62*t68*t84*t85*t86*t136*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t136*4.0;
  _H[1][9] = t334-e1*t23*t62*t66*(e2*t22*t68*(t19*t21*t85*t86*t149*4.0+t19*t21*t43*t64*t149*t199*t200*4.0)+t22*t43*t67*t85*t86*t90*t151*2.0)*2.0-e1*t43*t62*t68*t84*t85*t86*t160*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t160*4.0;
  _H[1][10] = t387-e1*t23*t62*t66*(e2*t22*t68*(t19*t21*t85*t86*t173*4.0+t19*t21*t43*t64*t173*t199*t200*4.0)+t22*t43*t67*t85*t86*t90*t175*2.0)*2.0-e1*t43*t62*t68*t84*t85*t86*t183*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t183*4.0;
  _H[2][0] = t204;
  _H[2][1] = t208;
  _H[2][2] = t77*t80*t91*t194*(-1.0/4.0)+t62*t84*t209*t210*t213*8.0+t23*t32*t62*t66*t93*t252*1.2E1+t23*t62*t66*t92*t209*t210*t215*8.0-a*b*t23*t32*t66*t78*t93*t94*4.0+t22*t23*t62*t65*t88*t209*t210*t213*8.0;
  _H[2][3] = t221+t23*t62*(e1*t66*((t34*t95*2.0)/c+t32*t93*t94*t98*t195*2.0)-t32*t66*t93*t94*t97*2.0+t32*t65*t88*t93*t94*t101*2.0-t22*t32*t61*t93*t94*t196*2.0)*2.0-t32*t62*t66*t93*t94*t103*4.0;
  _H[2][4] = t231;
  _H[2][5] = t240-e1*t23*t62*t66*(t9*t13*t22*t93*t94*4.0+t9*t13*t22*t32*t92*t215*t216*4.0)*2.0-e1*t32*t62*t84*t93*t94*t121*4.0-t23*t32*t62*t65*t88*t93*t94*t121*4.0;
  _H[2][6] = t259+t260+e1*t23*t62*t66*(t11*t13*t22*t93*t94*4.0+t11*t13*t22*t32*t92*t215*t216*4.0)*2.0-a*b*e1*t23*t66*t78*t128;
  _H[2][7] = t283-e1*t23*t62*t66*(t4*t12*t13*t22*t93*t94*4.0+t4*t12*t13*t22*t32*t92*t215*t216*4.0)*2.0-e1*t32*t62*t84*t93*t94*t134*4.0-t23*t32*t62*t65*t88*t93*t94*t134*4.0;
  _H[2][8] = t319-e1*t23*t62*t66*(t13*t19*t22*t93*t94*4.0+t13*t19*t22*t32*t92*t215*t216*4.0)*2.0-e1*t32*t62*t84*t93*t94*t136*4.0-t23*t32*t62*t65*t88*t93*t94*t136*4.0;
  _H[2][9] = t335+e1*t23*t62*t66*(t13*t22*t93*t94*t159*4.0+t13*t22*t32*t92*t159*t215*t216*4.0)*2.0-e1*t32*t62*t84*t93*t94*t160*4.0-t23*t32*t62*t65*t88*t93*t94*t160*4.0;
  _H[2][10] = t388-e1*t23*t62*t66*(t13*t22*t93*t94*t181*4.0+t13*t22*t32*t92*t181*t215*t216*4.0)*2.0-e1*t32*t62*t84*t93*t94*t183*4.0-t23*t32*t62*t65*t88*t93*t94*t183*4.0;
  _H[3][0] = t217-t53*t62*t66*t68*t76*t81*t103*4.0-t23*t53*t62*t68*t76*t81*t220*4.0+e2*t23*t53*t62*t66*t68*t76*t81*t95*t96*4.0;
  _H[3][1] = t218-t43*t62*t66*t68*t85*t86*t103*4.0-t23*t43*t62*t68*t85*t86*t220*4.0+e2*t23*t43*t62*t66*t68*t85*t86*t95*t96*4.0;
  _H[3][2] = t221-t32*t62*t66*t93*t94*t103*4.0-t23*t32*t62*t93*t94*t220*4.0+t23*t32*t62*t66*t93*t94*t95*t98*4.0;
  _H[3][3] = t62*(t103*t103)*2.0-t23*t62*(t66*t101-t97*t103+e1*t101*t220+t61*t101*t196-e1*t66*(t34*(t98*t98)*t223+t34*t98*t195*2.0+e2*t58*t96*t195*2.0+t58*t223*t224*t225))*2.0;
  _H[3][4] = t233+t23*t62*t66*t112*2.0+e1*t23*t62*t112*t220*2.0-e1*t23*t62*t66*(t232-e2*t68*t95*t110+e2*t58*t195*t224-t68*t96*t110*t195*t225)*2.0;
  _H[3][5] = t243+t23*t62*t66*t121*2.0-e1*t23*t62*t66*(t241+t242+e2*t68*t95*t117+t9*t13*t24*t93*t95*2.0)*2.0+e1*t23*t62*t121*t220*2.0;
  _H[3][6] = t23*t62*t66*t128*-2.0+e1*t23*t62*t66*(t262+t263+e2*t68*t95*t125+t11*t13*t24*t93*t95*2.0)*2.0-e1*t62*t66*t103*t128*2.0-e1*t23*t62*t128*t220*2.0;
  _H[3][7] = t286+t23*t62*t66*t134*2.0+e1*t23*t62*t134*t220*2.0-e1*t23*t62*t66*(t284+t285+e2*t68*t95*t130+t4*t12*t13*t24*t93*t95*2.0)*2.0;
  _H[3][8] = t321+t23*t62*t66*t136*2.0+e1*t23*t62*t136*t220*2.0-e1*t23*t62*t66*(t320+t13*t19*t24*t93*t95*2.0-t13*t19*t35*t68*t85*t95*2.0-e2*t13*t19*t35*t68*t85*t96*t195*2.0)*2.0;
  _H[3][9] = t337+t23*t62*t66*t160*2.0+e1*t23*t62*t160*t220*2.0-e1*t23*t62*t66*(t336+e2*t68*t95*t151-t13*t24*t93*t95*t159*2.0-t13*t24*t93*t98*t159*t195*2.0)*2.0;
  _H[3][10] = t391+t23*t62*t66*(t178+t182)*2.0-e1*t23*t62*t66*(t389+t390+e2*t68*t95*t175+t13*t24*t93*t95*t181*2.0)*2.0+e1*t23*t62*t220*(t178+t182)*2.0;
  _H[4][0] = t226-t23*t53*t62*t66*t76*t81*t228*4.0-e1*t53*t62*t68*t76*t81*t84*t112*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t112*4.0+t23*t53*t62*t66*t68*t76*t81*t105*t106*4.0;
  _H[4][1] = t229-t23*t43*t62*t66*t85*t86*t228*4.0-e1*t43*t62*t68*t84*t85*t86*t112*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t112*4.0+t23*t43*t62*t66*t68*t85*t86*t105*t107*4.0;
  _H[4][2] = t231;
  _H[4][3] = t233+t23*t62*(t61*t112*t196-e1*t66*(t232+e2*t95*t96*t112-e2*t58*t95*t104*t110)+e1*t66*t97*t112-e1*t65*t88*t101*t112)*2.0;
  _H[4][4] = t62*t84*t236*t237*2.0-e1*t23*t62*t66*(t22*t68*t110-t22*t96*t112+e2*t22*t110*t228+t22*t58*t104*t110-e2*t22*t68*(t45*(t107*t107)*t235+t55*(t106*t106)*t235+t45*t107*t198*2.0+t55*t106*t198*2.0))*2.0+e1*t23*t62*t65*t88*t236*2.0;
  _H[4][5] = t245+t246+e1*t23*t62*t66*(t22*t68*t117+e2*t22*t117*t228-e2*t22*t68*(t244+t15*t19*t35*t85*t105*2.0-t2*t12*t20*t46*t76*t105*2.0-t2*t12*t20*t46*t76*t106*t198*2.0))*2.0;
  _H[4][6] = e1*t23*t62*t66*(t22*t68*t125+e2*t22*t125*t228-e2*t22*t68*(t266+t267+t18*t19*t35*t85*t105*2.0+t6*t12*t20*t46*t76*t105*2.0))*-2.0-t62*t84*t112*t128*t237*2.0-e1*t23*t62*t65*t88*t112*t128*2.0;
  _H[4][7] = t288+t289+e1*t23*t62*t66*(t22*t68*t130+e2*t22*t130*t228-e2*t22*t68*(t287+t5*t20*t46*t76*t105*2.0-t3*t12*t19*t35*t85*t105*2.0-t3*t12*t19*t35*t85*t107*t198*2.0))*2.0;
  _H[4][8] = t322+t323-e1*t23*t62*t66*(t13*t19*t22*t35*t85*t228*2.0-t13*t19*t22*t35*t68*t85*t105*t107*2.0)*2.0;
  _H[4][9] = t338+t341+e1*t23*t62*t66*(t22*t68*t151+e2*t22*t151*t228-e2*t22*t68*(t339+t340+t20*t46*t76*t105*t141*2.0+t19*t35*t85*t105*t149*2.0))*2.0;
  _H[4][10] = t392+t395+e1*t23*t62*t66*(t22*t68*(t170+t174)+e2*t22*(t170+t174)*(t227-t234)-e2*t22*t68*(t393+t394+t20*t46*t76*t105*t167*2.0+t19*t35*t85*t105*t173*2.0))*2.0;
  _H[5][0] = t238-e1*t53*t62*t68*t76*t81*t84*t121*4.0+t2*t12*t20*t23*t62*t66*t68*t76*t81*8.0-t23*t53*t62*t66*t67*t76*t81*t90*t117*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t121*4.0+t2*t12*t20*t23*t53*t62*t64*t66*t68*t115*t122*8.0;
  _H[5][1] = t239-e1*t43*t62*t68*t84*t85*t86*t121*4.0-t15*t19*t23*t62*t66*t68*t85*t86*8.0-t23*t43*t62*t66*t67*t85*t86*t90*t117*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t121*4.0-t15*t19*t23*t43*t62*t64*t66*t68*t199*t200*8.0;
  _H[5][2] = t240-e1*t32*t62*t84*t93*t94*t121*4.0-t9*t13*t23*t62*t66*t93*t94*8.0-t23*t32*t62*t65*t88*t93*t94*t121*4.0-t9*t13*t23*t32*t62*t66*t92*t215*t216*8.0;
  _H[5][3] = t243-t23*t62*(e1*t66*(t241+t242+t9*t34*t95*t261*2.0+e2*t58*t95*t104*t117)-t61*t121*t196-e1*t66*t97*t121+e1*t65*t88*t101*t121)*2.0;
  _H[5][4] = t245+t246+e1*t23*t62*t66*(t22*t58*t104*t117-e2*t22*t68*(t244+t15*t45*t105*t264*2.0-t2*t12*t55*t105*t265*2.0-t2*t12*t20*t46*t76*t106*t198*2.0)+e2*t68*t95*t96*t117-e2*t22*t67*t90*t110*t117)*2.0;
  _H[5][5] = t62*t84*t237*t254*2.0+e1*t23*t62*t66*(t22*t24*t93*t253*2.0+e2*t22*t68*(t21*t35*t85*t248*2.0+t21*t46*t76*t250*t251*2.0+t21*t43*t64*t199*t247*t248*4.0+t21*t53*t64*t115*t249*t250*t251*4.0)+e2*t22*t67*t90*(t117*t117)+t22*t32*t92*t215*t252*t253*4.0)*2.0+e1*t23*t62*t65*t88*t254*2.0;
  _H[5][6] = t276;
  _H[5][7] = t302;
  _H[5][8] = t325+t326-e1*t23*t62*t66*(t324-t9*t19*t22*t24*t93*2.0-t13*t15*t22*t24*t93*2.0+t9*t19*t22*t35*t68*t85*2.0+t13*t15*t22*t35*t68*t85*2.0-t9*t19*t22*t32*t92*t215*t252*4.0+t13*t15*t22*t43*t64*t68*t199*t247*4.0)*2.0;
  _H[5][9] = t353;
  _H[5][10] = t407;
  _H[6][0] = t255+t256-b*c*e1*t23*t66*t78*t128+t6*t12*t20*t23*t62*t66*t68*t76*t81*8.0+t23*t53*t62*t66*t67*t76*t81*t90*t125*4.0+t6*t12*t20*t23*t53*t62*t64*t66*t68*t115*t122*8.0;
  _H[6][1] = t257+t258-a*c*e1*t23*t66*t78*t128+t18*t19*t23*t62*t66*t68*t85*t86*8.0+t23*t43*t62*t66*t67*t85*t86*t90*t125*4.0+t18*t19*t23*t43*t62*t64*t66*t68*t199*t200*8.0;
  _H[6][2] = t259+t260-a*b*e1*t23*t66*t78*t128+t11*t13*t23*t62*t66*t93*t94*8.0+t11*t13*t23*t32*t62*t66*t92*t215*t216*8.0;
  _H[6][3] = t23*t62*(e1*t66*(t262+t263+t11*t34*t95*t261*2.0+e2*t58*t95*t104*t125)-t61*t128*t196-e1*t66*t97*t128+e1*t65*t88*t101*t128)*2.0-e1*t62*t66*t103*t128*2.0;
  _H[6][4] = e1*t23*t62*t66*(t22*t58*t104*t125-e2*t22*t68*(t266+t267+t18*t45*t105*t264*2.0+t6*t12*t55*t105*t265*2.0)+e2*t68*t95*t96*t125-e2*t22*t67*t90*t110*t125)*-2.0-t62*t84*t112*t128*t237*2.0-e1*t23*t62*t65*t88*t112*t128*2.0;
  _H[6][5] = t276;
  _H[6][6] = t62*t84*t237*t280*2.0+e1*t23*t62*t66*(t22*t24*t93*t279*2.0+e2*t22*t68*(t21*t35*t85*t277*2.0+t21*t46*t76*t250*t278*2.0+t21*t43*t64*t199*t247*t277*4.0+t21*t53*t64*t115*t249*t250*t278*4.0)+e2*t22*t67*t90*(t125*t125)+t22*t32*t92*t215*t252*t279*4.0)*2.0+e1*t23*t62*t65*t88*t280*2.0;
  _H[6][7] = t310;
  _H[6][8] = t62*t84*t128*t136*t237*-2.0+e1*t23*t62*t66*(t327-t11*t19*t22*t24*t93*2.0-t13*t18*t22*t24*t93*2.0+t11*t19*t22*t35*t68*t85*2.0+t13*t18*t22*t35*t68*t85*2.0-t11*t19*t22*t32*t92*t215*t252*4.0+t13*t18*t22*t43*t64*t68*t199*t247*4.0)*2.0-e1*t23*t62*t65*t88*t128*t136*2.0;
  _H[6][9] = t365;
  _H[6][10] = t418;
  _H[7][0] = t281-e1*t53*t62*t68*t76*t81*t84*t134*4.0-t5*t20*t23*t62*t66*t68*t76*t81*8.0-t23*t53*t62*t66*t67*t76*t81*t90*t130*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t134*4.0-t5*t20*t23*t53*t62*t64*t66*t68*t115*t122*8.0;
  _H[7][1] = t282-e1*t43*t62*t68*t84*t85*t86*t134*4.0+t3*t12*t19*t23*t62*t66*t68*t85*t86*8.0-t23*t43*t62*t66*t67*t85*t86*t90*t130*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t134*4.0+t3*t12*t19*t23*t43*t62*t64*t66*t68*t199*t200*8.0;
  _H[7][2] = t283-e1*t32*t62*t84*t93*t94*t134*4.0-t4*t12*t13*t23*t62*t66*t93*t94*8.0-t23*t32*t62*t65*t88*t93*t94*t134*4.0-t4*t12*t13*t23*t32*t62*t66*t92*t215*t216*8.0;
  _H[7][3] = t286-t23*t62*(e1*t66*(t284+t285+e2*t58*t95*t104*t130+t4*t12*t34*t95*t261*2.0)-t61*t134*t196-e1*t66*t97*t134+e1*t65*t88*t101*t134)*2.0;
  _H[7][4] = t288+t289+e1*t23*t62*t66*(t22*t58*t104*t130-e2*t22*t68*(t287+t5*t55*t105*t265*2.0-t3*t12*t45*t105*t264*2.0-t3*t12*t19*t35*t85*t107*t198*2.0)+e2*t68*t95*t96*t130-e2*t22*t67*t90*t110*t130)*2.0;
  _H[7][5] = t302;
  _H[7][6] = t310;
  _H[7][7] = t62*t84*t237*t314*2.0+e1*t23*t62*t66*(e2*t22*t68*(t21*t46*t76*t311*2.0+t21*t35*t85*t250*t312*2.0+t21*t53*t64*t115*t249*t311*4.0+t21*t43*t64*t199*t247*t250*t312*4.0)+t22*t24*t93*t250*t313*2.0+e2*t22*t67*t90*(t130*t130)+t22*t32*t92*t215*t250*t252*t313*4.0)*2.0+e1*t23*t62*t65*t88*t314*2.0;
  _H[7][8] = t330+t331+e1*t23*t62*t66*(t328+t329-t3*t12*t13*t22*t24*t93*2.0+t3*t12*t13*t22*t35*t68*t85*2.0-t4*t12*t19*t22*t35*t68*t85*2.0-t13*t19*t22*t35*t67*t85*t90*t130*2.0+t3*t12*t13*t22*t43*t64*t68*t199*t247*4.0)*2.0;
  _H[7][9] = t377;
  _H[7][10] = t429;
  _H[8][0] = t317;
  _H[8][1] = t318-e1*t43*t62*t68*t84*t85*t86*t136*4.0+t13*t19*t23*t62*t66*t68*t85*t86*8.0-t23*t43*t62*t65*t68*t85*t86*t88*t136*4.0+t13*t19*t23*t43*t62*t64*t66*t68*t199*t200*8.0+t13*t19*t21*t23*t43*t62*t66*t67*t90*t193*t200*8.0;
  _H[8][2] = t319-e1*t32*t62*t84*t93*t94*t136*4.0-t13*t19*t23*t62*t66*t93*t94*8.0-t23*t32*t62*t65*t88*t93*t94*t136*4.0-t13*t19*t23*t32*t62*t66*t92*t215*t216*8.0;
  _H[8][3] = t321-t23*t62*(e1*t66*(t320+t19*t34*t95*t261*2.0-t13*t19*t35*t58*t85*t95*t104*2.0-e2*t13*t19*t35*t68*t85*t96*t195*2.0)-t61*t136*t196-e1*t66*t97*t136+e1*t65*t88*t101*t136)*2.0;
  _H[8][4] = t322+t323+e1*t23*t62*t66*(e2*t22*t68*(t13*t45*t105*t264*2.0+t13*t19*t35*t85*t107*t198*2.0)-t13*t19*t35*t68*t85*t95*t96*2.0-t13*t19*t21*t22*t35*t58*t85*t104*2.0+t13*t19*t22*t35*t67*t85*t90*t110*2.0)*2.0;
  _H[8][5] = t325+t326+e1*t23*t62*t66*(-t324-e2*t22*t68*(t9*t19*t21*t35*t85*2.0+t13*t15*t21*t35*t85*2.0+t13*t15*t21*t43*t64*t199*t247*4.0)+t9*t19*t22*t24*t93*2.0+t13*t15*t22*t24*t93*2.0+t9*t19*t22*t32*t92*t215*t252*4.0)*2.0;
  _H[8][6] = e1*t23*t62*t66*(-t327-e2*t22*t68*(t11*t19*t21*t35*t85*2.0+t13*t18*t21*t35*t85*2.0+t13*t18*t21*t43*t64*t199*t247*4.0)+t11*t19*t22*t24*t93*2.0+t13*t18*t22*t24*t93*2.0+t11*t19*t22*t32*t92*t215*t252*4.0)*-2.0-t62*t84*t128*t136*t237*2.0-e1*t23*t62*t65*t88*t128*t136*2.0;
  _H[8][7] = t330+t331+e1*t23*t62*t66*(t328+t329+e2*t22*t68*(t3*t12*t13*t21*t35*t85*2.0-t4*t12*t19*t21*t35*t85*2.0+t3*t12*t13*t21*t43*t64*t199*t247*4.0)-t3*t12*t13*t22*t24*t93*2.0-t13*t19*t22*t35*t67*t85*t90*t130*2.0)*2.0;
  _H[8][8] = t62*t84*t237*t332*2.0+e1*t23*t62*t66*(t22*t24*t32*t93*-2.0+t22*t24*t43*t93*2.0+t22*t32*t35*t68*t85*2.0-t22*t35*t43*t68*t85*2.0+t22*t32*t43*t92*t215*t252*4.0+t22*t32*t43*t64*t68*t199*t247*4.0+t21*t22*t32*t43*t67*t90*t193*t247*4.0)*2.0+e1*t23*t62*t65*t88*t332*2.0;
  _H[8][9] = t378+t382-e1*t23*t62*t66*(t379+t380+t381+e2*t22*t68*(t13*t21*t35*t85*t149*2.0-t19*t21*t35*t85*t159*2.0+t13*t21*t43*t64*t149*t199*t247*4.0)-t13*t22*t24*t93*t149*2.0)*2.0;
  _H[8][10] = t433+t434+e1*t23*t62*t66*(t430+t431+t432-e2*t22*t68*(t13*t21*t35*t85*t173*2.0+t19*t21*t35*t85*t181*2.0+t13*t21*t43*t64*t173*t199*t247*4.0)-t13*t19*t22*t35*t67*t85*t90*t175*2.0)*2.0;
  _H[9][0] = t333-e1*t53*t62*t68*t76*t81*t84*t160*4.0-t20*t23*t62*t66*t68*t76*t81*t141*8.0-t23*t53*t62*t66*t67*t76*t81*t90*t151*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t160*4.0-t20*t23*t53*t62*t64*t66*t68*t115*t122*t141*8.0;
  _H[9][1] = t334-e1*t43*t62*t68*t84*t85*t86*t160*4.0-t19*t23*t62*t66*t68*t85*t86*t149*8.0-t23*t43*t62*t66*t67*t85*t86*t90*t151*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t160*4.0-t19*t23*t43*t62*t64*t66*t68*t149*t199*t200*8.0;
  _H[9][2] = t335-e1*t32*t62*t84*t93*t94*t160*4.0+t13*t23*t62*t66*t93*t94*t159*8.0-t23*t32*t62*t65*t88*t93*t94*t160*4.0+t13*t23*t32*t62*t66*t92*t159*t215*t216*8.0;
  _H[9][3] = t337+t23*t62*(t61*t160*t196-e1*t66*(t336-t34*t95*t159*t261*2.0+e2*t58*t95*t104*t151-t13*t24*t93*t98*t159*t195*2.0)+e1*t66*t97*t160-e1*t65*t88*t101*t160)*2.0;
  _H[9][4] = t338+t341+e1*t23*t62*t66*(t22*t58*t104*t151-e2*t22*t68*(t339+t340+t45*t105*t149*t264*2.0+t55*t105*t141*t265*2.0)+e2*t68*t95*t96*t151-e2*t22*t67*t90*t110*t151)*2.0;
  _H[9][5] = t353;
  _H[9][6] = t365;
  _H[9][7] = t377;
  _H[9][8] = t378+t382-e1*t23*t62*t66*(t379+t380+t381-t13*t22*t24*t93*t149*2.0+t13*t22*t35*t68*t85*t149*2.0-t19*t22*t35*t68*t85*t159*2.0+t13*t22*t43*t64*t68*t149*t199*t247*4.0)*2.0;
  _H[9][9] = t62*t84*t237*t386*2.0+e1*t23*t62*t66*(t22*t24*t93*t385*2.0+e2*t22*t68*(t21*t46*t53*t76*-2.0+t21*t35*t85*t383*2.0+t21*t46*t76*t384*2.0+t19*t21*t35*t85*(t41-t42+px*t2*t3*t5+py*t3*t5*t6-t2*t3*t5*x-t3*t5*t6*y)*2.0+t21*t53*t64*t115*t249*t384*4.0+t21*t43*t64*t199*t247*t383*4.0)-t13*t22*t24*t93*(t30-t31+px*t2*t4*t5+py*t4*t5*t6-t2*t4*t5*x-t4*t5*t6*y)*2.0+e2*t22*t67*t90*(t151*t151)+t22*t32*t92*t215*t252*t385*4.0)*2.0+e1*t23*t62*t65*t88*t386*2.0;
  _H[9][10] = t454;
  _H[10][0] = b*c*e1*t23*t66*t78*(t178+t182)-e1*t53*t62*t68*t76*t81*t84*t183*4.0-t20*t23*t62*t66*t68*t76*t81*t167*8.0-t23*t53*t62*t66*t67*t76*t81*t90*t175*4.0-t23*t53*t62*t65*t68*t76*t81*t88*t183*4.0-t20*t23*t53*t62*t64*t66*t68*t115*t122*t167*8.0;
  _H[10][1] = t387-e1*t43*t62*t68*t84*t85*t86*t183*4.0-t19*t23*t62*t66*t68*t85*t86*t173*8.0-t23*t43*t62*t66*t67*t85*t86*t90*t175*4.0-t23*t43*t62*t65*t68*t85*t86*t88*t183*4.0-t19*t23*t43*t62*t64*t66*t68*t173*t199*t200*8.0;
  _H[10][2] = t388-e1*t32*t62*t84*t93*t94*t183*4.0-t13*t23*t62*t66*t93*t94*t181*8.0-t23*t32*t62*t65*t88*t93*t94*t183*4.0-t13*t23*t32*t62*t66*t92*t181*t215*t216*8.0;
  _H[10][3] = t391-t23*t62*(e1*t66*(t389+t390+t34*t95*t181*t261*2.0+e2*t58*t95*t104*t175)-t61*t183*t196-e1*t66*t97*t183+e1*t65*t88*t101*t183)*2.0;
  _H[10][4] = t392+t395+e1*t23*t62*t66*(t22*t58*t104*(t170+t174)-e2*t22*t68*(t393+t394+t45*t105*t173*t264*2.0+t55*t105*t167*t265*2.0)+e2*t68*t95*t96*(t170+t174)-e2*t22*t67*t90*t110*t175)*2.0;
  _H[10][5] = t407;
  _H[10][6] = t418;
  _H[10][7] = t429;
  _H[10][8] = t433+t434-e1*t23*t62*t66*(-t430-t431-t432+t13*t22*t35*t68*t85*t173*2.0+t19*t22*t35*t68*t85*t181*2.0+t13*t19*t22*t35*t67*t85*t90*t175*2.0+t13*t22*t43*t64*t68*t173*t199*t247*4.0)*2.0;
  _H[10][9] = t454;
  _H[10][10] = t62*t84*t237*(t455*t455)*2.0+e1*t23*t62*t66*(t22*t24*t93*t460*2.0+e2*t22*t68*(t21*t35*t85*t458*2.0+t21*t46*t76*(t456*t456)*2.0-t19*t21*t35*t85*(t37-t38-t39+t40)*2.0+t20*t21*t46*t76*(t49-t50+t51-t52)*2.0+t21*t43*t64*t199*t247*t458*4.0+t21*t53*t64*t115*t249*(t457*t457)*4.0)-t13*t22*t24*t93*(t25-t27-t28+t29)*2.0+e2*t22*t67*t90*(t459*t459)+t22*t32*t92*t215*t252*t460*4.0)*2.0+e1*t23*t62*t65*t88*(t461*t461)*2.0;

}


/**
 * @function levmar_fx
 */
void levmar_fx( double *p, double* x,
		int m, int n, void *data ) {
    
    struct levmar_data* dptr;
    dptr = (struct levmar_data*) data;
    
    double xi, yi, zi;
    double t2, t3, t4, t5, t6;
    double t7, t8, t9, t10;
    double t11, t12, t13, t14, t15;
    double t16, t17, t18, t19, t20;
    double t21, t22;
        
    for( int i = 0; i < n; ++i ) {
	
	xi = dptr->x[i];
	yi = dptr->y[i];
	zi = dptr->z[i];
	
	t2 = cos(p[10]);
	t3 = sin(p[8]);
	t4 = cos(p[8]);
	t5 = sin(p[9]);
	t6 = sin(p[10]);
	t7 = t3*t6;
	t8 = t2*t4*t5;
	t9 = t7+t8;
	t10 = t2*t3;
	t11 = t10-t4*t5*t6;
	t12 = cos(p[9]);
	t13 = p[5]*t9-p[6]*t11-t9*xi+t11*yi+p[7]*t4*t12-t4*t12*zi;
	t14 = t4*t6;
	t15 = t14-t2*t3*t5;
	t16 = t2*t4;
	t17 = t3*t5*t6;
	t18 = t16+t17;
	t19 = p[5]*t15-p[6]*t18-t15*xi+t18*yi-p[7]*t3*t12+t3*t12*zi;
	t20 = p[7]*t5-t5*zi-p[5]*t2*t12-p[6]*t6*t12+t2*t12*xi+t6*t12*yi;
	t21 = 1.0/p[4];
	t22 = 1.0/p[3];
	
	x[i] = (pow(pow( pow(1.0/(p[0]*p[0])*(t20*t20),t21)+
			 pow(1.0/(p[1]*p[1])*(t19*t19),t21),p[4]*t22 )+
		    pow(1.0/(p[2]*p[2])*(t13*t13),t22), p[3] )
		-1.0) * sqrt(p[0]*p[1]*p[2]);       		
    } // end for
    
}


/**
 * @function levmar_jac
 */
void levmar_jac( double* p, double* jac,
		 int m, int n, void* data ) {
    
    struct levmar_data* dptr;
    dptr = (struct levmar_data*) data;
    double xi, yi, zi;
    
    register double t2,t3,t4,t5;
    register double t17, t18, t19, t20, t21, t22;
    register double t6,t7,t8,t9;
    register double t27, t10, t11, t12, t13;
    register double t28, t29, t30, t31, t32, t33;
    register double t14, t15, t16;
    register double t23, t24, t25, t26;


    register double t34, t35, t36, t37, t38, t39;
    register double t40, t41, t42, t43, t44, t45, t46;
    register double t50, t47, t49, t51, t52, t53, t54, t55;
    register double t48, t56, t57, t58, t59;
    register double t60, t61, t62, t63, t64, t65,t66;
    register double t67, t68, t69, t70, t71, t72, t73, t74, t75;

    for( int i = 0; i < n; ++i ) {

	xi = dptr->x[i];
	yi = dptr->y[i];
	zi = dptr->z[i];

	t2 = cos(p[10]);
	t3 = sin(p[8]);
	t4 = cos(p[8]);
	t5 = sin(p[9]);
	t6 = sin(p[10]);
	t7 = t3*t6;
	t8 = t2*t4*t5;
	t9 = t7+t8;
	t10 = t2*t3;
	t25 = t4*t5*t6;
	t11 = t10-t25;
	t12 = cos(p[9]);
	t24 = p[5]*t9;
	t26 = p[6]*t11;
	t27 = t9*xi;
	t28 = t11*yi;
	t29 = p[7]*t4*t12;
	t30 = t4*t12*zi;
	t13 = t24-t26-t27+t28+t29-t30;
	t14 = t4*t6;
	t35 = t2*t3*t5;
	t15 = t14-t35;
	t16 = t2*t4;
	t17 = t3*t5*t6;
	t18 = t16+t17;
	t36 = p[5]*t15;
	t37 = p[6]*t18;
	t38 = t15*xi;
	t39 = t18*yi;
	t40 = p[7]*t3*t12;
	t41 = t3*t12*zi;
	t19 = t36-t37-t38+t39-t40+t41;
	t46 = p[7]*t5;
	t47 = t5*zi;
	t48 = p[5]*t2*t12;
	t49 = t2*t12*xi;
	t50 = p[6]*t6*t12;
	t51 = t6*t12*yi;
	t20 = t46-t47-t48+t49-t50+t51;
	t21 = 1.0/p[4];
	t22 = 1.0/p[3];
	t23 = 1.0/(p[2]*p[2]);
	t31 = t13*t13;
	t32 = t23*t31;
	t33 = pow(t32,t22);
	t34 = 1.0/(p[1]*p[1]);
	t42 = t19*t19;
	t43 = t34*t42;
	t44 = pow(t43,t21);
	t45 = 1.0/(p[0]*p[0]);
	t52 = t20*t20;
	t53 = t45*t52;
	t54 = pow(t53,t21);
	t55 = t44+t54;
	t56 = p[4]*t22;
	t57 = pow(t55,t56);
	t58 = t33+t57;
	t59 = p[0]*p[1]*p[2];
	t60 = pow(t58,p[3]);
	t61 = t60-1.0;
	t62 = 1.0/sqrt(t59);
	t63 = p[3]-1.0;
	t64 = pow(t58,t63);
	t65 = t21-1.0;
	t66 = t56-1.0;
	t67 = pow(t55,t66);
	t68 = sqrt(t59);
	t69 = 1.0/(p[3]*p[3]);
	t70 = log(t55);
	t71 = 1.0/(p[4]*p[4]);
	t72 = pow(t43,t65);
	t73 = pow(t53,t65);
	t74 = t22-1.0;
	t75 = pow(t32,t74);
	jac[11*i+0] = p[1]*p[2]*t61*t62*(1.0/2.0)-1.0/(p[0]*p[0]*p[0])*t52*t64*t67*t68*t73*2.0;
	jac[11*i+1] = p[0]*p[2]*t61*t62*(1.0/2.0)-1.0/(p[1]*p[1]*p[1])*t42*t64*t67*t68*t72*2.0;
	jac[11*i+2] = p[0]*p[1]*t61*t62*(1.0/2.0)-1.0/(p[2]*p[2]*p[2])*t31*t64*t68*t75*2.0;
	jac[11*i+3] = t68*(t60*log(t58)-p[3]*t64*(t33*t69*log(t32)+p[4]*t57*t69*t70));
	jac[11*i+4] = p[3]*t64*t68*(t22*t57*t70-p[4]*t22*t67*(t44*t71*log(t43)+t54*t71*log(t53)));
	jac[11*i+5] = p[3]*t64*t68*(p[4]*t22*t67*(t15*t19*t21*t34*t72*2.0-t2*t12*t20*t21*t45*t73*2.0)+t9*t13*t22*t23*t75*2.0);
	jac[11*i+6] = -p[3]*t64*t68*(p[4]*t22*t67*(t18*t19*t21*t34*t72*2.0+t6*t12*t20*t21*t45*t73*2.0)+t11*t13*t22*t23*t75*2.0);
	jac[11*i+7] = p[3]*t64*t68*(p[4]*t22*t67*(t5*t20*t21*t45*t73*2.0-t3*t12*t19*t21*t34*t72*2.0)+t4*t12*t13*t22*t23*t75*2.0);
	jac[11*i+8] = p[3]*t64*t68*(t13*t19*t22*t23*t75*2.0-t13*t19*t22*t34*t67*t72*2.0);
	jac[11*i+9] = p[3]*t64*t68*(p[4]*t22*t67*(t20*t21*t45*t73*(p[7]*t12-t12*zi+p[5]*t2*t5+p[6]*t5*t6-t2*t5*xi-t5*t6*yi)*2.0+t19*t21*t34*t72*(p[7]*t3*t5-t3*t5*zi-p[5]*t2*t3*t12-p[6]*t3*t6*t12+t2*t3*t12*xi+t3*t6*t12*yi)*2.0)-t13*t22*t23*t75*(p[7]*t4*t5-t4*t5*zi-p[5]*t2*t4*t12-p[6]*t4*t6*t12+t2*t4*t12*xi+t4*t6*t12*yi)*2.0);
	jac[11*i+10] = p[3]*t64*t68*(p[4]*t22*t67*(t20*t21*t45*t73*(p[5]*t6*t12-p[6]*t2*t12-t6*t12*xi+t2*t12*yi)*2.0+t19*t21*t34*t72*(p[5]*t18+p[6]*t15-t18*xi-t15*yi)*2.0)+t13*t22*t23*t75*(p[5]*t11+p[6]*t9-t11*xi-t9*yi)*2.0);
    } // end for
    


}


