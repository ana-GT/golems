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

double error_SQ_tampering( const SQ_parameters &_par,  
			   const double &x, 
			   const double &y, 
			   const double &z ) {

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
  double t = _par.tamp;

  return pow(pow(pow(1.0/(c*c)*pow(px*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))-py*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))-x*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))+y*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))+pz*cos(pa)*cos(ra)-z*cos(pa)*cos(ra),2.0),1.0/e1)+pow(pow(1.0/(b*b)*1.0/pow((t*(px*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))-py*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))-x*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))+y*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))+pz*cos(pa)*cos(ra)-z*cos(pa)*cos(ra)))/c-1.0,2.0)*pow(px*(cos(ra)*sin(ya)-cos(ya)*sin(pa)*sin(ra))-py*(cos(ra)*cos(ya)+sin(pa)*sin(ra)*sin(ya))-x*(cos(ra)*sin(ya)-cos(ya)*sin(pa)*sin(ra))+y*(cos(ra)*cos(ya)+sin(pa)*sin(ra)*sin(ya))-pz*cos(pa)*sin(ra)+z*cos(pa)*sin(ra),2.0),1.0/e2)+pow(1.0/(a*a)*1.0/pow((t*(px*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))-py*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))-x*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))+y*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))+pz*cos(pa)*cos(ra)-z*cos(pa)*cos(ra)))/c-1.0,2.0)*pow(pz*sin(pa)-z*sin(pa)-px*cos(pa)*cos(ya)+x*cos(pa)*cos(ya)-py*cos(pa)*sin(ya)+y*cos(pa)*sin(ya),2.0),1.0/e2),e2/e1),e1)-1.0,2.0);
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

/**
 * @function levmar_tampering_fx
 */
void levmar_tampering_fx( double *p, double* x, int m, int n, void* data ) {

    struct levmar_data* dptr;
    dptr = (struct levmar_data*) data;
    
    double xi, yi, zi;
    double a, b, c, e1, e2, ra, pa, ya, px,py,pz,t;
    a = p[0]; b = p[1]; c = p[2];
    e1 = p[3]; e2 = p[4];
    px = p[5]; py = p[6]; pz = p[7];
    ra = p[8]; pa = p[9]; ya = p[10];
    t = p[11];
        
    for( int i = 0; i < n; ++i ) {
	
	xi = dptr->x[i];
	yi = dptr->y[i];
	zi = dptr->z[i];
		
    x[i] = (pow(pow(1.0/(c*c)*pow(px*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))-py*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))-xi*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))+yi*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))+pz*cos(pa)*cos(ra)-zi*cos(pa)*cos(ra),2.0),1.0/e1)+pow(pow(1.0/(b*b)*1.0/pow((t*(px*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))-py*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))-xi*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))+yi*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))+pz*cos(pa)*cos(ra)-zi*cos(pa)*cos(ra)))/c-1.0,2.0)*pow(px*(cos(ra)*sin(ya)-cos(ya)*sin(pa)*sin(ra))-py*(cos(ra)*cos(ya)+sin(pa)*sin(ra)*sin(ya))-xi*(cos(ra)*sin(ya)-cos(ya)*sin(pa)*sin(ra))+yi*(cos(ra)*cos(ya)+sin(pa)*sin(ra)*sin(ya))-pz*cos(pa)*sin(ra)+zi*cos(pa)*sin(ra),2.0),1.0/e2)+pow(1.0/(a*a)*1.0/pow((t*(px*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))-py*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))-xi*(sin(ra)*sin(ya)+cos(ra)*cos(ya)*sin(pa))+yi*(cos(ya)*sin(ra)-cos(ra)*sin(pa)*sin(ya))+pz*cos(pa)*cos(ra)-zi*cos(pa)*cos(ra)))/c-1.0,2.0)*pow(pz*sin(pa)-zi*sin(pa)-px*cos(pa)*cos(ya)+xi*cos(pa)*cos(ya)-py*cos(pa)*sin(ya)+yi*cos(pa)*sin(ya),2.0),1.0/e2),e2/e1),e1)-1.0)*sqrt(a*b*c);

    } // end for

}


/**
 * @function levmar_jac
 */
void levmar_tampering_jac( double* p, double* jac,
		 int m, int n, void* data ) {
    
    struct levmar_data* dptr;
    dptr = (struct levmar_data*) data;

    double xi, yi, zi;
    double a, b, c, e1, e2, ra, pa, ya, px,py,pz,t;
    a = p[0]; b = p[1]; c = p[2];
    e1 = p[3]; e2 = p[4];
    px = p[5]; py = p[6]; pz = p[7];
    ra = p[8]; pa = p[9]; ya = p[10];
    t = p[11];

    
    register double t3, t4, t5, t6, t7, t8, t9;
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

	xi = dptr->x[i];
	yi = dptr->y[i];
	zi = dptr->z[i];

	t3 = cos(ya);
	t4 = sin(ra);
	t5 = cos(ra);
	t6 = sin(pa);
	t7 = sin(ya);
	t8 = t4*t7;
	t9 = t3*t5*t6;
	t10 = t8+t9;
	t11 = t3*t4;
	t16 = t5*t6*t7;
	t12 = t11-t16;
	t13 = cos(pa);
	t15 = px*t10;
	t17 = py*t12;
	t18 = t10*xi;
	t19 = t12*yi;
	t20 = pz*t5*t13;
	t21 = t5*t13*zi;
	t14 = t15-t17-t18+t19+t20-t21;
	t22 = t5*t7;
	t40 = t3*t4*t6;
	t23 = t22-t40;
	t24 = t3*t5;
	t25 = t4*t6*t7;
	t26 = t24+t25;
	t41 = px*t23;
	t42 = py*t26;
	t43 = t23*xi;
	t44 = t26*yi;
	t45 = pz*t4*t13;
	t46 = t4*t13*zi;
	t27 = t41-t42-t43+t44-t45+t46;
	t28 = 1.0/c;
	t29 = t*t14*t28;
	t30 = t29-1.0;
	t31 = 1.0/(t30*t30);
	t51 = pz*t6;
	t52 = t6*zi;
	t53 = px*t3*t13;
	t54 = t3*t13*xi;
	t55 = py*t7*t13;
	t56 = t7*t13*yi;
	t32 = t51-t52-t53+t54-t55+t56;
	t33 = 1.0/e2;
	t34 = 1.0/e1;
	t35 = 1.0/(c*c);
	t36 = t14*t14;
	t37 = t35*t36;
	t38 = pow(t37,t34);
	t39 = 1.0/(b*b);
	t47 = t27*t27;
	t48 = t31*t39*t47;
	t49 = pow(t48,t33);
	t50 = 1.0/(a*a);
	t57 = t32*t32;
	t58 = t31*t50*t57;
	t59 = pow(t58,t33);
	t60 = t49+t59;
	t61 = e2*t34;
	t62 = pow(t60,t61);
	t63 = t38+t62;
	t64 = a*b*c;
	t65 = pow(t63,e1);
	t66 = t65-1.0;
	t67 = 1.0/sqrt(t64);
	t68 = e1-1.0;
	t69 = pow(t63,t68);
	t70 = t61-1.0;
	t71 = pow(t60,t70);
	t72 = sqrt(t64);
	t73 = t33-1.0;
	t74 = pow(t48,t73);
	t75 = 1.0/(t30*t30*t30);
	t76 = pow(t58,t73);
	t77 = 1.0/(e1*e1);
	t78 = log(t60);
	t79 = 1.0/(e2*e2);
	t80 = t34-1.0;
	t81 = pow(t37,t80);
	t82 = pz*t5*t6;
	t83 = t3*t5*t13*xi;
	t84 = t5*t7*t13*yi;
	t86 = t5*t6*zi;
	t87 = px*t3*t5*t13;
	t88 = py*t5*t7*t13;
	t85 = t82+t83+t84-t86-t87-t88;
	t89 = px*t12;
	t90 = py*t10;
	t92 = t12*xi;
	t93 = t10*yi;
	t91 = t89+t90-t92-t93;

  jac[12*i+0] = b*c*t66*t67*(1.0/2.0)-1.0/(a*a*a)*t31*t57*t69*t71*t72*t76*2.0;
  jac[12*i+1] = a*c*t66*t67*(1.0/2.0)-1.0/(b*b*b)*t31*t47*t69*t71*t72*t74*2.0;
  jac[12*i+2] = a*b*t66*t67*(1.0/2.0)+e1*t69*t72*(e2*t34*t71*(t*t14*t33*t35*t39*t47*t74*t75*2.0+t*t14*t33*t35*t50*t57*t75*t76*2.0)-1.0/(c*c*c)*t34*t36*t81*2.0);
  jac[12*i+3] = t72*(t65*log(t63)-e1*t69*(t38*t77*log(t37)+e2*t62*t77*t78));
  jac[12*i+4] =e1*t69*t72*(t34*t62*t78-e2*t34*t71*(t49*t79*log(t48)+t59*t79*log(t58)));
  jac[12*i+5] = -e1*t69*t72*(e2*t34*t71*(t33*t76*(t3*t13*t31*t32*t50*2.0+t*t10*t28*t50*t57*t75*2.0)-t33*t74*(t23*t27*t31*t39*2.0-t*t10*t28*t39*t47*t75*2.0))-t10*t14*t34*t35*t81*2.0);
  jac[12*i+6] = -e1*t69*t72*(e2*t34*t71*(t33*t76*(t7*t13*t31*t32*t50*2.0-t*t12*t28*t50*t57*t75*2.0)+t33*t74*(t26*t27*t31*t39*2.0-t*t12*t28*t39*t47*t75*2.0))+t12*t14*t34*t35*t81*2.0);
  jac[12*i+7] = e1*t69*t72*(e2*t34*t71*(t33*t76*(t6*t31*t32*t50*2.0-t*t5*t13*t28*t50*t57*t75*2.0)-t33*t74*(t4*t13*t27*t31*t39*2.0+t*t5*t13*t28*t39*t47*t75*2.0))+t5*t13*t14*t34*t35*t81*2.0);
  jac[12*i+8] = -e1*t69*t72*(e2*t34*t71*(t33*t74*(t14*t27*t31*t39*2.0+t*t27*t28*t39*t47*t75*2.0)+t*t27*t28*t33*t50*t57*t75*t76*2.0)-t14*t27*t34*t35*t81*2.0);
  jac[12*i+9] = e1*t69*t72*(e2*t34*t71*(t33*t74*(t27*t31*t39*(pz*t4*t6-t4*t6*zi-px*t3*t4*t13-py*t4*t7*t13+t3*t4*t13*xi+t4*t7*t13*yi)*2.0+t*t28*t39*t47*t75*t85*2.0)+t33*t76*(t31*t32*t50*(pz*t13-t13*zi+px*t3*t6+py*t6*t7-t3*t6*xi-t6*t7*yi)*2.0+t*t28*t50*t57*t75*t85*2.0))-t14*t34*t35*t81*t85*2.0);
  jac[12*i+10] = e1*t69*t72*(e2*t34*t71*(t33*t76*(t31*t32*t50*(px*t7*t13-py*t3*t13-t7*t13*xi+t3*t13*yi)*2.0-t*t28*t50*t57*t75*t91*2.0)+t33*t74*(t27*t31*t39*(px*t26+py*t23-t26*xi-t23*yi)*2.0-t*t28*t39*t47*t75*t91*2.0))+t14*t34*t35*t81*t91*2.0);
  jac[12*i+11] = -e2*t69*t71*t72*(t14*t28*t33*t39*t47*t74*t75*2.0+t14*t28*t33*t50*t57*t75*t76*2.0);


    } // end for
    
}


