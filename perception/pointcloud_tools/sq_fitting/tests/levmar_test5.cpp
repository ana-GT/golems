/**
 * @dilw levmar_test4.cpp
 * @brief Curve fitting
 */
#include <pcl/io/pcd_io.h>
#include <levmar/levmar.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <SQ_utils.h>

void ex5_fx( double *p, double* x,
	     int m, int n, void *data );
void ex5_jac( double* p, double* jac,
	      int m, int n, void* data );


struct myData {
    double* ix;
    double* iy;
    double* iz;
    int num;
};


int main( int argc, char* argv[] ) {


    // 1. Read pointcloud
    if( argc != 2 ) {
	std::cout << "Syntax: "<< argv[0]<< " filename.pcd"<< std::endl;
	return 1;
    }

    std::string filename = std::string( argv[1] );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFit( new pcl::PointCloud<pcl::PointXYZ>() );
    if( pcl::io::loadPCDFile<pcl::PointXYZ>( filename, *cloud ) == -1 ) {
	std::cout << "\t Could NOT read pointcloud file! Exiting" << std::endl;
	return 1;
    }
    /*
    for( int i = 0; i < cloud0->points.size()/2; ++i ) {
	cloud->points.push_back( cloud0->points[i] );
    }
    cloud->width = 1;
    cloud->height = cloud->points.size();
    */
     
    std::cout << "Pointcloud loaded with "<< cloud->points.size() 
	      <<" points."<< std::endl;
    

    // 3. Fit it
    
    int n = cloud->points.size(); // Size of measurements
    int m = 11; // Size of parameters

    double p[m]; double x[n];

    double opts[LM_OPTS_SZ];
    double info[LM_INFO_SZ];

    opts[0] = LM_INIT_MU;
    opts[1] = 1E-15;
    opts[2] = 1E-15;
    opts[3] = 1E-20;
    opts[4] = LM_DIFF_DELTA;


    struct myData adata;
    adata.ix = new double[n];
    adata.iy = new double[n];
    adata.iz = new double[n];
    adata.num = n;

    for( int i = 0; i < n; ++i ) {
	adata.ix[i] = cloud->points[i].x;
	adata.iy[i] = cloud->points[i].y;
	adata.iz[i] = cloud->points[i].z;
    }


    int i, ret;
    
    // We want the function to be closest to their measurements
    for( int i = 0; i < n; ++i ) {
	x[i] = 0.0;
    }

    // Initial values
    p[0] = 0.05; p[1] = 0.05; p[2] = 0.095;
    p[3] = 0.5; p[4] = 1;
    p[5] = -0.21; p[6] = -0.10; p[7] = 0.79;
    p[8] = -2.81; p[9] = -1.92; p[10] = 1.24;
 
    std::cout << "BEFORE: Initial parameters:"<<std::endl;
    std::cout << "\t * Axes: "<< p[0]<<", "<<p[1]<<", "<< p[2] << std::endl;
    std::cout << "\t  *e1, e2: "<< p[3]<<", "<< p[4] << std::endl;
    std::cout << "\t * trans: "<< p[5]<<", "<< p[6]<<", "<<p[7]<<std::endl;
    std::cout << "\t * rot: "<< p[8]<<", "<< p[9]<<", "<<p[10]<<std::endl;

    double dt; clock_t ts, tf;
    ts = clock();
    double ub[11]; double lb[11];
    lb[0] = 0.01; ub[0] = 1.0;
    lb[1] = 0.01; ub[1] = 1.0;
    lb[2] = 0.01; ub[2] = 1.0;

    lb[3] = 0.1; ub[3] = 1.9;
    lb[4] = 0.1; ub[4] = 1.9;

    lb[5] = -3; ub[5] = 3;
    lb[6] = -3; ub[6] = 3;
    lb[7] = -3; ub[7] = 3;

    lb[8] = -6; ub[8] = 6;
    lb[9] = -6; ub[9] = 6;
    lb[10] = -6; ub[10] = 6;


    ret = dlevmar_bc_der( ex5_fx, ex5_jac,
			  p, x,
			  m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&adata );
    tf = clock();
    
    dt = (double)(tf-ts)/(double)CLOCKS_PER_SEC;

    SQ_parameters s;
    s.dim[0] = p[0]; s.dim[1] = p[1]; s.dim[2] = p[2];
    s.e[0] = p[3]; s.e[1] = p[4];
    s.trans[0] = p[5]; s.trans[1] = p[6]; s.trans[2] = p[7];
    s.rot[0] = p[8]; s.rot[1] = p[9]; s.rot[2] = p[10];
    cloudFit = sampleSQ_uniform( s );


    std::cout << "Calculation time: "<< dt << std::endl;
    std::cout << " Levenberg Marquardt returned in "<<info[5]<<" iterations "<<
	", reason: "<< info[6] << " sumsq: "<< info[1] <<"["<<info[0]<<"]"<<std::endl;

    std::cout << "AFTER: Best fit parameter: "<<std::endl;
    std::cout <<"\t * Axes: "<< p[0]<<", "<<p[1]<<", "<< p[2] << std::endl;
    std::cout << "\t * e1, e2: "<< p[3]<<", "<< p[4] << std::endl;
    std::cout << "\t * trans: "<< p[5]<<", "<< p[6]<<", "<<p[7]<<std::endl;
    std::cout << "\t * rot: "<< p[8]<<", "<< p[9]<<", "<<p[10]<<std::endl;


    // 2. Visualize pointcloud start and end
    // 2. Visualize pointcloud
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("input pointcloud") );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(cloud, 255,0,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> comp(cloudFit, 255,255,0);
    viewer->addPointCloud( cloud, col, "input1"  );
    viewer->addPointCloud( cloudFit, comp, "fit"  );
    while( !viewer->wasStopped() ) {
	viewer->spinOnce(100);
	boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
    }

    return 0;

}

/**
 * @function SQ
 */
void ex5_fx( double *p, double* x,
	     int m, int n, void *data ) {

    struct myData* dptr;
    dptr = (struct myData*) data;

    double xi, yi, zi;
    double t2, t3, t4, t5, t6;
    double t7, t8, t9, t10;
    double t11, t12, t13, t14, t15;
    double t16, t17, t18, t19, t20;
    double t21, t22;

    // std::cout <<  "a: "<<p[0]<< "b: "<< p[1] << "c: "<< p[2] << std::endl; 

    for( int i = 0; i < n; ++i ) {
	
	xi = dptr->ix[i];
	yi = dptr->iy[i];
	zi = dptr->iz[i];
	
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
	/*
	x[i] = (pow(pow( pow(1.0/(p[0]*p[0])*(t20*t20),t21)+
			 pow(1.0/(p[1]*p[1])*(t19*t19),t21),p[4]*t22 )+
		    pow(1.0/(p[2]*p[2])*(t13*t13),t22), p[3] )
		-1.0) * sqrt(p[0]*p[1]*p[2]);
	*/


	double f1 = pow(1.0/(p[0]*p[0])*(t20*t20),t21);
	double f2 = pow(1.0/(p[1]*p[1])*(t19*t19),t21);
	double f3 = pow(f1+f2, p[4]*t22 );
	double f4 = pow(1.0/(p[2]*p[2])*(t13*t13),t22);
	double f5 = f3 + f4;
	double f6 = pow(f5, p[3] );
	double f7 = p[0]*p[1]*p[2];
	double f8 = sqrt(f7);
	/*
	if( f1 != f1 ) { std::cout << "["<<i<<"] F1: "<< f1 << std::endl; }
	if( f2 != f2 ) { std::cout << "["<<i<<"] F2: "<< f2 << std::endl; }
	if( f3 != f3 ) { std::cout << "["<<i<<"] F3: "<< f3 << std::endl; }
	if( f4 != f4 ) { std::cout << "["<<i<<"] F4: "<< f4 << std::endl; }
	if( f5 != f5 ) { std::cout << "["<<i<<"] F5: "<< f5 << std::endl; }
	if( f6 != f6 ) { std::cout << "["<<i<<"] F6: "<< f6 << std::endl; }  
	if( f7 != f7 ) { std::cout << "["<<i<<"] F7: "<< f7 << std::endl; }
	if( f8 != f8 ) { std::cout << "["<<i<<"] F8: "<< f8 <<" f7: "<< f7<< "a: "<<p[0]<< "b: "<< p[1] << "c: "<< p[2] << std::endl; }
	*/
	x[i] = (f6 - 1.0) * f8;
	
	
    }


    for( int i = 0; i < n; ++i ) {
	if( x[i] != x[i] ) {
	    std::cout << "!!!! NAN in fx ("<<i<<"): "<< x[i]<<std::endl;
	    break;
	}
	
    }
    
    
}


void ex5_jac( double* p, double* jac,
	      int m, int n, void* data ) {

    struct myData* dptr;
    dptr = (struct myData*) data;
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

	xi = dptr->ix[i];
	yi = dptr->iy[i];
	zi = dptr->iz[i];

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
    }
    
    for( int i = 0; i < n; ++i ) {
	for( int j = 0; j < 11; ++j ) {
	    if( jac[11*i+j] != jac[11*i+j] ) {
		std::cout << "NAN in jacobian ("<<i<<","<<j<<")"<<std::endl;
	    }
	}
    }

}

/**
 * @function SQ
 */
/*
void ex5_fx( double *p, double* x,
	     int m, int n, void *data ) {

    struct myData* dptr;
    dptr = (struct myData*) data;

    double xi, yi, zi;
    double t2, t3, t4, t5, t6;
    double t7, t8, t9, t10;
    double t11, t12, t13, t14, t15;
    double t16, t17, t18, t19, t20;
    double t21, t22;

    for( int i = 0; i < n; ++i ) {
	
	xi = dptr->ix[i];
	yi = dptr->iy[i];
	zi = dptr->iz[i];

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
       
	x[i]= pow(pow(1.0/(p[0]*p[0])*(t20*t20),t21)+pow(1.0/(p[1]*p[1])*(t19*t19),t21),p[4]*t22)+pow(1.0/(p[2]*p[2])*(t13*t13),t22);
	
    }
    
}
*/
/*
void ex5_jac( double* p, double* jac,
	      int m, int n, void* data ) {

    struct myData* dptr;
    dptr = (struct myData*) data;
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

    for( int i = 0; i < n; ++i ) {

	xi = dptr->ix[i];
	yi = dptr->iy[i];
	zi = dptr->iz[i];



	t2 = sin(p[9]);
	t3 = cos(p[9]);
	t4 = cos(p[10]);
	t5 = sin(p[10]);
	t17 = p[7]*t2;
	t18 = t2*zi;
	t19 = p[5]*t3*t4;
	t20 = t3*t4*xi;
	t21 = p[6]*t3*t5;
	t22 = t3*t5*yi;
	t6 = t17-t18-t19+t20-t21+t22;
	t7 = cos(p[8]);
	t8 = sin(p[8]);
	t9 = t5*t7;
	t27 = t2*t4*t8;
	t10 = t9-t27;
	t11 = t4*t7;
	t12 = t2*t5*t8;
	t13 = t11+t12;
	t28 = p[5]*t10;
	t29 = p[6]*t13;
	t30 = t10*xi;
	t31 = t13*yi;
	t32 = p[7]*t3*t8;
	t33 = t3*t8*zi;
	t14 = t28-t29-t30+t31-t32+t33;
	t15 = 1.0/p[4];
	t16 = 1.0/(p[0]*p[0]);
	t23 = t6*t6;
	t24 = t16*t23;
	t25 = 1.0/p[3];
	t26 = 1.0/(p[1]*p[1]);
	t34 = t14*t14;
	t35 = t26*t34;
	t36 = t15-1.0;
	t37 = pow(t35,t15);
	t38 = pow(t24,t15);
	t39 = t37+t38;
	t40 = p[4]*t25;
	t41 = t40-1.0;
	t42 = pow(t39,t41);
	t43 = t5*t8;
	t44 = t2*t4*t7;
	t45 = t43+t44;
	t46 = t4*t8;
	t50 = t2*t5*t7;
	t47 = t46-t50;
	t49 = p[5]*t45;
	t51 = p[6]*t47;
	t52 = t45*xi;
	t53 = t47*yi;
	t54 = p[7]*t3*t7;
	t55 = t3*t7*zi;
	t48 = t49-t51-t52+t53+t54-t55;
	t56 = t48*t48;
	t57 = 1.0/(p[2]*p[2]);
	t58 = t56*t57;
	t59 = 1.0/(p[3]*p[3]);
	t60 = log(t39);
	t61 = pow(t39,t40);
	t62 = 1.0/(p[4]*p[4]);
	t63 = pow(t35,t36);
	t64 = pow(t24,t36);
	t65 = t25-1.0;
	t66 = pow(t58,t65);
	jac[11*i+0] = 1.0/(p[0]*p[0]*p[0])*t23*t25*t42*t64*-2.0;
	jac[11*i+1] = 1.0/(p[1]*p[1]*p[1])*t25*t34*t42*t63*-2.0;
	jac[11*i+2] = 1.0/(p[2]*p[2]*p[2])*t25*t56*t66*-2.0;
	jac[11*i+3] = -pow(t58,t25)*t59*log(t58)-p[4]*t59*t60*t61;
	jac[11*i+4] = t25*t60*t61-p[4]*t25*t42*(t38*t62*log(t24)+t37*t62*log(t35));
	jac[11*i+5] = p[4]*t25*t42*(t10*t14*t15*t26*t63*2.0-t3*t4*t6*t15*t16*t64*2.0)+t25*t45*t48*t57*t66*2.0;
	jac[11*i+6] = -p[4]*t25*t42*(t13*t14*t15*t26*t63*2.0+t3*t5*t6*t15*t16*t64*2.0)-t25*t47*t48*t57*t66*2.0;
	jac[11*i+7] = p[4]*t25*t42*(t2*t6*t15*t16*t64*2.0-t3*t8*t14*t15*t26*t63*2.0)+t3*t7*t25*t48*t57*t66*2.0;
	jac[11*i+8] = t14*t25*t48*t57*t66*2.0-t14*t25*t26*t42*t48*t63*2.0;
	jac[11*i+9] = p[4]*t25*t42*(t6*t15*t16*t64*(p[7]*t3-t3*zi+p[5]*t2*t4+p[6]*t2*t5-t2*t4*xi-t2*t5*yi)*2.0+t14*t15*t26*t63*(p[7]*t2*t8-t2*t8*zi-p[5]*t3*t4*t8-p[6]*t3*t5*t8+t3*t4*t8*xi+t3*t5*t8*yi)*2.0)-t25*t48*t57*t66*(p[7]*t2*t7-t2*t7*zi-p[5]*t3*t4*t7-p[6]*t3*t5*t7+t3*t4*t7*xi+t3*t5*t7*yi)*2.0;
	jac[11*i+10] = p[4]*t25*t42*(t6*t15*t16*t64*(p[5]*t3*t5-p[6]*t3*t4-t3*t5*xi+t3*t4*yi)*2.0+t14*t15*t26*t63*(p[5]*t13+p[6]*t10-t13*xi-t10*yi)*2.0)+t25*t48*t57*t66*(p[5]*t47+p[6]*t45-t47*xi-t45*yi)*2.0;
    }

}

*/
