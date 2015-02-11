
#include <levmar/levmar.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>


void ex3_fx( double *p, double* x,
	     int m, int n, void *data );
void ex3_jac( double* p, double* jac,
	      int m, int n, void* data );


int main( int argc, char* argv[] ) {
    
    srandom(0);
    const int n = 4; // Size of measurements
    const int m = 4; // Size of parameters

    double p[m]; double x[n];

    double opts[LM_OPTS_SZ];
    double info[LM_INFO_SZ];

    int i, ret;
    
    // We want the function to be the minimum possible
    x[0] = 0; x[1] = 0; x[2] = 0; x[3] = 0;
    p[0] = 3; p[1] = -1; p[2] = 0; p[3] = 1;
 
    double dt; clock_t ts, tf;
    ts = clock();
    ret = dlevmar_der( ex3_fx, ex3_jac,
		       p, x,
		       m, n,
		       1000,
		       NULL, info,
		       NULL, NULL, NULL );
    tf = clock();
    
    dt = (double)(tf-ts)/(double)CLOCKS_PER_SEC;
    std::cout << "Calculation time: "<< dt << std::endl;
    std::cout << " Levenberg Marquardt returned in "<<info[5]<<" iterations "<<
	", reason: "<< info[6] << " sumsq: "<< info[1] <<"["<<info[0]<<"]"<<std::endl;

    std::cout << "Best fit parameter: "<< p[0]<<", "<<p[1]<<", "<<p[2]<<", "<<p[3]<< std::endl;

    return 0;

}


void ex3_fx( double *p, double* x,
	     int m, int n, void *data ) {
    x[0] = p[0] + 10*p[1];
    x[1] = sqrt(5)*( p[2] - p[3] );
    x[2] = (p[1]-2*p[2])*(p[1]-2*p[2]);
    x[3] = sqrt(10)*(p[0] - p[3])*(p[0] - p[3]);
}

void ex3_jac( double* p, double* jac,
	      int m, int n, void* data ) {
    jac[0] = 1;
    jac[1] = 10;
    jac[2] = 0;
    jac[3] = 0;

    jac[4] = 0;
    jac[5] = 0;
    jac[6] = sqrt(5);
    jac[7] = -sqrt(5);

    jac[8] = 0;
    jac[9] = 2*(p[1] - 2*p[2]);
    jac[10] = -4*(p[1] -2*p[2]);
    jac[11] = 0;


    jac[12] = 2*sqrt(10)*(p[0] - p[3]);
    jac[13] = 0;
    jac[14] = 0;
    jac[15] = -2*sqrt(10)*(p[0] - p[3]);

}

