
#include <levmar/levmar.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>


void ex1_fx( double *p, double* x,
	     int m, int n, void *data );
void ex1_jac( double* p, double* jac,
	      int m, int n, void* data );


int main( int argc, char* argv[] ) {
    
    srandom(0);
    const int n = 1; // Size of measurements
    const int m = 1; // Size of parameters

    double p[m]; double x[n];

    double opts[LM_OPTS_SZ];
    double info[LM_INFO_SZ];

    int i, ret;
    
    x[0] = 0;
    p[0] = 5;
 
    double dt; clock_t ts, tf;
    ts = clock();
    ret = dlevmar_der( ex1_fx, ex1_jac,
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

    std::cout << "Best fit parameter: "<< p[0] << std::endl;

    return 0;

}


void ex1_fx( double *p, double* x,
	     int m, int n, void *data ) {
    x[0] = (0.5)*(10.0-p[0])*(10-p[0]);
}

void ex1_jac( double* p, double* jac,
	      int m, int n, void* data ) {
    jac[0] = p[0] = 10;
}

