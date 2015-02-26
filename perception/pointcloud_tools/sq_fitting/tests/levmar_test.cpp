/**
 * @file levmar_test.cpp
 */
extern "C" {
#include <levmar/levmar.h>
}
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

double gNoise( double m, double s ) {
    double r1, r2, val;
    r1 = ((double)random())/RAND_MAX;
    r2 = ((double)random())/RAND_MAX;

    val = sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2);
    val = s*val + m;

    return val;
}

void expfunc( double *p, double *x, int m, int n, void *data ) {
    register int i;
    for( i = 0; i < n; ++i ) {
	x[i] = p[0]*exp(-p[1]*i) + p[2];
    }
}

void jacexpfunc( double *p, 
		 double *jac,
		 int m, int n,
		 void* data ) {

    register int i, j;
    for( i = j = 0; i <n; ++i ) {
	jac[j++] = exp(-p[1]*i );
	jac[j++] = -p[0]*i*exp(-p[1]*i);
	jac[j++] = 1.0;
    }

}

int main( int argc, char* argv[] ) {

    srandom(0);

    const int n  = 40; // # of measurements
    const int m = 3; // # of parameters to estimate
    
    double p[m];
    double x[n];
    double opts[LM_OPTS_SZ];
    double info[LM_INFO_SZ];

    int i;
    int ret;

    // Generate measurements with some error on it
    // Params: (5.0, 0.1, 1.0) - error: Gaussian noise with s = 0.1
    for( i=0; i <n; ++i ) {
	x[i] = (5.0*exp(-0.1*i)+1.0) + gNoise(0.0, 0.1);
    }


    // Initial values for parameters
    p[0] = 1.0;
    p[1] = 0.0;
    p[2] = 0.0;

    // Optimization control parameters
    opts[0] = LM_INIT_MU;
    opts[1] = 1E-15;
    opts[2] = 1E-15;
    opts[3] = 1E-20;
    opts[4] = LM_DIFF_DELTA;

    // Call optimization with analytic jacobian
    double dt; clock_t ts,tf;
    ts = clock();
    ret = dlevmar_der( expfunc, jacexpfunc,
		       p, x, 
		       m, n,
		       1000,
		       opts, info,
		       NULL, NULL, NULL );


    tf = clock();
    dt = (double)(tf-ts)/(double)CLOCKS_PER_SEC;
    std::cout << "Calculation time: "<< dt << std::endl;
    std::cout << " Levenberg Marquardt returned in "<<info[5]<<" iterations "<<
	", reason: "<< info[6] << " sumsq: "<< info[1] <<"["<<info[0]<<"]"<<std::endl;

    std::cout << "Best fit parameters: "<< p[0]<<", "<< p[1]<<", "<< p[2] << std::endl;

    return 0;
}


