/**
 * @file levmar_eqs.h
 */
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <levmar/levmar.h>
#include <iostream>

/**
 * @struct Contains the matching robot-kinect points info
 */
struct levmar_data {
  double* xr;
  double* yr;
  double* zr;
  double* xk;
  double* yk;
  double* zk;
  int num;
};

void levmar_fx( double *p, double* x, int m, int n, void *data );
void levmar_jac( double* p, double* jac,
		 int m, int n, void* data );
