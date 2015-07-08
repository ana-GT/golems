/**
 * @file es_tests_2.cpp
 * @brief Test an individual case, pointcloud is a real one entered from terminal
 */
#include "evaluated_eqs.h"
#include "evaluated_eqs_t.h"
#include <SQ_utils.h>
#include <SQ_fitter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>



/**
 * @function main
 */
int main( int argc, char*argv[] ) {

  // Front
  double alpha, k;
  double x, y, z;
  double X, Y, Z;

  alpha = 0.87; k = 5.0;
  x = 0.3; y = 0.2; z = 0.8;

  
  //
  double r, R, beta, gamma;
  
  gamma = z*k;
  beta = atan2(y,x);
  r = cos(alpha - beta)*sqrt(x*x+y*y);
  R = (1.0/k) - cos(gamma)*(1.0/k - r);

  X = x + cos(alpha)*(R-r);
  Y = y + sin(alpha)*(R-r);
  Z = sin(gamma)*(1.0/k - r );
  
  printf("1. x: %f y: %f z: %f X: %f Y: %f Z: %f \n", x,y,z,X,Y,Z);
  
  // Inverse
  R = cos( alpha - atan2(Y,X))*sqrt(X*X+Y*Y);
  r = 1/k - sqrt( Z*Z + (1.0/k-R)*(1.0/k-R) );
  gamma = atan2(Z,(1.0/k-R));

  x = X - (R-r)*cos(alpha);
  y = Y - (R-r)*sin(alpha);
  z = gamma/k;
  
  printf("2. x: %f y: %f z: %f X: %f Y: %f Z: %f \n", x,y,z,X,Y,Z);
}
