/**
 * @file deformations_tests_0.cpp
 * @brief Testing simple 1-DOF bending deformation
 */
#include "evaluated_eqs.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_deformations.h"

/**
 * @function main
 */
int main( int argc, char*argv[] ) {

  double Rfactor = 1.4;
  double tamp = 0.5;
  
  int v;
  while( (v=getopt(argc, argv, "t:b:")) != -1 ) {
    switch(v) {

    case 't' : {
      tamp = atof(optarg);
    } break;
    case 'b' : {
      Rfactor = atof(optarg);
    } break;
    } // switch end
  }

  
  SQ_deformations sd;

  double a3 = 0.1;
  double a2 = 0.025;
  double a1 = 0.025;
  
  sd.bending( a1, a2, a3, 0.1, 1.0, Rfactor*a3);
  sd.tampering( a1, a2, a3, 0.1, 1.0, tamp);
  
  return 0;  
}
