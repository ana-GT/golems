/**
 * @file deformations_tests_1.cpp
 * @brief Testing bending
 */
#include "perception/pointcloud_tools/sq_fitting/evaluated_eqs.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_deformations.h"

/**
 * @function main
 */
int main( int argc, char*argv[] ) {

  double k, alpha;
  std::string name = std::string("default_b2.pcd");
  
  k = 1.0; alpha = 0.87;
  
  int v;
  while( (v=getopt(argc, argv, "k:a:n:h")) != -1 ) {
    switch(v) {

    case 'k' : {
      k = atof(optarg);
    } break;
    case 'a' : {
      alpha = atof(optarg);
    } break;
    case 'n' : {
      name = std::string(optarg);
    } break;
    case 'h' : {
      printf("Syntax: ./executable -k K_bending -a alpha_bending -n name_bent_pcd_output\n");
      return 0;
    } break;
    } // switch end
  }

  

  SQ_parameters par;
  par.dim[0] = 0.03; par.dim[1] = 0.03; par.dim[2] = 0.12;
  par.e[0] = 0.1; par.e[1] = 1.0;
  par.trans[0] = 0; par.trans[1] = 0; par.trans[2] = 0;
  par.rot[0] = 0; par.rot[1] = 0; par.rot[2] = 0;
  par.alpha = alpha; par.k = k;

  pcl::PointCloud<pcl::PointXYZ>::Ptr approx( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr really( new pcl::PointCloud<pcl::PointXYZ>() );
  approx = sampleSQ_uniform_b2( par );
  really = sampleSQ_uniform(par);

  printf("ALpha: %f \n", par.alpha);

  pcl::io::savePCDFileASCII( name, *approx );
  pcl::io::savePCDFileASCII( "real.pcd", *really );
  
  return 0;  
}
