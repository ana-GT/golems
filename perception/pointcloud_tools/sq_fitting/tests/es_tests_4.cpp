/**
 * @file es_tests_2.cpp
 * @brief Test an individual case, pointcloud is a real one entered from terminal
 */
#include "evaluated_eqs.h"
#include <SQ_utils.h>
#include <SQ_fitter_evaluated.h>
#include <tabletop_symmetry/mindGapper.h>

char* fx_names[6] = { "Radial", "Solina", "Ichim", "Chevalier", "F5", "F6"};
int fx_sq[6] = {SQ_FX_RADIAL, SQ_FX_SOLINA, SQ_FX_ICHIM, SQ_FX_CHEVALIER, SQ_FX_5, SQ_FX_6 };
std::string gInput, gOutput;

/**
 * @function main
 */
int main( int argc, char*argv[] ) {

  int v;
  while( (v=getopt(argc, argv, "n:o:")) != -1 ) {
    switch(v) {

    case 'n' : {
      gInput = std::string(optarg);
    } break;
    case 'o' : {
      gOutput = std::string(optarg);
    } break;

    } // switch end
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr input( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::io::loadPCDFile<pcl::PointXYZ> ( gInput.c_str(), *input ); 


  mindGapper<pcl::PointXYZ> mg;
  std::vector<double> gTableCoeffs(4);
  gTableCoeffs[0] = -0.0163878;
  gTableCoeffs[1] = -0.526281;
  gTableCoeffs[2] = 0.850153;
  gTableCoeffs[3] = -0.667902;

  mg.setTablePlane( gTableCoeffs );
  mg.setFittingParams();
  mg.setDeviceParams();
  mg.setFocalDist(574);
  
  // Fit pointcloud to superquadric
  pcl::PointCloud<pcl::PointXYZ>::Ptr completed( new pcl::PointCloud<pcl::PointXYZ>() );
  Eigen::Isometry3d Tsymm; Eigen::Vector3d Bb;
  
  *completed = *input;
  mg.reset();

  mg.complete( completed );
  printf("Finished completing \n");
  mg.getSymmetryApprox( Tsymm, Bb );
  /*
  clock_t ts, tf; double dt;
  SQ_fitter_evaluated<pcl::PointXYZ> sfe;
  sfe.setInitialApprox( Tsymm, Bb );
  sfe.setInputCloud( input );
  
 
  double er1, er2, er4;
  SQ_parameters par;
  pcl::PointCloud<pcl::PointXYZ>::Ptr approx( new pcl::PointCloud<pcl::PointXYZ>() );
  int n; n = input->points.size();
  
  for( int i = 0; i < 6; ++i ) {

    printf( "Function: %s \n", fx_names[i] );
    ts = clock();
    sfe.fit( fx_sq[i] );
    tf = clock();
    dt = (tf-ts) / (double) CLOCKS_PER_SEC;

    sfe.getFinalParams( par );
    error_metric( par, input, er1, er2, er4 );
    printf("Dim: %f %f %f \t e: %f %f \t t: %f \t er_g: %f er_r: %f  \n",
	   par.dim[0], par.dim[1], par.dim[2],
	   par.e[0], par.e[1], dt,
	   er1, er2);
    
    approx = sampleSQ_uniform( par );
    char name[50];
    sprintf( name, "%s_hierarchical_%d.pcd", gOutput.c_str(), i );
    pcl::io::savePCDFileASCII( name, *approx );
  }
  */
}

