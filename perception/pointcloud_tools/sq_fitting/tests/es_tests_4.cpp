/**
 * @file es_tests_2.cpp
 * @brief Test an individual case, pointcloud is a real one entered from terminal
 */
#include "perception/pointcloud_tools/sq_fitting/evaluated_eqs.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"
#include "perception/pointcloud_tools/tabletop_symmetry/mindGapper.h"

char* fx_names[6] = { "Radial", "Solina", "Ichim", "Chevalier", "F5", "F6"};
int fx_sq[6] = {SQ_FX_RADIAL, SQ_FX_SOLINA, SQ_FX_ICHIM, SQ_FX_CHEVALIER, SQ_FX_5, SQ_FX_6 };
std::string gInput, gOutput;

typedef pcl::PointXYZ PointT;

/**
 * @function downsampling
 */
pcl::PointCloud<PointT>::Ptr downsampling( const pcl::PointCloud<PointT>::Ptr &_input,
						  const double &_voxelSize ) {

  pcl::PointCloud<PointT>::Ptr cloud_downsampled( new pcl::PointCloud<PointT>() );
  
  // Create the filtering object
  pcl::VoxelGrid< PointT > downsampler;
  // Set input cloud
  downsampler.setInputCloud( _input );
  // Set size of voxel
  downsampler.setLeafSize( _voxelSize, _voxelSize, _voxelSize );
  // Downsample
  downsampler.filter( *cloud_downsampled );

  return cloud_downsampled;
}

/**
 * @function main
 */
int main( int argc, char*argv[] ) {

  int v;
  while( (v=getopt(argc, argv, "n:o:h")) != -1 ) {
    switch(v) {

    case 'n' : {
      gInput = std::string(optarg);
    } break;
    case 'o' : {
      gOutput = std::string(optarg);
    } break;
    case 'h': {
      printf("Usage: %s -n input_pcd -o output_data \n", argv[0]);
      return 0;
    } break;
    } // switch end
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr input( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr down( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::io::loadPCDFile<pcl::PointXYZ> ( gInput.c_str(), *input ); 
   down = downsampling(input,0.01);

  mindGapper<pcl::PointXYZ> mg;
  std::vector<double> gTableCoeffs(4);
  gTableCoeffs[0] = 0.0163878;
  gTableCoeffs[1] = 0.526281;
  gTableCoeffs[2] = -0.850153;
  gTableCoeffs[3] = 0.667902;

  mg.setTablePlane( gTableCoeffs );
  mg.setFittingParams();
  mg.setDeviceParams();
  mg.setFocalDist(574);
  
  // Fit pointcloud to superquadric
  pcl::PointCloud<pcl::PointXYZ>::Ptr completed( new pcl::PointCloud<pcl::PointXYZ>() );
  Eigen::Isometry3d Tsymm; Eigen::Vector3d Bb;
  
  *completed = *input;
  mg.reset();

  mg.complete( completed, false );
  printf("Finished completing \n");
  mg.getSymmetryApprox( Tsymm, Bb );
/*
  std::cout << "mBB: \n"<< Bb.transpose() << std::endl;
  std::cout << "mSymmTf: \n"<< Tsymm.matrix() << std::endl;
  std::cout << "Translation: "<< Tsymm.translation()(0)<<","
	    << Tsymm.translation()(1)<<","
	    << Tsymm.translation()(2)  << std::endl;
  Eigen::Quaterniond q( Tsymm.linear() );
  char command[200];
  std::cout << "Rotation q: "<< q.x() << ","<<q.y()<<","<<q.z()<<","<<q.w()<<std::endl;
  sprintf( command, "./visualization_cube_cloud -f %s -x %f -y %f -z %f -r %f -p %f -q %f -n %f -a %f -b %f -c %f \n",
          gInput.c_str(),
	  Tsymm.translation()(0), Tsymm.translation()(1), Tsymm.translation()(2),
	  q.x(), q.y(), q.z(), q.w(),
	  Bb(0), Bb(1), Bb(2) );
  system(command);
*/
  
  clock_t ts, tf; double dt;
 // SQ_fitter_evaluated<pcl::PointXYZ> sfe;
  //sfe.setInitialApprox( Tsymm, Bb );
//  sfe.setInputCloud( input );
  
 
  double er1, er2, er4;
  SQ_parameters par;
  pcl::PointCloud<pcl::PointXYZ>::Ptr approx( new pcl::PointCloud<pcl::PointXYZ>() );
  int n; n = input->points.size();
  
  for( int i = 0; i < 6; ++i ) {

    printf( "Function: %s \n", fx_names[i] );
    ts = clock();
//    sfe.fit( fx_sq[i] );
    evaluated_sqs ee;
    ee.minimize( down, par, er1, er2, er4, fx_sq[i] );
    tf = clock();
    dt = (tf-ts) / (double) CLOCKS_PER_SEC;

    //sfe.getFinalParams( par );
/*
    error_metric( par, input, er1, er2, er4 );
    printf("Dim: %f %f %f \t e: %f %f \t t: %f \t er_g: %f er_r: %f  \n",
	   par.dim[0], par.dim[1], par.dim[2],
	   par.e[0], par.e[1], dt,
	   er1, er2);
    */
    approx = sampleSQ_uniform<pcl::PointXYZ>( par );
    char name[50];
    sprintf( name, "%s_hierarchical_%d.pcd", gOutput.c_str(), i );
    pcl::io::savePCDFileASCII( name, *approx );
  }
  
}

