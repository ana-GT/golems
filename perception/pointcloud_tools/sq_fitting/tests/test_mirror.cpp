/**
 * @file test_mirror.cpp
 * @brief Test SQ fitting with mirror
 */
#include <pcl/io/pcd_io.h>
#include <string>
#include <unistd.h>

#include <SQ_fitter.h>
#include <tabletop_symmetry/mindGapper.h>

typedef typename  pcl::PointXYZ PointT;

pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
pcl::PointCloud<PointT>::Ptr mirrored( new pcl::PointCloud<PointT>() );
pcl::PointCloud<PointT>::Ptr cloud_fitted( new pcl::PointCloud<PointT>() );
pcl::PointCloud<PointT>::Ptr mirror_fitted( new pcl::PointCloud<PointT>() );
const int num_poses = 5;
std::vector<double> cloud_times(num_poses);
std::vector<double> mirror_times(num_poses);

std::vector<double> cloud_errors(num_poses);
std::vector<double> mirror_errors(num_poses);


std::vector<double> tableCoeffs(4);
std::string mirrorFilename;

std::string dataset_loc = std::string("/home/ana/Research/sq_dataset");

std::string obj_name;

void printHelp( char* argv0 ) {
	printf( "Syntax: %s \n",  argv0 );
	printf( " -p obj_name (i.e. macaroni, orange) \n" );
}



/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // 0. Variables needed
  SQ_fitter<PointT> fitter;
  
  // 0. Read pointcloud from input
  int v;
  std::string filename;
  int minType = LEVMAR_MINIMIZER;
  while( (v=getopt( argc, argv, "p:h")) != -1 ) {
	
    switch(v) {
    case 'p': {
		obj_name = std::string( optarg );
    } break;
      
	case 'h': {
		printHelp( argv[0] ); return 1;
	} break;
    } // end switch
	
  }

  // For all poses
  for( int i = 0; i < num_poses; ++i ) {
	  
	  // Load pointcloud
	  char pcd_name[100];
	  sprintf( pcd_name, "%s/%s_%d.pcd", dataset_loc.c_str(), obj_name.c_str(), i );
      if( pcl::io::loadPCDFile<PointT>( pcd_name,
										*cloud ) == -1 ) {
		  printf( "\t [ERROR] Could not read pointcloud %s \n", pcd_name );
		  return 1;
	  }
	  
	  // Load mirror file
	  char mirror_name[100];
	  sprintf( mirror_name, "%s/%s_%d.txt", dataset_loc.c_str(), obj_name.c_str(), i );

      FILE* pFile; pFile = fopen( mirror_name, "r");
      float a, b, c, d;
      fscanf( pFile, " %f %f %f %f", &a, &b, &c, &d );
      fclose( pFile );
      tableCoeffs[0] = (double) a; tableCoeffs[1] = (double) b;
      tableCoeffs[2] = (double) c; tableCoeffs[3] = (double) d;
	  if( tableCoeffs[0] == 0 && tableCoeffs[1] == 0 && tableCoeffs[2] == 0 && tableCoeffs[3] == 0 ) {
		  printf("\t [ERROR] Could not read mirror file %s \n", mirror_name );
		return 1;
	  }

	  //////////////////////// COMPARISON ///////////////////////////////

	  // 1. Generate a mirror version of the pointcloud
	  mindGapper<PointT> mg;
	  mg.setTablePlane( tableCoeffs );
	  mg.setFittingParams();
	  mg.setDeviceParams();
	  
	  *mirrored = *cloud;
	  mg.complete( mirrored );

	  char mirrored_name[100];
	  sprintf( mirrored_name, "%s_mirrored_%d.pcd", obj_name.c_str(), i );
	  pcl::io::savePCDFileASCII ( mirrored_name, *mirrored );  
	  
	  double smin, smax;
	  int N;
	  double thresh;
	  
	  thresh = 0.1;
	  N = 5;

	  /*
		double dim[3]; double trans[3]; double rot[3];
		double minDist;
		fitter.getBoundingBox( cloud, dim, trans, rot, false );
		if( dim[0] <= dim[1] && dim[0] <= dim[2] ) { minDist = dim[0]; }
		if( dim[1] <= dim[0] && dim[1] <= dim[2] ) { minDist = dim[1]; }
		if( dim[2] <= dim[0] && dim[2] <= dim[1] ) { minDist = dim[2]; }
		smin = minDist*2.0 / 15.0;
		smax = minDist*2.0 / 5.0;
	  */
  
	  smin = 0.005;
	  smax = 0.03;
	  
	  //std::cout << "\t * [FIT] Voxel lim: ( "<<smin<<","<<smax<<"), N: "<< N<<"   thresh: "<< thresh << std::endl;

	  // 2. Fitting comparison
	  SQ_parameters cloud_params;
	  
	  SQ_parameters mirror_params;
	  bool status;
	  
	  // 2.a. Fit using no mirror
	  fitter.setInputCloud( cloud );
	  time_t ts, tf; double dt;

	  ts = clock();
	  status = fitter.fit( minType, 
					  smax, smin,
						   N, thresh);
	  tf = clock();
	  cloud_times[i] = (tf - ts) / (double) CLOCKS_PER_SEC;
	  printf("\t [%d] Cloud fitting time: %f \n", i, cloud_times[i] );
	  if( status == true ) {
		  
		  fitter.getFinalParams( cloud_params );
		  cloud_fitted = fitter.getSampledOutput();
		  
		  char cloud_fitted_name[100];
		  sprintf( cloud_fitted_name, "%s_%d_cloud_fitted.pcd", obj_name.c_str(), i );
		  pcl::io::savePCDFileASCII ( cloud_fitted_name, *cloud_fitted ); 
		  cloud_errors[i] = fitter.error_metric( cloud_params, cloud );
		  printf("\t [%d] Cloud error: %f \n", i, cloud_errors[i] );
	  } else {
		  std::cout << "\t [BAD] NO Fit superquadric CLOUD" << std::endl;
		  
	  }



	  // 2.b. Fit using mirrored version
	  fitter.setInputCloud( mirrored );

	  ts = clock();
	  status = fitter.fit( minType, 
						   smax, smin,
						   N, thresh);
	  tf = clock();
	  mirror_times[i] = (tf - ts) / (double) CLOCKS_PER_SEC;
	  printf("\t [%d] Mirror fitting time: %f \n", i, mirror_times[i] );

	  if( status == true ) {
		  
		  fitter.getFinalParams( mirror_params );
		  mirror_fitted = fitter.getSampledOutput();

		  char mirror_fitted_name[100];
		  sprintf( mirror_fitted_name, "%s_%d_mirror_fitted.pcd", obj_name.c_str(), i );

		  pcl::io::savePCDFileASCII ( mirror_fitted_name, *mirror_fitted ); 
		  mirror_errors[i] = fitter.error_metric( mirror_params, cloud );
		  printf("\t [%d] Mirror error: %f \n", i, mirror_errors[i] );
		  
	  } else {
		  std::cout << "\t [BAD] NO Fit superquadric MIRROR" << std::endl;
	  }




  } // end for
  
  // Print
  FILE* pFile;
  char result_name[50];
  double mirror_time_avg = 0;
  double cloud_time_avg = 0;
  double mirror_error_avg = 0;
  double cloud_error_avg = 0;

  sprintf( result_name, "%s_results.txt", obj_name.c_str() );
  pFile = fopen( result_name, "w" );
  for( int i = 0; i < num_poses; ++i ) {
	  fprintf( pFile, "Mirror time [%d]: %f \n", i, mirror_times[i] );
	  fprintf( pFile, "Mirror error [%d]: %f \n", i, mirror_errors[i] );

	  mirror_time_avg += mirror_times[i];
	  mirror_error_avg += mirror_errors[i];

	  fprintf( pFile, "Cloud time [%d]: %f \n", i, cloud_times[i] );
	  fprintf( pFile, "Cloud error [%d]: %f \n", i, cloud_errors[i] );

	  cloud_time_avg += cloud_times[i];
	  cloud_error_avg += cloud_errors[i];

  } 			

  mirror_time_avg /= (double) num_poses;
  mirror_error_avg /= (double) num_poses;

  cloud_time_avg /= (double) num_poses;
  cloud_error_avg /= (double) num_poses;

  fprintf( pFile, "\n");

  fprintf( pFile, "Mirror avg time: %f \n", mirror_time_avg);
  fprintf( pFile, "Mirror avg error: %f \n", mirror_error_avg);

  fprintf( pFile, "Cloud avg time: %f \n", cloud_time_avg);
  fprintf( pFile, "Cloud avg error: %f \n", cloud_error_avg);


  fclose( pFile );

  return 0; 
}


// Local Variables:
// mode: c++
// tab-width: 4
// End:
