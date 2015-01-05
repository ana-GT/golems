/**
 * @file test_noMirror.cpp
 * @brief Test SQ fitting with no mirror
 */
#include <pcl/io/pcd_io.h>
#include <string>
#include <unistd.h>

#include <SQ_fitter.h>
#include <tabletop_symmetry/mindGapper.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
pcl::PointCloud<pcl::PointXYZ>::Ptr mirrored( new pcl::PointCloud<pcl::PointXYZ>() );
std::vector<double> tableCoeffs(4);
std::string mirrorFilename;
void printHelp( char* argv0 ) {
  printf("Syntax: %s \n", argv0 );
  printf("\t -p filename.pcd \n" );
}


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // 0. Variables needed
  SQ_fitter<pcl::PointXYZ> fitter;
  
  // 0. Read pointcloud from input
  int v;
  std::string filename;
  int minType = LEVMAR_MINIMIZER;
  while( (v=getopt( argc, argv, "p:h")) != -1 ) {
	
    switch(v) {
    case 'p': {
      filename = std::string( optarg );
      if( pcl::io::loadPCDFile<pcl::PointXYZ>( filename,
											   *cloud ) == -1 ) {
		std::cout<<"\t [ERROR] Could not read pointcloud "<<filename<< std::endl;
		return 1;
      } 
    } break;

	case 'h': {
      printHelp( argv[0] ); return 1;
	} break;
    } // end switch
	
  }
  
  if( filename.size() == 0 ) {
    printHelp( argv[0] );
	return 1;
  }

  // 1. Generate a mirror version of the pointcloud
  mindGapper<pcl::PointXYZ> mg;
  mg.setTablePlane( tableCoeffs );
  mg.setFittingParams();
  mg.setDeviceParams();

  *mirrored = *cloud;
  mg.complete( mirrored );


  pcl::io::savePCDFileASCII ( "mirrored.pcd", *mirrored );  


  // 2. Load pointcloud in fitter object
  fitter.setInputCloud( mirrored );
  
  double smin, smax;
  int N;
  double thresh;

  thresh = 0.1;
  N = 5;
  
  smin = 0.005;
  smax = 0.03;

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

  // 2. Fit. If successful, visualize and spit out summary

  std::cout << "\t * [FIT] Voxel lim: ( "<<smin<<","<<smax<<"), N: "<< N<<"   thresh: "<< thresh << std::endl;
  if( fitter.fit( minType, 
				  smax, smin,
				  N, thresh) ) {

	std::cout << "\t [GOOD] Fit superquadric"<< std::endl;

	// 3. Visualize
	fitter.visualize();
	pcl::PointCloud<pcl::PointXYZ>::Ptr fitted;
	fitted = fitter.getSampledOutput();
	char name[50];
	sprintf( name, "fit.pcd", filename.c_str() );
	pcl::io::savePCDFile( name, *fitted, true );

  } else {
	std::cout << "\t [BAD] NO Fit superquadric" << std::endl;
  }
			
  return 0; 
}


// Local Variables:
// mode: c++
// tab-width: 4
// End:
