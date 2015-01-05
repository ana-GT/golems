/**
 * @file SQ_fitter_test1.cpp
 * @brief Test Fit function for Super quadrics
 */
#include <pcl/io/pcd_io.h>
#include <string>
#include <unistd.h>

#include <SQ_fitter.h>

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // 0. Variables needed
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  SQ_fitter<pcl::PointXYZ> fitter;
  
  
  // 1. Read pointcloud from input
  int v;
  std::string filename;
  int minType = LEVMAR_MINIMIZER;
  while( (v=getopt( argc, argv, "p:t:h")) != -1 ) {
	
    switch(v) {
    case 'p': {
      filename = std::string( optarg );
      if( pcl::io::loadPCDFile<pcl::PointXYZ>( filename,
											   *cloud ) == -1 ) {
		std::cout<<"\t [ERROR] Could not read pointcloud "<<filename<< std::endl;
		return 1;
      } 
    } break;
		// Set type of minimizer (ceres or levmar)
	case 't': {
		minType = atoi(optarg);
	} break;
	case 'h': {
	  std::cout <<"Syntax: "<<argv[0]<<" filename.pcd"<< std::endl;
	} break;
    } // end switch
	
  }
  
  if( filename.size() == 0 ) {
	std::cout << "Syntax: "<< argv[0]<< " -p filename.pcd"<< std::endl;
	return 1;
  }


  // 2. Load pointcloud in fitter object
  fitter.setInputCloud( cloud );

  
  double dim[3]; double trans[3]; double rot[3];
  double minDist;
  fitter.getBoundingBox( cloud, dim, trans, rot, false );
  if( dim[0] <= dim[1] && dim[0] <= dim[2] ) { minDist = dim[0]; }
  if( dim[1] <= dim[0] && dim[1] <= dim[2] ) { minDist = dim[1]; }
  if( dim[2] <= dim[0] && dim[2] <= dim[1] ) { minDist = dim[2]; }
  double smin = minDist*2.0 / 15.0;
  double smax = minDist*2.0 / 5.0;

  // 2. Fit. If successful, visualize and spit out summary
  double thresh = 0.1;
  int N = 5;
  std::cout << "\t * Call fitting function with voxel limits (: "<<smin<<","<<smax<<"), N: "<< N<<" and  thresh: "<< thresh << std::endl;
  if( fitter.fit( minType, 
				  smax, smin,
				  N, thresh) ) {

	std::cout << "\t [GOOD] Fit superquadric!"<< std::endl;

	// 3. Visualize
	fitter.visualize();
	pcl::PointCloud<pcl::PointXYZ>::Ptr fitted;
	fitted = fitter.getSampledOutput();
	char name[50];
	sprintf( name, "fit.pcd", filename.c_str() );
	pcl::io::savePCDFile( name, *fitted, true );

  } else {
	std::cout << "\t [BAD] Did not fit properly" << std::endl;
  }
			
  return 0; 
}


// Local Variables:
// mode: c++
// tab-width: 4
// End:
