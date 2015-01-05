/**
 * @file tabletop_symmetry_test
 * @brief Test with hard-coded segmented pointcloud
 */

#include <tabletop_symmetry/mindGapper.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>

void help( char* argv0 ) {
  std::cout << " Help"<< std::endl;
  std::cout << "Usage: \t "<<argv0<< "-i CANDIDATE_INDEX -c CLOUD_INDEX"<<std::endl;
}

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  int candidate_index = 0;
  int cloud_index = 0;
  int c;

  while( (c = getopt(argc, argv,"i:c:h")) != -1 ) {
  
    switch(c) {
    case 'i':
      candidate_index = atoi(optarg);
      break;
    case 'c':
      cloud_index = atoi(optarg);
      break;
    case 'h':
    case '?':
      help( argv[0] );
      return 0;
    }

  }


  std::cout<<"\t -- Setting candidate index: "<<candidate_index<<" and cloud index: "<< cloud_index <<std::endl;
  

  mindGapper mG;
  std::vector<double> coeffs(4);
  coeffs[0] = 0.0354126; 
  coeffs[1] = -0.830739;
  coeffs[2] = -0.555534;
  coeffs[3] =  0.470276;
  
  // Set plane
  mG.setTablePlane( coeffs );
  
  // Set parameters for the optimization search of the best plane
  mG.setFittingParams( 6, 5, 0.01, M_PI / 9.0 );
  mG.setDeviceParams();

  // Convert (Debug version fires up a 
  char name[100];
  sprintf( name, "/home/ana/Research/toolsLWA4/tabletop_object_perception/bin/testSim/cluster_%d.pcd",
	   cloud_index );
  std::string filename( name );

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  
  if( pcl::io::loadPCDFile<pcl::PointXYZ>( filename.c_str(), *cloud ) == -1 ) {
    std::cout << "\t -- Did not load correctly point cloud "<< filename << std::endl;
    return 1;
  }

  int index = mG.complete( cloud );
  mG.viewMirror( index );
  mG.printMirror( candidate_index );

  return 0;
}
