
#include <pcl/io/pcd_io.h>
#include <perception/pointcloud_tools/sq_fitting/SQ_fitter_m.h>
#include <stdio.h>

typedef pcl::PointXYZ PointT;

/**
 * 
 */
int main( int argc, char* argv[] ) {

  // Load pointcloud
  pcl::PointCloud<PointT>::Ptr input( new pcl::PointCloud<PointT> );
  if( pcl::io::loadPCDFile<PointT>( argv[1], *input ) == -1 ){ 
    printf("Error loading %s cloud \n", argv[1]);
    return 1;
  }
  
  
  // Create fitter multiple
  SQ_fitter_m<PointT> fitter;
  fitter.setInputCloud( input );

  // Run in pointcloud 
  std::vector<int> types(2); 
  types[0] = TAMPERED; 
  types[1] = TAMPERED;
  fitter.fit( SQ_FX_ICHIM, 
	      types,
	      2,
	      PERPENDICULAR_TO_Z,
	      0.03, 0.005, 5, 0.005 );
  
  // Tell results
  
}
