
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
  types[0] = REGULAR; 
  types[1] = REGULAR;
  fitter.fit( SQ_FX_ICHIM, 
	      types,
	      2,
	      PERPENDICULAR_TO_Z,
	      0.03, 0.005, 5, 0.005 );
  
  std::vector<SQ_parameters> ps;
  fitter.getFinalParams( ps );
  pcl::PointCloud<PointT>::Ptr P0( new pcl::PointCloud<PointT> );
  pcl::PointCloud<PointT>::Ptr P1( new pcl::PointCloud<PointT> );
  P0 = sampleSQ_uniform<PointT>( ps[0], true );
  P1 = sampleSQ_uniform<PointT>( ps[1], true );

  // Tell results
  pcl::io::savePCDFile( "final_part_0.pcd", *P0 );
  pcl::io::savePCDFile( "final_part_1.pcd", *P1 );

  printf("Final params: \n");
  printParamsInfo( ps[0] );
  printParamsInfo( ps[1] );
  // Get the mesh
  SQ_utils::convertMeshes( ps, "final_mesh.ply" );
}
