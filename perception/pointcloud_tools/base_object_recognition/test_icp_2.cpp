
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <stdio.h>


int main( int argc, char* argv[] ) {
 
  // 0. Make sure input name of pcd for model is entered
  if( argc != 3 ) {
    printf("Syntax: %s MODEL.pcd TEST.pcd \n", argv[0]);
    return 1;
  }


  // 1. Read the pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr model( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr test( new pcl::PointCloud<pcl::PointXYZ> );
	
  int i = pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *model );
  if( i == -1 ) {
    printf("Could not read pcd file for model \n");
    return 1;
  }

  i = pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *test );
  if( i == -1 ) {
    printf("Could not read pcd file for test \n");
    return 1;
  }

  // 3. Test ICP and show result
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource( model );
  icp.setInputTarget( test );
  pcl::PointCloud<pcl::PointXYZ>::Ptr final( new pcl::PointCloud<pcl::PointXYZ> );
  icp.align( *final );

  std::cout << "Has ICP converged? : "<< icp.hasConverged() << std::endl;
  std::cout<< " Final transformation: \n"<< icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f tf;
  tf = icp.getFinalTransformation();  
  Eigen::Matrix4f tfi;
  tfi = tf.inverse();

  pcl::transformPointCloud( *test, *final, tfi );
  pcl::io::savePCDFileASCII( "transformed.pcd", *final );
  
  // 4. Do it again
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
  icp2.setInputSource( model );
  icp2.setInputTarget( final );
  pcl::PointCloud<pcl::PointXYZ> final2;
  icp2.align( final2 );


  std::cout << "2. Has ICP converged? : "<< icp2.hasConverged() << std::endl;
  std::cout<< " 2. Final transformation: \n"<< icp2.getFinalTransformation() << std::endl;

  Eigen::Matrix4f tf2;
  tf2 = icp2.getFinalTransformation();  
  Eigen::Matrix4f tfi2;
  tfi2 = tf2.inverse();

  pcl::transformPointCloud( *final, final2, tfi2 );
  pcl::io::savePCDFileASCII( "transformed2.pcd", final2 );
  
  
  return 0;
}
