
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <stdio.h>


int main( int argc, char* argv[] ) {
 
  // 0. Make sure input name of pcd for model is entered
  if( argc != 2 ) {
    printf("Syntax: %s MODEL.pcd \n", argv[0]);
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

  // 2. Modify it
  Eigen::Affine3d Tf;
  Eigen::Matrix3d rot;
//  rot << 0.866, -0.5, 0, 0.5, 0.866, 0, 0, 0, 1;
  rot << 0, -1, 0, 1, 0, 0, 0, 0, 1;

  Tf.setIdentity();
  Tf.translation() << 0.03, 0.02, 0.01;
  Tf.linear() = rot; 
  printf("Read model okay. Now modify it \n");  
  pcl::transformPointCloud( *model, *test, Tf );
  printf("Transformed fine \n");

  // 3. Test ICP and show result
  pcl::io::savePCDFileASCII( "test.pcd", *test );

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource( model );
  icp.setInputTarget( test );
  pcl::PointCloud<pcl::PointXYZ> final;
  icp.align( final );

  std::cout << "Has ICP converged? : "<< icp.hasConverged() << std::endl;
  std::cout<< " Final transformation: \n"<< icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f tf;
  tf = icp.getFinalTransformation();  
  Eigen::Matrix4f tfi;
  tfi = tf.inverse();

  pcl::transformPointCloud( *test, final, tfi );
  pcl::io::savePCDFileASCII( "transformed.pcd", final );
  return 0;
}
