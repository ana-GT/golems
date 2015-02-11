/**
 * @file cloud_basicOperations
 */

#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <stdlib.h>

#include "SQ_fitter.h"

/** Function declarations */
void printHelp();
void show( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud );
void getBoundingBox( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
		     Eigen::Vector3f &_centroid,
		     Eigen::Matrix3f &_rot,
		     Eigen::Vector3f &_dim,
		     pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud_transf );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  int v;
  std::string output_name = "output_name.pcd";
  std::string input_name;

  bool scaleCloud_flag = false;
  bool centerCloud_flag = false;

  double scale = 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr mid_cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud( new pcl::PointCloud<pcl::PointXYZ>() );

  while( (v=getopt(argc, argv, "p:hs:o:c") ) != -1 ) {
    switch(v) {
      
      /** Input pointcloud */
    case 'p' : {
      
      input_name = std::string( optarg );
      if( pcl::io::loadPCDFile<pcl::PointXYZ>( input_name,
					       *input_cloud ) == -1 ) {
	std::cout << "\t [ERROR] Could not read pointcloud "<<
	  input_name<< std::endl;
	return 1;
      }

    } break;

      /**Help */
    case 'h' : {
      printHelp();
    } break;

      /** Scale pointcloud */
    case 's' : {
      scaleCloud_flag = true;
      scale = atof( optarg );
    } break;

      /** Store pointcloud in filename  */      
    case 'o' : {
      output_name = std::string( optarg );
    } break;

      /** Center and normalize pointcloud */
    case 'c' : {
      centerCloud_flag = true;
    } break;
      
    } // switch
  }
  
  // If input pointcloud not provided, scream at user:
  if( input_name.size() == 0 ) {
    printHelp();
    return 1;
  }

  // Get bounding box information
  Eigen::Vector3f centroid, dim; 
  Eigen::Matrix3f rot;

  getBoundingBox( input_cloud,
		  centroid,
		  rot,
		  dim,
		  mid_cloud );
  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tf_center = Eigen::Matrix4f::Identity();
  tf_center.block(0,3,3,1) = centroid;
  tf_center.block(0,0,3,3) = rot;  

  // If scale
  if( scaleCloud_flag ) {

    Eigen::Matrix4f tf_sc = Eigen::Matrix4f::Identity();
    for( int i = 0; i < 3; ++i ) { tf_sc(i,i) = scale; }
    
    if( centerCloud_flag ) {
      transf = tf_sc*tf_center.inverse();
    }
    else {
      transf = tf_center*tf_sc*tf_center.inverse();      
    }

  } else {
    if( centerCloud_flag ) {
      transf = tf_center.inverse();
    }
  }


  // Show
  if( centerCloud_flag || scaleCloud_flag ) {
    std::cout << "\t Show centered or scale cloud"<<std::endl;
    pcl::transformPointCloud( *input_cloud, *output_cloud, transf );
    // Save
    pcl::io::savePCDFileASCII( output_name, *output_cloud );        
    std::cout <<"\t [*] Saved "<< output_name  << std::endl;
    show( output_cloud );

  } else {
    std::cout << "\t Show just input cloud"<<std::endl;
    show( input_cloud );
  }

  return 0;
}

/**
 * @function printHelp
 */
void printHelp() {
  std::cout<<"Syntax: ./cloud_basicOperations [i | s | o] "<<std::endl;
  std::cout<<"\t -p P: Input pointcloud"<<std::endl;
  std::cout<<"\t -i: Display bounding box information "<<std::endl;
  std::cout<<"\t -s S: Scale pointcloud"<<std::endl;
  std::cout<<"\t -o O: Output pcd filename"<<std::endl;
}

/**
 * @function getBoundingBox
 * @brief Get bounding box information
 */
void getBoundingBox( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
		     Eigen::Vector3f &_centroid,
		     Eigen::Matrix3f &_rot,
		     Eigen::Vector3f &_dim,
		     pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud_transf ) {

  double dim[3]; double trans[3]; double rot[3];

  SQ_fitter<pcl::PointXYZ> sqf;
  sqf.setInputCloud( _cloud );  
  sqf.getBoundingBox( _cloud,
		      dim, trans, rot );
  _centroid << (float) trans[0], (float) trans[1], (float) trans[2];
  _dim << (float) dim[0], (float) dim[1], (float) dim[2];
  _rot = Eigen::AngleAxisf( _rot(2), Eigen::Vector3f::UnitZ() )*
    Eigen::AngleAxisf( _rot(1), Eigen::Vector3f::UnitY() )*
    Eigen::AngleAxisf( _rot(0), Eigen::Vector3f::UnitX() );
  
  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  transf.block(0,3,3,1) = _centroid;
  transf.block(0,0,3,3) = _rot;
  Eigen::Matrix4f tinv = transf.inverse();

  pcl::transformPointCloud( *_cloud, *_cloud_transf, tinv );
}

/**
 * @function showInfo
 * @brief Show pointcloud in viewer and bounding box information
 */
void show( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud ) {

    // [DEBUG] Visualize 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("showInfo viewer") );
    
    // [DEBUG] Visualize input pointcloud (GREEN)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(_cloud, 0,255,0);
    viewer->addPointCloud( _cloud, col, "input cloud"  );

    
    // [DEBUG] Visualize bounding box
    Eigen::Vector3f centroid, dim; Eigen::Matrix3f rot;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2( new pcl::PointCloud<pcl::PointXYZ>() );

    getBoundingBox( _cloud, centroid, rot, dim, cloud2 );        
    viewer->addCube( centroid,
		     Eigen::Quaternionf(rot),
		     dim(0)*2, dim(1)*2, dim(2)*2, 
		     "OBB" );

    viewer->addCoordinateSystem(dim[2], 0);
    viewer->setBackgroundColor(0,0,0);


    // [PRINT INFO]
    std::cout << "\t [INFO] Bounding box dimensions: "<< dim.transpose() << std::endl;
    std::cout << "\t [INFO] Centroid: "<< centroid.transpose() << std::endl;   
    std::cout << "\t [INFO] Rotation: "<< std::endl;
    std::cout << rot << std::endl;
 
    while( !viewer->wasStopped() ) {
	viewer->spinOnce(100);
	boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
    }

}

