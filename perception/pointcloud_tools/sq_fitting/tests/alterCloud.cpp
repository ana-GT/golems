/**
 * @function alterCloud.cpp
 * @brief Rotate / translate cloud + add noise
 */
#include <unistd.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <random>

/** Functions declaration */
void printHelp( char* argv0 );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  std::random_device rd;
  std::mt19937 gen( rd() );

  int v;
  float ra, pa, ya; 
  float x, y, z;
  bool noise_flag;
  float noise_level;
  std::string output_name;
  std::string input_name;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud( new pcl::PointCloud<pcl::PointXYZ>() );  
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud( new pcl::PointCloud<pcl::PointXYZ>() );  

  // 0. Init values
  ra = 0; pa = 0; ya = 0;
  x = 0; y = 0; z = 0;
  noise_flag = false;

  // 1. Read arguments
  while( (v=getopt(argc,argv,"hR:P:Y:x:y:z:n:o:p:"))!= -1 ) {
  
    switch(v) {
    
    case 'h':{
      printHelp( argv[0] );
    } break;

    case 'R': {
      ra = atof( optarg );
    } break;

    case 'P': {
      pa = atof( optarg );
    } break;

    case 'Y': {
      ya = atof( optarg );
    } break;

    case 'x': {
      x = atof( optarg );
    } break;

    case 'y': {
      y = atof( optarg );
    } break;

    case 'z': {
      z = atof( optarg );
    } break;

    case 'n': {
      noise_flag = true;
      noise_level = atof( optarg );
    } break;

    case 'o': {
      output_name = std::string( optarg );
    } break;

    case 'p': {
      input_name = std::string( optarg );
      if( pcl::io::loadPCDFile<pcl::PointXYZ>( input_name,
					       *input_cloud ) == -1 ) {
	std::cout << "\t [ERROR] Could not read pointcloud "<<
	  input_name<< std::endl;
	return 1;
      }
    } break;
    case '?': {
      printHelp( argv[0] );
      return 1;
    } break;
      
    }// end of switch

  } // end of while

  if( input_name.size() == 0 || output_name.size() == 0 ) {
    printf("\t [ERROR] Input or output name of pointcloud not set! \n");
    printHelp(argv[0]);
    return 1;
  }

  // Create transform rotate/translate
  Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
  tf.block(0,3,3,1) = Eigen::Vector3f( x,y,z );
  Eigen::Matrix3f rot;
  rot = Eigen::AngleAxisf( ya, Eigen::Vector3f::UnitZ() )*
    Eigen::AngleAxisf( pa, Eigen::Vector3f::UnitY() )*
    Eigen::AngleAxisf( ra, Eigen::Vector3f::UnitX() );
  tf.block(0,0,3,3) = rot;

  pcl::transformPointCloud( *input_cloud,
			    *output_cloud,
			    tf );

  // If noise active
  if( noise_flag ) {

    std::uniform_real_distribution<> dis( -1*noise_level, noise_level);
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    for( it = output_cloud->begin(); it != output_cloud->end(); ++it ) {
      (*it).x += dis(gen);
      (*it).y += dis(gen);
      (*it).z += dis(gen);
    }

  }
  

  // Save output cloud
  pcl::io::savePCDFileASCII( output_name,
			     *output_cloud );
  

  return 0;

}


/**
 * @function printHelp
 */
void printHelp( char* argv0 ) {
  std::cout<<"Syntax: "<< argv0<<std::endl;
  std::cout<<"\t -p P: Input pointcloud"<<std::endl;
  std::cout<<"\t -o O: Output pcd filename"<<std::endl;
  std::cout<<"\t -x/y/z: Translation "<<std::endl;
  std::cout<<"\t -R/P/Y: Rotation "<<std::endl;
  std::cout<<"\t -h This help"<<std::endl;
  std::cout<<"\t -n Max. noise"<< std::endl;

}
