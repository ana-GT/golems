/**
 * @file basic_kMeans.cpp
 */
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "kmeans.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGB PointT;

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  int num_clusters = 2;
  srand(time(NULL));

  // Read pointcloud
  pcl::PointCloud<PointT>::Ptr input (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);


  if( pcl::io::loadPCDFile (argv[1], *input) == -1 )  {
    printf("Pointcloud cuold not be read: %s \n", argv[1] );
    return 1;
  }

  // Get min max
  PointT minP, maxP;
  pcl::getMinMax3D( *input, minP, maxP );
  printf( "min: %f %f %f and max: %f %f %f \n", minP.x, minP.y, minP.z, maxP.x, maxP.y, maxP.z );
  
  // Transform pointcloud to data format (vector of points)
  std::vector< std::vector<float> > data;
  for( pcl::PointCloud<PointT>::iterator it = input->begin(); it != input->end(); ++it ) {
    std::vector<float> p;
    for( int j = 0; j < 3; ++j ) {
      p.push_back( (*it).x );
      p.push_back( (*it).y );
      p.push_back( (*it).z );
    }
    data.push_back( p );
  }
  

  // For 20 random starts
  int N_trials = 20;
  std::vector< pcl::PointCloud<PointT> > Pis;

  for( int i = 0; i < N_trials; ++i ) {
    
    // Initialize kmeans
    Kmeans km( input->points.size(), 3 );
    km.setMinMax( Eigen::Vector3f(minP.x, minP.y, minP.z), Eigen::Vector3f(maxP.x, maxP.y, maxP.z) );
    km.setClusterSize( num_clusters );
    km.setInputData( data );
    km.kMeans();

    // Visualize clouds
    std::vector< std::vector<float> > centroids =  km.get_centroids();
      // Get colorized version
    std::vector< std::set<unsigned int> > ctp =  km.getClustersToPoints();
    uint8_t r, g, b;

    for( int j = 0; j < num_clusters; ++j ) {
      printf("[%d] Centroid %d: %f %f %f . Size: %d \n", i, j, centroids[j][0],  centroids[j][1],  centroids[j][2], ctp[j].size() );
    } // for j

    pcl::PointCloud<PointT> Pi;

    for( int k = 0; k < ctp.size(); ++k ) {
      r = (uint8_t)(rand() % 255);
      g = (uint8_t)rand() % 255;
      b = (uint8_t)rand() % 255;
      
      for( std::set<unsigned int>::iterator it = ctp[k].begin();
	   it != ctp[k].end(); ++it ) {
	PointT p;
        p.x = input->points[*it].x;
        p.y = input->points[*it].y;
        p.z = input->points[*it].z;
	p.r = r; p.g = g; p.b = b;
	Pi.points.push_back( p );
      } // end for it

    } // end for k
    Pi.width = 1; Pi.height = Pi.points.size();
    
    char name[50];
    sprintf( name, "kmeans_trial_%d.pcd", i );
    pcl::io::savePCDFile ( name, Pi);
    Pis.push_back( Pi );
 
  } // trials

  // Save all pointclouds with a translation of x = 0.3 m so we can see them all together
  pcl::PointCloud<PointT> Ptogether;
  Ptogether.points.insert( Ptogether.begin(), Pis[0].begin(), Pis[0].end() );

  for( int i = 1; i < N_trials; ++i ) {

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() = Eigen::Vector3f(0.30*i, 0, 0);

  // Executing the transformation
  pcl::PointCloud<PointT> transformed_cloud;
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (Pis[i], transformed_cloud, transform);

      Ptogether.points.insert( Ptogether.end(), transformed_cloud.begin(), transformed_cloud.end() );
  }
  Ptogether.width = 1; Ptogether.height = Ptogether.points.size();
  pcl::io::savePCDFile ( "allCombined.pcd", Ptogether);

  return 0;
  
}
