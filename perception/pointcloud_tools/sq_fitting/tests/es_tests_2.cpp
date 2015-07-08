/**
 * @file es_tests_2.cpp
 * @brief Test an individual case, pointcloud is a real one entered from terminal
 */
#include "evaluated_eqs.h"
#include "evaluated_eqs_t.h"
#include <SQ_utils.h>
#include <SQ_fitter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>

char* fx_names[6] = { "Radial", "Solina", "Ichim", "Chevalier", "F5", "F6"};
int fx_sq[6] = {SQ_FX_RADIAL, SQ_FX_SOLINA, SQ_FX_ICHIM, SQ_FX_CHEVALIER, SQ_FX_5, SQ_FX_6 };

char* fx_t_names[5] = { "Radial_T", "Solina T", "Old T", "Chevalier T", "F5 T"};
int fx_t[5] = { SQ_FX_RADIAL_T, SQ_FX_SOLINA_T, SQ_FX_OLD_T, SQ_FX_CHEVALIER_T, SQ_FX_5_T};

const double gDev = 0.0025;

// Functions to get partial and noisy versions of original pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_input,
						  const double &_voxelSize );
pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud );
pcl::PointCloud<pcl::PointXYZ>::Ptr get_noisy( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
					       const double &_dev );
double getRand( const double &_minVal, const double &_maxVal );

std::string gInput;
std::string gOutput;

// Global variables to generate noise
std::random_device rd;
std::mt19937 gen(rd());


/**
 * @function main
 */
int main( int argc, char*argv[] ) {

  int v;
  while( (v=getopt(argc, argv, "n:o:")) != -1 ) {
    switch(v) {

    case 'n' : {
      gInput = std::string(optarg);
    } break;
    case 'o' : {
      gOutput = std::string(optarg);
    } break;

    } // switch end
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr input( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr down( new pcl::PointCloud<pcl::PointXYZ>() );

  pcl::io::loadPCDFile<pcl::PointXYZ> ( gInput.c_str(), *input ); 
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> testCloud(4);  
  down = downsampling( input, 0.01 );
  testCloud[0] = down;    
  testCloud[1]= get_noisy( testCloud[0], gDev );
  testCloud[2] = cut_cloud( testCloud[0] );
  testCloud[3] = cut_cloud( testCloud[1] );

  
  printf("Size of input: %d \n", input->points.size() );
  printf("Size of down: %d \n", down->points.size() );
  
  clock_t ts, tf; double dt;
  evaluated_sqs es;
  double er1, er2, er4;
  SQ_parameters par;
  pcl::PointCloud<pcl::PointXYZ>::Ptr approx( new pcl::PointCloud<pcl::PointXYZ>() );
  /*
  for( int i = 0; i < 6; ++i ) {

    printf( "Function: %s \n", fx_names[i] );
    ts = clock();
    es.minimize( down, par, er1, er2, er4, fx_sq[i] );
    tf = clock();
    dt = (tf-ts) / (double) CLOCKS_PER_SEC;
    printf("Dim: %f %f %f \t e: %f %f \t t: %f \t er1: %f er2: %f er4: %f \n",
	   par.dim[0], par.dim[1], par.dim[2],
	   par.e[0], par.e[1], dt,
	   er1, er2, er4);

    approx = sampleSQ_uniform( par );
    char name[50];
    sprintf( name, "%s_%d.pcd", gOutput.c_str(), i );
    pcl::io::savePCDFileASCII( name, *approx );
  }
  */
  // Try with simple
  evaluated_sqs_t est;
  for( int i = 0; i < 5; ++i ) {

    printf( "Function: %s \n", fx_t_names[i] );
    ts = clock();
    est.minimize( down, par, er1, er2, er4, fx_t[i] );
    tf = clock();
    dt = (tf-ts) / (double) CLOCKS_PER_SEC;
    printf("Dim: %f %f %f \t e: %f %f \t K: %f --- t: %f \t er1: %f er2: %f er4: %f \n",
	   par.dim[0], par.dim[1], par.dim[2],
	   par.e[0], par.e[1],
	   par.tamp, dt,
	   er1, er2, er4);
    approx = sampleSQ_uniform_t(par);
    char name[50];
    sprintf( name, "%s_t_%d.pcd", gOutput.c_str(), i );
    pcl::io::savePCDFileASCII( name, *approx );
    
  }
  
}


/**
 * @function downsampling
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_input,
						  const double &_voxelSize ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled( new pcl::PointCloud<pcl::PointXYZ>() );
  
  // Create the filtering object
  pcl::VoxelGrid< pcl::PointXYZ > downsampler;
  // Set input cloud
  downsampler.setInputCloud( _input );
  // Set size of voxel
  downsampler.setLeafSize( _voxelSize, _voxelSize, _voxelSize );
  // Downsample
  downsampler.filter( *cloud_downsampled );

  return cloud_downsampled;
}

/**
 * @function cut_cloud
 * @brief Slice cloud by half, leaving only the "visible"points (points towards the eye point (0,0,0))
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr output( new pcl::PointCloud<pcl::PointXYZ>() );
  
  // Get eigenvectors and centroid of pointcloud
  Eigen::Vector4d c;
  pcl::compute3DCentroid( *_cloud, c );
  
  // Get the normal w.r.t. the centroid
  Eigen::Vector3d N; N = Eigen::Vector3d(0,0,0) - Eigen::Vector3d(c(0), c(1), c(2) );
  double d;
  
  // Plane equation n(0)x + n(1)y + n(2)z + d = 0. Find d using the centroid
  d = -( N(0)*c(0) + N(1)*c(1) + N(2)*c(2) );
  
  // Cut it
  for( pcl::PointCloud<pcl::PointXYZ>::iterator it = _cloud->begin();
	   it != _cloud->end(); it++ ) {
    
    if( (N(0)*((*it).x) + N(1)*((*it).y) + N(2)*((*it).z) + d) > 0 ) {
      pcl::PointXYZ p;
      p = *it;
      output->points.push_back( p );
    }
    
  }
  
  output->height = 1; output->width = output->points.size();
  
  return output;  
}

/**
 * @function get_noisy
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr get_noisy( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
					       const double &_dev ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr output( new pcl::PointCloud<pcl::PointXYZ>() );
  std::normal_distribution<double> d(0,_dev);
  
  // Make it dirty
  for( pcl::PointCloud<pcl::PointXYZ>::iterator it = _cloud->begin();
       it != _cloud->end(); it++ ) {
    pcl::PointXYZ p;
    p.x = (*it).x + d(gen);
    p.y = (*it).y + d(gen);
    p.z = (*it).z + d(gen);
    output->points.push_back( p );
  }
  
  output->height = 1; output->width = output->points.size();
  return output;
}

/**
 * @function getRand
 */
double getRand( const double &_minVal, const double &_maxVal ) {

  return _minVal + (_maxVal - _minVal)*((double)rand() / RAND_MAX);
  
}
