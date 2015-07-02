/**
 * @file test_cases_functions.cpp
 * @brief Executable that generate random test cases to test diverse approximation functions for SQ
 * @brief So far: Radial, Solina
 */
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <random>
#include <stdlib.h>
#include <time.h>

#include <SQ_utils.h>
#include <evaluated_eqs.h>


// Functions to get partial and noisy versions of original pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
						  double _voxelSize );
pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud );
pcl::PointCloud<pcl::PointXYZ>::Ptr get_noisy( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
					       double _dev );
double getRand( double _minVal, double _maxVal );
void saveParams( std::ofstream &_output, const SQ_parameters &_par, double _t,
		 double _e1, double _e2, double _e4 );

// Global variables to generate noise
std::random_device rd;
std::mt19937 gen(rd());

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  srand(time(NULL));
  
  int T = 20; // Number of test cases
  double dev = 0.0025; // Standard deviation for noise
  double leaf_size = 0.01;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr clean( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr clean_down( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr noisy( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr clean_partial( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr noisy_partial( new pcl::PointCloud<pcl::PointXYZ>() );
  SQ_parameters par, par_res;
  double e1, e2, e4;
  clock_t ts, tf; double dt;
  evaluated_sqs es;
  
  std::ofstream output("test_case_functions_short2.txt", std::ofstream::out );
  output << T << std::endl;
  
  for( int i = 0; i < T; ++i ) {

    // Dimensions
    par.dim[0] = getRand(0.025, 0.15);
    par.dim[1] = getRand(0.025, 0.15);
    par.dim[2] = getRand(0.025, 0.15);

    // Translations
    par.trans[0] = getRand(-1.5, 1.5);
    par.trans[1] = getRand(-1.5, 1.5);
    par.trans[2] = getRand(0.3, 1.5);

    // Rotation
    par.rot[0] = getRand(-M_PI, M_PI);
    par.rot[1] = getRand(-M_PI, M_PI); 
    par.rot[2] = getRand(-M_PI, M_PI); 

    for( int j = 0; j < 5; j++ ) {
      for( int k = 0; k < 5; k++ ) {
	printf("T: %d j: %d k: %d \n", i, j, k);
	par.e[0] = 0.1 + 0.45*j;
	par.e[1] = 0.1 + 0.45*k;
	

	// Store original data
	saveParams( output, par, 0, 0, 0, 0 );
	
	// 1. Generate clean pointcloud
	clean = sampleSQ_uniform( par );
	clean_down = downsampling( clean, leaf_size );
	
	
	printf("Size pointcloud :%d \n", clean->points.size() );
	printf("Size pointcloud after :%d \n", clean_down->points.size() );
	// 2. Generate dirty pointcloud
	noisy = get_noisy( clean_down, dev );
	
	// 3. Divide both pointclouds in half using the
	// plane that goes through the center
	clean_partial = cut_cloud( clean_down );
	noisy_partial = cut_cloud( noisy );
	
	// Store and check if it is fine
	/*
	pcl::io::savePCDFileASCII ( "clean.pcd", *clean_down );
	pcl::io::savePCDFileASCII ( "clean_partial.pcd", *clean_partial );
	pcl::io::savePCDFileASCII ( "noisy.pcd", *noisy );
	pcl::io::savePCDFileASCII ( "noisy_partial.pcd", *noisy_partial );
	*/
	
	// Try it out with methods	
	// RADIAL
	ts = clock();
	es.minimize( clean_down, par_res, e1, e2, e4, SQ_FX_RADIAL );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy, par_res,  e1, e2, e4, SQ_FX_RADIAL );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( clean_partial, par_res, e1, e2, e4, SQ_FX_RADIAL );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy_partial, par_res, e1, e2, e4, SQ_FX_RADIAL );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );	

	// SOLINA
	ts = clock();
	es.minimize( clean_down, par_res, e1, e2, e4, SQ_FX_SOLINA );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy, par_res, e1, e2, e4, SQ_FX_SOLINA );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( clean_partial, par_res,  e1, e2, e4, SQ_FX_SOLINA );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy_partial, par_res,  e1, e2, e4, SQ_FX_SOLINA );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );	

	// ICHIM
	ts = clock();
	es.minimize( clean_down, par_res,  e1, e2, e4, SQ_FX_ICHIM );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy, par_res,  e1, e2, e4, SQ_FX_ICHIM );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( clean_partial, par_res,  e1, e2, e4, SQ_FX_ICHIM );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy_partial, par_res,  e1, e2, e4, SQ_FX_ICHIM );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );	

	// CHEVALIER
	ts = clock();
	es.minimize( clean_down, par_res,  e1, e2, e4, SQ_FX_CHEVALIER );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy, par_res,  e1, e2, e4, SQ_FX_CHEVALIER );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( clean_partial, par_res,  e1, e2, e4, SQ_FX_CHEVALIER );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy_partial, par_res, e1, e2, e4,  SQ_FX_CHEVALIER );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );	

	// 5
	ts = clock();
	es.minimize( clean_down, par_res, e1, e2, e4,  SQ_FX_5 );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy, par_res, e1, e2, e4,  SQ_FX_5 );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( clean_partial, par_res, e1, e2, e4,  SQ_FX_5 );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy_partial, par_res, e1, e2, e4,  SQ_FX_5 );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );	

	// 6
	ts = clock();
	es.minimize( clean_down, par_res, e1, e2, e4,  SQ_FX_6 );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy, par_res, e1, e2, e4,  SQ_FX_6 );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( clean_partial, par_res, e1, e2, e4,  SQ_FX_6 );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );

	ts = clock();
	es.minimize( noisy_partial, par_res, e1, e2, e4,  SQ_FX_6 );
	tf = clock();
	dt = (tf-ts) / (double) CLOCKS_PER_SEC;
	saveParams( output, par_res, dt,e1,e2,e4 );	

	
	

      }      
    }

    

  } // end test cases

  output.close();
  return 0;  
}

/**
 * @function downsampling
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling( pcl::PointCloud<pcl::PointXYZ>::Ptr _input,
						  double _voxelSize ) {

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
pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud ) {
  
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
pcl::PointCloud<pcl::PointXYZ>::Ptr get_noisy( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
					       double _dev ) {
  
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
double getRand( double _minVal, double _maxVal ) {

  return _minVal + (_maxVal - _minVal)*((double)rand() / RAND_MAX);
  
}

/**
 * @function saveParams
 */
void saveParams( std::ofstream &_output,
		 const SQ_parameters &_par,
		 double _t,
		 double _e1, double _e2, double _e4 ) {
  
  _output << _par.dim[0] << " " << _par.dim[1] << " " << _par.dim[2] << " "
	  << _par.e[0] << " " << _par.e[1] << " "
	  << _par.trans[0] << " " << _par.trans[1] << " " << _par.trans[2] << " "
	  << _par.rot[0] << " " << _par.rot[1] << " " << _par.rot[2]
	  << " " << _t << " "<< _e1 << " " << _e2 << " " << _e4 << std::endl;
}

