/**
 * @file test_cases_generation_base.cpp
 * @brief Executable that generate random test cases to test diverse approximation functions for SQ
 * @brief Functions tested: RADIAL, SOLINA, ICHIM, CHEVALIER, 5 AND 6
 */
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <random>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string>

#include <SQ_utils.h>
#include <evaluated_eqs.h>

#include <future>
#include <thread>

// Global variables
const int fx_types[6] = {SQ_FX_RADIAL, SQ_FX_SOLINA, SQ_FX_ICHIM, SQ_FX_CHEVALIER, SQ_FX_5, SQ_FX_6};
const double dev = 0.005; // Standard deviation for noise
const double leaf_size = 0.02;
const int gNum_threads = 7;

// Variables that user can set as input
int gT = 100;
double gE1, gE2;
std::string gFilename;

/**
 * @brief Structure used to store output lines
 */
struct output_sq{
  SQ_parameters par;
  double t;
  double er1, er2, er4;
};


// Functions to get partial and noisy versions of original pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_input,
						  const double &_voxelSize );
pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud );
pcl::PointCloud<pcl::PointXYZ>::Ptr get_noisy( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
					       const double &_dev );
double getRand( const double &_minVal, const double &_maxVal );
void saveParams( std::ofstream &_output, const SQ_parameters &_par, double _t,
		 double _e1, double _e2, double _e4 );

std::vector<output_sq> createCases( int _id );
std::vector<output_sq> createCase();

// Global variables to generate noise
std::random_device rd;
std::mt19937 gen(rd());



/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Initialize, in case user does not enter values of e1 and e2
  gE1 = 1.0; gE2 = 1.0;
  gFilename = std::string("omp_result.txt");

  int v;
  while( (v=getopt(argc, argv, "e:f:t:n:")) != -1 ) {
    switch(v) {
    case 'e' : {
      gE1 = atof(optarg);
    } break;
    case 'f' : {
      gE2 = atof(optarg);
    } break;
    case 't' : {
      gT = atoi(optarg);
    } break;
    case 'n' : {
      gFilename.assign( optarg );
    } break;
    } // switch end
  }

  
  struct timespec start, finish;
  double elapsed;
  clock_gettime(CLOCK_MONOTONIC, &start);
  
  srand(time(NULL));
      
  std::ofstream output( gFilename.c_str(), std::ofstream::out );
  output << gT << std::endl;
  
  // Launch threads
  std::vector<std::future< std::vector<output_sq> > > futures;
  for( int i = 0; i < gNum_threads; ++i ) {
    futures.push_back( std::async( std::launch::async, &createCases, i) );
  }

  for( size_t i = 0; i < futures.size(); ++i ) {
    futures[i].wait();
  }

  for( size_t i = 0; i < futures.size(); ++i ) {
    std::vector<output_sq> s;
    s = futures[i].get();
    for( int m = 0; m < s.size(); ++m ) {
      saveParams( output, s[m].par, s[m].t, s[m].er1, s[m].er2, s[m].er4 );
    }
  }  
   
  clock_gettime(CLOCK_MONOTONIC, &finish);
  
  elapsed = (finish.tv_sec - start.tv_sec);
  elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
  printf("* Total time: %f \n", elapsed );
  
  output.close();
  return 0;
}

/**
 * @function createCases
 * @brief Create cases per each thread
 */
std::vector<output_sq> createCases(int _id ) {

  std::vector<output_sq> ss;
  std::vector<output_sq> si;
  for( int i = _id; i < gT; i = i + gNum_threads ) {
    si = createCase();
    for( int j = 0; j < si.size(); ++j ) {
      ss.push_back( si[j] );
    }    
  }

  return ss;
}


/**
 * @function createCase
 * @brief Create individual case (base + 6*4 tests)
 */
std::vector<output_sq> createCase() {

  std::vector<output_sq> output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr clean( new pcl::PointCloud<pcl::PointXYZ>() );
  
  SQ_parameters par, par_res;
  double er1, er2, er4;
  evaluated_sqs es;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> testCloud(4);
    
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

  // E parameters
  par.e[0] = gE1;
  par.e[1] = gE2;

  // Store original data
  output_sq base;
  base.par = par;
  base.er1 = 0; base.er2 = 0; base.er4 = 0; base.t = 0;
  output.push_back( base );
    
  // 1. Generate clean pointcloud
  clean = sampleSQ_uniform( par );
  testCloud[0] = downsampling( clean, leaf_size );    
  testCloud[1]= get_noisy( testCloud[0], dev );
  testCloud[2] = cut_cloud( testCloud[0] );
  testCloud[3] = cut_cloud( testCloud[1] );

  struct timespec ts, tf;
  double elapsed;
  // Try it out with methods
  for( int i = 0; i < 6; ++i ) {
    for( int j = 0; j < 4; ++j ) {

      clock_gettime(CLOCK_MONOTONIC, &ts);    
      es.minimize( testCloud[j], par_res, er1, er2, er4, fx_types[i] );
      clock_gettime(CLOCK_MONOTONIC, &tf);
      
      elapsed = (tf.tv_sec - ts.tv_sec);
      elapsed += (tf.tv_nsec - ts.tv_nsec) / 1000000000.0;
     
      output_sq oi;
      oi.par = par_res;
      oi.er1 = er1; oi.er2 = er2; oi.er4 = er4; oi.t = elapsed;
      output.push_back( oi );
    }
  }

  return output;
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

