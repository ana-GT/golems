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

#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"
#include "perception/pointcloud_tools/sq_fitting/evaluated_eqs.h"

#include <future>
#include <thread>

typedef pcl::PointXYZ PointT;

/**
 * @brief Structure used to store output lines
 */
struct output_sq{
  SQ_parameters par;
  double t;
  double er_g, er_r;
  double er_v;
  double er_e1;
  double er_e2;
};

// Global variables
const int gNum_threads = 7;

int gnD = 7;
double gdD = 0.005;

// Variables that user can set as input
int gT = 50;
std::string gFilename;

// Functions to get partial and noisy versions of original pointcloud
pcl::PointCloud<PointT>::Ptr downsampling( const pcl::PointCloud<PointT>::Ptr &_input,
						  const double &_voxelSize );

// Global variables to generate noise
double getRand( const double &_minVal, const double &_maxVal );
double beta( double z,double w);
double volSQ( double a, double b, double c, double e1, double e2 );
void saveParams( std::ofstream &_output, const SQ_parameters &_par, double _t,
                 double _eg, double _er, 
                 double _e_e1, double _e_e2,
                 double _e_v );
std::vector<output_sq> createCases( int _id );
std::vector<output_sq> createCase();




/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  /* initialize random seed: */
  srand (time(NULL));
  
  // Initialize, in case user does not enter values of e1 and e2
  gFilename = std::string("omp_result.txt");

  int v;
  while( (v=getopt(argc, argv, "t:n:h")) != -1 ) {
    switch(v) {
    case 't' : {
      gT = atoi(optarg);
    } break;
    case 'n' : {
      gFilename.assign( optarg );
    } break;
    case 'h' : {
      printf("Executable to save T randomized runs to test downsampling effect \n");
      printf("Usage: ./executable -t T -n output_filename.txt \n");
      return 0;
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
      saveParams( output, s[m].par, s[m].t, s[m].er_g, s[m].er_r,
       s[m].er_e1, s[m].er_e2, s[m].er_v );
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
  pcl::PointCloud<PointT>::Ptr input( new pcl::PointCloud<PointT>() );
  pcl::PointCloud<PointT>::Ptr down( new pcl::PointCloud<PointT>() );
  
  SQ_parameters par, par_res;
  double er_g, er_r, er_d;
  evaluated_sqs es;
    
  // Dimensions
  par.dim[0] = getRand(0.025, 0.39);
  par.dim[1] = getRand(0.025, 0.39);
  par.dim[2] = getRand(0.025, 0.39);
  
  // Translations
  par.trans[0] = getRand(-0.8, 0.8);
  par.trans[1] = getRand(-0.8, 0.8);
  par.trans[2] = getRand(0.3, 1.4);
  
  // Rotation
  par.rot[0] = getRand(-M_PI, M_PI);
  par.rot[1] = getRand(-M_PI, M_PI); 
  par.rot[2] = getRand(-M_PI, M_PI); 

  // E parameters
  par.e[0] = getRand(0.1,1.9);
  par.e[1] = getRand(0.1,1.9);


  // Store original data
  output_sq base;
  base.par = par;
  base.er_r = 0; base.er_g = 0; base.t = 0; base.er_v = 0;
  base.er_e1 = 0; base.er_e2 = 0;
  output.push_back( base );
    
  // 1. Generate clean pointcloud
  input = sampleSQ_uniform<PointT>( par );

  struct timespec ts, tf;
  double elapsed;
  double d;
  double vr, vc;
  output_sq oi;
	
  // Try it out
  for( int i = 1; i < gnD; ++i ) {
    
    d = 0 + gdD*i;
    if( d == 0 ) { down = input; }
    else { down = downsampling( input, d ); }

    clock_gettime(CLOCK_MONOTONIC, &ts);    
    es.minimize( down, par_res, er_g, er_r, er_d, SQ_FX_RADIAL );
    clock_gettime(CLOCK_MONOTONIC, &tf);
    error_metric<PointT>( par_res,input, er_g, er_r, er_d );
      
    elapsed = (tf.tv_sec - ts.tv_sec);
    elapsed += (tf.tv_nsec - ts.tv_nsec) / 1000000000.0;
     
    oi.par = par_res;
    oi.er_g = er_g; oi.er_r = er_r; oi.t = elapsed;
    oi.er_e1 = fabs(base.par.e[0] - par_res.e[0]);
    oi.er_e2 = fabs(base.par.e[1] - par_res.e[1]);

    vc =  volSQ(par_res.dim[0], par_res.dim[1], par_res.dim[2],     par_res.e[0], par_res.e[1]);
    vr = volSQ(base.par.dim[0],base.par.dim[1],base.par.dim[2],  base.par.e[0], base.par.e[1] );
    oi.er_v = (vc - vr)/vr*100.0;

    output.push_back( oi );    
  }


  return output;
}

/**
 * @function downsampling
 */
pcl::PointCloud<PointT>::Ptr downsampling( const pcl::PointCloud<PointT>::Ptr &_input,
						  const double &_voxelSize ) {

  pcl::PointCloud<PointT>::Ptr cloud_downsampled( new pcl::PointCloud<PointT>() );
  
  // Create the filtering object
  pcl::VoxelGrid< PointT > downsampler;
  // Set input cloud
  downsampler.setInputCloud( _input );
  // Set size of voxel
  downsampler.setLeafSize( _voxelSize, _voxelSize, _voxelSize );
  // Downsample
  downsampler.filter( *cloud_downsampled );

  return cloud_downsampled;
}


/**
 * @function saveParams
 */
void saveParams( std::ofstream &_output,
		 const SQ_parameters &_par,
		 double _t,
		 double _eg, double _er, 
                 double _e_e1, double _e_e2,
                 double _e_v ) {
  
  _output << _par.dim[0] << " " << _par.dim[1] << " " << _par.dim[2] << " "
	  << _par.e[0] << " " << _par.e[1] << " "
	  << _par.trans[0] << " " << _par.trans[1] << " " << _par.trans[2] << " "
	  << _par.rot[0] << " " << _par.rot[1] << " " << _par.rot[2]
	  << " " << _t << " "<< _eg << " " << _er << " " << _e_e1 << "  " 
          << _e_e2 << " "<< _e_v << std::endl;
}


/**
 * @function getRand
 */
double getRand( const double &_minVal, const double &_maxVal ) {

  return _minVal + (_maxVal - _minVal)*((double)rand() / (double)RAND_MAX);
  
}

double beta( double z,double w) {
  double gz, gw, gzw;
  
  gz = tgamma(z);
  gw = tgamma(w);
  gzw = tgamma(z+w);
  return  gz*gw/gzw;
}

double volSQ( double a, double b, double c, double e1, double e2 ) {
  return 2*a*b*c*e1*e2*beta(e1*0.5, e1+1)*beta(e2*0.5, e2*0.5+1);
}
