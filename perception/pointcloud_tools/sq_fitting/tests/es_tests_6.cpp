/**
 * @file es_tests_6.cpp
 * @brief Executable that generate random test cases to test diverse approximation functions for SQ
 * @brief Functions tested: RADIAL, SOLINA, ICHIM, CHEVALIER, 5
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
const int gnF = 5;
double gD = 0.01;
int fx_sq[gnF] = {SQ_FX_RADIAL, SQ_FX_SOLINA, SQ_FX_ICHIM, SQ_FX_CHEVALIER, SQ_FX_5};

std::string gFilename, gOutput;
pcl::PointCloud<PointT>::Ptr gInput( new pcl::PointCloud<PointT>() );
pcl::PointCloud<PointT>::Ptr gDown( new pcl::PointCloud<PointT>() );
output_sq createCase(int i);

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
output_sq createCase( int _id );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  /* initialize random seed: */
  srand (time(NULL));
  
  int v;
  while( (v=getopt(argc, argv, "n:o:h")) != -1 ) {
    switch(v) {
    case 'n' : {
      gFilename.assign( optarg );
    } break;
    case 'o' : {
      gOutput.assign( optarg );
   } break;
    case 'h' : {
      printf("Executable to evaluate simple input pointcloud w.r.t. 5 functions \n");
      printf("Usage: ./executable -n input_filename.txt \n");
      return 0;
    } break;
    } // switch end
  }

  if( pcl::io::loadPCDFile<PointT> (gFilename, *gInput) == -1) {
    printf("Error loading pcd : %s \n", gFilename.c_str() );
    return 0;
  } else {
    gDown = downsampling( gInput, gD );
    printf("OK loading pcd : %s with %d points  and downsampled to %d \n", gFilename.c_str(), gInput->points.size(), gDown->points.size() ); 
  }

  struct timespec start, finish;
  double elapsed;
  clock_gettime(CLOCK_MONOTONIC, &start);
  std::vector< output_sq > rs(gnF);
      
  // Launch threads
  std::vector<std::future< output_sq > > futures;
  for( int i = 0; i < gnF; ++i ) {
    futures.push_back( std::async( std::launch::async, &createCase, i) );
  }

  for( size_t i = 0; i < futures.size(); ++i ) {
    futures[i].wait();
  }

  for( size_t i = 0; i < futures.size(); ++i ) {
    output_sq s;
    s = futures[i].get();
    rs[i] = s;
    printf("Result [%d]: %f %f %f \t %f %f \t %f %f %f \t %f %f %f \t Errors: g: %f r: %f \t t: %f \n", 
    i, s.par.dim[0], s.par.dim[1], s.par.dim[2], s.par.e[0], s.par.e[1], s.par.trans[0], s.par.trans[1], s.par.trans[2], s.par.rot[0], s.par.rot[1], s.par.rot[2], s.er_g, s.er_r, s.t);

  }  
   
  // Save
  // 1. Store name
  char* name = new char[50];
  char inna[50];
  strcpy( inna, gOutput.c_str() );
  name = strtok( inna, "." );
  for( int i = 0; i < gnF; ++i ) {

    char iname[60];
    sprintf( iname, "res/%s_%d.pcd", name, i );
    pcl::PointCloud<PointT>::Ptr sample( new pcl::PointCloud<PointT>() );
    sample = sampleSQ_uniform<PointT>( rs[i].par );
    pcl::io::savePCDFileASCII ( iname, *sample);

  }

  clock_gettime(CLOCK_MONOTONIC, &finish);
  
  elapsed = (finish.tv_sec - start.tv_sec);
  elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
  printf("* Total time: %f \n", elapsed );

  return 0;
}


/**
 * @function createCase
 * @brief Create individual case (base + 6*4 tests)
 */
output_sq createCase(int i) {

  SQ_parameters par;
  double er_g, er_r, er_d;
  evaluated_sqs es;
    
  struct timespec ts, tf;
  double elapsed;
  output_sq oi;
	

    clock_gettime(CLOCK_MONOTONIC, &ts);    
    es.minimize( gDown, par, er_g, er_r, er_d, fx_sq[i] );
    clock_gettime(CLOCK_MONOTONIC, &tf);
    error_metric<PointT>( par,gInput, er_g, er_r, er_d );
      
    elapsed = (tf.tv_sec - ts.tv_sec);
    elapsed += (tf.tv_nsec - ts.tv_nsec) / 1000000000.0;
     
    oi.par = par;
    oi.er_g = er_g; oi.er_r = er_r; oi.t = elapsed;
    oi.er_e1 = 0;
    oi.er_e2 = 0;
    oi.er_v = 0;

  return oi;
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
