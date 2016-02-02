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

// Global variables
int gnD = 7;

// Variables that user can set as input
int gT;
std::string gFilename;


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


// Functions to get partial and noisy versions of original pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_input,
						  const double &_voxelSize );
void readSQline( std::stringstream &_input,
		 output_sq &_ou );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  /* initialize random seed: */
  srand (time(NULL));
  
  int v;
  while( (v=getopt(argc, argv, "n:h")) != -1 ) {
    switch(v) {
    case 'n' : {
      gFilename.assign( optarg );
    } break;
    case 'h' : {
      printf("Executable to read T randomized runs to test downsampling effect \n");
      printf("Usage: ./executable -n input_filename_es_test_1.txt \n");
      return 0;
    } break;
    } // switch end
  }

  // Read
  std::ifstream input( gFilename, std::ifstream::in );
  std::string line;
  
  std::getline( input, line );
  std::stringstream st(line);
  st >> gT;
  
  printf("Reading %d input cases from es_test_1 results \n", gT);

  output_sq ou;
  double acc_er_g[gnD-1];
  double acc_er_r[gnD-1];
  double acc_er_v[gnD-1];
  double acc_t[gnD-1];

  for( int j = 1-1; j < gnD-1; ++j ) {
    acc_er_g[j] = 0;
    acc_er_r[j] = 0;
    acc_er_v[j] = 0;
    acc_t[j] = 0;
  }

  
  for( int i = 0; i < gT; ++i ) {

    // Read base
    std::getline( input, line );
    std::stringstream ss(line);
    readSQline( ss, ou );

    // Read downsampling results
    for( int j = 1-1; j < gnD-1; ++j ) {
      std::getline( input, line );
      std::stringstream ss(line);
      readSQline( ss, ou );

      // Store accumulated error
      acc_er_g[j] += ou.er_g;
      acc_er_r[j] += ou.er_r;
      acc_er_v[j] += fabs(ou.er_v);
      acc_t[j] += ou.t;
    }
  }

  for( int j = 1-1; j < gnD-1; ++j ) {
    acc_er_g[j] /= (double)gT;
    acc_er_r[j] /= (double)gT;
    acc_er_v[j] /= (double)gT;
    acc_t[j] /= (double)gT;
  }

  for( int j = 1-1; j < gnD-1; ++j ) {
    printf("* Downsampling level %d: \t Er_g: %f \t Er_r: %f \t Er_v: %f \t t: %f \n",
	   j, acc_er_g[j],  acc_er_r[j],  acc_er_v[j], acc_t[j]);
  }
  
  input.close();
  return 0;
}

/**
 * @function saveParams
 */
void readSQline( std::stringstream &_input,
		 output_sq &_ou ) {
  
  _input >> _ou.par.dim[0] >> _ou.par.dim[1] >> _ou.par.dim[2] 
	 >> _ou.par.e[0]  >> _ou.par.e[1] 
	 >> _ou.par.trans[0] >> _ou.par.trans[1] >> _ou.par.trans[2] 
	 >> _ou.par.rot[0] >> _ou.par.rot[1] >> _ou.par.rot[2]
	 >> _ou.t >> _ou.er_g >> _ou.er_r >> _ou.er_e1 
	 >> _ou.er_e2 >> _ou.er_v;
}
