/**
 * @file es_read_tests_2.cpp
 * @brief Executable that reads output from es_tests_2.cpp with randomized cases to test effects
 * @brief Of noise and partiality
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

#include <SQ_utils.h>
#include <evaluated_eqs.h>

#include <future>
#include <thread>

int gnF = 5;

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
      printf("Executable to read T randomized runs to test noise and partial view effects \n");
      printf("Usage: ./executable -n input_filename_es_test_2.txt \n");
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
  
  printf("Reading %d input cases from es_test_2 results \n", gT);

  output_sq ou;
  double acc_er_g[gnF][4];
  double acc_er_r[gnF][4];
  double acc_er_v[gnF][4];
  double acc_t[gnF][4];

  for( int i = 0; i < gnF; ++i ) {
    for( int j = 0; j < 4; ++j ) {
      acc_er_g[i][j] = 0;
      acc_er_r[i][j] = 0;
      acc_er_v[i][j] = 0;
      acc_t[i][j] = 0;
    }
  }

  // Store
  output_sq fr[gT][4];
  output_sq fs[gT][4];
  output_sq fi[gT][4];
  output_sq fc[gT][4];
  output_sq f5[gT][4];

  for( int i = 0; i < gT; ++i ) {

    // Read base
    std::getline( input, line );
    std::stringstream ss(line);
    readSQline( ss, ou );
    
    // Read downsampling results
    for( int j = 0; j < gnF; ++j ) {
      for( int k = 0; k < 4; ++k ) {
	std::getline( input, line );
	std::stringstream ss(line);
	readSQline( ss, ou );
	
	if( j == 0 ) {
	  fr[i][k] = ou;
	} else if( j == 1 ) {
	  fs[i][k] = ou;
	} else if( j == 2 ) {
	  fi[i][k] = ou;
	} else if( j == 3 ) {
	  fc[i][k] = ou;
	} else if( j == 4 ) {
	  f5[i][k] = ou;
	}

	// Store accumulated error
	acc_er_g[j][k] += ou.er_g;
	acc_er_r[j][k] += ou.er_r;
	acc_er_v[j][k] += fabs(ou.er_v);
	acc_t[j][k] += ou.t;
        if( ou.er_g > 10 || ou.er_g < -1 ) {
          printf("Detected extreme value in er_g for: %f - line:  %d - check time: %f \n", ou.er_g, i*20 +j*4 + k, ou.t ); 
        }
      }
    }
    
  }

  for( int i = 0; i < gnF; ++i ) {
    for( int j = 0; j < 4; ++j ) {
      acc_er_g[i][j] /= (double)gT;
      acc_er_r[i][j] /= (double)gT;
      acc_er_v[i][j] /= (double)gT;
      acc_t[i][j] /= (double)gT;
    }
  }


  for( int i = 0; i < gnF; ++i ) {
    for( int j = 0; j < 4; ++j ) {
      printf("* Downsampling Fx: %d cloud: %d: \t Er_g: %f \t Er_r: %f \t Er_v: %f \t t: %f \n",
	     i, j, acc_er_g[i][j],  acc_er_r[i][j],  acc_er_v[i][j], acc_t[i][j]);
    }
  }
  /*
  std::ofstream outing("outing.txt", std::ofstream::out );
  for( int i = 0; i < gT; ++i ) {
    printf("[%d] F0 - 4: \t er_r[3]: %f \t er_s[3]: %f \t er_i[3]: %f \t er_c[3]: %f \t er_5[3]: %f \n", i, fr[i][3].er_v, fs[i][3].er_v, fi[i][3].er_v, fc[i][3].er_v, f5[i][3].er_v );
    char line[200];
    //sprintf( line, "%f \t %f %f " );
    outing << fr[i][2].par.e[0] << " \t " << fr[i][2].par.e[1] << " \t "
	   << fr[i][2].er_g << " \t " << fr[i][2].er_r << " \t "
	   << fabs(fr[i][2].er_v) << " \t " << fr[i][2].t << std::endl;
   }
  
  outing.close();
*/
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
