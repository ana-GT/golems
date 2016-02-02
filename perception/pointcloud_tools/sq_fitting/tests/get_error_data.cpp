
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>


#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"
#include <iostream>
#include <fstream>
#include <stdio.h>

const int gnF = 5;


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

void readSQline( std::stringstream &_input,
		 output_sq &_ou );

int main( int argc, char* argv[] ) {

  int gT;
  std::string gFilename;
  
  int v;
  while( (v=getopt(argc,argv,"t:n:h")) != -1 ) {

    switch(v) {
    case 't': { gT = atoi(optarg); } break;
    case 'n': { gFilename.assign(optarg); } break;
    case 'h': { printf("Syntax: %s -t NUMBER_OF_OBJECTS -n FILENAME \n", argv[0]); return 0; } break;
    }    
  }

  // Store
  output_sq out_r[gnF][gT];
  output_sq out_h[gnF][gT];
  
  //Read input
  std::ifstream input( gFilename.c_str(), std::ifstream::in );
  output_sq ou;
  std::string line;
  
  for( int i = 0; i < gT; ++i ) {
    for( int j = 0; j < gnF; ++j ) {

    // Read regular
    std::getline( input, line );
    std::stringstream ssr(line);
    readSQline( ssr, ou );
    out_r[j][i] = ou;

    // Read hierarchical
    std::getline( input, line );
    std::stringstream ssh(line);
    readSQline( ssh, ou );
    out_h[j][i] = ou;
    
    }
  } // end for i

  // Calculate means
  double means_eg_r[gnF];
  double means_er_r[gnF];
  double means_t_r[gnF];

  double means_eg_h[gnF];
  double means_er_h[gnF];
  double means_t_h[gnF];

  double std_eg_r[gnF];
  double std_er_r[gnF];
  double std_t_r[gnF];

  double std_eg_h[gnF];
  double std_er_h[gnF];
  double std_t_h[gnF];

  

  for( int j = 0; j < gnF; ++j ) {
      means_eg_r[j] = 0;
      means_er_r[j] = 0;
      means_t_r[j] = 0;

      means_eg_h[j] = 0;
      means_er_h[j] = 0;
      means_t_h[j] = 0;
      
      std_eg_r[j] = 0;
      std_er_r[j] = 0;
      std_t_r[j] = 0;

      std_eg_h[j] = 0;
      std_er_h[j] = 0;
      std_t_h[j] = 0;           
  }

  
  for( int j = 0; j < gnF; ++j ) {
    for( int i = 0; i < gT; ++i ) {
      means_eg_r[j] += out_r[j][i].er_g;
      means_er_r[j] += out_r[j][i].er_r;
      means_t_r[j] += out_r[j][i].t;

      means_eg_h[j] += out_h[j][i].er_g;
      means_er_h[j] += out_h[j][i].er_r;
      means_t_h[j] += out_h[j][i].t;      
    }

    means_eg_r[j] /= (double) gT;
    means_er_r[j] /= (double) gT;
    means_t_r[j] /= (double) gT;
    
    means_eg_h[j] /= (double) gT;
    means_er_h[j]/= (double) gT ;
    means_t_h[j]/= (double) gT;      
    
  }

  for( int j = 0; j < gnF; ++j ) {
    for( int i = 0; i < gT; ++i ) {
      std_eg_r[j] += (out_r[j][i].er_g - means_eg_r[j])*(out_r[j][i].er_g - means_eg_r[j]);
      std_er_r[j] += (out_r[j][i].er_r - means_er_r[j])*(out_r[j][i].er_r - means_er_r[j]);
      std_t_r[j] += (out_r[j][i].t - means_t_r[j])*(out_r[j][i].t - means_t_r[j]);

      std_eg_h[j] += (out_h[j][i].er_g - means_eg_h[j])*(out_h[j][i].er_g - means_eg_h[j]);
      std_er_h[j] += (out_h[j][i].er_r - means_er_h[j])*(out_h[j][i].er_r - means_er_h[j]);
      std_t_h[j] += (out_h[j][i].t - means_t_h[j])*(out_h[j][i].t - means_t_h[j]);      
    }

    std_eg_r[j] = sqrt( std_eg_r[j]/(double) gT);
    std_er_r[j] = sqrt( std_er_r[j]/(double) gT);
    std_t_r[j] = sqrt( std_t_r[j]/(double) gT);
    
    std_eg_h[j] = sqrt(std_eg_h[j]/(double) gT);
    std_er_h[j] = sqrt(std_er_h[j]/(double) gT);
    std_t_h[j] = sqrt(std_t_h[j]/(double) gT);      
    
  }
  
  printf("REGULAR \n");
  for( int j= 0; j < gnF; ++j ) {
    printf("F[%d] Mean eg: %f std eg: %f Mean er: %f std er: %f Mean t: %f std t: %f \n", j,
	   means_eg_r[j],std_eg_r[j],
	   means_er_r[j],std_er_r[j],
	   means_t_r[j], std_t_r[j]);
  }
  
  printf("HIERARCHICAL \n");
  for( int j= 0; j < gnF; ++j ) {
    printf("F[%d] Mean eg: %f std eg: %f Mean er: %f std er: %f Mean t: %f std t: %f \n", j,
	   means_eg_h[j],std_eg_h[j],
	   means_er_h[j],std_er_h[j],
	   means_t_h[j], std_t_h[j]);
  }

  std::ofstream output("output.txt", std::ofstream::out );
  for( int i = 0; i < gT; ++i ) {
    output << i <<" ";
    for( int j = 0; j < gnF; ++j ) {
      output << out_r[j][i].er_g <<" " << out_r[j][i].er_r <<" " << out_r[j][i].t << " ";
    }
    output << "\n";
  }
  output.close();
  
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
