
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>


#include <SQ_utils.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

const int gnF = 5;
//const std::string datapath("/media/ana/ICRA_2015/data_july_18");
const std::string datapath("/home/ana/Research/golems/bin/data_july_18");
const int gnG = 5;
std::string groupnames[gnG] = {"cylinders", "boxes", "balls","fruits", "shaped_1"};

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
  std::string group;
};

//////////////////////////////////
struct temp_t { double metric; unsigned int ind; };
bool sorting( temp_t a, temp_t b ) { return  (a.metric < b.metric); }

std::vector<unsigned int> sortIndices( std::vector<double> _metrics ) {

  std::vector<unsigned int> sortedIndices;

  std::vector<temp_t> sorted;
  for( unsigned int i = 0; i < _metrics.size(); ++i ) {
    temp_t tt;
    tt.ind = i;
    tt.metric = _metrics[i];
    sorted.push_back(tt);
  }

  // Sort
  std::sort( sorted.begin(), sorted.end(), sorting );
  
  // Return
  for( unsigned int i = 0; i < sorted.size(); ++i ) {
    sortedIndices.push_back( sorted[i].ind );
  }
  
  return sortedIndices;
}
/////////////////////////

void readSQline( std::stringstream &_input,
		 output_sq &_ou );

int main( int argc, char* argv[] ) {
  int v;
  while( (v=getopt(argc,argv,"h")) != -1 ) {
    switch(v) {
    case 'h': { printf("Syntax: %s \n", argv[0]); return 0; } break;
    }    
  }

  // Store
  output_sq out_r[gnF][50];
  output_sq out_h[gnF][50];
  
  //Read input
  std::ifstream input;
  std::string line; int lT; int gT = 0;

  // cylinders, balls, boxes, fruits, tampered
  for( int k = 0; k < gnG; ++k ) {

    // Get number of objects
    char dataFile[200]; 
    sprintf( dataFile, "%s/%s/%s_data.txt",
	     datapath.c_str(),
	     groupnames[k].c_str(),
	     groupnames[k].c_str() );
    
    // Read data file to get number of objects 
    input.open( dataFile, std::ifstream::in );
    std::getline(input, line); // gF
    std::getline(input, line); // gTableCoeff
    input >> lT;
    input.close();
    printf("lt: %d group: %s\n", lT, groupnames[k].c_str());

    // Get name of file
    char filename[200];
    sprintf( filename, "%s/%s_output.txt", datapath.c_str(), 
	     groupnames[k].c_str() );
    input.open( filename, std::ifstream::in );
    output_sq ou;
    std::string line;
    
    for( int i = 0; i < lT; ++i ) {
      for( int j = 0; j < gnF; ++j ) {
	
	// Read regular
	std::getline( input, line );
	std::stringstream ssr(line);
	readSQline( ssr, ou );
	ou.group = groupnames[k].c_str();
	out_r[j][gT + i] = ou;
	
	// Read hierarchical
	std::getline( input, line );
	std::stringstream ssh(line);
	readSQline( ssh, ou );
	ou.group = groupnames[k].c_str();
	out_h[j][gT + i] = ou;
	
      } // end j
    } // end for i
   
    input.close();    

    gT += lT;
  }// for k

  printf("Total objects: %d \n", gT);

  // Order according to time in radial
  std::vector<double> tm;
  std::vector<unsigned int> ti;
  for( int i = 0; i < gT; ++i ) {
     tm.push_back( out_r[0][i].t );
  }
    ti = sortIndices(tm);  

  // Output time in order
  std::ofstream output_time("output_time.txt", std::ofstream::out);
  std::ofstream output_time_h("output_time_hier.txt", std::ofstream::out);
  for( int i = 0; i  < gT; ++i ) {
     output_time << i << " ";
     output_time_h << i << " ";
     for( int j = 0; j < gnF; ++j ) {
        output_time << out_r[j][ti[i]].t <<" ";
        output_time_h << out_h[j][ti[i]].t <<" ";
     }
     output_time << std::endl;
     output_time_h << std::endl;
  }
  output_time.close();
  output_time_h.close();

  // Order according to Er in radial
  std::vector<double> rm;
  std::vector<unsigned int> ri;
  for( int i = 0; i < gT; ++i ) {
     rm.push_back( out_r[0][i].er_r );
  }
    ri = sortIndices(rm);  

  // Output time in order
  std::ofstream output_radial("output_radial.txt", std::ofstream::out);
  std::ofstream output_radial_h("output_radial_hier.txt", std::ofstream::out);
  for( int i = 0; i  < gT; ++i ) {
     output_radial << i << " ";
     output_radial_h << i << " ";
     for( int j = 0; j < gnF; ++j ) {
        output_radial << out_r[j][ri[i]].er_r <<" ";
        output_radial_h << out_h[j][ri[i]].er_r <<" ";
     }
     output_radial << std::endl;
     output_radial_h << std::endl;
  }
  output_radial.close();
  output_radial_h.close();

  // Order according to Eg in radial
  std::vector<double> gm;
  std::vector<unsigned int> gi;
  for( int i = 0; i < gT; ++i ) {
     gm.push_back( out_r[0][i].er_g );
  }
    ri = sortIndices(rm);  

  // Output time in order
  std::ofstream output_goodness("output_goodness.txt", std::ofstream::out);
  std::ofstream output_goodness_h("output_goodness_hier.txt", std::ofstream::out);
  for( int i = 0; i  < gT; ++i ) {
     output_goodness << i << " ";
     output_goodness_h << i << " ";
     for( int j = 0; j < gnF; ++j ) {
        output_goodness << out_r[j][ri[i]].er_g <<" ";
        output_goodness_h << out_h[j][ri[i]].er_g <<" ";
     }
     output_goodness << std::endl;
     output_goodness_h << std::endl;
  }
  output_goodness.close();
  output_goodness_h.close();


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
