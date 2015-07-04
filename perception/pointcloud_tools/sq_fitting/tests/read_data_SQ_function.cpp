/**
 * @function 
 */
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

struct minks{
  double dim[3];
  double e[2];
  double trans[3];
  double rot[3];
  double t;
  double er1, er2, er4;
};

void readSQline( std::stringstream &_input, minks &_d );

int main( int argc, char* argv[] ) {

  printf("Name: %s \n", argv[1]);
  std::ifstream input( argv[1], std::ifstream::in );
  int T;
  minks d;

  // Read number of files
  std::string line;
  std::getline( input, line );
  std::stringstream st(line);
  st >> T;
  printf("Input: %d \n", T);
  minks info[6][T][4];
  minks baseline[T];


  for( int i = 0; i < T; ++i ) {
    std::getline( input, line );
    std::stringstream ss(line);
    readSQline( ss, d );
    baseline[i] = d;

    for( int j = 0; j < 6; ++j ) {
      for( int k = 0; k < 4; ++k ) {
	std::getline( input, line );
	std::stringstream se(line);
	readSQline( se, d );
	info[j][i][k] = d;
      }
    }

  } // end for i
  printf("Finished reading fine \n");
  input.close();

  // Save errors
  for( int i = 0; i < 4; ++i ) {
    char name[50];
    sprintf(name, "er1_%d.txt", i );
    std::ofstream output_er1( name, std::ofstream::out );

    sprintf(name, "er2_%d.txt", i );
    std::ofstream output_er2( name, std::ofstream::out );
   
    sprintf(name, "er4_%d.txt", i );
    std::ofstream output_er4( name, std::ofstream::out );
 
    sprintf(name,"t_%d.txt", i);
    std::ofstream output_t( name, std::ofstream::out );

    sprintf(name,"e1_%d.txt", i);
    std::ofstream output_e1( name, std::ofstream::out );

    sprintf(name,"e2_%d.txt", i);
    std::ofstream output_e2( name, std::ofstream::out );
    
    
    for( int j = 0; j < T; ++j ) {
      
      output_e1 << baseline[j].e[0] << " ";
      output_e2 << baseline[j].e[1] << " ";
      
      for( int k = 0; k < 6; ++k ) {	
	output_er1 << info[k][j][i].er1 << " ";
	output_er2 << info[k][j][i].er2 << " ";
	output_er4 << info[k][j][i].er4 << " ";
        output_t << info[k][j][i].t << " ";
	output_e1 << info[k][j][i].e[0] << " ";
	output_e2 << info[k][j][i].e[1] << " ";
	
      }
      output_er1 << std::endl;
      output_er2 << std::endl;
      output_er4 << std::endl;
      output_t << std::endl;
      output_e1 << std::endl;
      output_e2 << std::endl;      
    }
    output_er1.close();
    output_er2.close();
    output_er4.close();
    output_t.close();
    output_e1.close();
    output_e2.close();    
  }
  
}

/**
 * @function readSQline
 */
void readSQline( std::stringstream &_input, 
		 minks &_d ) {

  _input >> _d.dim[0] >> _d.dim[1] >> _d.dim[2] 
	 >> _d.e[0] >> _d.e[1] 
	 >> _d.trans[0] >> _d.trans[1] >> _d.trans[2]
	 >> _d.rot[0] >> _d.rot[1] >> _d.rot[2]
	 >> _d.t >> _d.er1 >> _d.er2 >> _d.er4;
}
