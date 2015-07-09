/**
 * @function 
 */
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>

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

  // Calculate the mean time, er1, er2, and er4
  double sum_er1[6][4];
  double sum_er2[6][4];
  double sum_er4[6][4];
  double sum_time[6][4];
  double sum_volume[6][4];

  double sum_e1_error[6][4];
  double sum_e2_error[6][4];

  for( int i = 0; i < 6; ++i ) { 
    for(int j = 0; j < 4; ++j ) { 
      sum_er1[i][j] = 0; 
      sum_er2[i][j] = 0; 
      sum_er4[i][j] = 0; 
      sum_time[i][j] = 0; 
      sum_volume[i][j] = 0;
      sum_e1_error[i][j] = 0;
      sum_e2_error[i][j] = 0;
    } 
  }

  for( int i = 0; i < 6; ++i ) { // each function
    for( int j = 0; j < 4; ++j ) {  // each type of input
      for( int k = 0; k < T; ++k ) { // each instance
	
  minks d = info[i][k][j];
  double vol = baseline[k].dim[0]*baseline[k].dim[1]*baseline[k].dim[2];

	sum_er1[i][j] += d.er1/(double)T;
	sum_er2[i][j] += d.er2/(double)T;
	sum_er4[i][j] += d.er4/(double)T; 
	sum_time[i][j] += d.t/(double)T;
  sum_volume[i][j] += fabs((d.dim[0]*d.dim[1]*d.dim[2]) - vol)/vol*100.0/(double)T;
  sum_e1_error[i][j] += fabs(d.e[0] - baseline[k].e[0] ) / baseline[k].e[0]*100.0/(double)T;
  sum_e2_error[i][j] += fabs(d.e[1] - baseline[k].e[1] ) / baseline[k].e[1]*100.0/(double)T;
      }
    }
  }



  printf("FOR LATEX START \n");
  char car[6] = {'r','s','i','c','5','6'};
  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
      if( i == 2 || i == 5 ) { continue; }
      printf("& $F_{%c}$ & %f & %.3f\\% & %.3f\\% & %.3f\\% & %.3f \\\ \n", car[i], sum_er1[i][j], sum_volume[i][j],
	     sum_e1_error[i][j], sum_e2_error[i][j], sum_time[i][j] ); 
    }
    printf("\n");
  }
  printf("FOR LATEX END \n");


  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("Er1 average for F %d in pointcloud %d: %f \n", i, j, sum_er1[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");

  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("Er2 average for F %d in pointcloud %d: %f \n", i, j, sum_er2[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");

 
  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("Er4 average for F %d in pointcloud %d: %f \n", i, j, sum_er4[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");



  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("Time average for F %d in pointcloud %d: %f \n", i, j, sum_time[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");

  
  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("Error volume for F %d in pointcloud %d: %f \n", i, j, sum_volume[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");


  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("Error e1 for F %d in pointcloud %d: %f \n", i, j, sum_e1_error[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");


  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("Error e2 for F %d in pointcloud %d: %f \n", i, j, sum_e2_error[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");

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
