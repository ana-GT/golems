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
  double er_g, er_r;
  double er_e1, er_e2, er_v;
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
  double sum_er_e1[6][4];
  double sum_er_e2[6][4];
  double sum_er_er[6][4];
  double sum_er_eg[6][4];
  double sum_time[6][4];
  double sum_er_v[6][4];

  for( int i = 0; i < 6; ++i ) { 
    for(int j = 0; j < 4; ++j ) { 
      sum_er_e1[i][j] = 0; 
      sum_er_e2[i][j] = 0; 
      sum_er_er[i][j] = 0; 
      sum_time[i][j] = 0; 
      sum_er_v[i][j] = 0;
      sum_er_eg[i][j] = 0;
    } 
  }

  for( int i = 0; i < 6; ++i ) { // each function
    for( int j = 0; j < 4; ++j ) {  // each type of input
      for( int k = 0; k < T; ++k ) { // each instance
	
         minks d = info[i][k][j];

	sum_er_e1[i][j] += fabs(d.er_e1);
	sum_er_e2[i][j] += fabs(d.er_e2);
	sum_er_er[i][j] += fabs(d.er_r); 
	sum_er_eg[i][j] += fabs(d.er_g);
	sum_time[i][j] += d.t;
        sum_er_v[i][j] += fabs(d.er_v);;
      }


	sum_er_e1[i][j] /= (double)T;
	sum_er_e2[i][j] /= (double)T;
	sum_er_er[i][j] /= (double)T; 
	sum_er_eg[i][j] /= (double)T;
	sum_time[i][j] /= (double)T;
        sum_er_v[i][j] /= (double)T;

    }
  }



  printf("FOR LATEX START \n");
  char car[6] = {'r','s','i','c','5','6'};
  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
      if( i == 2 || i == 5 ) { continue; }
      printf("& $F_{%c}$ & %f & %.3f & %.3f & %.3f\\% & %.3f \\\ \n", car[i], sum_er_er[i][j], sum_er_eg[i][j], sum_er_e1[i][j],
	     sum_er_e2[i][j], sum_er_v[i][j], sum_time[i][j] ); 
    }
    printf("\n");
  }
  printf("FOR LATEX END \n");


  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("E1 average for F %d in pointcloud %d: %f \n", i, j, sum_er_e1[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");

  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("E2 average for F %d in pointcloud %d: %f \n", i, j, sum_er_e2[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");

 
  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("Er_r average for F %d in pointcloud %d: %f \n", i, j, sum_er_er[i][j] ); 
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
       printf("Error volume for F %d in pointcloud %d: %f \n", i, j, sum_er_v[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");


  for(int j = 0; j < 4; ++j ) { 
    for( int i = 0; i < 6; ++i ) { 
       printf("Error eg for F %d in pointcloud %d: %f \n", i, j, sum_er_eg[i][j] ); 
    }
    printf("\n");
  }
  printf("**********************************\n");



  // Save errors
  for( int i = 0; i < 4; ++i ) {
    char name[50];
    sprintf(name, "e1_%d.txt", i );
    std::ofstream output_e1( name, std::ofstream::out );

    sprintf(name, "e2_%d.txt", i );
    std::ofstream output_e2( name, std::ofstream::out );
   
    sprintf(name, "er_r_%d.txt", i );
    std::ofstream output_er_r( name, std::ofstream::out );
 
    sprintf(name,"t_%d.txt", i);
    std::ofstream output_t( name, std::ofstream::out );

    sprintf(name,"er_g_%d.txt", i);
    std::ofstream output_er_g( name, std::ofstream::out );

    sprintf(name,"er_v_%d.txt", i);
    std::ofstream output_er_v( name, std::ofstream::out );
        
    for( int j = 0; j < T; ++j ) {
      
      output_e1 << baseline[j].e[0] << " ";
      output_e2 << baseline[j].e[1] << " ";
      
      for( int k = 0; k < 6; ++k ) {	
	//output_er1 << info[k][j][i].er1 << " ";
	//output_er2 << info[k][j][i].er2 << " ";
	//output_er4 << info[k][j][i].er4 << " ";
        //output_t << info[k][j][i].t << " ";
	//output_e1 << info[k][j][i].e[0] << " ";
	//output_e2 << info[k][j][i].e[1] << " ";
	
      }
      //output_er1 << std::endl;
      //output_er2 << std::endl;
      //output_er4 << std::endl;
      //output_t << std::endl;
      //output_e1 << std::endl;
      //output_e2 << std::endl;      
    }
    //output_er1.close();
    //output_er2.close();
    //output_er4.close();
    //output_t.close();
    //output_e1.close();
    //output_e2.close();    
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
	 >> _d.t >> _d.er_g >> _d.er_r >> _d.er_e1 >> _d.er_e2 >> _d.er_v;
}
