

#include <iostream>
#include "pick_pd.h"

int main( int argc, char* argv[] ) {

  if( argc < 2 ) { std::cout << "Syntax: "<< argv[0] << " filename" << std::endl; return 1; }
  
  std::string name(argv[1]);

  Pick_PD p;
  p.read_taskfile( name );

  return 0;
}
