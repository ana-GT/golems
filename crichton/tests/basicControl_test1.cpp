/**
 * @file basicControl_test.cpp
 */
#include <Eigen/Dense>
#include "basicControl/bimanualControl.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  BimanualControl bc;
  std::vector<Eigen::VectorXd> path, tightPath;


  const int N = 5;
  Eigen::VectorXd p[N];
  
  for( int i = 0; i < N; ++i ) {
    Eigen::VectorXd t(7);
    t << 0, 0, 0, 0, 0, 0, 0;
    p[i] = t;
  }

  p[0] << -1.57, 1.57, 0.0, 0.0, 0.0, 0.0, 0.5;
  
  path.push_back( p[0] );
  
  p[1] << -1.57, 1.57, 0.0, 0.0, 0.0, 0.3, 0.5;
  path.push_back( p[1] );
  
  p[2] << -1.57, 1.57, 0.0, 0.0, 0.0, 0.0, 0.5;
  path.push_back( p[2] );
    


  tightPath = bc.preprocessPath( path );

  std::cout <<"Old path of size: "<< path.size() << std::endl;
  for( int i = 0; i < path.size(); i++ ) {
    std::cout << "P["<< i <<"]: "<< path[i].transpose() << std::endl;
  }
  
  std::cout << "************************" << std::endl;

  std::cout <<"New path of size: "<< tightPath.size() << std::endl;
  for( int i = 0; i < tightPath.size(); i++ ) {
    std::cout << "P["<< i <<"]: "<< tightPath[i].transpose() << std::endl;
  }
  
}
