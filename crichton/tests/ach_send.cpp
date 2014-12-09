
#include <unistd.h>
#include <sns.h>
#include <ach.h>
#include <stdio.h>
#include <Eigen/Core>
#include <list>

ach_channel_t chan;
struct timespec now;
double tnano;

int main( int argc, char* argv[] ) {
  
  // Open channel
  sns_chan_open( &chan, "dare-chan", NULL );
  
  // Send continuously
  int n = 7;

  ach_status_t r;

  Eigen::VectorXd start( 7 );
  start << -0.3178, 1.2605, -0.3241, 0.4568, -0.320146, -1.493, 1.997;
  Eigen::VectorXd goal( 7 );
  goal = start;
  goal(5) = start(5) + 0.2;
  goal(6) = start(6) - 0.3;
  
  std::list<Eigen::VectorXd> path;
  path.push_back( start );
  path.push_back( goal );
  

  while( true ) {

    struct sns_msg_path_dense* msg = sns_msg_path_dense_alloc( path.size(), 7 );
    sns_msg_header_fill( &msg->header );
    msg->header.n = 7;

    // Fill
    int counter = 0;
    std::list<Eigen::VectorXd>::iterator it;
    for( it=path.begin(); it != path.end(); ++it ) {
      for( int j = 0; j < (*it).size(); ++j ) {
	msg->x[counter] = (*it)(j);
	counter++;
      }
    } 


    if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) { 
      printf("ERROR grabbing time \n"); 
      return 1; 
    }

    tnano = 0.01*1e9;
    sns_msg_set_time( &msg->header, &now, tnano );
    
    r = ach_put( &chan, msg, sns_msg_path_dense_size(msg) );
    if( r != ACH_OK ) { printf("[WARNING!!] Did not send the message all right \n"); }

    usleep(0.1*1e6);
  }

  return 0;
  
}
