/**
 * @function base_control_client.cpp
 */
#include "base_control.h"
#include <sns.h>
#include <unistd.h>
#include <ach.h>

ach_channel_t traj_chan;
int num_joints = 2;

int main( int argc, char* argv[] ) {

  // Open trajectory channel
  sns_chan_open( &traj_chan, "traj-left", NULL );

  // Set path
  std::list<Eigen::VectorXd> path;
  Eigen::VectorXd po(num_joints);
  Eigen::VectorXd pf(num_joints);

  po(0) = 0.1;
  po(1) = 0.3;
  
  pf(0) = po(0) + 0.1;
  pf(1) = po(1) - 0.25;
  
  path.push_back( po );
  path.push_back( pf );
  
  // Build a message
  struct sns_msg_path_dense* msg = sns_msg_path_dense_alloc( path.size(),
							     (*(path.begin())).size() );

  msg->n_dof = (*(path.begin())).size();
  msg->n_steps = path.size();
  
  sns_msg_header_fill( &msg->header );
  msg->header.n = (*(path.begin())).size();

  // Fill path
  int counter = 0;
  std::list<Eigen::VectorXd>::iterator it;
  for( it = path.begin(); it != path.end(); ++it ) {
    for( int j = 0; j < (*it).size(); ++j ) {
      msg->x[counter] = (*it)(j);
      counter++;
    }
  }

  // Set message duration
  double tsec = 0.05;
  int64_t dt_nsec = tsec*1e9;

  struct timespec now;
  if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );   
  }
  sns_msg_set_time( &msg->header, &now, dt_nsec );

  // Send message
  ach_status_t r;
  r = ach_put( &traj_chan, msg, sns_msg_path_dense_size(msg) );
  if( r!= ACH_OK ) { printf("\t * [ERROR] Error sending path \n"); }
  else { printf("\t * [INFO] Path was sent all right\n"); }
}
