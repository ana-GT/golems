/**
 * @file Test sending a velocity with Tobi's trajectory code
 * @brief Input: Start location, goal location. Output: Continuous velocity profile
 */
#include <sns.h>
#include <ach.h>
#include <unistd.h>
#include <basicControl/basicControl.h>
#include <list>
#include <basicControl/trajectories/Trajectory.h>

ach_channel_t chan_state;
ach_channel_t chan_ref;
ach_channel_t traj_chan[2];

struct timespec t_now;
struct timespec t_start;
struct timespec t_timeout;

double d_start; double d_now;
double d_duration = 2;

double tsec = 0.01;
double vel = 0.1;
const int NUM_JOINTS = 7;
BasicControl bc;

void zero();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    sns_init();
    sns_start();

    sns_chan_open( &traj_chan[0], "test-left", NULL );
    sns_chan_open( &traj_chan[1], "test-right", NULL );

    Eigen::VectorXd start( NUM_JOINTS );
    start << -0.3178, 1.2605, -0.3241, 0.4568, -0.320146, -1.493, 1.997;
    Eigen::VectorXd goal( NUM_JOINTS);
    goal = start;
    goal(5) = start(5) + 0.2;
    goal(6) = start(6) - 0.3;

    std::list<Eigen::VectorXd> path;
    path.push_back( start );
    path.push_back( goal );

  struct sns_msg_path_dense* msg = sns_msg_path_dense_alloc( path.size(), (*(path.begin())).size() );
  sns_msg_header_fill( &msg->header );
  
  msg->header.n = (*(path.begin())).size();
  printf("Header: %d \n", msg->header.n);
  // Fill
  int counter = 0;
  std::list<Eigen::VectorXd>::iterator it;
  for( it=path.begin(); it != path.end(); ++it ) {
    for( int j = 0; j < (*it).size(); ++j ) {
      msg->x[counter] = (*it)(j);
      counter++;
    }
  } 

  for( int i = 0; i < 14; ++i ) {
    printf("x[%d]: %f \n", i, msg->x[i]);
  }

 // Duration from now + tnano
  double tsec = 0.5;
  double tnano = tsec*1e9;
  struct timespec now;
  if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ){
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );
  }
  sns_msg_set_time( &msg->header, &now, tnano );

  // Send
  printf("** Start sending trajectory RIGHT over the network ** \n");
  ach_status_t r;
  r = ach_put( &traj_chan[1], msg, sns_msg_path_dense_size(msg) );
  std::cout << "\t ** Done sending trajectory over the network ** \n" << std::endl;
  
  if( r != ACH_OK ) { printf("Crap, sent it wrong \n"); } 
  else { printf("Sent ot right \n"); }
  usleep(3.0*1e6);
  return 0;

}
