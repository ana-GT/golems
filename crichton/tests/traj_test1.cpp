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

ach_channel_t traj_chan[2];
ach_channel_t rightstate_chan;

struct timespec t_now;
struct timespec t_start;
struct timespec t_timeout;

double d_start; double d_now;
double d_duration = 2;

double tsec = 0.01;
int64_t tnano = (int64_t)(tsec*1e9);


double vel = 0.1;
double q[7]; 
double dq[7];

BasicControl bc;


void zero();
bool update_n( size_t n, double *q, double *dq, ach_channel_t *chan );


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    sns_init();
    sns_start();

    sns_chan_open( &traj_chan[0], "traj-left", NULL );
    sns_chan_open( &traj_chan[1], "traj-right", NULL );

    sns_chan_open( &rightstate_chan, "state-left", NULL );

    // Get start right
    while( !update_n( 7, q, dq, &rightstate_chan ) ) {}


    Eigen::VectorXd start( 7 );
    for( int i = 0; i < 7; ++i ) { start(i) = q[i]; }

    Eigen::VectorXd goal( 7 );
    for( int i = 0; i < 7; ++i ) { goal(i) = start[i]; }
    goal(6) = start(6) - 0.2;

    std::list<Eigen::VectorXd> path;
    path.push_back( start );
    path.push_back( goal );

  struct sns_msg_path_dense* msg = sns_msg_path_dense_alloc( path.size(), (*(path.begin())).size() );
  sns_msg_header_fill( &msg->header );
  
  msg->header.n = (*(path.begin())).size();
  // Fill
  int counter = 0;
  std::list<Eigen::VectorXd>::iterator it;
  for( it=path.begin(); it != path.end(); ++it ) {
    for( int j = 0; j < (*it).size(); ++j ) {
      msg->x[counter] = (*it)(j);
      counter++;
    }
  } 

  counter = 0;
  for( it = path.begin(); it != path.end(); ++it ) {
   std::cout << "]"<< counter <<"Path point: "<< (*it).transpose() << std::endl;
   counter++;
  }  

 // Duration from now + tnano
  if( clock_gettime( ACH_DEFAULT_CLOCK, &t_now ) ){
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );
  }
  sns_msg_set_time( &msg->header, &t_now, tnano );

  // Send
  printf("** Start sending trajectory LEFT ** \n");
  ach_status_t r;
  r = ach_put( &traj_chan[0], msg, sns_msg_path_dense_size(msg) );
  std::cout << "\t ** Done sending trajectory ** \n" << std::endl;
  
  if( r != ACH_OK ) { printf("Crap, sent it wrong \n"); } 
  else { printf("Sent ot left \n"); }
  usleep(3.0*1e6);
  return 0;

}

/**
 *
 */
bool update_n( size_t n, double *q, double *dq, ach_channel_t *chan ) {
  

  struct timespec ts;

  if( clock_gettime( ACH_DEFAULT_CLOCK, &t_now ) ){
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );
  }
  ts = sns_time_add_ns( t_now, tnano );

  size_t frame_size;
  void *buf = NULL;
  ach_status_t r = sns_msg_local_get( chan, &buf,
				      &frame_size,
				      &ts, 
				      ACH_O_LAST | ACH_O_WAIT );
  switch(r) {
    
  case ACH_OK:
  case ACH_MISSED_FRAME: {
    struct sns_msg_motor_state *msg = (struct sns_msg_motor_state*)buf;
    if( n == msg->header.n &&
	frame_size == sns_msg_motor_state_size_n((uint32_t)n) ) {
      for( size_t j = 0; j < n; ++j ) {
	q[j] = msg->X[j].pos;
	dq[j] = msg->X[j].vel;
      }
      return true;
    } else {
      SNS_LOG( LOG_ERR, "Invalid motor_state message \n" );
      return false;
    }
  } break;
    
  case ACH_TIMEOUT:
  case ACH_STALE_FRAMES:
  case ACH_CANCELED:
    break;
  default:
    SNS_LOG( LOG_ERR, "Failed ach_get: %s \n", ach_result_to_string(r) );
  } // end switch

  return false;

}
