
#include <unistd.h>
#include <sns.h>
#include <ach.h>
#include <stdio.h>

bool check_traj_chan( struct timespec *ts );

double mDt = 0.01;


ach_channel_t chan;
struct timespec now;
struct timespec ts;
double tnano;

std::string traj_name[2];
ach_channel_t* traj_chan[2];


int main( int argc, char* argv[] ) {
  
  sns_init();
  sns_start();
  
  traj_name[0] = "test-left";
  traj_name[1] = "test-right";

  // Create channels  
  for( int i = 0; i < 2; ++i ) {
    traj_chan[i] = new ach_channel_t();
  }

  // Open & set channels
  for( int i = 0; i < 2; ++i ) {
    sns_chan_open( traj_chan[i], traj_name[i].c_str(), NULL );
  }
  
  
  // Receive continuously
  int n = 7;

  ach_status_t r;
  while( true ) {
    
    if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
      SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
      return 0;
    }
    ts = sns_time_add_ns( now, 1000*1000*1 );
    

    check_traj_chan( &ts );
    usleep((1e6*mDt) );
  } // end while

  return 0;
  
}


bool check_traj_chan( struct timespec *ts ) {

    size_t frame_size;
    void *buf = NULL;
    ach_status_t r = sns_msg_local_get( traj_chan[1], &buf,
					&frame_size,
					ts, 
					ACH_O_LAST | (ts ? ACH_O_WAIT : 0 ) );
    switch(r) {
      
    case ACH_OK:
    case ACH_MISSED_FRAME: {
      printf("Gotten something \n");
      /*
      struct sns_msg_motor_ref *msg = (struct sns_msg_motor_ref*)buf;
      if( n == msg->header.n &&
	  frame_size == sns_msg_motor_ref_size_n((uint32_t)n) ) {
	printf("Gotten message: \n");
	for( size_t j = 0; j < n; ++j ) {
	  printf(" %f ", msg->u[j]);
	}
	printf("\n");
      } else {
	SNS_LOG( LOG_ERR, "Invalid motor_state message \n" );
	return false;
      }
      */
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
