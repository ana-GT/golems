
#include <unistd.h>
#include <sns.h>
#include <ach.h>
#include <stdio.h>


ach_channel_t chan;
struct timespec now;
double tnano;

int main( int argc, char* argv[] ) {
  
  // Open channel
  sns_chan_open( &chan, "dare-chan", NULL );
  
  // Receive continuously
  int n = 7;

  ach_status_t r;
  while( true ) {
    
    if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
      SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
      return false;
    }
    struct timespec ts = sns_time_add_ns( now, 1000*1000*1 );
    
    size_t frame_size;
    void *buf = NULL;
    ach_status_t r = sns_msg_local_get( &chan, &buf,
					&frame_size,
					&ts, 
					ACH_O_LAST | (&ts ? ACH_O_WAIT : 0 ) );
    switch(r) {
      
    case ACH_OK:
    case ACH_MISSED_FRAME: {
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
    } break;
      
    case ACH_TIMEOUT:
    case ACH_STALE_FRAMES:
    case ACH_CANCELED:
      break;
    default:
      SNS_LOG( LOG_ERR, "Failed ach_get: %s \n", ach_result_to_string(r) );
    } // end switch

    usleep(0.01*1e6);

  } // end while

  return 0;
  
}
