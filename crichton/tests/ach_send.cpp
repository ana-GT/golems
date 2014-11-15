
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
  
  // Send continuously
  int n = 7;

  ach_status_t r;
  while( true ) {

    struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc(n);
    sns_msg_header_fill( &msg->header );
    msg->mode = SNS_MOTOR_MODE_VEL;

    for( int i = 0; i < n; ++i ) {
      msg->u[i] = 2*i;
    }

    if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) { 
      printf("ERROR grabbing time \n"); 
      return 1; 
    }

    tnano = 0.01*1e9;
    sns_msg_set_time( &msg->header, &now, tnano );

    r = ach_put( &chan, msg, sns_msg_motor_ref_size(msg) );
    if( r != ACH_OK ) { printf("[WARNING!!] Did not send the message all right \n"); }

    usleep(0.1*1e6);
  }

  return 0;
  
}
