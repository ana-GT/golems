#include <sns.h>
#include <ach.h>
#include <unistd.h>

ach_channel_t chan_state;
ach_channel_t chan_ref;
struct timespec now;
double tsec = 0.01;

/** Only send zero velocities to a joint */
void control() {

  double x[1];
  x[0] = 0;

  struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc( 1 );
  sns_msg_header_fill( &msg->header );
  msg->mode = SNS_MOTOR_MODE_VEL;
  msg->header.n = 1;
  AA_MEM_CPY( msg->u, x, 1 );

  
 double tnano = tsec*1e9;
 if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
   printf("Clock gettime failed \n");	
 } 	
 printf("* Time sent : %d %d \n", now.tv_sec, now.tv_nsec);
 sns_msg_set_time( &msg->header, &now, tnano );
 // Send
 ach_put( &chan_ref, msg, sns_msg_motor_ref_size(msg) );

}

/** Update states */
void update() {

  if( clock_gettime(ACH_DEFAULT_CLOCK, &now ) ) {
    
  }

}

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

	sns_init();
  sns_start();

  sns_chan_open( &chan_state, "state-right", NULL ); 
  sns_chan_open( &chan_ref, "ref-right", NULL );

  {
    ach_channel_t *chans[] = { &chan_state, &chan_ref, NULL};
    sns_sigcancel( chans, sns_sig_term_default );
  }

  while( !sns_cx.shutdown ) {
    update();
    control();
    aa_mem_region_local_release();
    usleep(tsec*1e6);
  }

   sns_end();
   return 0;
}



