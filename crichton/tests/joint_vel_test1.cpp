/**
 * @file Test sending a velocity for a fixed amount of time
 */
#include <sns.h>
#include <ach.h>
#include <unistd.h>

ach_channel_t chan_state;
ach_channel_t chan_ref;

struct timespec t_now;
struct timespec t_start;

double d_start; double d_now;
double d_duration = 2;

double tsec = 0.01;
double vel = 0.1;


/** Only send zero velocities to a joint */
bool control( double _vel ) {

  double x[1];
  x[0] = _vel;

  struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc( 1 );
  sns_msg_header_fill( &msg->header );
  msg->mode = SNS_MOTOR_MODE_VEL;
  msg->header.n = 1;
  AA_MEM_CPY( msg->u, x, 1 );

  
 double tnano = tsec*1e9;
 if( clock_gettime( ACH_DEFAULT_CLOCK, &t_now ) ) {
   printf("Clock gettime failed \n"); 
   return false;	
 } 	

 sns_msg_set_time( &msg->header, &t_now, tnano );
 // Send
 ach_put( &chan_ref, msg, sns_msg_motor_ref_size(msg) );
 return true;
}

/** Update states */
bool update() {

  if( clock_gettime(ACH_DEFAULT_CLOCK, &t_now ) ) {
    return false;
  }
  d_now = t_now.tv_sec + t_now.tv_nsec/1.0e9;
  return true;
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
  
  if( clock_gettime( ACH_DEFAULT_CLOCK, &t_start ) ) {
    printf("[ERROR] Could not update start time. Exiting \n");
    return 0;
  }

  d_start = t_start.tv_sec + t_start.tv_nsec/1.0e9;
  d_now = d_start;

  printf("Time start: %f \n", d_now );

  while( d_now < d_start + d_duration ) {
    if( !update() ) { printf("[ERROR] Failed updating. Exiting \n"); return 0; }
    if( !control(vel) ) { printf("[ERROR] Failed control send. Exiting \n"); return 0; }
    aa_mem_region_local_release();
    usleep(tsec*1e6);
  }

  // Send zero command
  if( !control(0) ) { printf("[ERROR] Could not send zero vel \n"); }

  printf("Time end: %f \n", d_now );

   sns_end();
   return 0;
}



