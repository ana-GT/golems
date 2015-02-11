/**
 * @file readHandStates.cpp
 * @brief Read current hand states
 */
#include <unistd.h>
#include <sns.h>
#include <ach.h>

#define SDH_AXES 7

enum SDH_SIDE {
    SDH_LEFT= 0,
    SDH_RIGHT = 1
};

ach_channel_t chan_sdhstate_left;
ach_channel_t chan_sdhstate_right;
double ql[SDH_AXES]; double dql[SDH_AXES];
double qr[SDH_AXES]; double dqr[SDH_AXES];
struct timespec now;

//***********************************************
// Functions prototype
static void update( void );
static int update_n( size_t n,
		     double* q, double *dq,
		     ach_channel_t *chan,
		     struct timespec *ts );

/************************************************
 * @function main
 *************************************************/
int main( int argc, char* argv[] ) {
    
    // Open log channel and set some stuff
    sns_init();
    
    // Shoots a message to parent (Achcop?)
    sns_start();

    // Open sdh state channels
    sns_chan_open( &chan_sdhstate_left, "sdhstate-left", NULL );
    sns_chan_open( &chan_sdhstate_right, "sdhstate-right", NULL );

    {
	ach_channel_t *chans[] = { &chan_sdhstate_left, 
				   &chan_sdhstate_right, 
				   NULL};
	sns_sigcancel( chans, sns_sig_term_default );
    }

    
    // Update 
    while( !sns_cx.shutdown ) {
	update();
	aa_mem_region_local_release();
	usleep(0.5*1e6);
    }

    sns_end();
    return 0;

}

/**
 * @function update
 */
static void update( void ) {

    if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
	SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n",
		 strerror(errno) );
    }

    struct timespec timeout = sns_time_add_ns( now, 1000*1000*1 );
    int is_updated = 0;

    // Get SDH
    int u_sl = update_n( SDH_AXES, ql, dql, 
			 &chan_sdhstate_left,
			 &timeout );

    printf("SDH Left: Pos: (");
    for( int i = 0; i < SDH_AXES; ++i ) {
	printf(" %f ", ql[i] );
    } printf("\n");
    printf("SDH Right: Vel: (");
    for( int i = 0; i < SDH_AXES; ++i ) {
	printf(" %f ", dql[i] );
    } printf("\n");


    int u_sr = update_n( SDH_AXES, qr, dqr,
			 &chan_sdhstate_right,
			 &timeout );

    printf("SDH Right: Pos: (");
    for( int i = 0; i < SDH_AXES; ++i ) {
	printf(" %f ", qr[i] );
    } printf("\n");
    printf("SDH Right: Vel: (");
    for( int i = 0; i < SDH_AXES; ++i ) {
	printf(" %f ", dqr[i] );
    } printf("\n");
    
    is_updated = is_updated || u_sl || u_sr;
    if( !is_updated ) {
	printf("Not updated with frame or OK \n");
    }
}

/**
 * @function update_n
 *
 */
static int update_n( size_t n,
		     double* q, double *dq,
		     ach_channel_t *chan,
		     struct timespec *ts ) {

    size_t frame_size;
    void *buf = NULL;
    ach_status_t r = sns_msg_local_get( chan, &buf, 
					&frame_size,
					ts, ACH_O_LAST | (ts ? ACH_O_WAIT : 0 ) );
    
    switch(r) {
    case ACH_OK:
    case ACH_MISSED_FRAME:
	{
	    struct sns_msg_motor_state *msg = (struct sns_msg_motor_state*)buf;
	    if( n == msg->header.n &&
		frame_size == sns_msg_motor_state_size_n((uint32_t)n) ) {
		for( size_t j = 0; j < n; ++j ) {
		    q[j] = msg->X[j].pos;
		    dq[j] = msg->X[j].vel;
		} // end for
		return 1;
	    } // end if
	    else {
		SNS_LOG( LOG_ERR, "Invalid motor_state message \n" );
	    }
	} break;

    case ACH_TIMEOUT:
    case ACH_STALE_FRAMES:
    case ACH_CANCELED:
	break;
    default:
	{SNS_LOG( LOG_ERR, "Failed ach_get: %s \n", ach_result_to_string(r) ); }
    } // end switch

    return 0;

}
