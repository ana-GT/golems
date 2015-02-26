/**
 * @file moveArmBasic.cpp
 * @brief Move left arm
 */
#include <unistd.h>
#include <sns.h>
#include <ach.h>
#include <poll.h>

#define ARM_AXES 7
#define VEL_STEP 0.08726 // 5 degrees


//*******************************
// Global variables            
//*******************************
ach_channel_t chan_state;
ach_channel_t chan_ref;

double q[ARM_AXES]; double dq[ARM_AXES];
struct timespec now;

//***************************
// Functions prototype
//***************************
static int update( void );
static int update_n( size_t n,
		     double* q, double *dq,
		     ach_channel_t *chan,
		     struct timespec *ts );
static void control_n( size_t n,
		       double *x,
		       double tsec,
		       ach_channel_t* chan );
void printHelp( char* argv0 );
bool check_userInput( int &_arm_drive_ind,  
		      double &_dv );

/************************************************
 * @function main
 *************************************************/
int main( int argc, char* argv[] ) {

    // Variable to move a finger at a time
    double tsec = 2;
    int arm_drive_ind; double dv;
    struct pollfd stdin_poll;
    stdin_poll.fd = STDIN_FILENO;
    stdin_poll.events = POLLIN;
    //--------------------
    // 1. Parse
    //--------------------
    int c; 
    while((c = getopt(argc, argv, "h" )) != -1 ) {
	switch(c) {
	case 'h': { printHelp(argv[0]); return 1; } break;
	} // end switch
    } // end while

    
    //-----------------------------------------
    // 2. Open log channel and set some stuff
    //-----------------------------------------
    sns_init();
    
    //---------------------------------------------------
    // 3. Shoots a signal to let parent know we are OK
    //---------------------------------------------------
    sns_start();
    
    //--------------------------------
    // 4. Open sdh state channels
    //--------------------------------
    sns_chan_open( &chan_state, "state-left", NULL );
    sns_chan_open( &chan_ref, "ref-left", NULL );

    //-----------------------------------------------------------------------
    // 5. Set sighandlers to die gracefully due to SIGTERM and other animals
    //------------------------------------------------------------------------
    {
	ach_channel_t *chans[] = { &chan_state, 
				   &chan_ref,
				   NULL};
	sns_sigcancel( chans, sns_sig_term_default );
    }

    //-----------------------
    // 6. Loop
    //-----------------------
    while( !sns_cx.shutdown ) {
	// Update, read pos and vel of hands
	update();
	// Check if user put input
	if( poll(&stdin_poll, 1, 0 ) == 1 ) {
	    if( check_userInput( arm_drive_ind, dv ) == true ) {
		printf("Should send arm message \n");
		
		for( int j = 0; j < ARM_AXES; ++j ) { dq[j] = 0; }
		    dq[arm_drive_ind] = dv;
		    control_n( ARM_AXES, dq, tsec, &chan_ref );	 
		
		
	    }
	} // end polling
	aa_mem_region_local_release();
	usleep(0.5*1e6);	
    }
    
    printf("\t * [INFO] Killing me softly with SIG-TERM \n");
    sns_end();
    return 0;

}

/**
 * @function control_n
 */
static void control_n( size_t n,
		       double *x,
		       double tsec,
		       ach_channel_t* chan ) {

    struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc( ARM_AXES );
    sns_msg_header_fill( &msg->header );
    msg->mode = SNS_MOTOR_MODE_VEL;
    msg->header.n = ARM_AXES;
    AA_MEM_CPY( msg->u, x, n );
    std::cout << "WOULD BE SENDING: "<<std::endl;
    aa_dump_vec( stdout, x, n );


    // Duration from now + tnano
    double tnano = tsec*1e9;
    if( clock_gettime( ACH_DEFAULT_CLOCK, &now )  ){
	SNS_LOG( LOG_ERR, "clock_gettime failed: %s \n", strerror(errno) );
    }
    sns_msg_set_time( &msg->header, &now, tnano );

    // Send
    ach_put( chan, msg, sns_msg_motor_ref_size(msg) );

}


/**
 * @function check_userInput
 */
bool check_userInput( int &_arm_drive_ind,  
		      double &_dv ) {
   
    char drive_id[4];
    char dir[2];
    char line[40];
    gets( line );
    int r = sscanf( line, "%s%s", drive_id, dir );
    if( r == 2 ) {
	
	// Check direction (increase/decrease)
	if( strcmp(dir, "+") == 0 ) { _dv = VEL_STEP; }
	else if( strcmp(dir, "-") == 0 ) { _dv = -VEL_STEP; }
	else { printf("\t * [WARNING] Did not set direction +/- \n"); return false; }
	
	// Check which finger
	if( strcmp(drive_id,"l0") == 0 ) {
	    _arm_drive_ind = 0;
	} else if( strcmp(drive_id, "l1") == 0 ) {
	    _arm_drive_ind = 1;
	} else if( strcmp(drive_id, "l2") == 0 ) {
	    _arm_drive_ind = 2;
	} else if( strcmp(drive_id, "l3") == 0 ) {
	    _arm_drive_ind = 3;
	} else if( strcmp(drive_id, "l4") == 0 ) {
	    _arm_drive_ind = 4;
	} else if( strcmp(drive_id, "l5") == 0 ) {
	    _arm_drive_ind = 5;
	} else if( strcmp(drive_id, "l6") == 0 ) {
	    _arm_drive_ind = 6; 
	} else {
	    printf("\t *[check_userInput] No recognized option \n");
	    return false;
	}
	printf("\t -- [INFO] Moving drive %d with dv:%f of left arm \n",
	       _arm_drive_ind, _dv );
	return true;
    } else {
	return false;
    }


}


/**
 * @function update
 */
static int update( void ) {

    if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
	SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n",
		 strerror(errno) );
    }

    struct timespec timeout = sns_time_add_ns( now, 1000*1000*1 );
    int is_updated = 0;

    // Get ARM States
    int u_al = update_n( ARM_AXES, q, dq, 
			 &chan_state,
			 &timeout );

    
    return u_al;
}

/**
 * @function update_n
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

/**
 * @function printHelp
 */
void printHelp( char* argv0 ) {
    
    printf("Usage: \n");
    printf("\t -h: This help \n");
    printf("\t * Once the program started running: \n");
    printf("\t ARM_DRIVE DIRECTION [ENTER] : \n" );
    printf("\t \t - ARM_DRIVE={l0,l1,l2,l3,l4,l5,l6,r0,r1,r2,r3,r4,r5,r6}\n");
    printf("\t \t - DIRECTION={+,-}\n");
}
