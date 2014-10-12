/**
 * @file moveFingers.cpp
 * @brief Move fingers of left OR right hand at a time
 */
#include <unistd.h>
#include <sns.h>
#include <ach.h>
#include <poll.h>

#define SDH_AXES 7
#define DJ 0.08726 // 5 degrees

enum SDH_SIDE {
    SDH_LEFT= 0,
    SDH_RIGHT = 1
};

//*******************************
// Global variables            
//*******************************
ach_channel_t chan_sdhstate_left;
ach_channel_t chan_sdhstate_right;
ach_channel_t chan_sdhref_left;
ach_channel_t chan_sdhref_right;

double ql[SDH_AXES]; double dql[SDH_AXES];
double qr[SDH_AXES]; double dqr[SDH_AXES];
struct timespec now;

//***************************
// Functions prototype
//***************************
static void update( void );
static int update_n( size_t n,
		     double* q, double *dq,
		     ach_channel_t *chan,
		     struct timespec *ts );
void sdh_pos( double* x,
	      int side = SDH_LEFT,
	      double tsec = 5 );
bool sdh_within_limits( double* x );
void printHelp( char* argv0 );
bool check_userInput( int &_sdh_finger_ind,  
		      double &_dj,
		      int _sdh_side );

/************************************************
 * @function main
 *************************************************/
int main( int argc, char* argv[] ) {

    // Variable to move a finger at a time
    int sdh_side = SDH_LEFT;
    double tsec = 3;
    int sdh_finger_ind; double dj;
    struct pollfd stdin_poll;
    stdin_poll.fd = STDIN_FILENO;
    stdin_poll.events = POLLIN;
    //--------------------
    // 1. Parse
    //--------------------
    int c; 
    while((c = getopt(argc, argv, "hm:" )) != -1 ) {
	switch(c) {
	case 'h': { printHelp(argv[0]); return 1; } break;
	case 'm': {
	    if( strcmp( optarg, "SDH_RIGHT" ) == 0 ) {
		sdh_side = SDH_RIGHT;
	    } else if( strcmp( optarg, "SDH_LEFT" ) == 0 ) {
		sdh_side = SDH_LEFT;
	    }
	} break;
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
    sns_chan_open( &chan_sdhstate_left, "sdhstate-left", NULL );
    sns_chan_open( &chan_sdhstate_right, "sdhstate-right", NULL );
    sns_chan_open( &chan_sdhref_left, "sdhref-left", NULL );
    sns_chan_open( &chan_sdhref_right, "sdhref-right", NULL );

    //-----------------------------------------------------------------------
    // 5. Set sighandlers to die gracefully due to SIGTERM and other animals
    //------------------------------------------------------------------------
    {
	ach_channel_t *chans[] = { &chan_sdhstate_left, 
				   &chan_sdhstate_right, 
				   &chan_sdhref_left,
				   &chan_sdhref_right,
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
		printf("Data to search! \n");
		if( check_userInput( sdh_finger_ind, dj, sdh_side ) == true ) {
	    		printf("Should send finger message \n");
		
	    		if( sdh_side == SDH_LEFT ) {
				ql[sdh_finger_ind] = ql[sdh_finger_ind] + dj;
				sdh_pos( ql, SDH_LEFT, tsec );	 
	    		} else if( sdh_side == SDH_RIGHT ) {
	    			qr[sdh_finger_ind] = qr[sdh_finger_ind] + dj;
				sdh_pos( qr, SDH_RIGHT, tsec );	 
	    		}
			    
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
 * @function check_userInput
 */
bool check_userInput( int &_sdh_finger_ind,  
		      double &_dj,
		      int _sdh_side ) {
   
    char finger_id[4];
    char dir[2];
    char line[40];
    gets( line );
    int r = sscanf( line, "%s%s", finger_id, dir );
    if( r == 2 ) {
	
	// Check direction (increase/decrease)
	if( strcmp(dir, "+") == 0 ) { _dj = DJ; }
	else if( strcmp(dir, "-") == 0 ) { _dj = -DJ; }
	else { printf("\t * [WARNING] Did not set direction +/- \n"); return false; }
	
	// Check which finger
	if( strcmp(finger_id,"lrf") == 0 ) {
	    _sdh_finger_ind = 0;
	} else if( strcmp(finger_id, "lf1") == 0 ) {
	    _sdh_finger_ind = 1;
	} else if( strcmp(finger_id, "lf2") == 0 ) {
	    _sdh_finger_ind = 2;
	}   else if( strcmp(finger_id, "tf1") == 0 ) {
	    _sdh_finger_ind = 3;
	}  else if( strcmp(finger_id, "tf2") == 0 ) {
	    _sdh_finger_ind = 4;
	} else if( strcmp(finger_id, "rf1") == 0 ) {
	    _sdh_finger_ind = 5;
	} else if( strcmp(finger_id, "rf2") == 0 ) {
	    _sdh_finger_ind = 6;
	} else {
	    printf("\t *[check_userInput] No recognized option \n");
	    return false;
	}


	printf("\t -- [INFO] Moving finger %d with dj:%f of hand: %s \n",
	       _sdh_finger_ind, _dj,
	       _sdh_side == SDH_LEFT? "sdh_left" : "sdh_right" );
	return true;
    } else {
	return false;
    }


}

/****/
bool sdh_within_limits( double* x ) {
    // Common [0 and +90]
    if( x[0] < 0 || x[0] > M_PI*0.5 ) {
	printf("\t [WARNING] Index %d of values for fingers exceed limits:", 0);
	return false;
    }
    // First joint [-90, +90]
    int ind_proximal[3] = {1,3,5};
    for( int i = 0; i < 3; ++i ) {
	if( x[ind_proximal[i]] < -M_PI*0.5 ||
	    x[ind_proximal[i]] > M_PI*0.5 ) {
	    printf("\t [WARNING] Index %d of values for fingers exceed limits:", ind_proximal[i]);
	    return false;
	}
    }
    // Second joint [-90, +90]
    int ind_distal[3] = {2,4,6};
    for( int i = 0; i < 3; ++i ) {
	if( x[ind_distal[i]] < -M_PI*0.5 ||
	    x[ind_distal[i]] > M_PI*0.5 ) {
	    printf("\t [WARNING] Index %d of values for fingers exceed limits:", ind_distal[i]);
	    return false;
	}
    }

    return true;
}

/**
 * @function sdh_pos
 * @brief Send position motor msg to hand
 */
void sdh_pos( double* x, int side, double tsec ) {

    // Time in seconds -> nanoseconds
    double tnano = tsec*1e9;

    if( tnano < 1000000000 ) {
	printf("\t [NO!] Time less than 3 sec is too small and dangerous! \n");
	return;
    }

    // Check limits
    if( !sdh_within_limits( x ) ) { 
       return;
    }

    // Update time
    if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
	SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n",
		 strerror(errno) );
    }


    struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc( SDH_AXES );
    sns_msg_header_fill( &msg->header );
    msg->mode = SNS_MOTOR_MODE_POS;
    AA_MEM_CPY( msg->u, x, SDH_AXES );
    aa_dump_vec( stdout, x, SDH_AXES );


    // Duration from now till now + tnano
    sns_msg_set_time( &msg->header, &now, tnano );

    switch( side ) {
    case SDH_LEFT: {
	ach_put( &chan_sdhref_left, msg, sns_msg_motor_ref_size(msg) );
	break; 
    }
    case SDH_RIGHT: {
	ach_put( &chan_sdhref_right, msg, sns_msg_motor_ref_size(msg) );
	break;
    }
    } // end switch

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

    int u_sr = update_n( SDH_AXES, qr, dqr,
			 &chan_sdhstate_right,
			 &timeout );
    
    is_updated = is_updated || u_sl || u_sr;
			 
}

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
    printf("\t -m: SDH_LEFT | SDH_RIGHT \n");
    printf("\t * Once the program started running: \n");
    printf("\t FINGER DIRECTION [ENTER] : \n" );
    printf("\t \t - FINGER={lf1,lf2,rf1,rf2,tf1,tf2}\n");
    printf("\t \t - DIRECTION={+,-}\n");
}
