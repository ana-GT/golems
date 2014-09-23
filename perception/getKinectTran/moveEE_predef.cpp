/**
 * @file moveEE_predef.cpp
 * @brief Move the left arm of Alita through a set of predefined points and send the position of
 * @brief a marked point on top of the EE to do the Kinect extrinsic calibration
 */
#include <unistd.h>
#include <stdio.h>
#include <Eigen/Core>
#include <amino.h>
#include <sns.h>
#include <ach.h>
#include <poll.h>

#include <dart/simulation/World.h>
#include <dart/dynamics/BodyNode.h>
#include <dart/dynamics/Skeleton.h>
#include <dart/utils/urdf/DartLoader.h>

// Variables
std::vector<Eigen::VectorXd> waypoints;

ach_channel_t ee_pos_chan;
ach_channel_t left_arm_ref;
ach_channel_t left_arm_state;
double q[7]; double dq[7];
Eigen::Vector3d EEoffset;

struct timespec now;
std::string path_leftarm("/home/cerdogan/Research/commonData/scenes/alita/alita_leftarm_calib.urdf");

dart::simulation::World* world = NULL;
dart::dynamics::Skeleton* leftArm = NULL;

/****************/
/** FUNCTIONS   */
/****************/
void setHardcode();
bool initKin();
void go( int from, int to );
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
void updateEEPos();
void test();


/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  //--------------------
  // 0. Logistics
  //--------------------
  int c;
  while((c = getopt(argc, argv, "ht" )) != -1 ) {
    switch(c) {
    case 'h': { printHelp(argv[0]); return 1; } break;
    case 't': {setHardcode(); initKin(); test(); return 1; }
    } // end switch
  } // end while

  //--------------------
  // 1. Parse
  //--------------------
  setHardcode();
  if( !initKin() ) {
    printf("\t * [CRAP] Kinematics not initialized well \n");
    return -1;
  }

  struct pollfd stdin_poll;
  stdin_poll.fd = STDIN_FILENO;
  stdin_poll.events = POLLIN;

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
  sns_chan_open( &left_arm_state, "left-arm-state", NULL );
  sns_chan_open( &left_arm_ref, "left-arm-ref", NULL );
  sns_chan_open( &ee_pos_chan, "ee-pos", NULL );

  //-----------------------------------------------------------------------
  // 5. Set sighandlers to die gracefully due to SIGTERM and other animals
  //------------------------------------------------------------------------
  {
    ach_channel_t *chans[] = { &left_arm_state,
			       &left_arm_ref,
			       &ee_pos_chan,
			       NULL};
    sns_sigcancel( chans, sns_sig_term_default );
  }
  
  //-----------------------
  // 6. Loop
  //-----------------------
  char cmd[5];
  char line[40];
  int r;

  while( !sns_cx.shutdown ) {
    
    // Update, read pos and vel of hands
    update();
    // Send pos of marker in EE
    updateEEPos();
    // Check if user put input
    if( poll(&stdin_poll, 1, 0 ) == 1 ) {
      
      gets(line);
      r = sscanf( line, "%s", cmd );
      if( r == 1 ) {
	if( strcmp(cmd, "0->1") == 0 ) { go(0,1); } 
	else if( strcmp(cmd, "1->2") == 0 ) { go(1,2); }
	else if( strcmp(cmd,"2->3") == 0 ) { go(2,3); }
	else if( strcmp(cmd, "3->4") == 0 ) { go(3,4); }
	else if( strcmp(cmd,"4->5") == 0 ) { go(4,5); }

	else if( strcmp(cmd, "5->4") == 0 ) { go(5,4); }
	else if( strcmp(cmd,"4->3") == 0 ) { go(4,3); }
	else if( strcmp(cmd, "3->2") == 0 ) { go(3,2); }
	else if( strcmp(cmd,"2->1") == 0 ) { go(2,1); }
	else if( strcmp(cmd,"1->0") == 0 ) { go(1,0); }
	
      } // end if r == 1
      
      
    } // end if poll
    
    aa_mem_region_local_release();
    usleep(0.015*1e6);	

  } // end while

  printf("\t * [INFO] Killing me softly with SIG-TERM \n");
  sns_end();
  return 0;


}

/**
 * @function setHardcode
 */
void setHardcode() {

  waypoints.resize(0);
  Eigen::VectorXd p(7); 

  p << M_PI*0.5, -M_PI*0.5, 0, 0, 0, 0, 0;
  waypoints.push_back(p);

  p << M_PI*0.25, -M_PI*0.5, 0, 0, 0, M_PI*-0.25, 0;
  waypoints.push_back(p);

  p << M_PI*0.25, -M_PI*0.5, 0, -M_PI*0.25, 0, M_PI*-0.25, 0;
  waypoints.push_back(p);

  p << M_PI*0.166, -M_PI*0.5, 0, -M_PI*0.25, -M_PI*0.25, M_PI*-0.25, 0;
  waypoints.push_back(p);

  p << M_PI*0.166, -M_PI*0.5, 0, -M_PI*0.5, -M_PI*0.66, M_PI*-0.25, 0;
  waypoints.push_back(p);

  p << M_PI*0.166, -M_PI*0.33, 0, -M_PI*0.5, 0, M_PI*-0.25, 0;
  waypoints.push_back(p);
}


/**
 * @function initKin
 */
bool initKin() {

  // Load world
  dart::utils::DartLoader dl;
  world = dl.parseWorld( path_leftarm );

  if(!world ) { return false; }

  leftArm =  world->getSkeleton("alitaLeftArm"); 
  if( leftArm == NULL ) { return false; }

  // Set EEoffset
  EEoffset << -0.055, 0, 0.07;

  printf("\t * [OK] Good. Alita left arm has been loaded \n");  
  return true;

}


/**
 * @function test
 */
void test() {
  std::cout << "Test Kinematics "<<std::endl;
  std::vector<int> armDofs(7);
  for( int i = 0; i < 7; ++i ) { armDofs[i] = i + 6; }

  for( int i = 0; i < waypoints.size(); ++i ) {
    leftArm->setConfig( armDofs, waypoints[i] );

  Eigen::Isometry3d T7;
  T7 = leftArm->getBodyNode("L7")->getWorldTransform();

  // Offset of EE marker
  Eigen::Vector3d EEp;
  EEp = T7.linear() * EEoffset + T7.translation();
  
  std::cout << "Supposed EE point ["<<i<<"] :"<< EEp.transpose() << std::endl;
  } // end for	

}


/**
 * @function updateEEPos
 */
void updateEEPos() {

  // Set arm configuration
  Eigen::VectorXd current(7);
  std::vector<int> armDofs(7);
  for( int i = 0; i < 7; ++i ) {
    current(i) = q[i];
    armDofs[i] = i + 6;
  }

  leftArm->setConfig( armDofs, current );


  Eigen::Isometry3d T7;
  T7 = leftArm->getBodyNode("L7")->getWorldTransform();

  // Offset of EE marker
  Eigen::Vector3d EEp;
  EEp = T7.linear() * EEoffset + T7.translation();

  // Send through channel
  double msg[3];
  for( int i = 0; i < 3; ++i ) { msg[i] = EEp(i); }
  ach_status r;
  r = ach_put( &ee_pos_chan, msg, sizeof(msg) );
  if( r != ACH_OK ) {
    printf("\t * [BAD] Something bad happened while sending EE Pos \n");
  }
}





/**
 * @function go
 */
void go( int from, int to ) {

  // Check both waypoints exist
  if( from < 0 || from >= waypoints.size() ) { printf("From is out of range \n"); return; }
  if( to < 0 || to >= waypoints.size() ) { printf("To is out of range \n"); return; }

  // Check start waypoint is current position (or close enough)
  Eigen::VectorXd current(7);
  for( int i = 0; i < 7; ++i ) { current(i) = q[i]; }

  if( (current-waypoints[from]).norm() > 0.08 ) {
    printf("From is a bit far from current joint conf. Are you sure? \n");
    return;
  }

  // Go
  control_n( 7, waypoints[to].data(), 3, &left_arm_ref );
}


/**
 * @function control_n
 */
static void control_n( size_t n,
		       double *x,
		       double tsec,
		       ach_channel_t* chan ) {
 
  struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc( n );
  sns_msg_header_fill( &msg->header );
  msg->mode = SNS_MOTOR_MODE_POS;
  msg->header.n = n;
  AA_MEM_CPY( msg->u, x, n );
  
  // Duration from now + tnano
  double tnano = tsec*1e9;
  if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ){
    SNS_LOG( LOG_ERR, "clock_gettime failed: %s \n", strerror(errno) );
  }
  sns_msg_set_time( &msg->header, &now, tnano );
  // Send
  ach_put( chan, msg, sns_msg_motor_ref_size(msg) );
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
  
  // Get arm/gripper States
  int u_al = update_n( 7, q, 
		      dq,
		      &left_arm_state,
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
  printf("\t -t: Test kinematics \n");
  printf("\t * Once the program started running: \n");
  printf("\t COMMAND [ENTER] : \n" );
  printf("\t \t - COMMAND={0->1, 1->2...4->5}\n");
  printf("\t \t - COMMAND={5->4, 4->3...1->0}\n");
}
