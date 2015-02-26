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
#include <iostream>
#include <fstream>
#include <string>

// Variables
enum DIR {
  FORWARD = 5,
  BACKWARDS = 6,
  NONE = 7
};


std::vector<Eigen::VectorXd> waypoints;

ach_channel_t ee_pos_chan;
ach_channel_t left_arm_ref;
ach_channel_t left_arm_state;
double q[7]; double dq[7];
Eigen::Vector3d EEoffset;
double freq = 10;
double dt = 1.0 / freq;
double dq_thresh = 0.1;
int direction = NONE;
double clickTime = 5;
std::string path_filename;
bool readPath = false;

struct timespec now;
std::string path_leftarm("/home/cerdogan/Research/commonData/scenes/alita/alita_leftarm_calib.urdf");

dart::simulation::World* world = NULL;
dart::dynamics::Skeleton* leftArm = NULL;

int goTime = 3;
int startHere = 35;

/****************/
/** FUNCTIONS   */
/****************/
void setHardcode();
bool readHardcode( std::string filename );
bool initKin();
void go( int from, int to );
int update( void );
int update_n( size_t n,
	      double* _q, double *_dq,
	      ach_channel_t *chan,
	      struct timespec *ts );
void control_n( size_t n,
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
  while((c = getopt(argc, argv, "htm:d:r:" )) != -1 ) {
    switch(c) {
    case 'h': { printHelp(argv[0]); return 1; } break;
    case 't': { direction = FORWARD; setHardcode(); initKin(); test(); return 1; }
    case 'm': { 
      if(strcmp(optarg, "FORWARD") == 0 ) {
	direction = FORWARD;
      } else if( strcmp(optarg, "BACKWARDS") == 0 ) {
	direction = BACKWARDS;
      } else {
	printf("Send me either FORWARD or BACKWARDS with -m option. Exiting! \n");
	return 1;
      }
    } break;
    case 'd':{
      clickTime = atof(optarg);
      printf("Click time: %f \n", clickTime );
    } break;
    case 'r': {
      path_filename = std::string(optarg);
      readPath = true;
    } break;      
    } // end switch
  } // end while

  if( direction == NONE ) {
    printf("\t * You must choose a direction! (FORWARD / BACKWARDS) \n");
    return 1;
  }


  //--------------------
  // 1. Parse
  //--------------------
  if( readPath ) { 
    if( readHardcode( path_filename ) == false ) {
      return 1;
    } else {
      std::cout << "\t * Waypoints loaded: "<< waypoints.size() << std::endl;
    }   
  }
  else { 
    setHardcode(); 
  }

  if( !initKin() ) {
    printf("\t * [CRAP] Kinematics not initialized well \n");
    return -1;
  }

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

  // Start behavior
  size_t key_idx = 0;
  Eigen::VectorXd current(7);
  Eigen::VectorXd currentGoal(7);
  double dist;
  bool stillFollow = true;


  //---------------------------------------------------------------------
  // Before starting: MAKE SURE THE FIRST POINT ON THE TRAJECTORY
  // IS CLOSE TO THE CURRENT POSE OF THE ARM, OTHERWISE WE ARE SCREWED
  //---------------------------------------------------------------------
  while( update() == 0 ) {}

  std::cout << "\t * Checking everything is in order "<<std::endl;

  for(int i =0; i < 7; ++i ) { current(i) = q[i]; }
  if( (current - waypoints[0]).norm() > 0.08 ) {
    printf(" The first trajectory point is too far from the current state. ARE YOU SURE? \n");
    std::cout << "* Current point: "<< current.transpose() << std::endl;
    std::cout << "* Trajectory point: "<< waypoints[0].transpose() << std::endl;
    return 1;
  }
  // END SECURITY
  //------------------------------------------------------------------------
  std::cout << "\t * Start"<<std::endl;
  while(stillFollow) {
    // Get the current state
    while( update_n(7, q, dq, &left_arm_state, NULL ) == 0 ) {}
    updateEEPos();

    for(int i =0; i < 7; ++i ) { current(i) = q[i]; }
    
    // Check if the target point keypoint is reached, if so, give command 
    // for the next one
    currentGoal = waypoints[key_idx];
    dist = ( currentGoal - current ).norm();
    if( dist < dq_thresh ) {
      if( key_idx == waypoints.size() - 1 ) {
	stillFollow = false;
      }

      key_idx = std::min( waypoints.size() - 1, key_idx + 1 );
      
      //------------------------------
      // CLICK MOMENT
      if( key_idx > startHere ) {
  	    printf("\t * Reached point to point %d, you got f seconds to click! \n", key_idx -1, clickTime );
      	    int maxCounter = (int)( (double)clickTime / 0.01 );
      	    int counter = 0;
      	    while( counter < maxCounter ) {
	    	update();
		updateEEPos();
		usleep(0.01*1e6);
		counter++;
      	   }
     }   
      //------------------------------

      currentGoal = waypoints[key_idx];
    }
    
    // Tell the robot to go to the current goal
    control_n( 7, currentGoal.data(), dt, &left_arm_ref );
    
    // Sleep and clean up
    if( freq > 0 ) { usleep ((useconds_t)(1e6*dt) ); }
    aa_mem_region_local_release();

  } // end while
  

  printf("\t * [INFO] Killing me softly with SIG-TERM \n");
  sns_end();
  return 0;

}

/**
 * @function readHardcode
 */
bool readHardcode( std::string filename ) {

  std::string line;
  std::ifstream file( filename.c_str() );
  Eigen::VectorXd p(7);
  std::vector<Eigen::VectorXd> temp;

  if( file.is_open() ) {
  
    while( getline(file, line) ) {
      std::istringstream iss(line);
      iss >> p(0) >> p(1) >> p(2) >> p(3) >> p(4) >> p(5) >> p(6);
      temp.push_back(p);
    }

    // Get ready
    waypoints.resize(0);
    
    // If forward
    if( direction == FORWARD ) {
      waypoints = temp;
    } 
    // If backwards
    else if( direction == BACKWARDS ) {
      for(int i = temp.size()-1; i >= 0; --i ) {
	waypoints.push_back(temp[i] );
      }
    }
    
    return true;
    
  } else {
    return false;
  }

}

/**
 * @function setHardcode
 */
void setHardcode() {

  std::vector<Eigen::VectorXd> temp;
  temp.resize(0);
  Eigen::VectorXd p(7); 

  p << 1.57, -1.57, 0, 0, 0, 0, 0;
  temp.push_back(p);

  p << 0.8, -1.57, 0, 0, 0, 0, 0;
  temp.push_back(p);

  p << 0.8, -1.57, 0, -0.8, 0, 0, 0;
  temp.push_back(p);

  p << 0.8, -1.57, 0, -0.8, 0, 0, -0.8;
  temp.push_back(p);

  p << 0.8, -1.57, 0, -0.8, 0, -0.8, -0.8;
  temp.push_back(p);

  p << 0.4, -1.57, 0, -0.8, 0, -0.8, -0.8;
  temp.push_back(p);

  p << 0.4, -1.57, 0, -0.8, 0, -0.8, -1.57;
  temp.push_back(p);

  p << 0.0, -1.57, 0, -0.8, 0, -1.2, -1.57;
  temp.push_back(p);

  p << 0.0, -1.57, 0.8, -0.8, 0, -1.2, -1.57;
  temp.push_back(p);

  p << 0.0, -1.57, 0.8, -0.8, 0.8, -1.2, -1.57;
  temp.push_back(p);

  p << 0.0, -1.57, 0.8, -0.4, 0.8, -1.2, -1.57;
  temp.push_back(p);

  p << 0.0, -1.57, 0.6, -0.6, 0.8, -1.2, -1.57;
  temp.push_back(p);

  p << 0.0, -0.8, 0.6, -0.6, 0.8, -1.2, -1.57;
  temp.push_back(p);

  p << 0.0, -0.8, 0.0, -0.6, 0.8, -1.2, -1.57;
  temp.push_back(p);

  p << 0.0, -0.8, 0.0, -1.2, 0.4, -1.57, -1.57;
  temp.push_back(p);

  p << 0.0, -0.8, 0.0, -1.2, 0.4, -1.57, -2.0;
  temp.push_back(p);

  p << 0.4, -0.8, 0.0, -1.2, 0.4, -1.2, -2.0;
  temp.push_back(p);

  p << 0.8, -0.8, 0.0, -1.2, 0.6, -1.2, -1.3;
  temp.push_back(p);



  // Get ready
  waypoints.resize(0);

  // If forward
  if( direction == FORWARD ) {
    waypoints = temp;
  } 
  // If backwards
  else if( direction == BACKWARDS ) {
    for(int i = temp.size()-1; i >= 0; --i ) {
      waypoints.push_back(temp[i] );
    }
  }

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
  control_n( 7, waypoints[to].data(), goTime, &left_arm_ref );
}


/**
 * @function control_n
 */
void control_n( size_t n,
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
int update( void ) {
  
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
		       NULL );
  return u_al;
}

/**
 * @function update_n
 */
int update_n( size_t n,
	      double* _q, double *_dq,
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
	  _q[j] = msg->X[j].pos;
	  _dq[j] = msg->X[j].vel;
	} // end for

	return 1;
      } // end if
      else {
	SNS_LOG( LOG_ERR, "Invalid motor_state message \n" );
	return 0;
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
  printf("\t -d: Wait time in seconds \n");
  printf("\t -m: Mode (FORWARD OR BACKWARDS): \n");
  printf("\t -r: File with path to follow \n");

}
