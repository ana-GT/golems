/**
 * @file base_control.cpp
 */
#include "base_control.h"

#include "trajectories/Trajectory.h"
#include "trajectories/Path.h"

#include <fstream>

/**
 * @function BaseControl
 */
BaseControl::BaseControl() {
  mFreq = 100;
  mDt = 1.0 / mFreq;
  mDq_thresh = 0.184; // sqrt(7)*4degrees

  mMaxDev = 0.1;
	mVALID_NS = (int64_t)((1000000000) / 5);
  mN = 1;
}

/**
 * @function ~BaseControl
 */
BaseControl::~BaseControl() {}

/**
 * @function set_numJoints
 */
void BaseControl::set_numJoints( int _N ) {
  mN = _N;
  mq.resize( mN );
  mdq.resize( mN );
}

/**
 * @function set_channels
 */
void BaseControl::set_channels( ach_channel_t* _ref_chan,
				ach_channel_t* _state_chan ) {

  mChan_ref = _ref_chan;
  mChan_state = _state_chan;
}

/**
 * @function update 
 */
bool BaseControl::update() {

  if( clock_gettime( ACH_DEFAULT_CLOCK, &mNow ) ) {
    SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
    return false;
  }

  struct timespec timeout = sns_time_add_ns( mNow, 1000*1000*1 );
 return update_n( mN,
		   mq, mdq, mChan_state, &timeout );
}

/**
 * @function get_state
 */
void BaseControl::get_state( Eigen::VectorXd &_q,
			     Eigen::VectorXd &_dq ) {

  while( !this->update() ) {}
  _q = mq;
  _dq = mdq;
}

/**
 * @function followTrajectory
 */
bool BaseControl::followTrajectory( const std::list<Eigen::VectorXd> &_path,
				    const Eigen::VectorXd &_maxAccel,
				    const Eigen::VectorXd &_maxVel ) {

  std::list<Eigen::VectorXd> path;
  path = _path;

  // Safety check
  while( !update() ) {}
  if( ( *(path.begin()) - mq).norm() > mDq_thresh ) {
    printf("\t [followTrajectory] First point of trajectory is too far from init path point \n");
    std::cout <<"Path first element: "<< (*(path.begin())).transpose() << std::endl;
    std::cout << "Current joint configuration: "<< mq.transpose() << std::endl;
    std::cout << "Norm: "<< (*(path.begin()) - mq).norm() <<" limit in code: "<< mDq_thresh<< std::endl;
    return false;
  }
  

  // Create trajectory
  Trajectory trajectory( Path(path, mMaxDev), _maxVel, _maxAccel );
  trajectory.outputPhasePlaneTrajectory();
  if( !trajectory.isValid() ) {
    printf("\t [followTrajectory] ERROR: Trajectory is not valid \n");
    return false;
  }
  double duration = trajectory.getDuration();
  double start_time;
  double current_time;
  Eigen::VectorXd vel_cmd;

  printf("\t [followTrajectory] DEBUG: Duration of trajectory: %f \n", duration );

  // Update current state and time
  double tn;
  Eigen::VectorXd vel_t, pos_t;
  double Kp = 0.005;  

  while( !update() ) {}

  start_time = mNow.tv_sec + (mNow.tv_nsec)/1.0e9;
  current_time = start_time;
  tn = current_time - start_time;

  // Send velocity commands
  char name[50];
  std::time_t t = std::time(NULL);
  std::strftime( name, 50, "output_%A_%H_%M_%S.txt", std::localtime(&t) );
  std::ofstream output( name, std::ofstream::out );
  Eigen::VectorXd vp = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd acc;

  double tn_old = tn;
  while( tn < duration ) {

    // Get current time and state
    while( !update() ) {}
    current_time = mNow.tv_sec + mNow.tv_nsec / (1.0e9);
    tn = current_time - start_time;   

    // Build velocity command
    
    vel_t = trajectory.getVelocity( tn );
    pos_t = trajectory.getPosition( tn );
    vel_cmd = vel_t + Kp*( pos_t - mq );

    for( int j = 0; j < vel_cmd.size(); ++j ) { if( fabs(vel_cmd(j)) > _maxVel(j) ) { printf("VELOCITY OUT OF LIMITS, STOPPING!!! \n"); sendZeroVel(); break; } }

    acc = (vel_cmd - vp)/0.01;
    vp = vel_cmd;

     if( tn - tn_old > 0.5 ) {
     tn_old = tn;
     std::cout << "["<< tn << "]: "<< vel_cmd.transpose() << std::endl;
     Eigen::VectorXd ep; ep = trajectory.getPosition(tn).transpose();
     Eigen::VectorXd cp; cp = mq.transpose();
     Eigen::VectorXd errp; errp = (ep - cp);
     std::cout << "Error position: "<< errp.norm() << ": "<< errp.transpose() << std::endl;
   }
    output << tn << " " << trajectory.getPosition(tn).transpose() << " " << mq.transpose() << " " <<
	vel_cmd.transpose() << " " << mdq.transpose() << " " << acc.transpose()  << std::endl; 

 
    // Send command to robot    
    if( !control_n( mN, vel_cmd, 2*mDt, mChan_ref, SNS_MOTOR_MODE_VEL ) ) {
      printf("\t[followTrajectory] Sending vel msg error \n");
      return false;
      }
    
    // Sleep and clean up
    usleep( (useconds_t)(1e6*mDt)) ;
    aa_mem_region_local_release();
    
  } // end while
  printf("Closing file with data of trajectory pos and vel \n");
  output.close();

  printf("\t * [trajectoryFollowing] DEBUG: Finish and sending zero vel for good measure \n");
  sendZeroVel();

  return true;  
}

/**
 * @function sendZeroVel
 * @brief Send zero velocities for emergency stops
 */
void BaseControl::sendZeroVel() {

  Eigen::VectorXd zeros(mN); for( int i = 0; i < mN; ++i ) { zeros(i) = 0; }
  control_n( mN, zeros, mDt, mChan_ref, SNS_MOTOR_MODE_VEL );

}

////////////////////////////////////////////////////

/**
 * @function update_n
 */
bool BaseControl::update_n( size_t n,
			    Eigen::VectorXd &_q,
			    Eigen::VectorXd &_dq,
			    ach_channel_t* chan,
			    struct timespec *ts ) {
  
  size_t frame_size;
  void* buf = NULL;
  ach_status_t r = sns_msg_local_get( chan, &buf,
				      &frame_size,
				      ts,
				      ACH_O_LAST | (ts ? ACH_O_WAIT : 0) );

  switch(r) {
  case ACH_OK:
  case ACH_MISSED_FRAME: {
    struct sns_msg_motor_state *msg = (struct sns_msg_motor_state*) buf;
    if( n == msg->header.n &&
	frame_size == sns_msg_motor_state_size_n((uint32_t)n) ) {
      for( size_t j = 0; j < n; ++j ) {
	_q(j) = msg->X[j].pos;
	_dq(j) = msg->X[j].vel;
      }
      return true;
    } else {
      SNS_LOG( LOG_ERR, "[update_n] Invalid motor_state message: frame size: %d sns motor size: %d, n: %d, header n: %d  \n", frame_size, sns_msg_motor_state_size_n(n), n, msg->header.n );
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

  return false;
}


/**
 * @function control_n
 */
bool BaseControl::control_n( size_t n,
			     const Eigen::VectorXd &_x,
			     double tsec,
			     ach_channel_t* chan,
			     int mode ) {

  // Safety check
  if( _x.size() != n ) {
    printf("\t [control_n] Size of control vector is diff than expected \n");
    return false;
  }
  

  // Create the message
  struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc( n );
  sns_msg_header_fill( &msg->header );

  switch( mode ) {
  case SNS_MOTOR_MODE_POS:
    msg->mode = SNS_MOTOR_MODE_POS;
    break;
  case SNS_MOTOR_MODE_VEL:
    msg->mode = SNS_MOTOR_MODE_VEL;
    break;
  default:
    return false;
  }

  msg->header.n = n;
  AA_MEM_CPY( msg->u, _x.data(), n );
  
  // Duration from now + tnano
  int64_t dur_nsec = (int64_t) ( tsec*1e9 );
  if( clock_gettime( ACH_DEFAULT_CLOCK, &mNow ) ) {
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );
  }

  sns_msg_set_time( &msg->header, &mNow, mVALID_NS ); // mVALID_NS value taken from piranha/src/pirctrl.c

  // Send
  ach_status_t r;
  r = ach_put( chan, msg, sns_msg_motor_ref_size(msg) );

  return (r == ACH_OK);  
}


/**
 * @function reset_n
 */
bool BaseControl::reset_n( size_t n,
			   double tsec,
			   ach_channel_t* chan ) {

  // Create the message
  struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc( n );
  sns_msg_header_fill( &msg->header );
  msg->mode = SNS_MOTOR_MODE_RESET;
  msg->header.n = n;

  // Duration from now to tnano
  int64_t dur_nsec = (int64_t)(tsec*1e9);
  if( clock_gettime( ACH_DEFAULT_CLOCK, &mNow ) ) {
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );
  }
  sns_msg_set_time( &msg->header, &mNow, dur_nsec );

  // Send
  ach_status_t r;
  r = ach_put( chan, msg, sns_msg_motor_ref_size(msg) );

  return (r == ACH_OK); 
} 
