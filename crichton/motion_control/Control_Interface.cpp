/**
 * @function Control_Interface.cpp
 */
#include "Control_Interface.h"

/**
 * @function Control_Interface
 * @brief Constructor
 */
Control_Interface::Control_Interface() {
  mVALID_NS = (int64_t)((1000000000)/5);
}

/**
 * @function Control_Interface
 * @brief Destructor
 */
Control_Interface::~Control_Interface() {
}

/**
 * @function set_numJoints
 * @brief Set number of drives in the message
 */
void Control_Interface::set_numJoints( int _N ) {
  mN = _N;
  mq.resize( mN );
  mdq.resize( mN );
}

/**
 * @function set_channels
 */
void Control_Interface::set_channels( ach_channel_t* _chan_state,
				      ach_channel_t* _chan_output ) {
  mChan_state = _chan_state;
  mChan_output = _chan_output;
}


/**
 * @function update 
 */
bool Control_Interface::update() {

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
void Control_Interface::get_state( Eigen::VectorXd &_q,
				   Eigen::VectorXd &_dq ) {

  while( !this->update() ) {}
  _q = mq;
  _dq = mdq;
}


////////////////////////////////////////////////////

/**
 * @function update_n
 * @brief
 */
bool Control_Interface::update_n( size_t n,
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
      SNS_LOG( LOG_ERR, "[update_n] Invalid motor_state message \n" );
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
bool Control_Interface::control_n( size_t n,
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
  
  // Set time duration
  if( clock_gettime( ACH_DEFAULT_CLOCK, &mNow ) ) {
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );
  }

  sns_msg_set_time( &msg->header, &mNow, mVALID_NS ); // mVALID_NS value taken from piranha/src/pirctrl.c

  // Send
  ach_status_t r;
  r = ach_put( chan, msg, sns_msg_motor_ref_size(msg) );

  return (r == ACH_OK);  
}
