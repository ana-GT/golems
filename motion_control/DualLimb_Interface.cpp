/**
 * @file DualLimb_Interface.cpp
 */
#include "DualLimb_Interface.h"

/**
 * @function DualLimb_Interface
 * @brief Constructor
 */
DualLimb_Interface::DualLimb_Interface() {

}

/**
 * @function ~DualLimb_Interface
 * @brief Destructor
 */
DualLimb_Interface::~DualLimb_Interface() {
  
}

/**
 * @function 
 * @brief
 */
void DualLimb_Interface::set_numJoints( int _num_arm_joints,
					int _num_hand_joints ) {
  for( int i = 0; i < 2; ++i ) {
    mLi[i].set_numJoints( _num_arm_joints, _num_hand_joints );
  }
}

/**
 * @function set_hand_channels
 * @brief Set states input and dual hand output
 */
void DualLimb_Interface::set_hand_channels( ach_channel_t* _hand_left_state_chan,
					    ach_channel_t* _hand_right_state_chan,
					    ach_channel_t* _hand_output_chan ) {

  mChan_bimanualHand = _hand_output_chan;
  // arm output chan just to fill the argument. We send messages not using the Limb interface
  mLi[0].set_hand_channels( _hand_left_state_chan,
			   _hand_output_chan ); // This output is rather wrong. Should not be used
  mLi[1].set_hand_channels( _hand_right_state_chan,
			   _hand_output_chan ); // This output is rather wrong. Should not be used
}

/**
 * @function set_arm_channels
 * @brief Set arm[i] state and output channels
 */
void DualLimb_Interface::set_arm_channels( ach_channel_t* _arm_left_state_chan,
					   ach_channel_t* _arm_right_state_chan,
					   ach_channel_t* _arm_output_chan ) {

  mChan_bimanualArm = _arm_output_chan;
  
  // arm output chan just to fill the argument. We send messages not using the Limb interface
  mLi[0].set_arm_channels( _arm_left_state_chan,
			   _arm_output_chan ); // This output is rather wrong. Should not be used
  mLi[1].set_arm_channels( _arm_right_state_chan,
			   _arm_output_chan ); // This output is rather wrong. Should not be used
  
}

/**
 * @function update
 * @brief
 */
bool DualLimb_Interface::update() {
  return update(0) && update(1);
}

/**
 * @function 
 * @brief
 */
bool DualLimb_Interface::update( int _i ) {
  return mLi[_i].update();
}

/**
 * @function update_arm
 * @brief
 */
bool DualLimb_Interface::update_arm( int _i ) {
  return mLi[_i].update_arm();
}

/**
 * @function update_hand
 * @brief
 */
bool DualLimb_Interface::update_hand( int _i ) {
  return mLi[_i].update_hand();
}

/**
 * @function get_arm_state
 * @brief
 */
void DualLimb_Interface::get_arm_state( int _i,
					Eigen::VectorXd &_q,
					Eigen::VectorXd &_dq ) {
  mLi[_i].get_arm_state( _q, _dq );
}

/**
 * @function get_hand_state
 * @brief
 */
void DualLimb_Interface::get_hand_state( int _i,
					 Eigen::VectorXd &_q,
					 Eigen::VectorXd &_dq ) {
  mLi[_i].get_hand_state( _q, _dq );
}


/**
 * @function 
 * @brief
 */
bool DualLimb_Interface::follow_arm_trajectory( int _i,
						const std::list<Eigen::VectorXd> &_path ) {

  // Build a message  
  struct sns_msg_bimanual* msg;

  if( _i == 0 ) {
    msg = sns_msg_bimanual_alloc( _path.size(), 0,
				  (*(_path.begin())).size() );

    msg->n_dof = (*(_path.begin())).size();
    msg->n_steps_left = _path.size();
    msg->n_steps_right = 0; // alloc
    msg->mode = 0;
printf("N dof: %d, steps left: %d steps right: %d \n", msg->n_dof, msg->n_steps_left, msg->n_steps_right );
  
    sns_msg_header_fill( &msg->header );
    msg->header.n = msg->n_dof;
    
    // Fill data in message
    int counter = 0;
    std::list<Eigen::VectorXd>::const_iterator it;
    for( it = _path.begin(); it != _path.end(); ++it ) {
      for( int j = 0; j < (*it).size(); ++j ) {
	msg->x[counter] = (*it)(j);
	counter++;
      }
    }
  } else if( _i == 1 ) {
    msg = sns_msg_bimanual_alloc( 0, _path.size(),
				  (*(_path.begin())).size() );
    
    msg->n_dof = (*(_path.begin())).size();
    msg->n_steps_left = 0; // alloc
    msg->n_steps_right = _path.size();
    msg->mode = 1;
    
    sns_msg_header_fill( &msg->header );
    msg->header.n = msg->n_dof;
    
    // Fill data in message
    int counter = 0;
    std::list<Eigen::VectorXd>::const_iterator it;
    for( it = _path.begin(); it != _path.end(); ++it ) {
      for( int j = 0; j < (*it).size(); ++j ) {
	msg->x[counter] = (*it)(j);
	counter++;
      }
    }

  } else {
    printf("[ERROR] Neither left or right! \n");
    return false;
  }

  // Set message duration
  double tsec = 0.05;
  int64_t dt_nsec = tsec*1e9;
  
  if( clock_gettime( ACH_DEFAULT_CLOCK, &mNow ) ) {
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );   
  }
  sns_msg_set_time( &msg->header, &mNow, dt_nsec );

  // Send message
  ach_status_t r;
  r = ach_put( mChan_bimanualArm, msg, sns_msg_bimanual_size(msg) );
  
  if( r!= ACH_OK ) { printf("\t * [ERROR] Error sending arm path \n"); }
  else { printf("\t * [INFO] Arm path was sent all right\n"); }
  
  return true;  
  
}

/**
 * @function follow_dual_arm_trajectory
 * @brief
 */
bool DualLimb_Interface::follow_dual_arm_trajectory( const std::list<Eigen::VectorXd> &_leftPath,
						     const std::list<Eigen::VectorXd> &_rightPath ) {

  // Build a message  
  struct sns_msg_bimanual* msg;
  
  msg = sns_msg_bimanual_alloc( _leftPath.size(),
				_rightPath.size(),
				(*(_leftPath.begin())).size() );
  
  msg->n_dof = (*(_leftPath.begin())).size();
  msg->n_steps_left = _leftPath.size();
  msg->n_steps_right = _rightPath.size();
  msg->mode = 2;
  
  sns_msg_header_fill( &msg->header );
  msg->header.n = msg->n_dof;
  
  // Fill data in message
  int counter = 0;
  std::list<Eigen::VectorXd>::const_iterator it;
  for( it = _leftPath.begin(); it != _leftPath.end(); ++it ) {
    for( int j = 0; j < (*it).size(); ++j ) {
      msg->x[counter] = (*it)(j);
      counter++;
    }
  }
  
  counter = msg->n_steps_left * msg->n_dof;
  for( it = _rightPath.begin(); it != _rightPath.end(); ++it ) {
    for( int j = 0; j < (*it).size(); ++j ) {
      msg->x[counter] = (*it)(j);
      counter++;
    }
  }
  
  
  // Set message duration
  double tsec = 0.05;
  int64_t dt_nsec = tsec*1e9;
  
  if( clock_gettime( ACH_DEFAULT_CLOCK, &mNow ) ) {
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );   
  }
  sns_msg_set_time( &msg->header, &mNow, dt_nsec );
  
  // Send message
  ach_status_t r;
  r = ach_put( mChan_bimanualArm, msg, sns_msg_bimanual_size(msg) );
  
  if( r!= ACH_OK ) { printf("\t * [ERROR] Error sending arm path \n"); }
  else { printf("\t * [INFO] Arm path was sent all right\n"); }
  
  return true;  
  
}

/**
 * @function go_hand_configuration
 */
bool DualLimb_Interface::go_hand_configuration( int _i,
						const Eigen::VectorXd &_config,
						double _dt ) {

  // Build a message  
  struct sns_msg_bimanual* msg;
  
  if( _i == 0 ) {
    msg = sns_msg_bimanual_alloc( 1, 0,
				  _config.size() );
    
    msg->n_dof = _config.size();
    msg->n_steps_left = 1;
    msg->n_steps_right = 0;
    msg->mode = 0;
    
    sns_msg_header_fill( &msg->header );
    msg->header.n = msg->n_dof;
    
    // Fill data in message
    for( int j = 0; j < _config.size(); ++j ) {
      msg->x[j] = _config(j);
    }
  } else if( _i == 1 ) {
    msg = sns_msg_bimanual_alloc( 0, 1,
				  _config.size() );
    
    msg->n_dof = _config.size();
    msg->n_steps_left = 0;
    msg->n_steps_right = 1;
    msg->mode = 1;
    
    sns_msg_header_fill( &msg->header );
    msg->header.n = msg->n_dof;
    
    // Fill data in message
    for( int j = 0; j < _config.size(); ++j ) {
      msg->x[j] = _config(j);
    }
  } else {
    printf("[ERROR] Neither left or right! \n");
    return false;
  }
  
  // Set message duration
  double tsec = 0.05;
  int64_t dt_nsec = tsec*1e9;
  
  if( clock_gettime( ACH_DEFAULT_CLOCK, &mNow ) ) {
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );   
  }
  sns_msg_set_time( &msg->header, &mNow, dt_nsec );
  
  // Send message
  ach_status_t r;
  r = ach_put( mChan_bimanualHand, msg, sns_msg_bimanual_size(msg) );
  
  if( r!= ACH_OK ) { printf("\t * [ERROR] Error sending hand config \n"); }
  else { printf("\t * [INFO] Hand config was sent all right\n"); }
  
  return true;  

  
}

/**
 * @function go_dual_hand_configuration
 */
bool DualLimb_Interface::go_dual_hand_configuration( const std::list<Eigen::VectorXd> &_leftPath,
						     const std::list<Eigen::VectorXd> &_rightPath ) {


}
