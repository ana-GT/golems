/**
 * @file basicControl_test2.cpp
 */


#include "basicControl/bimanualControl.h"


int main( int argc, char* argv[] ) {

  sns_init();
  sns_start();

  ach_channel_t* state_chan[2];
  ach_channel_t* ref_chan[2];
  ach_channel_t* gripper_state_chan[2];
  ach_channel_t* gripper_ref_chan[2];

  for( int i = 0; i < 2; ++i ) {
    state_chan[i] = new ach_channel_t();
    ref_chan[i] = new ach_channel_t();
    gripper_state_chan[i] = new ach_channel_t();
    gripper_ref_chan[i] = new ach_channel_t();
  } 

  // Open channels
  sns_chan_open( state_chan[LEFT_BIM], "state-left", NULL );
  sns_chan_open( ref_chan[LEFT_BIM], "ref-left", NULL );


  BimanualControl bc;
  bc.setChannels( state_chan, ref_chan,
		  gripper_state_chan, gripper_ref_chan );

  while( bc.update() == false ) {}

  // Get states
  Eigen::VectorXd q_a[2];
  Eigen::VectorXd dq_a[2];
  Eigen::VectorXd q_h[2];
  Eigen::VectorXd dq_h[2];

  bc.getStates( q_a, dq_a, q_h, dq_h );
  
  // Get left arm
  std::cout<< "* State of left arm: "<< q_a[LEFT_BIM].transpose() << std::endl;


  // Move it a tiny bit
  std::vector<Eigen::VectorXd> path, densePath;

  // Fill normal path
  Eigen::VectorXd q0, q1;
  q0 = q_a[LEFT_BIM];
  q1 = q0; 
  q1(5) = q1(5) + 0.628;
  q1(6) = q1(6) - 0.949;

  path.push_back( q0 );
  path.push_back( q1 );

  densePath = bc.preprocessPath( path );

  for( int i = 0; i < densePath.size(); ++i ) {
    std::cout << "["<<i<<"]: "<< densePath[i].transpose() << std::endl;
  }

  
  bc.followArmPath( LEFT_BIM,
		    densePath );
  

  return 0;

}
