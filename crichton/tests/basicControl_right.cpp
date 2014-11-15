/**
 * @file basicControl_test2.cpp
 */


#include "basicControl/unimanualControl.h"


int main( int argc, char* argv[] ) {
  sns_init();
  sns_start();

  ach_channel_t* state_chan;
  ach_channel_t* ref_chan;
  ach_channel_t* hand_state_chan;
  ach_channel_t* hand_ref_chan;

    state_chan = new ach_channel_t();
    ref_chan = new ach_channel_t();
    hand_state_chan = new ach_channel_t();
    hand_ref_chan = new ach_channel_t();
 

  // Open channels
  sns_chan_open( state_chan, "state-right", NULL );
  sns_chan_open( ref_chan, "ref-right", NULL );
  sns_chan_open( hand_state_chan, "sdhstate-right", NULL );
  sns_chan_open( hand_ref_chan, "sdhref-right", NULL );

  UnimanualControl uc;
  uc.setChannels( state_chan, ref_chan,
		  hand_state_chan, hand_ref_chan );
   
  while( uc.update() == false ) { }
  // Get states
  Eigen::VectorXd q_a;
  Eigen::VectorXd dq_a;
  Eigen::VectorXd q_h;
  Eigen::VectorXd dq_h;
  uc.getStates( q_a, dq_a, q_h, dq_h );
  
  // Get arm state
  std::cout<< "* State of right arm: "<< q_a.transpose() << std::endl;


  // Move it a tiny bit
  std::vector<Eigen::VectorXd> path, densePath;

  // Fill normal path
  Eigen::VectorXd q0, q1;
  q0 = q_a;
  q1 = q0; 
  q1(0) = q1(0) + 0.1;

  path.push_back( q0 );
  path.push_back( q1 );

  densePath = uc.preprocessPath( path );

  for( int i = 0; i < densePath.size(); ++i ) {
    std::cout << "["<<i<<"]: "<< densePath[i].transpose() << std::endl;
  }

  
  uc.followArmPath( densePath );
  

  return 0;

}
