
#include <sns.h>
#include <unistd.h>
#include <ach.h>
#include "motion_control/base_control.h"

#include <list>

ach_channel_t chan_state;
ach_channel_t chan_ref;

int main( int argc, char* argv[] ) {

  sns_init();
  sns_start();

  sns_chan_open( &chan_state, "state-left", NULL );
  sns_chan_open( &chan_ref, "ref-left", NULL );

  {
    ach_channel_t *chans[] = { &chan_state, 
			       &chan_ref,
			       NULL};
    sns_sigcancel( chans, sns_sig_term_default );
  }

  BaseControl mBs;

  int n = 2;
  mBs.set_numJoints( n );
  mBs.set_channels( &chan_ref, &chan_state );

  std::list<Eigen::VectorXd> path;

  // Set start and goal point
  Eigen::VectorXd qo(n), dqo(n);
  Eigen::VectorXd qf(n);
  mBs.get_state( qo, dqo );

	qf(0) = qo(0) + 0.2;
  qf(1) = qo(1) + 0.2;

  std::cout << "\t * Start: "<< qo.transpose() << std::endl;
  std::cout << "\t * Goal: "<< qf.transpose() << std::endl;
  
  path.push_back( qo );
  path.push_back( qf );
  
  Eigen::VectorXd maxAccel(n); 
  Eigen::VectorXd maxVel(n);
  for( int i = 0; i < n; ++i ) { maxAccel(i) = 0.1; maxVel(i) = 0.18; }
  
  mBs.followTrajectory( path, maxAccel, maxVel );
  
  
}
