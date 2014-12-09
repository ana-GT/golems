/**
 * @file Test sending a velocity with Tobi's trajectory code
 * @brief Input: Start location, goal location. Output: Continuous velocity profile
 */
#include <sns.h>
#include <ach.h>
#include <unistd.h>
#include <basicControl/basicControl.h>
#include <list>
#include <basicControl/trajectories/Trajectory.h>

ach_channel_t chan_state;
ach_channel_t chan_ref;

struct timespec t_now;
struct timespec t_start;
struct timespec t_timeout;

double d_start; double d_now;
double d_duration = 2;

double tsec = 0.01;
double vel = 0.1;
const int NUM_JOINTS = 7;
BasicControl bc;

void zero();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  sns_init();
  sns_start();
  
  sns_chan_open( &chan_state, "state-right", NULL ); 
  sns_chan_open( &chan_ref, "ref-right", NULL );
  
  {
    ach_channel_t *chans[] = { &chan_state, &chan_ref, NULL};
    sns_sigcancel( chans, sns_sig_term_default );
  }

  // 0. Define needed 
  double q[NUM_JOINTS];
  double dq[NUM_JOINTS];
  Eigen::VectorXd start(NUM_JOINTS);
  Eigen::VectorXd goal(NUM_JOINTS);
  Eigen::VectorXd maxVel( NUM_JOINTS );
  Eigen::VectorXd maxAccel( NUM_JOINTS );
  Eigen::VectorXd vel_cmd;

  // 1. Get start pose
  if( clock_gettime(ACH_DEFAULT_CLOCK, &t_now ) ) { printf("No tnow obtained. Exiting \n"); return 0; }
  t_timeout = sns_time_add_ns( t_now, 1000*1000*1 );
  while( !bc.update_n( NUM_JOINTS, q, dq, &chan_state, &t_timeout ) ) {};
  for( int i = 0; i < NUM_JOINTS; ++i ) { 
    start(i) = q[i]; 
    goal(i) = q[i]; 
  }
  std::cout << "Start: "<< start.transpose() << std::endl;

  // 2. Define goal pose
  goal(5) = start(5) - 0.1;
  goal(6) = start(6) - 0.1;
  std::cout << "Goal: "<< goal.transpose() << std::endl;
  
  // 3. Create path
  std::list<Eigen::VectorXd> path;
  path.push_back( start );
  path.push_back( goal );
  
  // 4. Create trajectory
  maxVel = Eigen::VectorXd::Ones(NUM_JOINTS) * 0.1;
  maxAccel = Eigen::VectorXd::Ones(NUM_JOINTS) * 0.18;
  
  Trajectory trajectory( Path(path, 0.1), maxVel, maxAccel );
  trajectory.outputPhasePlaneTrajectory();

  if( trajectory.isValid() == false ) {
    printf("Trajectory is not valid \n");
    return 0;
  }

  // 4. Loop
  double d_start, d_goal, d_now;
  timespec t_start, t_goal, t_now;
  double d_duration;

  clock_gettime(ACH_DEFAULT_CLOCK, &t_now );
  t_timeout = sns_time_add_ns( t_now, 1000*1000*1 );
  
  while( !bc.update_n( NUM_JOINTS, q, dq, &chan_state, &t_timeout ) ) {}

  if( clock_gettime( ACH_DEFAULT_CLOCK, &t_start ) ) {
    printf("Clock gettime failed \n"); 
    return false;	
  } 	

  d_start = t_start.tv_sec + t_start.tv_nsec/1.0e9;
  d_now = d_start;

  
  // Send velocity commands
  d_duration = trajectory.getDuration();
  printf("Trajectory duration: %f \n", d_duration );

  while( d_now < d_start + d_duration ) {

    // Get current time and state
    if( clock_gettime(ACH_DEFAULT_CLOCK, &t_now ) ) { zero(); printf("Did not get time control \n"); return 0; }
    d_now = t_now.tv_sec + t_now.tv_nsec/1.0e9;

    t_timeout = sns_time_add_ns( t_now, 1000*1000*10 );
    if( !bc.update_n( NUM_JOINTS, q, dq, &chan_state, &t_timeout ) ) { zero(); printf("Did not get time update \n");return 0; }


    // Get velocity command
    vel_cmd = trajectory.getVelocity(d_now - d_start);
    //std::cout << "["<< (d_now - d_start)<<"] Vel: "<< vel_cmd.transpose() << std::endl;
    

    if( !bc.control_n( NUM_JOINTS, 
		       vel_cmd.data(), 
		       tsec, &chan_ref, 
		       SNS_MOTOR_MODE_VEL ) ) {
      printf("[followTrajectory] Sending velocity message did not go well \n");
      return 0;
    }
    

    // Sleep and clean up
    usleep ((useconds_t)(1e6*0.01) ); 
    aa_mem_region_local_release();
  } // end while

  printf("Finished and sending zero \n");
  zero();

   sns_end();
   return 0;
}

/**
 *
 */
void zero() {
  double q[NUM_JOINTS];
  for( int i = 0; i < NUM_JOINTS; ++i ) { q[i] = 0; }

  bc.control_n( NUM_JOINTS, 
		q, 
		tsec, &chan_ref, 
		SNS_MOTOR_MODE_VEL );
}

