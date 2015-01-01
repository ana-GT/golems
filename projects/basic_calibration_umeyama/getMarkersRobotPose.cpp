
#include <unistd.h>
#include <sns.h>
#include <ach.h>

#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/BodyNode.h>

#include <aoi/utils/robotData.h>
#include <aoi/utils/globalStructs.h>
#include <dart/utils/urdf/DartLoader.h>
#include <motion_control/DualLimb_Interface.h>


std::vector<Eigen::Isometry3d> Tjm; // Tf from joint to marker
std::vector<std::string> jointNames;

subject_t mS;
DualLimb_Interface mDli;
ach_channel_t mBiTraj_chan;
dart::simulation::World* mWorld;

void loadDefault();
void setupComm();
void updateStates();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Always set this first
  setHardCodedStructures();

  std::string path_world("/home/ana/Research/commonData/scenes/benchmark_tests/test0_followBall.urdf");
  
  // Load known knowledge
  loadDefault();
  
  // Load world
  dart::utils::DartLoader dl;
  mWorld = dl.parseWorld( path_world );
  
  if( !mWorld ) {
    printf("\t [ERROR] Was not able to load world \n");
    return 1;
  }
  
  setSubject( mS, mWorld, LWA4_ROBOT );

  // Setup comm after setting subject
  setupComm();


  
  // Update poses
  Eigen::Isometry3d Taw, Twa, Twj, Twm, Tam;
  Eigen::Vector3d pam;
    
  while( true ) {

    // Get state of arm and hand joints
    updateStates();
    
    // Update the subject
    mS.update();

    // Get the poses of the finger markers
    for( int i = 0; i < 2; ++i ) {

      Twa = mS.limb[i].arm->getRootBodyNode()->getWorldTransform();
      Taw = Twa.inverse();
      for( int j = 0; j < 4; ++j ) {

	Twj = mS.limb[i].hand->getBodyNode(jointNames[j])->getWorldTransform();
	Twm = Twj*Tjm[j];
	Tam = Taw* Twm;
	pam = Tam.translation();
	printf(" Arm [%d] Marker [%d]: Position w.r.t. arm (%f %f %f) \n",
	       i,j, pam(0), pam(1), pam(2) );
      }
      
    }

    // Sleep and clean up
    usleep( (useconds_t)(1e6*0.1)) ;
    aa_mem_region_local_release();    

  }


}

/**
 * @function loadDefault
 */
void loadDefault() {
    
  // Load default Tjoint_markers
  Tjm.resize(4);
  jointNames.resize(4);
  for( int i = 0; i < 4; ++i ) { Tjm[i].setIdentity(); }
  
  // F12: Knuckle
  Tjm[0].translation() << 0, 0.0275, 0;
  jointNames[0] = std::string("finger_12_link");
  // F13: Tip
  Tjm[1].translation() << 0, 0.0176, 0;
  jointNames[1] = std::string("finger_13_link");
  // F22: Knuckle
  Tjm[2].translation() << 0, -0.0208, 0;
  jointNames[2] = std::string("finger_22_link");
  // F23: Tip
  Tjm[3].translation() << 0, -0.015, 0;
  jointNames[3] = std::string("finger_23_link");

}


/**
 * @function updateStates
 */
void updateStates() {
  
  Eigen::VectorXd arm_q[2];
  Eigen::VectorXd arm_dq[2];
  
  Eigen::VectorXd hand_q[2];
  Eigen::VectorXd hand_dq[2];
  
  for( int i = 0; i < 2; ++i ) { 
    
    if( mDli.update(i) == true ) {
      
      // Get states
      mDli.get_arm_state( i, arm_q[i], arm_dq[i] );
      mDli.get_hand_state( i, hand_q[i], hand_dq[i] );
      
      // Update simulation      
      mS.limb[i].arm->setConfig( mS.limb[i].armDofs, 
				 arm_q[i] );
      
      if( hand_dq[i].size() == 7 ) { 
	Eigen::VectorXd q(8); 
	q << hand_q[i](0), hand_q[i](5), hand_q[i](6), hand_q[i](0), hand_q[i](1), hand_q[i](2), hand_q[i](3), hand_q[i](4);
	mS.limb[i].hand->setConfig( mS.limb[i].fingerDofs, q );
      } // end if hand_dq
      
    } // end if update
    
  }  // end for


}

/** 
 * @function setupComm
 */
void setupComm() {
  
  // Start sns
  sns_init();
  sns_start();
  
  // Open channels to read motor states
  ach_channel_t* arm_state_chan[2];
  ach_channel_t* hand_state_chan[2];
  ach_channel_t* hand_ref_chan[2];
  
  for( int i = 0; i < 2; ++i ) {
    arm_state_chan[i] = new ach_channel_t();
    hand_state_chan[i] = new ach_channel_t();
    hand_ref_chan[i] = new ach_channel_t();
  }
  
  for( int i = 0; i < 2; ++i ) {
    printf("Opening arm %d \n", i);
    sns_chan_open( arm_state_chan[i], mS.limb[i].arm_state_chan.c_str(), NULL );
    sns_chan_open( hand_state_chan[i], mS.limb[i].hand_state_chan.c_str(), NULL );
    sns_chan_open( hand_ref_chan[i], mS.limb[i].hand_ref_chan.c_str(), NULL );
  }
  printf("Next \n");
  // BIMANUAL
  sns_chan_open( &mBiTraj_chan, "bimanual_chan", NULL );
  mDli.set_numJoints(7,7);
  for( int i = 0; i < 2; ++i ) {
    mDli.set_hand_channels(i, hand_state_chan[i], hand_ref_chan[i] );
  }
  mDli.set_arm_channels( arm_state_chan[0], arm_state_chan[1], &mBiTraj_chan );
  
  // Ready
  printf("\t * [SET_COMM] We should be ready to go \n");
}
