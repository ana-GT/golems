/**
 * @file skeleton_tracker.cpp
 */
#include <stdio.h>
#include "skeleton_tracker.h"

/**
 * @function skeleton_tracker 
 * @brief Constructor
 */
skeleton_tracker::skeleton_tracker() {
  mUserTracker = new nite::UserTracker;
}


/**
 * @function ~skeleton_tracker
 * @brief Destructor
 */
skeleton_tracker::~skeleton_tracker() {
  finalize();
}

/**
 * @function init
 * @brief Initialize
 */
openni::Status skeleton_tracker::init( int argc, char* argv[] ) {
  
  openni::Status rc = openni::OpenNI::initialize();
  if( rc!= openni::STATUS_OK ) {
    printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError() );
    return rc;
  }

  rc = mDevice.open( openni::ANY_DEVICE );
  if( rc != openni::STATUS_OK ) {
    printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError() );
    return rc;    
  }

  nite::NiTE::initialize();
  if( mUserTracker->create( &mDevice ) != nite::STATUS_OK ) {
    return openni::STATUS_ERROR;
  }

  // Open channel to communicate
  sns_chan_open( &mJoints_chan, "skel_chan", NULL );
  
}

void skeleton_tracker::run() {

  while(true) {
    update();
    usleep(0.5*1e6);
  }

}


void skeleton_tracker::update() {

  nite::UserTrackerFrameRef userTrackerFrame;
  //openni::VideoFrameRef depthFrame;
  nite::Status rc = mUserTracker->readFrame( &userTrackerFrame );
  if( rc != nite::STATUS_OK ) {
    printf("[X] Get Next data failed \n");
    return;
  }

  const nite::Array<nite::UserData> &users = userTrackerFrame.getUsers();
  
  //printf("Number of users detected: %d \n", users.getSize() );
  for( int i = 0; i < users.getSize(); ++i ) {
    const nite::UserData& user = users[i];

    // If it is new, start tracking
    if( user.isNew() ) {
      printf("\n New user [%d] \n", i );
      mUserTracker->startSkeletonTracking( user.getId() );
      mUserTracker->startPoseDetection( user.getId(), nite::POSE_CROSSED_HANDS );
    } else if( !user.isLost() ) {
      if( user.getSkeleton().getState() == nite::SKELETON_TRACKED ) {
	printf("[V] Tracked user [%d] \n", i);
	nite::Point3f Point;
	// 0. HEAD
	Point = users[i].getSkeleton().getJoint(nite::JOINT_HEAD).getPosition();
	mJoints[HEAD] = Eigen::Vector3d( Point.x, Point.y, Point.z );
	// 1. NECK
	Point = users[i].getSkeleton().getJoint(nite::JOINT_NECK).getPosition();
	mJoints[NECK] = Eigen::Vector3d( Point.x, Point.y, Point.z );
	// 2. LEFT_SHOULDER
	Point = users[i].getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition();
	mJoints[LEFT_SHOULDER] = Eigen::Vector3d( Point.x, Point.y, Point.z );
	// 3. LEFT_ELBOW
	Point = users[i].getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition();
	mJoints[LEFT_ELBOW] = Eigen::Vector3d( Point.x, Point.y, Point.z );
	// 4. LEFT_HAND
	Point = users[i].getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition();
	mJoints[LEFT_HAND] = Eigen::Vector3d( Point.x, Point.y, Point.z );
	// 5. RIGHT_SHOULDER
	Point = users[i].getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition();
	mJoints[RIGHT_SHOULDER] = Eigen::Vector3d( Point.x, Point.y, Point.z );
	// 6. RIGHT_ELBOW
	Point = users[i].getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition();
	mJoints[RIGHT_ELBOW] = Eigen::Vector3d( Point.x, Point.y, Point.z );
	// 7. RIGHT_HAND
	Point = users[i].getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition();
	mJoints[RIGHT_HAND] = Eigen::Vector3d( Point.x, Point.y, Point.z );
	// 8. TORSO
	Point = users[i].getSkeleton().getJoint(nite::JOINT_TORSO).getPosition();
	mJoints[TORSO] = Eigen::Vector3d( Point.x, Point.y, Point.z );

	for( unsigned int i = 0; i < NUM_SKEL_JOINTS; ++i ) {
	  mMsg.x[i] = -1*mJoints[i](0)/1000.0;
	  mMsg.y[i] = mJoints[i](1)/1000.0;
	  mMsg.z[i] = mJoints[i](2)/1000.0;
	}
	
	int r = ach_put( &mJoints_chan,
			 (void*)&mMsg,
			 sizeof(mMsg) );
	if( r != ACH_OK ) {
	  printf("[X] An error occurred sending the skeleton pose \n");
	} else {
	  printf("Sending pose through skel_chan \n");
	}
	
      } // end if SKELETON_TRACKED

      else {
	//printf("[X] Skeleton is not lost but it is also not tracked yet [%d] \n", i);
      }

      
      
    } // end if isLost

  } // end for
 
  
}

void skeleton_tracker::finalize() {
  delete mUserTracker;
  nite::NiTE::shutdown();
  openni::OpenNI::shutdown();
}


