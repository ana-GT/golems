
#include "skeleton_tracker/skeleton_tracker.h"

int main( int argc, char* argv[] ) {

  skeleton_tracker mTracker;
  openni::Status rc = openni::STATUS_OK;
  rc = mTracker.init( argc, argv );
  if( rc != openni::STATUS_OK ) {
    printf(">> Tracker initialization did not finish OK \n");
    return 1;
  }

  mTracker.run();
  printf("Finalize...\n");

}
