/**
 * @file perception_pick_ftts_2.cpp
 * @brief Recognizes objects in bounding boxes and uses it to select what fitting to do
 * @brief Adding bounding box
 * @date 2016/01/13
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>

#include <global/crichton_global.h>
#include <global/fsa_data.h>

#include "perception/ftts/fast_tabletop_segmentation.h"
#include "perception/msgs/perception_msgs.h"
#include "perception/pointcloud_tools/tabletop_symmetry/mindGapper.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter_t.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter_b.h"
#include "object_recognition/ObjectsDatabase.h"
#include <pcl/filters/statistical_outlier_removal.h>

#include <mutex>

#include <ach.h>
#include <sns.h>

#include "CrichtonView.h"

typedef pcl::PointXYZRGBA PointT;

CrichtonView<PointT> mCv;
void EonMouse( int event, int x, int y, int flags, void* userdata ) { mCv.onMouse( event, x, y, flags, userdata); }
void EmirrorState( int state, void* data ) { mCv.mirrorState( state, data ); }
void EstartComm( int state, void* data ) { mCv.startComm( state, data ); }
void Esend( int state, void* data ) { mCv.send( state, data ); }

/**
 * @function main
 */
int main( int argc, char* argv[] ) {


  // Set configuration values for Tts (tabletop segmentor)
  mCv.mRgbImg = cv::Mat( 480, 640, CV_8UC3 );
  mCv.mTts.setMinClusterSize(300);  
  mCv.mTts.setMinMaxFilter( -1.0, 1.0, -1.0, 1.0, 0.35, 1.0 ); // PrimeSense
  //mCv.mTts.setMinMaxFilter( -0.35, 0.35, -0.70, 0.70, 1.5, 2.4 ); // Kinect
  
  // Set capture
  mCv.setGrabber();

  // Set window
  mCv.setGUI();

 
 cv::setMouseCallback( mCv.mWindowName, EonMouse, NULL );
  cv::createButton( "Mirror", EmirrorState, NULL, cv::QT_CHECKBOX, mCv.mMirror );
  cv::createButton( "Start comm", EstartComm, NULL, cv::QT_PUSH_BUTTON, false );
  cv::createButton( "Send", Esend, NULL, cv::QT_PUSH_BUTTON, false );

  // Object database
  mCv.mOd.init_classifier();
  mCv.mOd.load_dataset();

  //Loop
  mCv.mGrabber->start();

  for(;;) {
    
    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * Pressed ESC. Finishing program! \n");
      mCv.mGrabber->stop();
      break;
    }
    cv::imshow( mCv.mWindowName, mCv.mRgbImg );

  } // end for

  return 0;

}

