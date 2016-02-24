
#include "crichton_markerDetector.h"
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @function crichton_markerDetector
 * @brief Constructor
 */
crichton_markerDetector::crichton_markerDetector( int _markerSize ) {
  setMarkerSize( _markerSize );
}

/**
 * @function setMarkerSize
 */
void crichton_markerDetector::setMarkerSize( int _markerSize ) {
  mMarkerSize = _markerSize;
  mMarkerDetector.SetMarkerSize( mMarkerSize );
}

/**
 * @function detect
 * @brief True if at least one marker was found
 */
bool crichton_markerDetector::detect( cv::Mat &_img ) {

  IplImage ipl = _img;
  mMarkerDetector.Detect( &ipl, &mCam, false, true ); // track and visualize
  mCalibMarkers.resize(0);

  
  for( size_t i = 0; i < mMarkerDetector.markers->size(); ++i ) {
    
    int id = (*(mMarkerDetector.markers))[i].GetId();

//    printf("[%d] Marker ID: %d \n", i, id );
    
    double sum_x = 0; double sum_y = 0;
//    printf("Corners:");
    for( int j = 0; j < 4; ++j ) {
      alvar::PointDouble p = (*(mMarkerDetector.markers))[i].marker_corners_img[j];
      sum_x += p.x;
      sum_y += p.y;
    //  printf("%f %f -- ", p.x, p.y);
    }
  //  printf("\n");
    sum_x /= 4.0;
    sum_y /= 4.0;

    // Pose
    alvar::Pose p = (*(mMarkerDetector.markers))[i].pose;
    double transf[16];
    p.GetMatrixGL( transf, false);

    calib_marker cm;
    cm.id = id; 
    cm.found = true;
    cm.xc = transf[12]; cm.yc = transf[13]; cm.zc = transf[14];
    cm.px = sum_x; cm.py = sum_y;
    mCalibMarkers.push_back( cm );

    // DEBUG: Draw it:
    //if( id == 8 ) {
    cv::circle( _img, cv::Point( sum_x, sum_y ), 5, cv::Scalar(0,0,255), -1 );
    //}
  }
  if( mCalibMarkers.size() > 1 ) {
    return true;
  }  
  return false;
}
