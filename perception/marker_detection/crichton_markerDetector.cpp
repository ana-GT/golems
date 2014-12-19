
#include "crichton_markerDetector.h"

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
  mMarkerDetector.Detect( &ipl, &mCam, false, false ); // track and visualize

  for( size_t i = 0; i < mMarkerDetector.markers->size(); ++i ) {
    int id = (*(mMarkerDetector.markers))[i].GetId();
    printf("Detected marker with id: %d \n", id );

    // DEBUG: Draw it:
    double sum_x = 0; double sum_y = 0;
    for( int j = 0; j < 4; ++j ) {
      alvar::PointDouble p = (*(mMarkerDetector.markers))[i].marker_corners_img[j];
      sum_x += p.x;
      sum_y += p.y;
    }
    
    cv::circle( _img, cv::Point( sum_x/4.0, sum_y/4.0 ), 5, cv::Scalar(0,0,255), -1 );
    printf("Center: %f %f \n", sum_x, sum_y );

    // Pose
    alvar::Pose p = (*(mMarkerDetector.markers))[i].pose;
    double transf[16];


    p.GetMatrixGL( transf, false);
    // Set message
    /*
    gMarkerMsgs[i].trans[0][0] = transf[0];
    gMarkerMsgs[i].trans[0][1] = transf[4];
    gMarkerMsgs[i].trans[0][2] = transf[8];
    gMarkerMsgs[i].trans[0][3] = transf[12];
    gMarkerMsgs[i].trans[1][0] = transf[1];
    gMarkerMsgs[i].trans[1][1] = transf[5];
    gMarkerMsgs[i].trans[1][2] = transf[9];
    gMarkerMsgs[i].trans[1][3] = transf[13];
    gMarkerMsgs[i].trans[2][0] = transf[2];
    gMarkerMsgs[i].trans[2][1] = transf[6];
    gMarkerMsgs[i].trans[2][2] = transf[10];
    gMarkerMsgs[i].trans[2][3] = transf[14];
    for( int row = 0; row < 3; ++row ) {
      for( int col = 0; col <4; ++col ) {
	//std::cout << gMarkerMsgs[i].trans[row][col] << " ";
      } //std::cout << std::endl;
    } 
    */
    
  }
  printf("*******************\n");
  
}
