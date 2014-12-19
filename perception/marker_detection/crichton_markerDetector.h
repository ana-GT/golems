/**
 * @file crichton_markerDetector.h
 */
#pragma once


#include <MarkerDetector.h>
#include <Camera.h>
#include <vector>

/**
 * @class crichton_markerDetector
 */
class crichton_markerDetector {

 public:
  crichton_markerDetector( int _marker_size = 5 );
  void setMarkerSize( int _markerSize );
  bool detect( cv::Mat &_img );

  
 private:
  std::vector<alvar::Marker> mMarkers;
  alvar::MarkerDetector<alvar::MarkerData> mMarkerDetector;
  std::vector<int> mDetectedMarkersId;
  
  alvar::Camera mCam;
  int mMarkerSize;
};
