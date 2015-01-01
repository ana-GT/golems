/**
 * @file crichton_markerDetector.h
 */
#pragma once


#include <MarkerDetector.h>
#include <Camera.h>
#include <vector>

/****/
struct calib_marker {
  int id;
  bool found;
  double xc, yc, zc; // Position w.r.t. camera
  double px,py; // Position in pixels coordinates 
};

/**
 * @class crichton_markerDetector
 */
class crichton_markerDetector {

 public:
  crichton_markerDetector( int _marker_size = 5 );
  void setMarkerSize( int _markerSize );
  bool detect( cv::Mat &_img );
  std::vector<calib_marker> getCalibMarkers() { return mCalibMarkers; }
  
  
  
 private:
  std::vector<alvar::Marker> mMarkers;
  std::vector<calib_marker> mCalibMarkers;
  alvar::MarkerDetector<alvar::MarkerData> mMarkerDetector;
  std::vector<int> mDetectedMarkersId;
  
  alvar::Camera mCam;
  int mMarkerSize;
};
