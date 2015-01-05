/**
 * @file mindGapper.h
 * @brief Generates an object's full pointcloud from a partial view using symmetries w.r.t. a given resting plane
 */
#pragma once

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//-- OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @class mindGapper
 */
template<typename PointT>
class mindGapper {

 public:
  
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;
  typedef typename PointCloud::iterator PointCloudIter;


  mindGapper();
  ~mindGapper();
  
  /**< Set table plane where the object rests and its partial cloud */
  void setTablePlane( std::vector<double> planeCoeff );


  /**< Set parameters for symmetry plane search */
  void setFittingParams( int _n = 6, int _m = 5, 
			 double _dj = 0.01, double _alpha = 20.0*M_PI / 180.0 );

  /**< Set Kinect params to calculate mirror errors */
  void setDeviceParams( int _width = 640, int _height = 480, 
			double _focal_length_in_pixels = 525,
			double _cx = 320, double _cy = 240 );

  /** Reset storage variables for new mirror operation */
  void reset();

  /**< Generates mirroring cloud */
  int complete( PointCloudPtr &_cloud );


  bool generate2DMask( PointCloudPtr _segmented_cloud,
		       cv::Mat &_markMask,
		       cv::Mat &_depthMask );
  cv::Mat get2DMask() { return mMarkMask; }
  

  // Debug functions
  bool viewMirror( int _ind );
  void printMirror( int _ind );
  PointCloudPtr getCandidate(int _ind) { return mCandidates[_ind]; }
  bool viewInitialParameters();

 private:
  
  /** Helper functions for complete */
  PointCloudPtr projectToPlane( PointCloudPtr _cloud );
  PointCloudPtr mirrorFromPlane( PointCloudPtr _cloud,
				 Eigen::VectorXd _plane,
				 bool _joinMirrored = true );

  PointCloudPtr mProjected;
  std::vector<PointCloudPtr> mCandidates;
  std::vector<double> mDelta;
  std::vector<double> mDelta1;
  std::vector<double> mDelta2;
  PointCloudPtr mCloud;

  /**< Variables */
  Eigen::VectorXd mPlaneCoeffs;
  int mN;
  int mM;
  double mDj;
  double mAlpha;
  
  // Mask variables
  double mF, mCx, mCy;
  cv::Mat mMarkMask; cv::Mat mDepthMask;
  cv::Mat mDTMask;
  int mWidth; int mHeight;

  Eigen::Vector3d mC;
  Eigen::Vector3d mEa, mEb;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
};


#include "impl/mindGapper.hpp"
