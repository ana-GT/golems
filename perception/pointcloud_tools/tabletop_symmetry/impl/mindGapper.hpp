/**
 * @file mindGapper.hpp
 */
#pragma once

#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "../dt/dt.h"

#include "perception/pointcloud_tools/tabletop_symmetry/mindGapper_utils.h"


#define MAX_VALUE_DELTA 1000000

/**
 * @function fillProjection
 * @brief Fill holes in an object mask by a Open-Close operation
 */
cv::Mat fillProjection( const cv::Mat &_input ) {

  int morph_size = 2;
  int morph_elem = cv::MORPH_RECT;
  
  cv::Mat output = cv::Mat::zeros( _input.rows, _input.cols, CV_8UC1 );
  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Scalar color(255);

  cv::findContours( _input, contours, hierarchy,
		    CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
  int idx = 0;
  for( ; idx >= 0; idx = hierarchy[idx][0] ) {
    drawContours( output, contours, idx, color, CV_FILLED, 8, hierarchy, 0 );
  }
  
  cv::morphologyEx( output, output, cv::MORPH_OPEN,
		    cv::getStructuringElement(morph_elem,
					      cv::Size(2*morph_size+1, 2*morph_size+1),
					      cv::Point(morph_size,morph_size) ) );
   cv::morphologyEx( output, output, cv::MORPH_CLOSE,
		    cv::getStructuringElement(morph_elem,
					      cv::Size(2*morph_size+1, 2*morph_size+1),
					      cv::Point(morph_size,morph_size) ) );
  
  return output;
}


/**
 * @function sortIndices
 */
struct temp_t { double metric; double dist; unsigned int ind; };
bool sorting( temp_t a, temp_t b ) { 
  if( a.metric != b.metric ) { return  (a.metric < b.metric); }
  else { return (a.dist < b.dist); }
}

std::vector<unsigned int> sortIndices( std::vector<double> _metrics,
				       std::vector<double> _dists ) {

  std::vector<unsigned int> sortedIndices;

  std::vector<temp_t> sorted;
  for( unsigned int i = 0; i < _metrics.size(); ++i ) {
    temp_t tt;
    tt.ind = i;
    tt.metric = _metrics[i];
    tt.dist = _dists[i];
    sorted.push_back(tt);
  }

  // Sort
  std::sort( sorted.begin(), sorted.end(), sorting );
  
  // Return
  for( unsigned int i = 0; i < sorted.size(); ++i ) {
    sortedIndices.push_back( sorted[i].ind );
  }
  
  return sortedIndices;
}


/**
 * @function complete
 * @brief Use symmetries on plane to complete pointcloud 
 */
template<typename PointT>
int mindGapper<PointT>::complete( PointCloudPtr &_cloud,
				  bool _completeCloud ) {
  
  // Variables
  PointCloudIter it;

  this->reset();
  // 0. Store cloud, visibility mask, depth and 
  // the distance transform (DT) in 2D matrices
  mCloud = _cloud;
  if( !this->generate2DMask( mCloud,
			    mMarkMask,
			    mDepthMask ) ) {
    printf("[ERROR] Error generating 2D mask \n");
    return 0;
  }
  
  mDTMask = matDT( mMarkMask );
  cv::imwrite( "dtmask.png", mDTMask );
  cv::imwrite( "markMask.png", mMarkMask );
  // 1. Project pointcloud to plane 
  double dmin, dmax; double da, db;
  mProjected = projectToPlane<PointT>( mCloud, mPlaneCoeffs, dmin, dmax );

  // 2. Find eigenvalues (first two,the last one will be zero since projected cloud is in 2D)
  getInfoFromProjectedCloud<PointT>( mProjected, 0.01, 
				     mC, mEa, mEb, da, db );
  
  // 3. Choose the eigen vector most perpendicular to the viewing direction as initial guess for symmetry plane
  Eigen::Vector3d v, s, s_sample;
  v = mC; // viewing vector from Kinect origin (0,0,0) to centroid of projected cloud (mC)
  // s: Line which is the intersection between symmetry and table planes
  if( fabs(v.dot(mEa)) <= fabs(v.dot(mEb)) ) { s = mEa; } 
  else { s = mEb; }

  
  // 4. Get candidate planes by shifting centroid and rotating initial guess
  Eigen::Vector3d Np; 
  double ang, dang;
  Eigen::VectorXd sp(4);
  Eigen::Vector3d np, cp, dir;

  Eigen::Isometry3d symmRt; symmRt.setIdentity();
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > candidateSymmRts;
  std::vector<double>  candidateDists;

  Np << mPlaneCoeffs(0), mPlaneCoeffs(1), mPlaneCoeffs(2); 
  dang = 2*mAlpha / (double) (mM-1);
  
  int count = 0;
  std::vector<Eigen::VectorXd> planes;
  std::vector<Eigen::VectorXd> centers;

  for( int i = 0; i < mM; ++i ) {
        
    ang = -mAlpha +i*dang;
    s_sample = Eigen::AngleAxisd( ang, Np )*s;
    np = s_sample.cross( Np ); np.normalize();
    

    // Store symmetry reference Transformation
    symmRt.linear().col(2) = Np; 
    symmRt.linear().col(1) = s_sample; 
    symmRt.linear().col(0) = np;

    for( int j = 0; j < mN; ++j ) {
      
      if( np.dot(v) > -np.dot(v) ) { dir = np; } else { dir = -np; }
      cp = mC + dir*mDj*j;
      
      symmRt.translation() = cp;
      //Set symmetry plane coefficients
      sp << np(0), np(1), np(2), -1*np.dot( cp );

      // 5. Mirror
      mCandidates.push_back( mirrorFromPlane(_cloud, sp, false) );

      planes.push_back( sp ); centers.push_back(cp);

      mValidity.push_back( true );
      candidateSymmRts.push_back( symmRt );
      candidateDists.push_back( mDj*j );

      // DEBUG//////
      /*
      cv::Mat markMaskd, depthMaskd;
      this->generate2DMask( mCandidates[mCandidates.size()-1],
			    markMaskd,
			    depthMaskd );
      char named[50]; sprintf(named, "cand_%d_%d.png", i, j );
      cv::imwrite( named, markMaskd );
      sprintf(named, "cand_%d_%d.pcd", i, j );
      pcl::io::savePCDFile( named, *mCandidates[mCandidates.size()-1] );
      */
      ///////////////


    } // end for N    
  } // end for M

  // 6. Evaluate (optimization)
  mDelta1.resize( mCandidates.size() );
  mDelta2.resize( mCandidates.size() );
 
  for( int i = 0; i < mCandidates.size(); ++i ) {

    // Check inliers and outliers
    int px, py; PointT P;
    int outOfMask = 0; int frontOfMask = 0;
    double delta_1 = 0; double delta_2 = 0;
    for( it = mCandidates[i]->begin(); 
	 it != mCandidates[i]->end(); ++it ) {
      P = (*it);
      px = (int)( mF*(-P.x / P.z) + mCx );
      py = (int)( mF*(-P.y / P.z) + mCy );
      
      if( px < 0 || px >= mWidth ) { mValidity[i] = false; break; }
      if( py < 0 || py >= mHeight ) { mValidity[i] = false; break; }
           
      // MEASURE 1: OUT-OF-MASK PIXELS DISTANCE TO CLOSEST MASK PIXEL
      if( mMarkMask.at<uchar>(py,px) == 0 ) {
	outOfMask++;  delta_1 += mDTMask.at<float>(py,px);
      }
      
      // MEASURE 2: IN-MASK PIXELS IN FRONT OF VISIBLE PIXELS
      else {
	double dp = (double)(mDepthMask.at<float>(py,px));
	if( dp == 0 ) { continue; }
	double d = sqrt(P.z*P.z + P.y*P.y + P.x*P.x ) - dp;
	if( d < 0 ) {
	  frontOfMask++;  delta_2 += -d;
	}

      }
 
    } // end for it
    
    // Expected values
    if( mValidity[i] == false ||
	(delta_1 / (double) outOfMask) >= mMax_Out_Pixel_Avg ||
	(delta_2/(double)frontOfMask) >= mMax_Front_Dist_Avg )
      {
      mDelta1[i] = MAX_VALUE_DELTA; mDelta2[i] = MAX_VALUE_DELTA;
    }
    else {
      if( outOfMask == 0 ) { mDelta1[i] = MAX_VALUE_DELTA; }
      else{ mDelta1[i] =  delta_1 / (double) outOfMask; }

      if( frontOfMask == 0 ) { mDelta2[i] = MAX_VALUE_DELTA;  } 
      else { mDelta2[i] = (delta_2 / (double)frontOfMask); }
    } // end else mValidity
    
  } // for each candidate
  

  // First get the smallest delta1 per each rotation group
  int d1_ind; double d1_min;
  std::vector<int> first_pass;
  
  for( int i = 0; i < mM; ++i ) {

    for( int j = 0; j < mN; ++j ) {
      if( mDelta1[i*mN + j] < mCutoff_Pixel_MaxDist ) {
	first_pass.push_back( i*mN + j );
      } 
    }
  }

  // Second, now get the smallest according to delta 2
  int d2_ind; double d2_min;

  if( first_pass.size() == 0 ) { printf("FIrst pass is zero \n"); return -1; }

  d2_ind = first_pass[0]; d2_min = mDelta2[d2_ind];
  for( int i = 1; i < first_pass.size(); ++i ) {
    if( mDelta2[first_pass[i]] < d2_min ) {
      d2_ind = first_pass[i];
      d2_min = mDelta2[d2_ind];
    }
  }

  int minIndex = d2_ind;

  if( mDelta1[minIndex] == MAX_VALUE_DELTA || mDelta2[minIndex] == MAX_VALUE_DELTA ) {
    printf("Mirroring failed to give good results \n");
    return -1;
  }
  printf("Min index: %d \n", minIndex );
  mSymPlane = planes[minIndex];
  mCenterPlane = centers[minIndex];
  mSymPlanes = planes;
  mCenterPlanes = centers;
  // Set symmetry transformation
  symmRt = candidateSymmRts[minIndex];
  // Put in symmetry axis
  calculateSymmTf( symmRt, _cloud );


  // Complete pointcloud if required
  // (do this AFTER setting the symmRt and calclating symmTf above)

  if( _completeCloud ) {
    typename PointCloud::iterator it;
    PointT P; int px, py;
    for( it = mCandidates[minIndex]->begin(); it != mCandidates[minIndex]->end(); ++it ) {

      P = (*it);
      px = (int)( mF*(-P.x / P.z) + mCx );
      py = (int)( mF*(-P.y / P.z) + mCy );
                 
      // MEASURE 1: OUT-OF-MASK PIXELS DISTANCE TO CLOSEST MASK PIXEL
       
      if( mMarkMask.at<uchar>(py,px) == 0 ) {
       // continue;
      }      
      // MEASURE 2: IN-MASK PIXELS IN FRONT OF VISIBLE PIXELS
      else {
	double dp = (double)(mDepthMask.at<float>(py,px));
	if( dp != 0 ) {
	  double d = sqrt(P.z*P.z + P.y*P.y + P.x*P.x ) - dp;
	  if( d < 0 ) {
            continue;
	  }
        }
     }
      _cloud->points.push_back( *it );
   } // end for
  }

  _cloud->width = 1; _cloud->height = _cloud->points.size();

  return minIndex;
}

/**
 * @function calculateSymmTf
 * @brief Calculate the symmetry plane and bounding box approximation
 */
template<typename PointT>
void mindGapper<PointT>::calculateSymmTf( const Eigen::Isometry3d &_Twc,
					  const PointCloudPtr &_cloud ) {

  PointCloudPtr Ps( new PointCloud() );
  Eigen::Affine3f Tcw;
  Tcw = (_Twc.cast<float>()).inverse();
  pcl::transformPointCloud( *_cloud, *Ps, Tcw );

  pcl::MomentOfInertiaEstimation <PointT> feature_extractor;  
  feature_extractor.setInputCloud (Ps);
  feature_extractor.compute();
  PointT mp, Mp;
  feature_extractor.getAABB (mp, Mp);

  // Dimm
  if( fabs(mp.x) > fabs(Mp.x) ) { mBBDim(0) = fabs(mp.x); } else { mBBDim(0) = fabs(Mp.x); }
  if( fabs(mp.y) > fabs(Mp.y) ) { mBBDim(1) = fabs(mp.y); } else { mBBDim(1) = fabs(Mp.y); }
  mBBDim(2) = 0.5*Mp.z; // Z should always be pointing up. no need to fabs
  
  // Tf
  mSymmTf.setIdentity();
  mSymmTf.linear() = _Twc.linear();
  mSymmTf.translation() = _Twc.linear()*Eigen::Vector3d(0,0,0.5*Mp.z) + _Twc.translation();  
}



/**
 * @function getSymmetryApprox
 */
template<typename PointT>
void mindGapper<PointT>::getSymmetryApprox( Eigen::Isometry3d &_Tf,
					    Eigen::Vector3d &_dim ) {  
  _Tf = mSymmTf;
  _dim = mBBDim;
}


/**
 * @function generate2DMask
 * @brief Generates image with visible segmented pixels from _segmented_cloud, _depthMask stores the depth of each
 */
template<typename PointT>
bool mindGapper<PointT>::generate2DMask( PointCloudPtr _segmented_cloud,
					 cv::Mat &_markMask,
					 cv::Mat &_depthMask ) {

  _markMask = cv::Mat::zeros( mHeight, mWidth, CV_8UC1 );
  _depthMask = cv::Mat::zeros( mHeight, mWidth, CV_32FC1 );
  
  // Segmented pixels: 255, No-segmented: 0
  PointCloudIter it;
  PointT P; int px; int py;

  for( it = _segmented_cloud->begin(); 
       it != _segmented_cloud->end(); ++it ) {
    P = (*it);
    px = (int)( mCx - mF*(P.x / P.z) );
    py = (int)( mCy - mF*(P.y / P.z) );

    if( px < 0 || px >= mWidth ) { return false; }
    if( py < 0 || py >= mHeight ) { return false; }

    _markMask.at<uchar>(py,px) = 255;
    _depthMask.at<float>(py,px) = (float)sqrt( P.z*P.z + P.x*P.x + P.y*P.y );
  }

  // Fill projection
  cv::imwrite( "maskNotFilled.png", _markMask );
  _markMask = fillProjection( _markMask );
  
   cv::imwrite( "maskFilled.png", _markMask );

  return true;
}



/** 
 * @function mindGapper
 * @brief Constructor 
 */
template<typename PointT>
mindGapper<PointT>::mindGapper() :
  mCloud( new PointCloud() ),
  mProjected( new PointCloud() ){

  mMax_Out_Mask_Ratio = 0.5;
  mUpper_Ratio_Delta = 0.1;
  mMax_Front_Dist_Avg = 0.005; // 1 cm
  mMax_Out_Pixel_Avg = 4; // 8 pixels avg (too big already, usually more than 6 is wrong)
  mCutoff_Pixel_MaxDist = 2; // Distance in pixels as cut off (we set 2)
}

/**
 * @function ~mindGapper
 * @brief Destructor 
 */
template<typename PointT>
mindGapper<PointT>::~mindGapper() {

}

/**
 * @function setTablePlane
 * @brief Set resting plane and object to complete based on symmetry
 */
template<typename PointT>
void mindGapper<PointT>::setTablePlane( std::vector<double> _planeCoeffs ) {
  
  mPlaneCoeffs.resize( _planeCoeffs.size() );
  for( int i = 0; i < _planeCoeffs.size(); ++i ) {
    mPlaneCoeffs(i) = _planeCoeffs[i];
  }
  
  // Normalize (a,b,c), in case it has not been done already
  double norm = sqrt( pow(mPlaneCoeffs[0],2) + pow(mPlaneCoeffs[1],2) + pow(mPlaneCoeffs[2],2) );
  mPlaneCoeffs = mPlaneCoeffs / norm;
}


/**
 * @function setFittingParams
 * @brief n: Distance steps, m: Orientation steps, dj: Distance step size, alpha: +- rotation step size
 */
template<typename PointT>
void mindGapper<PointT>::setFittingParams( int _n, int _m, 
					   double _dj, double _alpha ) {
  mN = _n;
  mM = _m;
  mDj = _dj;
  mAlpha = _alpha;
}

/**
 * @function setDeviceParams
 * @brief Set parameters of Kinect to calculate fitness functions for optimization process
 */
template<typename PointT>
void mindGapper<PointT>::setDeviceParams( int _width, int _height, 
					  double _focal_length_in_pixels,
					  double _cx, double _cy ) {
  
  mWidth = _width; mHeight = _height;
  mF = _focal_length_in_pixels;
  mCx = _cx; mCy = _cy;
}

/**
 * @function reset
 * @brief Clear storage variables  
 */
template<typename PointT>
void mindGapper<PointT>::reset() {
  mCandidates.resize(0);
  mDelta1.resize(0);
  mDelta2.resize(0);
  mValidity.resize(0);
}



/**
 * @function mirrorFromPlane
 * @brief Mirror pointcloud around _plane 
 */
template<typename PointT>
typename mindGapper<PointT>::PointCloudPtr mindGapper<PointT>::mirrorFromPlane( PointCloudPtr _cloud,
						   Eigen::VectorXd _plane,
						   bool _joinMirrored ) {
  
  // 0. Init
  PointCloudPtr mirrored( new PointCloud() );

  // 1. Project and store
  PointCloudIter it;
  PointT p; double a;


  // Plane equation: c0*x + c1*y + c2*z + c3 = 0
  // Original 3d point P will be projected (P') into plane with normal [c0, c1, c2]
  // P' = -(c3 + c0*x + c1*y + c2*z) / (c0^2 + c1^2 + c2^2) 
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {

    p = (*it);
    a = -( _plane(3) + _plane(0)*p.x + _plane(1)*p.y + _plane(2)*p.z );
    
    PointT mp;
    mp.x = p.x + 2*_plane(0)*a;
    mp.y = p.y + 2*_plane(1)*a;
    mp.z = p.z + 2*_plane(2)*a;

    mirrored->points.push_back( mp );
  }

  // If you want the output to have the whole (original + mirrored points)
  if( _joinMirrored ) {    
    for( it = _cloud->begin(); it != _cloud->end(); ++it ) {
      mirrored->points.push_back( *it );
    }
  }

  mirrored->height = mirrored->points.size();
  mirrored->width = 1;
  
  return mirrored;
}


/**
 * @function printMirror
 */
template<typename PointT>
void mindGapper<PointT>::printMirror( int _ind ) {

  if( _ind < 0 || _ind >= mCandidates.size() ) { std::cout << "NO PRINTING"<< std::endl; return; }

  PointCloudIter it;
  int px, py; PointT P;
  int outside = 0; int front = 0; int behind = 0;
  
  cv::Mat mark_i = cv::Mat::zeros( mHeight, mWidth, CV_8UC3 );
  
  for( it = mCandidates[_ind]->begin(); 
       it != mCandidates[_ind]->end(); ++it ) {
    P = (*it);
    px = (int)( mCx - mF*(P.x / P.z) );
    py = (int)( mCy - mF*(P.y / P.z) );
    
    // If outside: YELLOW
    if( mMarkMask.at<uchar>(py,px) != 255 ) {
      cv::Vec3b col(0,255,255);
      mark_i.at<cv::Vec3b>(py,px) = col;
      outside++;
    } 
    // If inside
    else {
      // If in front of visible  - MAGENTA
      if( (float)P.z < mDepthMask.at<float>(py,px) ) {
	cv::Vec3b col(255,0,255);
	mark_i.at<cv::Vec3b>(py,px) = col;
	front++;
      } else {
	// If behind - CYAN
	cv::Vec3b col(255,255,0);
	mark_i.at<cv::Vec3b>(py,px) = col;
	behind++;
      }
    }

  } // end for it

  char name[50];
  sprintf( name, "candidate_%d.png", _ind );
  imwrite( name, mark_i );

}


