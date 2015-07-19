/**
 * @file mindGapper.hpp
 */
#pragma once

//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include "../dt/dt.h"

/** Helpers to improve mask */
int getNumNeighbors( const cv::Mat &_mat, 
		     int pi, int pj, 
		     int minVal );

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
    printf("Error generating 2D mask \n");
    return 0;
  }

  mDTMask = matDT( mMarkMask );
  generateBorder();

  // Refill mask holes 
  for( int c = 0; c < 2; ++c ) {
    growMask( mMarkMask, 4, 125, 0, 125 );
  }

  // 1. Project pointcloud to plane
  mProjected = projectToPlane( mCloud );

  // 2. Find eigenvalues (first two,the last one will be zero since projected cloud is in 2D)
  pcl::PCA<PointT> pca;
  pca.setInputCloud( mProjected );
  Eigen::Vector3f eval = pca.getEigenValues();
  Eigen::Matrix3f evec = pca.getEigenVectors();
  
  Eigen::Vector4d c;
  pcl::compute3DCentroid( *mProjected, c );
  
  mC << c(0), c(1), c(2);
  mEa << (double) evec(0,0), (double) evec(1,0), (double) evec(2,0);
  mEb << (double) evec(0,1), (double) evec(1,1), (double) evec(2,1);

  // 3. Choose the eigen vector most perpendicular to the viewing direction as initial guess for symmetry plane
  Eigen::Vector3d v, s, s_sample;
  v = mC; // viewing vector from Kinect origin (0,0,0) to centroid of projected cloud (mC)
  std::cout << "Centroid: "<< mC.transpose() << std::endl;
  // s: Line which is the intersection between symmetry and table planes
  if( abs(v.dot(mEa)) <= abs(v.dot(mEb)) ) { s = mEa; } 
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
      mValidity.push_back( true );
      candidateSymmRts.push_back( symmRt );
      candidateDists.push_back( mDj*j );

    } // end for N    
  } // end for M

  // 6. Evaluate (optimization)
  mDelta1.resize( mCandidates.size() );
  mDelta2.resize( mCandidates.size() );


  //cv::Mat mi = printDebugMask();
  for( int i = 0; i < mCandidates.size(); ++i ) {

    cv::Mat mi = printDebugMask();
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
	//cv::Vec3b col(0,255,0);  mi.at<cv::Vec3b>(py,px) = col;
      }
      
      // MEASURE 2: IN-MASK PIXELS IN FRONT OF VISIBLE PIXELS
      else {
	double dp = (double)(mDepthMask.at<float>(py,px));
	if( dp == 0 ) { continue; }
	double d = P.z - dp;
	if( d < 0 ) {
	  frontOfMask++;  delta_2 += -d;
	  //cv::Vec3b col(0,0,255);  mi.at<cv::Vec3b>(py,px) = col;
	}

      }
 
    } // end for it
    
    // Expected values
    if( mValidity[i] == false ) { mDelta1[i] = 1000000; mDelta2[i] = 1000000; printf("[%d] Candidate is NOT valid \n", i);}
    else {
      if( outOfMask == 0 ) { mDelta1[i] = 1000000; }
      else{ mDelta1[i] =  delta_1 / (double) outOfMask; }

      if( frontOfMask == 0 ) { mDelta2[i] = 1000000;  }
      else { mDelta2[i] =  (double) frontOfMask; } //(delta_2 / (double)frontOfMask); }
    
    } // end else mValidity

  } // for each candidate

  // Select the upper section according to delta_1
  std::vector<unsigned int> delta1_priority;
  delta1_priority = sortIndices( mDelta1, candidateDists );
  std::vector<double> delta2_selected;

  double d1min, d1max;
  d1min = mDelta1[delta1_priority[0]];
  for( int i = delta1_priority.size() - 1; i >= 0; --i ) {
    if( mDelta1[i] == 1000000 ) { continue; }
    else { d1max = mDelta1[i]; break; }
  }   
  double d1cut = d1min + 0.1*(d1max - d1min);

  for( int i = 0; i < (int)(0.1*mCandidates.size()); ++i ) {
      delta2_selected.push_back( mDelta2[ delta1_priority[i] ] );
  }

  // Prioritize according to delta_2
  std::vector<unsigned int> delta2_priority;
  delta2_priority = sortIndices( delta2_selected, candidateDists );
  
  int minIndex = delta1_priority[delta2_priority[0]];

  // Set symmetry transformation
  symmRt = candidateSymmRts[minIndex];
  // Put in symmetry axis
  calculateSymmTf( symmRt, _cloud );


  // Complete pointcloud if required
  // (do this AFTER setting the symmRt and calclating symmTf above)
  if( _completeCloud ) {
    _cloud->insert( _cloud->end(), mCandidates[minIndex]->begin(),
		    mCandidates[minIndex]->end() );
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

  pcl::io::savePCDFileASCII( "testOriginal.pcd", *_cloud );  
  PointCloudPtr Ps( new PointCloud() );
  Eigen::Affine3f Tcw;
  Tcw = (_Twc.cast<float>()).inverse();
  pcl::transformPointCloud( *_cloud, *Ps, Tcw );

  pcl::MomentOfInertiaEstimation <PointT> feature_extractor;  
  feature_extractor.setInputCloud (Ps);
  feature_extractor.compute();
  PointT mp, Mp;
  feature_extractor.getAABB (mp, Mp);
  Eigen::Vector3d dm, dM;
  dm << fabs(mp.x), fabs(mp.y), fabs(0.5*(Mp.z -mp.z));
  dM << fabs(Mp.x), fabs(Mp.y), fabs(0.5*(mp.z -Mp.z));

  // Dimm
  if( fabs(mp.x) > fabs(Mp.x) ) { mBBDim(0) = fabs(mp.x); } else { mBBDim(0) = fabs(Mp.x); }
  if( fabs(mp.y) > fabs(Mp.y) ) { mBBDim(1) = fabs(mp.y); } else { mBBDim(1) = fabs(Mp.y); }
  printf("Mp: %f %f %f \n", Mp.x, Mp.y, Mp.z);
  printf("mp: %f %f %f \n", mp.x, mp.y,mp.z);
  mBBDim(2) = fabs(Mp.z); //fabs(Mp.z - mp.z);
  
  // Tf
  mSymmTf.setIdentity();
  mSymmTf.linear() = _Twc.linear();
  mSymmTf.translation() = _Twc.linear()*Eigen::Vector3d(0,0,0.5*(mp.z+Mp.z)) + _Twc.translation();

  std::cout << "mBB: \n"<< mBBDim.transpose() << std::endl;
  std::cout << "mSymmTf: \n"<< mSymmTf.matrix() << std::endl;
  std::cout << "Translation: "<< mSymmTf.translation()(0)<<","
	    << mSymmTf.translation()(1)<<","
	    << mSymmTf.translation()(2)  << std::endl;
  Eigen::Quaterniond q( mSymmTf.linear() );
  char command[200];
  std::cout << "Rotation q: "<< q.x() << ","<<q.y()<<","<<q.z()<<","<<q.w()<<std::endl;
  sprintf( command, "./visualization_cube_cloud  -x %f -y %f -z %f -r %f -p %f -q %f -n %f -a %f -b %f -c %f \n",
	  mSymmTf.translation()(0), mSymmTf.translation()(1), mSymmTf.translation()(2),
	  q.x(), q.y(), q.z(), q.w(),
	  mBBDim(0), mBBDim(1), mBBDim(2) );
  system(command);
  
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
  int count = 0; int repeated = 0;
  for( it = _segmented_cloud->begin(); 
       it != _segmented_cloud->end(); ++it ) {
    P = (*it);
    px = (int)( mCx - mF*(P.x / P.z) );
    py = (int)( mCy - mF*(P.y / P.z) );

    if( px < 0 || px >= mWidth ) { return false; }
    if( py < 0 || py >= mHeight ) { return false; }
    count++;
    if( _markMask.at<uchar>(py,px) == 255 ) { repeated++; }
    _markMask.at<uchar>(py,px) = 255;

    _depthMask.at<float>(py,px) = (float)P.z;
  }
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

template<typename PointT>
void mindGapper<PointT>::generateBorder() {
  
  mBorders[0].resize( mMarkMask.rows );
  mBorders[1].resize( mMarkMask.rows );
  uchar prev, current; bool found0;

  
  for( size_t j = 0; j < mMarkMask.rows; ++j ) {
    
    uchar* ptr = mMarkMask.ptr<uchar>(j);
    mBorders[0][j] = -1; mBorders[1][j] = -1;
    prev = *ptr;
    found0 = false; 
    ptr++;

    for( size_t i = 1; i < mMarkMask.cols; ++i ) {
      current =  *ptr;
      
      // Left      
      if( !found0 ) {
	if( current  == 255 && prev == 0 ) { 
	  mBorders[0][j] = i;
	  found0 = true;
	} 
      }
      
      // Right
      if( current == 0 && prev == 255 ) {
	mBorders[1][j] = i-1;
      }       
      prev = current;
      ptr++;  
    
    } // for i    
  } // for j
  
}



/**
 * @function projectToPlane
 * @brief Project _cloud to plane (set by setTablePlane), results in a 2D cloud
 */
template<typename PointT>
typename mindGapper<PointT>::PointCloudPtr mindGapper<PointT>::projectToPlane( PointCloudPtr _cloud ) {

  // 0. Init
  PointCloudPtr projected( new PointCloud() );
  
  // 1. Project and store
  PointCloudIter it;
  PointT p; double a;
  

  // Plane equation: c0*x + c1*y + c2*z + c3 = 0
  // Original 3d point P will be projected (P') into plane with normal [c0, c1, c2]
  // P' = -(c3 + c0*x + c1*y + c2*z) / (c0^2 + c1^2 + c2^2) 
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {

    p = (*it);
    a = -( mPlaneCoeffs(3) + mPlaneCoeffs(0)*p.x + mPlaneCoeffs(1)*p.y + mPlaneCoeffs(2)*p.z );

    PointT pp;
    pp.x = p.x + mPlaneCoeffs(0)*a;
    pp.y = p.y + mPlaneCoeffs(1)*a;
    pp.z = p.z + mPlaneCoeffs(2)*a;

    projected->points.push_back( pp );
  }

  projected->height = projected->points.size();
  projected->width = 1;

  return projected;
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

int getNumNeighbors( const cv::Mat &_mat, 
		     int pi, int pj, 
		     int minVal ) {  
  int ei, ej;
  int n = 0;
  for( int i = -1; i <=1; ++i ) {
    for( int j = -1; j <=1; ++j ) {
      ei = pi + i;
      ej = pj + j;
      if( ei < 0 || ei >= _mat.cols ) { continue; }
      if( ej < 0 || ej >= _mat.rows ) { continue; }
      if( i == 0 && j == 0 ) { continue; }
      if( _mat.at<uchar>(ej,ei) >= minVal ) {
	n++;
      }
    }
  }

  return n;
} 

template<typename PointT>
int mindGapper<PointT>::growMask( cv::Mat &_mask,
				  int _numNeighbors,
				  int _setVal,
				  int _emptyVal,
				  int _minNeighborVal ) {
  int n;
  int count = 0;

  // Go only from border to border
  int is, ig, i;

  for( size_t j = 0; j < _mask.rows; ++j ) {
    
    is = 0; ig = _mask.cols;
    if( mBorders[0][j] > 0 ) { is = mBorders[0][j]; }
    if( mBorders[1][j] > 0 ) { ig = mBorders[1][j]; }

    uchar* ptr = _mask.ptr<uchar>(j);
    for( i = is; i < ig; ++i ) {

      if( ptr[i] == _emptyVal ) {
	
 	// Check num neighbors
	n = getNumNeighbors( _mask, i,j, _minNeighborVal );
	if( n >= _numNeighbors ) {
	  ptr[i] = _setVal;
	  count++;
	}
      } 
      //ptr++;
    } // for 
  } // for

  return count;
}

/**
 * @function printDebugMask
 */
template<typename PointT>
cv::Mat mindGapper<PointT>::printDebugMask() {
  
  cv::Mat mask = cv::Mat::zeros( mHeight, mWidth, CV_8UC3 );
  int count = 0;
  uchar* ptr;
  for( int j = 0; j < mMarkMask.rows; j++ ) {
    ptr = mMarkMask.ptr(j); 
    for( int i = 0; i < mMarkMask.cols; i++ ) {
      if( ptr[i] == 255 ) { cv::Vec3b col(240,240,240); mask.at<cv::Vec3b>(j,i) = col; count++; }
      else if( ptr[i] == 125 ) {  cv::Vec3b col(240,240,240); mask.at<cv::Vec3b>(j,i) = col; }
      else { cv::Vec3b col(0,0,0); mask.at<cv::Vec3b>(j,i) = col; }      
    }
    
//    if( mBorders[0][j] > 0 ) { cv::Vec3b col(0,255,0); mask.at<cv::Vec3b>(j,mBorders[0][j]) = col; }
//    if( mBorders[1][j] > 0 ) { cv::Vec3b col(0,255,0); mask.at<cv::Vec3b>(j,mBorders[1][j]) = col; }
  }
  return mask;
}

