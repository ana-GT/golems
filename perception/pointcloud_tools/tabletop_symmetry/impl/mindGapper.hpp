/**
 * @file mindGapper.hpp
 */
#pragma once

//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>

#include "../dt/dt.h"

/** Helpers to improve mask */
int getNumNeighbors( const cv::Mat &_mat, 
		     int pi, int pj, 
		     int minVal );

/**
 * @function sortIndices
 */
struct temp_t { double metric; unsigned int ind; };
bool sorting( temp_t a, temp_t b ) { return  (a.metric < b.metric); }

std::vector<unsigned int> sortIndices( std::vector<double> _metrics ) {

  std::vector<unsigned int> sortedIndices;

  std::vector<temp_t> sorted;
  for( unsigned int i = 0; i < _metrics.size(); ++i ) {
    temp_t tt;
    tt.ind = i;
    tt.metric = _metrics[i];
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
int mindGapper<PointT>::complete( PointCloudPtr &_cloud ) {
  
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
  for( int c = 0; c <= 1; ++c ) {
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

  // s: Line which is the intersection between symmetry and table planes
  if( abs(v.dot(mEa)) <= abs(v.dot(mEb)) ) { s = mEa; } 
  else { s = mEb; }

  
  // 4. Get candidate planes by shifting centroid and rotating initial guess
  Eigen::Vector3d Np; 
  double ang, dang;
  Eigen::VectorXd sp(4);
  Eigen::Vector3d np, cp, dir;

  Np << mPlaneCoeffs(0), mPlaneCoeffs(1), mPlaneCoeffs(2); 
  dang = 2*mAlpha / (double) (mM-1);
    
  for( int i = 0; i < mM; ++i ) {
        
    ang = -mAlpha +i*dang;
    s_sample = Eigen::AngleAxisd( ang, Np )*s;
    np = s_sample.cross( Np ); np.normalize();
    
    for( int j = 0; j < mN; ++j ) {

      if( np.dot(v) > -np.dot(v) ) { dir = np; } else { dir = -np; }
      cp = mC + dir*mDj*j;

      //Set symmetry plane coefficients
      sp << np(0), np(1), np(2), -1*np.dot( cp );

      // 5. Mirror
      mCandidates.push_back( mirrorFromPlane(_cloud, sp, false) );

    } // end for N    
  } // end for M

  // 6. Evaluate (optimization)
  mDelta.resize( mCandidates.size() );
  mDelta1.resize( mCandidates.size() );
  mDelta2.resize( mCandidates.size() );

  cv::Mat mark_i = printDebugMask();
  char name[50]; sprintf(name, "mask.png" ); cv::imwrite( name, mark_i );  

  for( int i = 0; i < mCandidates.size(); ++i ) {
    cv::Mat candMat = mark_i.clone();
    // Check inliers and outliers
    int px, py; PointT P;
    int outOfMask = 0; int frontOfMask = 0;
    double delta_1 = 0; double delta_2 = 0;
    for( it = mCandidates[i]->begin(); 
	 it != mCandidates[i]->end(); ++it ) {
      P = (*it);
      px = (int)( mF*(P.x / P.z) + mCx );
      py = (int)( mF*(P.y / P.z) + mCy );
      
      if( px < 0 || px >= mWidth ) { return -1; }
      if( py < 0 || py >= mHeight ) { return -1; }
           
      // MEASURE 1: OUT-OF-MASK PIXELS DISTANCE TO CLOSEST MASK PIXEL
      if( mMarkMask.at<uchar>(py,px) == 0 ) {
	outOfMask++;
	delta_1 += mDTMask.at<float>(py,px);
	cv::Vec3b col(0,255,0); candMat.at<cv::Vec3b>(py,px) = col;
      }

      // MEASURE 2: IN-MASK PIXELS IN FRONT OF VISIBLE PIXELS
      else {
	double dp = (double)(mDepthMask.at<float>(py,px));
	if( dp == 0 ) { continue; }
	double d = P.z - dp;
	if( d < 0 ) {
	  frontOfMask++;
	  delta_2 += -d;
	  cv::Vec3b col(255,0,0); candMat.at<cv::Vec3b>(py,px) = col;
	}

      }
 
      
    } // end for it
    
    // Expected values
    mDelta1[i] =  delta_1 / (double) outOfMask;
    if( frontOfMask == 0 ) { mDelta2[i] = 0; }
    else { mDelta2[i] =  (delta_2 / (double)frontOfMask); } // * 1000.0 / 0.00780; }// mmx pix/mm

  } // for each candidate

  // Select the upper section according to delta_1
  std::vector<unsigned int> delta1_priority;
  delta1_priority = sortIndices( mDelta1 );

  // Get the first 10% according to delta_1
  std::vector<double> delta2_selected;
  for( int i = 0; i < (int)((0.1)*mCandidates.size()); ++i ) {
    delta2_selected.push_back( mDelta2[ delta1_priority[i] ] );
  }
  // Prioritize according to delta_2
  std::vector<unsigned int> delta2_priority;
  delta2_priority = sortIndices( delta2_selected );
  
  int minIndex = delta1_priority[delta2_priority[0]];
    
  _cloud = mCandidates[ minIndex ];
  printf("Min index: %d \n", minIndex );
  for( it = mCloud->begin(); it != mCloud->end(); ++it ) {
    _cloud->points.push_back( *it );
  }
  _cloud->width = 1; _cloud->height = _cloud->points.size();
  
  return minIndex;
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
    px = (int)( mF*(P.x / P.z) + mCx );
    py = (int)( mF*(P.y / P.z) + mCy );

    if( px < 0 || px >= mWidth ) { return false; }
    if( py < 0 || py >= mHeight ) { return false; }
    
    _markMask.at<uchar>(py,px) = 255;
    _depthMask.at<float>(py,px) = (float)P.z;
  }
  
  // Mark mask with points with 8 neighbors
  /*
  int addedPoints = 0;
  int i = 4;
  for( int j = 0; j < 2; ++j ) {
    addedPoints += growMask( _markMask, i, 125, 0, 125 );
    i++;
  }
  */
  //  printf("Added %d points: %f \n", addedPoints, (double)addedPoints/(_segmented_cloud->points.size()));

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
  mDelta.resize(0);
  mDelta1.resize(0);
  mDelta2.resize(0);
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
 * @function viewMirror
 */
template<typename PointT>
bool mindGapper<PointT>::viewMirror( int _ind ) {
/*
  if( _ind >= mCandidates.size() || _ind < 0 ) {
    return false; 
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("Mind Gap") );
  viewer->setBackgroundColor(0,0,0);
  viewer->addCoordinateSystem(1.0, 0 );
  viewer->initCameraParameters();

  // Original - GREEN, mirror - BLUE
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color( mCloud, 0, 255, 0 );
  pcl::visualization::PointCloudColorHandlerCustom<PointT> mirror_color( mCandidates[_ind], 0, 0, 255 );
  viewer->addPointCloud( mCandidates[_ind], "mirror_cloud" );
  viewer->addPointCloud( mCloud, cloud_color, "cloud" );
  
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
  }
  
  return true;
*/
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
    px = (int)( mF*(P.x / P.z) + mCx );
    py = (int)( mF*(P.y / P.z) + mCy );
    
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


/**
 * @function viewInitialParameters
 * @brief View projected cloud, centroid and 2 eigenvalues Ea and Eb
 */
template<typename PointT>
bool mindGapper<PointT>::viewInitialParameters() {
/*
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("Initial") );
  viewer->setBackgroundColor(0,0,0);
  viewer->addCoordinateSystem(1.0, 0 );
  viewer->initCameraParameters();

  // Original green, mirror blue
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color( mCloud, 0, 255, 0 );
  pcl::visualization::PointCloudColorHandlerCustom<PointT> projected_color( mProjected, 0, 0, 255 );
  viewer->addPointCloud( mProjected, projected_color, "projected" );
  viewer->addPointCloud( mCloud, cloud_color, "cloud" );
  
  // Center red ball
  PointT c;
  c.x = mC(0); c.y = mC(1); c.z = mC(2);
  double r, g, b;
  r = 1; g = 0; b = 0;
  viewer->addSphere( c, 0.015, r, g, b, "centroid" );

  // Draw Eigen vectors: ea magenta, eb yellow
  PointT pea, peb;
  double l = 0.20;
  pea.x = c.x + mEa(0)*l;   pea.y = c.y + mEa(1)*l;   pea.z = c.z + mEa(2)*l;
  peb.x = c.x + mEb(0)*l;   peb.y = c.y + mEb(1)*l;   peb.z = c.z + mEb(2)*l;

  r = 1.0; g = 0.0; b = 1.0;
  viewer->addLine( c, pea, r, g, b, "ea", 0 );
  r = 1.0; g = 1.0; b = 0.0;
  viewer->addLine( c, peb,  r, g, b, "eb", 0 );

  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
  }

  
  return true;
*/
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

  uchar* ptr;
  for( int j = 0; j < mMarkMask.rows; j++ ) {
    ptr = mMarkMask.ptr(j); 
    for( int i = 0; i < mMarkMask.cols; i++ ) {
      if( ptr[i] == 255 ) { cv::Vec3b col(240,240,240); mask.at<cv::Vec3b>(j,i) = col; }
      else if( ptr[i] == 125 ) {  cv::Vec3b col(0,0,255); mask.at<cv::Vec3b>(j,i) = col; }
      else { cv::Vec3b col(0,0,0); mask.at<cv::Vec3b>(j,i) = col; }      
    }
    
    if( mBorders[0][j] > 0 ) { cv::Vec3b col(0,255,0); mask.at<cv::Vec3b>(j,mBorders[0][j]) = col; }
    if( mBorders[1][j] > 0 ) { cv::Vec3b col(0,255,0); mask.at<cv::Vec3b>(j,mBorders[1][j]) = col; }
  }

  return mask;
}
