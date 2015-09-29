/**
 * @file mindGapper.hpp
 */
#pragma once

//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "../dt/dt.h"

#define MAX_VALUE_DELTA 1000000

/**
 * @function fillProjection
 * @brief Fill holes in an object mask by a Open-Close operation
 */
cv::Mat fillProjection( const cv::Mat &_input ) {

  int morph_size = 0;
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

  // 1. Project pointcloud to plane
  mProjected = projectToPlane( mCloud );

  // 2. Find eigenvalues (first two,the last one will be zero since projected cloud is in 2D)
  pcl::PCA<PointT> pca;
  pca.setInputCloud( mProjected );
  Eigen::Vector3f eval = pca.getEigenValues();
  Eigen::Matrix3f evec = pca.getEigenVectors();

  // Calculate the centroid with the voxelized version of the projected cloud on the table
  // (otherwise the center is too influenced by the "front points" and might not
  // use the top information of the cloud, if available
  Eigen::Vector4d c;

  PointCloudPtr projected_voxelized( new PointCloud() );
  
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (mProjected);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*projected_voxelized);

  pcl::compute3DCentroid( *projected_voxelized, c );
    
  pcl::io::savePCDFile("voxelized.pcd", *projected_voxelized, true );
  mC << c(0), c(1), c(2);
  mEa << (double) evec(0,0), (double) evec(1,0), (double) evec(2,0);
  mEb << (double) evec(0,1), (double) evec(1,1), (double) evec(2,1);

  ////////////////
  pcl::PointCloud<pcl::PointXYZ> pab;
  Eigen::Vector3d cab_a; Eigen::Vector3d cab_b;
  for( int m =  0; m <= 10; ++m ) {
    cab_a = mC + mEa.normalized()*0.01*m;
    cab_b = mC + mEb.normalized()*0.01*m;
    pcl::PointXYZ cab_p;
    cab_p.x = cab_a(0); cab_p.y = cab_a(1); cab_p.z = cab_a(2);
    pab.points.push_back( cab_p );
    cab_p.x = cab_b(0); cab_p.y = cab_b(1); cab_p.z = cab_b(2);
    pab.points.push_back( cab_p );
  }
      
  pab.height = 1; pab.width = pab.points.size();
  char nameab[100]; sprintf(nameab, "eigenvels.pcd" );
  pcl::io::savePCDFile(nameab, pab, true );
  ///////////////
  
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

  Eigen::Isometry3d symmRt; symmRt.setIdentity();
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > candidateSymmRts;
  std::vector<double>  candidateDists;

  Np << mPlaneCoeffs(0), mPlaneCoeffs(1), mPlaneCoeffs(2); 
  dang = 2*mAlpha / (double) (mM-1);
  int count = 0;
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

      // Store cp and s_sample direction
      Eigen::Vector3d ss, cia;
      ss = s_sample.normalized();
      
      pcl::PointCloud<pcl::PointXYZ> pt;
      for( int m =  0; m <= 10; ++m ) {
	cia = cp + ss*0.01*m;
	pcl::PointXYZ can; can.x = cia(0); can.y = cia(1); can.z = cia(2);
	pt.points.push_back( can );
      }
      
      pt.height = 1; pt.width = pt.points.size();
      char name[100]; sprintf(name, "see_%d_%d.pcd", i, j );
      pcl::io::savePCDFile(name, pt, true );
      
      
      symmRt.translation() = cp;
      //Set symmetry plane coefficients
      sp << np(0), np(1), np(2), -1*np.dot( cp );

      // 5. Mirror
      mCandidates.push_back( mirrorFromPlane(_cloud, sp, false) );
      
      char name2[100];
      sprintf( name2, "candidate_%d.pcd", count );
      count++;
      pcl::io::savePCDFile(name2, *mCandidates[mCandidates.size()-1], true );
      
      mValidity.push_back( true );
      candidateSymmRts.push_back( symmRt );
      candidateDists.push_back( mDj*j );

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
	outOfMask >= mMax_Out_Mask_Ratio*(_cloud->points.size()) ) {
      mDelta1[i] = MAX_VALUE_DELTA; mDelta2[i] = MAX_VALUE_DELTA;
    }
    else {
      if( outOfMask == 0 ) { mDelta1[i] = MAX_VALUE_DELTA; }
      else{ mDelta1[i] =  delta_1 / (double) outOfMask; }

      if( frontOfMask == 0 ) { mDelta2[i] = MAX_VALUE_DELTA;  } 
      else { mDelta2[i] =  (double) (delta_2 / (double)frontOfMask); }

      printf("Candidate [%d] delta 1: %f delta 2: %f num out: %d front: %d \n", i, mDelta1[i], mDelta2[i],
	     outOfMask, frontOfMask);
            
    } // end else mValidity
    
    
  } // for each candidate

    // Select the upper section according to delta_2 (less frontal points)
  std::vector<unsigned int> delta2_priority;
  delta2_priority = sortIndices( mDelta2, candidateDists );

  std::vector<double> delta1_selected;

  for( int i = 0; i < (int)(mUpper_Ratio_Delta*mCandidates.size()); ++i ) {
      delta1_selected.push_back( mDelta1[ delta2_priority[i] ] );
  }
  
  // Prioritize according to delta_1
  std::vector<unsigned int> delta1_priority;
  delta1_priority = sortIndices( delta1_selected, candidateDists );
  
  int minIndex = delta2_priority[delta1_priority[0]];
 
  printf("Min index: %d  \n", minIndex );
  
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
  _markMask = fillProjection( _markMask );
  cv::imwrite( "mask_filled.png", _markMask );
  
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

  mMax_Out_Mask_Ratio = 0.25;
  mUpper_Ratio_Delta = 0.1;
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


