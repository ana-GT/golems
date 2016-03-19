/**
 * @file SQ_fitter.hpp
 * @brief Implementation
 */
#pragma once

#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include "perception/pointcloud_tools/sq_fitting/SQ_fitter.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"
#include "perception/pointcloud_tools/sq_fitting/evaluated_eqs.h"

#include "perception/pointcloud_tools/sq_fitting/levmar/levmar.h"

/**
 * @function SQ_fitter
 * @brief Constructor. Create initial pointers for cloud and its normalized version
 */
template<typename PointT>
SQ_fitter<PointT>::SQ_fitter() :
  cloud_( new pcl::PointCloud<PointT>() ),
  mGotInitApprox(false) {

  int i;
  for( i = 0; i < 3; ++i ) { mLowerLim_dim[i] = 0.003; mUpperLim_dim[i] = 0.30; }
  mLowerLim_e = 0.1;  mUpperLim_e = 1.9;
  for( i = 0; i < 3; ++i ) { mLowerLim_trans[i] = -2.0; mUpperLim_trans[i] = 2.0; }
  for( i = 0; i < 3; ++i ) { mLowerLim_rot[i] = -M_PI; mUpperLim_rot[i] = M_PI; } 
 
  mDimFactor = 1.1;
}

/**
 * @function ~SQ_fitter
 * @brief Destructor
 */
template<typename PointT>
SQ_fitter<PointT>::~SQ_fitter() {
}

/**
 * @function setInputCloud
 * @brief Set segmented cloud to be fitted
 */
template<typename PointT>
void SQ_fitter<PointT>::setInputCloud( const PointCloudPtr &_cloud ) {
  cloud_ = _cloud;
}

/**
 * @function setInitialApprox
 * @brief
 */
template<typename PointT>
void SQ_fitter<PointT>::setInitialApprox( const Eigen::Isometry3d &_Tsymm,
					  const Eigen::Vector3d &_Bb ) {
  
  // Get R,P,Y
  Eigen::Matrix3d rt = _Tsymm.linear();
  Eigen::Vector3d rpy = rt.eulerAngles(2,1,0);
  mInitRot[0] = rpy(2); mInitRot[1] = rpy(1); mInitRot[2] = rpy(0); 

  // Get x,y,z
  mInitTrans[0] = _Tsymm.translation()(0);
  mInitTrans[1] = _Tsymm.translation()(1);
  mInitTrans[2] = _Tsymm.translation()(2);

  // Get Bb sizes
  mInitDim[0] = _Bb(0); mInitDim[1] = _Bb(1); mInitDim[2] = _Bb(2);

  mGotInitApprox = true;
}

/**
 * @function fit
 * @brief Fit using Levenberg-Marquadt with box constraints
 */
template<typename PointT>
bool SQ_fitter<PointT>::fit( const int &_type,
			     const double &_smax,
			     const double &_smin,
			     const int &_N,
			     const double &_thresh ) {
    
  // 0. Store parameters
  smax_ = _smax;
  smin_ = _smin;
  N_ = _N;
  thresh_ = _thresh;

  double ds; double error_i; double error_i_1;
  double s_i; bool fitted; 
  SQ_parameters par_i, par_i_1;

  if( N_ == 1 ) { ds = 0; }
  else { ds = (smax_ - smin_) / (double) (N_-1); }
    
  // 1. Initialize par_in_ with bounding box values
  if( mGotInitApprox ) {

    for( int i = 0; i < 3; ++i ) {
      par_in_.dim[i] = mInitDim[i];
      par_in_.trans[i] = mInitTrans[i];
      par_in_.rot[i] = mInitRot[i]; 
    }

    mGotInitApprox = false;
  } else {
    getBoundingBox( cloud_, 
		    par_in_.dim,
		    par_in_.trans,
		    par_in_.rot );
  }

  for( int i = 0; i < 3; ++i ) { this->mUpperLim_dim[i] = this->mDimFactor*this->par_in_.dim[i]; }
  /*
  double dims;
  if( par_in_.dim[0] >= par_in_.dim[1] && par_in_.dim[0] >= par_in_.dim[2] ) { dims = par_in_.dim[0]; }
  if( par_in_.dim[1] >= par_in_.dim[0] && par_in_.dim[1] >= par_in_.dim[2] ) { dims = par_in_.dim[1]; }
  if( par_in_.dim[2] >= par_in_.dim[0] && par_in_.dim[2] >= par_in_.dim[1] ) { dims = par_in_.dim[2]; }
  for( int i = 0; i < 3; ++i ) { this->mUpperLim_dim[i] = this->mDimFactor*dims; }   
  */

  // 1.1. Set e1 and e2 to middle value in range
  par_in_.e[0] = 1.0; par_in_.e[1] = 1.0; 
  par_in_.type = REGULAR;

  // Run loop
  par_i = par_in_;
  double eg, er, ed;
  this->get_error( par_i, cloud_, eg, er, ed );
  error_i = er;
  fitted = false;

  ////////
  for( int i = 0; i < N_; ++i ) {

    s_i = smax_ - (i)*ds;
    par_i_1 = par_i;
    error_i_1 = error_i;

    // Update limits**********
      double dim_i[3];
      this->getBoundingBoxAlignedToTf( cloud_,
			               par_i_1.trans,
                                       par_i_1.rot,
				       dim_i );

      for( int j = 0; j < 3; ++j ) { 
	if( mUpperLim_dim[j] < mDimFactor*dim_i[j] ) { 
	  mUpperLim_dim[j] = mDimFactor*dim_i[j]; 
	} 
      }
    //****************************

    
    PointCloudPtr cloud_i( new pcl::PointCloud<PointT>() );
    downsampling<PointT>( cloud_, s_i, cloud_i ); 

    if( cloud_i->points.size() < 11 ) { continue; }
    
    minimize( _type,
	      cloud_i,
	      par_i_1,
	      par_i,
	      error_i );
    // Get the error of the whole cloud (the error from minimize is from the downsampled cloud)
    this->get_error( par_i, cloud_, eg, error_i, ed );
    
    // [CONDITION]
    double de = (error_i_1 - error_i);
    final_error_ = error_i;

    if( i > 0 ) {
      if( fabs(de) < thresh_ ) {
        fitted = true;
        break;
      } 
   } // end error i > 0
  }
 
  par_out_ = par_i;

  return fitted;
}

/**
 * @function get_error
 * @brief Set segmented cloud to be fitted
 */
template<typename PointT>
void SQ_fitter<PointT>::get_error( SQ_parameters _par,
				   const PointCloudPtr &_cloud,
				   double &_errA, double &_errB, double &_errC ) {
  return error_metric<PointT>( _par, _cloud, _errA, _errB, _errC );
}


/**
 * @function getBoundingBox
 * @brief Set segmented cloud to be fitted
 */
template<typename PointT>
void SQ_fitter<PointT>::getBoundingBox( const PointCloudPtr &_cloud,
					double _dim[3],
					double _trans[3],
					double _rot[3],
					bool _debug ) {

  // 1. Compute the bounding box center
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid( *_cloud, centroid );
  _trans[0] = centroid(0);
  _trans[1] = centroid(1); 
  _trans[2] = centroid(2);

  // 2. Compute main axis orientations
  pcl::PCA<PointT> pca;
  pca.setInputCloud( _cloud );
  Eigen::Vector3f eigVal = pca.getEigenValues();
  Eigen::Matrix3f eigVec = pca.getEigenVectors();
  // Make sure 3 vectors are normal w.r.t. each other
  
  eigVec.col(2) = eigVec.col(0); // Z
  Eigen::Vector3f v3 = (eigVec.col(1)).cross( eigVec.col(2) );
  eigVec.col(0) = v3; 


  Eigen::Vector3f rpy = eigVec.eulerAngles(2,1,0);
 
  _rot[0] = (double)rpy(2);
  _rot[1] = (double)rpy(1);
  _rot[2] = (double)rpy(0);


  // Transform _cloud
  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  transf.block(0,3,3,1) << (float)centroid(0), (float)centroid(1), (float)centroid(2);
  transf.block(0,0,3,3) = eigVec;

  Eigen::Matrix4f tinv; tinv = transf.inverse();
  PointCloudPtr cloud_temp( new pcl::PointCloud<PointT>() );
  pcl::transformPointCloud( *_cloud, *cloud_temp, tinv );

  // Get maximum and minimum
  PointT minPt; PointT maxPt;
  pcl::getMinMax3D( *cloud_temp, minPt, maxPt );
  double dx, dy, dz, dmax;
  dx = ( maxPt.x - minPt.x ) / 2.0;
  dy = ( maxPt.y - minPt.y ) / 2.0;
  dz = ( maxPt.z - minPt.z ) / 2.0;

  _dim[0] = dx;
  _dim[1] = dy;
  _dim[2] = dz;

}

template<typename PointT>
void SQ_fitter<PointT>::getBoundingBoxAlignedToTf( const PointCloudPtr &_cloud,
					double _trans[3],
					double _rot[3],
					double _dim[3] ) {

  Eigen::Matrix3f rot; rot = Eigen::AngleAxisf( _rot[2], Eigen::Vector3f::UnitZ() )*Eigen::AngleAxisf( _rot[1], Eigen::Vector3f::UnitY() )*Eigen::AngleAxisf( _rot[0], Eigen::Vector3f::UnitX() ); 

  // Transform _cloud
  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  transf.block(0,3,3,1) << (float)_trans[0], (float)_trans[1], (float)_trans[2];
  transf.block(0,0,3,3) = rot;

  Eigen::Matrix4f tinv; tinv = transf.inverse();
  PointCloudPtr cloud_temp( new pcl::PointCloud<PointT>() );
  pcl::transformPointCloud( *_cloud, *cloud_temp, tinv );

  // Get maximum and minimum
  PointT minPt; PointT maxPt;
  pcl::getMinMax3D( *cloud_temp, minPt, maxPt );
  double dx, dy, dz, dmax;
  dx = ( maxPt.x - minPt.x ) / 2.0;
  dy = ( maxPt.y - minPt.y ) / 2.0;
  dz = ( maxPt.z - minPt.z ) / 2.0;

  _dim[0] = dx;
  _dim[1] = dy;
  _dim[2] = dz;

} 


/**
 * @function minimize
 * @brief Apply bounded Levenberg-Marquadt with _in initial parameters
 * @output _out
 */
template<typename PointT>
bool SQ_fitter<PointT>::minimize( const int &_type,
				  const PointCloudPtr &_cloud, 
				  const SQ_parameters &_in,
				  SQ_parameters &_out,
				  double &_error ) {
      
    // Parameters initially _in:
    _out = _in; 

    // Set necessary parameters
    int n = _cloud->points.size();
    int m = 11; 
    double p[m]; // Parameters of SQ
    double y[n]; // Values we want to achieve

    double opts[LM_OPTS_SZ];
    double info[LM_INFO_SZ];
    
    opts[0] = LM_INIT_MU;
    opts[1] = 1E-15;
    opts[2] = 1E-15;
    opts[3] = 1E-20;
    opts[4] = LM_DIFF_DELTA;

    struct levmar_data data;
    data.x = new double[n];
    data.y = new double[n];
    data.z = new double[n];
    data.num = n;


    int i; int ret;
    typename pcl::PointCloud<PointT>::iterator pit;
    for( pit = _cloud->begin(), i = 0; pit != _cloud->end(); ++pit, ++i ) {
	data.x[i] = (*pit).x;
	data.y[i] = (*pit).y;
	data.z[i] = (*pit).z;
    }

    // Set minimizer value to zero (could be 1, depending of what equation you are minimizing)
    for( i = 0; i < n; ++i ) { y[i] = 0.0; }
  
    // Initialize values for parameters p
    for( i = 0; i < 3; ++i ) { p[i] = _in.dim[i]; }
    for( i = 0; i < 2; ++i ) { p[i+3] = _in.e[i]; }
    for( i = 0; i < 3; ++i ) { p[i+5] = _in.trans[i]; }
    for( i = 0; i < 3; ++i ) { p[i+8] = _in.rot[i]; }
    
    // Set limits
    double ub[m], lb[m];
    for( i = 0; i < 3; ++i ) { lb[i] = mLowerLim_dim[i]; ub[i] = mUpperLim_dim[i]; }
    for( i = 0; i < 2; ++i ) { lb[i+3] = mLowerLim_e; ub[i+3] = mUpperLim_e; }
    for( i = 0; i < 3; ++i ) { lb[i+5] = mLowerLim_trans[i]; ub[i+5] = mUpperLim_trans[i]; }
    for( i = 0; i < 3; ++i ) { lb[i+8] = mLowerLim_rot[i]; ub[i+8] = mUpperLim_rot[i]; }
    
    switch( _type ) {
    case SQ_FX_RADIAL: {
      
      ret = dlevmar_bc_der( fr_add,
			    Jr_add,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );
    } break;
      
    case SQ_FX_ICHIM: {
      
      ret = dlevmar_bc_der( fi_add,
			    Ji_add,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );
    } break;
      
    case SQ_FX_SOLINA: {
      
      ret = dlevmar_bc_der( fs_add,
			    Js_add,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );
    } break;
      
    case SQ_FX_CHEVALIER: {
      
      ret = dlevmar_bc_der( fc_add,
			    Jc_add,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );
    } break;
      
    case SQ_FX_5: {
      
      ret = dlevmar_bc_der( f5_add,
			    J5_add,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );
    } break;
      
    case SQ_FX_6: {
      
      ret = dlevmar_bc_der( f6_add,
			    J6_add,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );
    } break;

    
  } // end switch

    
    // Fill _out
    for( i = 0; i < 3; ++i ) { _out.dim[i] = p[i]; }
    for( i = 0; i < 2; ++i ) { _out.e[i] = p[i+3]; }
    for( i = 0; i < 3; ++i ) { _out.trans[i] = p[i+5]; }
    for( i = 0; i < 3; ++i ) { _out.rot[i] = p[i+8]; }
    _out.type = REGULAR;

    // Return status and error
    double eg, er, ed;
    get_error( _out, cloud_, eg, er, ed );
    _error = er;
    // If stopped by invalid (TODO: Add other reasons)
    if( info[6] == 7 ) {
	return false;
    } else {
	return true;
    }
}





/**
 * @function printResults
 * @brief Print results of optimization (initial vs final fitted parameters)
 */
template<typename PointT>
void SQ_fitter<PointT>::printResults() {
  
  // 1. Print initial parameters information
  std::cout << "\t *******************************"<<std::endl;
  std::cout << "\t [SQ_fitter] INITIAL PARAMETERS: "<< std::endl;
  std::cout << "\t *******************************"<<std::endl;
  printParamsInfo( par_in_ );
  // 2. Print final parameters information
  std::cout << "\t *******************************"<<std::endl;
  std::cout << "\t [SQ_fitter] FINAL PARAMETERS: "<< std::endl;
  std::cout << "\t *******************************"<<std::endl;
  printParamsInfo( par_out_ );

}


/**
 * @function getSampledOutput
 */
template<typename PointT>
typename SQ_fitter<PointT>::PointCloudPtr SQ_fitter<PointT>::getSampledOutput() {
    
    PointCloudPtr output( new pcl::PointCloud<PointT>() );
    output = sampleSQ_uniform<PointT>( par_out_ );

    // Downsample
    PointCloudPtr cloud_out( new pcl::PointCloud<PointT>() );
    downsampling<PointT>( output,
			  0.01,
			  cloud_out );

    return cloud_out;
}
