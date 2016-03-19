
#pragma once

#include "perception/pointcloud_tools/sq_fitting/evaluated_eqs_t.h"
#include "perception/pointcloud_tools/tabletop_symmetry/mindGapper_utils.h"

/**
 * @function SQ_fitter
 * @brief Constructor. Create initial pointers for cloud and its normalized version
 */
template<typename PointT>
SQ_fitter_t<PointT>::SQ_fitter_t() :
  SQ_fitter<PointT>() {
  
  int i;
  for( i = 0; i < 3; ++i ) { this->mLowerLim_dim[i] = 0.003; this->mUpperLim_dim[i] = 0.30; }
  this->mLowerLim_e = 0.1;  this->mUpperLim_e = 1.9;
  for( i = 0; i < 3; ++i ) { this->mLowerLim_trans[i] = -2.0; this->mUpperLim_trans[i] = 2.0; }
  for( i = 0; i < 3; ++i ) { this->mLowerLim_rot[i] = -M_PI; this->mUpperLim_rot[i] = M_PI; }
 
  this->mLowerLim_tamp = -1.0; this->mUpperLim_tamp = 1.0;

  this->mDimFactor = 1.1;
}

/**
 * @function ~SQ_fitter_t
 * @brief Destructor
 */
template<typename PointT>
SQ_fitter_t<PointT>::~SQ_fitter_t() {
}

/**
 * @function fit
 * @brief Fit using Levenberg-Marquadt with box constraints
 */
template<typename PointT>
bool SQ_fitter_t<PointT>::fit( const int &_type, 
			       const double &_smax,
			       const double &_smin,
			       const int &_N,
			       const double &_thresh ) {
    
  // 0. Store parameters
  this->smax_ = _smax;
  this->smin_ = _smin;
  this->N_ = _N;
  this->thresh_ = _thresh;
  
  double ds; double error_i; double error_i_1;
  double s_i; bool fitted; 
  SQ_parameters par_i, par_i_1;
  
  if( this->N_ == 1 ) { ds = 0; }
  else { ds = (this->smax_ - this->smin_)/(double)(this->N_-1); }

  
  // 1. Initialize par_in_ with bounding box values
  if( this->mGotInitApprox ) {
    
    for( int i = 0; i < 3; ++i ) {
      this->par_in_.dim[i] = this->mInitDim[i];
      this->par_in_.trans[i] = this->mInitTrans[i];
      this->par_in_.rot[i] = this->mInitRot[i]; 
    }
    
    this->mGotInitApprox = false;
  } else {

    
    this->getBoundingBox( this->cloud_, 
			  this->par_in_.dim,
			  this->par_in_.trans,
			  this->par_in_.rot,
			  false );
    
    printf(" BT Dim: %f %f %f, trans: %f %f %f, rot: %f %f %f \n",
	   this->par_in_.dim[0], this->par_in_.dim[1], this->par_in_.dim[2],
	   this->par_in_.trans[0], this->par_in_.trans[1], this->par_in_.trans[2],
	   this->par_in_.rot[0], this->par_in_.rot[1], this->par_in_.rot[2]);
    

    printf("Use table for orientation \n");
    double coeff[4]= {-0.002072, -0.666118, -0.745843, 0.667762 }; 
    getBoundingBoxTable<PointT>( this->cloud_, coeff, 
				 this->par_in_.dim, 
				 this->par_in_.trans, 
				 this->par_in_.rot );
    printf(" AT Dim: %f %f %f, trans: %f %f %f, rot: %f %f %f \n",
	   this->par_in_.dim[0], this->par_in_.dim[1], this->par_in_.dim[2],
	   this->par_in_.trans[0], this->par_in_.trans[1], this->par_in_.trans[2],
	   this->par_in_.rot[0], this->par_in_.rot[1], this->par_in_.rot[2]);
  }
   

  for( int i = 0; i < 3; ++i ) { this->mUpperLim_dim[i] = this->mDimFactor*this->par_in_.dim[i]; }

  // 1.0 Set tampering start to 1 (0: All tampering, 1: No tamp)
  this->par_in_.tamp = 0.1;
  this->par_in_.type = TAMPERED;

  // 1.1. Set e1 and e2 to middle value in range
  this->par_in_.e[0] = 0.5; this->par_in_.e[1] = 1.0;

  // Run loop
  par_i = this->par_in_;
  double eg, er, ed;
  this->get_error( par_i, this->cloud_, eg, error_i, ed );
  fitted = false;

  ///***********************************
  for( int i = 0; i < this->N_; ++i ) { 

    s_i = this->smax_ - (i)*ds;
    par_i_1 = par_i;
    error_i_1 = error_i;
    
    // Update limits**********
    /*
      double dim_i[3];
      this->getBoundingBoxAlignedToTf( this->cloud_,
			               par_i_1.trans,
                                       par_i_1.rot,
				       dim_i );
      
      for( int j = 0; j < 3; ++j ) { 
	if( this->mUpperLim_dim[j] < this->mDimFactor*dim_i[j] ) { 
	  this->mUpperLim_dim[j] = this->mDimFactor*dim_i[j]; 
	}
      }
    */
    //****************************
      printf("[%d] Lim dim: %f %f %f \n", i, this->mUpperLim_dim[0], this->mUpperLim_dim[1], this->mUpperLim_dim[2]);

    PointCloudPtr cloud_i( new pcl::PointCloud<PointT>() );
    if( this->N_ == 1 ) { cloud_i = this->cloud_; }
    else { downsampling<PointT>( this->cloud_, s_i, cloud_i ); }

    if( cloud_i->points.size() < 12 ) { continue; }

    minimize( _type, 
	      cloud_i,
	      par_i_1,
	      par_i,
	      error_i );
    // Get the error of the whole cloud (the error from minimize is from the downsampled cloud)
    this->get_error( par_i, this->cloud_, eg, error_i, ed );
 
    

    // [CONDITION]
    double de = (error_i_1 - error_i);
    this->final_error_ = error_i;
    if( fabs(de) < this->thresh_ ) {
      fitted = true;
      break;
    } 
    
  } // end for N_
 
  this->par_out_ = par_i;

  return fitted;
}

/**
 * @function minimize_tampering
 */
template<typename PointT>
bool SQ_fitter_t<PointT>::minimize( const int &_type, 
				    const PointCloudPtr &_cloud,
				    const SQ_parameters &_in,
				    SQ_parameters &_out,
				    double &_error ) {
    
    // Parameters initially _in:
    _out = _in; 

    // Set necessary parameters
    int n = _cloud->points.size();
    int m = 12; 
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
    p[11] = _in.tamp;

    
    // Set limits
    double ub[m], lb[m];
    for( i = 0; i < 3; ++i ) { lb[i] = this->mLowerLim_dim[i]; ub[i] = this->mUpperLim_dim[i]; }
    for( i = 0; i < 2; ++i ) { lb[i+3] = this->mLowerLim_e; ub[i+3] = this->mUpperLim_e; }
    for( i = 0; i < 3; ++i ) { lb[i+5] = this->mLowerLim_trans[i]; ub[i+5] = this->mUpperLim_trans[i]; }
    for( i = 0; i < 3; ++i ) { lb[i+8] = this->mLowerLim_rot[i]; ub[i+8] = this->mUpperLim_rot[i]; }
    lb[11] = mLowerLim_tamp; ub[11] = mUpperLim_tamp;
    switch( _type ) {
    case SQ_FX_RADIAL: {
      ret = dlevmar_bc_der( fr_add_t,
			    Jr_add_t,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );
 
    } break;

    case SQ_FX_ICHIM: {
      ret = dlevmar_bc_der( fi_add_t,
			    Ji_add_t,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );

    } break;
      
    case SQ_FX_SOLINA: {
      
      ret = dlevmar_bc_der( fs_add_t,
			    Js_add_t,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );
    } break;
      
    case SQ_FX_CHEVALIER: {
      
      ret = dlevmar_bc_der( fc_add_t,
			    Jc_add_t,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );
    } break;
      
    case SQ_FX_5: {
      
      ret = dlevmar_bc_der( f5_add_t,
			    J5_add_t,
			    p, y, m, n,
			    lb, ub,
			    NULL,
			    1000,
			    opts, info,
			    NULL, NULL, (void*)&data );
    } break;
      
    case SQ_FX_6: {
      
      ret = dlevmar_bc_der( f6_add_t,
			    J6_add_t,
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
    _out.tamp = p[11];
    _out.type = TAMPERED;
    
    // Return status and error
    double eg, er;
    get_error( _out, this->cloud_, eg, er, _error );
    
    // If stopped by invalid (TODO: Add other reasons)
    if( info[6] == 7 ) {
	return false;
    } else {
	return true;
    }
}


/**
 * @function error_metric
 * @brief Calculates the error 
 */
template<typename PointT>
void SQ_fitter_t<PointT>::get_error( SQ_parameters _par,
				     const PointCloudPtr &_cloud,
				     double &_errA,
				     double &_errB,
				     double &_errC ) {
  return error_metric_t<PointT>( _par, _cloud, _errA, _errB, _errC );

}


