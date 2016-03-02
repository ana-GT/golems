
/**
 * @function SQ_fitter
 * @brief Constructor. Create initial pointers for cloud and its normalized version
 */
template<typename PointT>
SQ_fitter_t<PointT>::SQ_fitter_t() :
  cloud_( new pcl::PointCloud<PointT>() ),
  mGotInitApprox(false) {
  
  int i;
  for( i = 0; i < 3; ++i ) { mLowerLim_dim[i] = 0.01; mUpperLim_dim[i] = 0.15; }
  mLowerLim_e = 0.1;  mUpperLim_e = 1.9;
  for( i = 0; i < 3; ++i ) { mLowerLim_trans[i] = -2.0; mUpperLim_trans[i] = 2.0; }
  for( i = 0; i < 3; ++i ) { mLowerLim_rot[i] = -M_PI; mUpperLim_rot[i] = M_PI; }
 
  mLowerLim_tamp = -1.0; mUpperLim_tamp = 1.0;
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
  smax_ = _smax;
  smin_ = _smin;
  N_ = _N;
  thresh_ = _thresh;
  
  double ds; double error_i; double error_i_1;
  double s_i; bool fitted; 
  SQ_parameters par_i, par_i_1;
  
  if( N_ == 1 ) { ds = 0; }
  else { ds = (smax_ - smin_)/(double)(N_-1); }

  
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
   
  // 1.0 Set tampering start to 1 (0: All tampering, 1: No tamp)
  par_in_.tamp = 0.1;
  par_in_.type = TAMPERED;

  // 1.1. Set e1 and e2 to middle value in range
  par_in_.e[0] = 0.5; par_in_.e[1] = 1.0;

  // Update limits according to this data, up to no more than original guess
  for( int i = 0; i < 3; ++i ) { mUpperLim_dim[i] = par_in_.dim[i]; }

  // Run loop
  par_i = par_in_;
  double eg, er;
  this->get_error( par_i, cloud_, eg, er, error_i );
  fitted = false;

  ///
  for( int i = 0; i < N_; ++i ) { 

    s_i = smax_ - (i)*ds;
    par_i_1 = par_i;
    error_i_1 = error_i;

    PointCloudPtr cloud_i( new pcl::PointCloud<PointT>() );
    if( N_ == 1 ) { cloud_i = cloud_; }
    else { downsampling<PointT>( cloud_, s_i, cloud_i ); }

    if( cloud_i->points.size() < 12 ) { continue; }

    minimize_tampering( cloud_i,
			par_i_1,
			par_i,
			error_i );
    
    // [CONDITION]
    double de = (error_i_1 - error_i);
    final_error_ = error_i;
    if( fabs(de) < thresh_ ) {
      fitted = true;
      break;
    } 
    
  }
 
  par_out_ = par_i;
  printf( "Dim: %f %f %f E: %f %f Tamp: %f \n", par_out_.dim[0], par_out_.dim[1], par_out_.dim[2],
	  par_out_.e[0], par_out_.e[1], par_out_.tamp );

  return fitted;
}

/**
 * @function minimize_tampering
 */
template<typename PointT>
bool SQ_fitter_t<PointT>::minimize( const PointCloudPtr &_cloud,
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

    printf("Initial p: %f %f %f %f %f %f %f %f %f %f %f %f, %f \n",
	   p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11]);

    
    // Set limits
    double ub[m], lb[m];
    for( i = 0; i < 3; ++i ) { lb[i] = mLowerLim_dim[i]; ub[i] = mUpperLim_dim[i]; }
    for( i = 0; i < 2; ++i ) { lb[i+3] = mLowerLim_e; ub[i+3] = mUpperLim_e; }
    for( i = 0; i < 3; ++i ) { lb[i+5] = mLowerLim_trans[i]; ub[i+5] = mUpperLim_trans[i]; }
    for( i = 0; i < 3; ++i ) { lb[i+8] = mLowerLim_rot[i]; ub[i+8] = mUpperLim_rot[i]; }
    lb[11] = mLowerLim_tamp; ub[11] = mUpperLim_tamp;

    ret = dlevmar_bc_der( levmar_tampering_fx,
			  levmar_tampering_jac,
			  p, y, m, n,
			  lb, ub,
			  NULL,
			  1000,
			  opts, info,
			  NULL, NULL, (void*)&data );
 
    // Fill _out
    for( i = 0; i < 3; ++i ) { _out.dim[i] = p[i]; }
    for( i = 0; i < 2; ++i ) { _out.e[i] = p[i+3]; }
    for( i = 0; i < 3; ++i ) { _out.trans[i] = p[i+5]; }
    for( i = 0; i < 3; ++i ) { _out.rot[i] = p[i+8]; }
    _out.tamp = p[11];

    
    // Return status and error
    _error = error_metric_tampering( _out, cloud_ );
    
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

  _errC = 0;
  typename pcl::PointCloud<PointT>::iterator it;
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {
    _errC += error_SQ_tampering( _par, it->x, it->y, it->z );
  }

  _errC =/ (double) _cloud->points.size();
}
