

template<typename PointT>
SQ_fitter_m<PointT>::SQ_fitter_m() :
  SQ_fitter<PointT>() {

}

template<typename PointT>
SQ_fitter_m<PointT>::~SQ_fitter_m() {

}


template<typename PointT>
bool SQ_fitter_m<PointT>::fit( const int &_type,
		       std::vector<int> _part_type,
		       int _num_parts,
		       int _hint_search,	    
		       const double &_smax,
		       const double &_smin,
		       const int &_N,
		       const double &_thresh ) {

  // So far we only operate for 2 objects

  // Get the bounding box limits
  if( !this->mGotInitApprox ) {
    printf("Get approx here\n");
    this->getBoundingBox( this->cloud_, 
			  this->mInitDim,
			  this->mInitTrans,
			  this->mInitRot );
  }
  printf("Init dim: %f %f %f, trans: %f %f %f, rot: %f %f %f\n",
	 this->mInitDim[0], this->mInitDim[1], this->mInitDim[2],
	 this->mInitTrans[0], this->mInitTrans[1], this->mInitTrans[2],
	 this->mInitRot[0], this->mInitRot[1], this->mInitRot[2]);

  // Get plane equation and direction towards which to move
  Eigen::Vector3d direction;
  Eigen::Vector4d plane, plane_i;
  double max_dist;
  int num_steps;
  double step_size = 0.01;
  getPlaneAndDirection( _hint_search, this->mInitDim, this->mInitTrans, this->mInitRot,
			plane, direction, max_dist );
  num_steps = max_dist*0.8 / step_size; // Say we move only till 80% of the maximum half and up to 1 cm

  // We start with setting the plane according to the hint_search and the initial approximation
  
  // We move the plane, say till a percentage of points is still there
  // or until limit of the bounding box
  double err_0, err_1, err_0_old;

  SQ_parameters p[2];
  std::vector<SQ_parameters> p0s;
  std::vector<SQ_parameters> p1s;

  std::vector<double> e0, e1, et;
  PointCloudPtr part_0( new PointCloud );
  PointCloudPtr part_1( new PointCloud );
  
  for( int i = 0; i < num_steps; ++i ) {

    plane_i = move_plane( plane, i, step_size );
    divide_cluster( part_0, part_1, plane_i );
    printf("[%d] Size part 0: %d size part 1: %d \n", i, part_0->points.size(), part_1->points.size() );
    // Fit
    err_0 = fit_part( part_0, _part_type[0], _smax, _smin, _N, _thresh, p[0] );
    err_1 = fit_part( part_1, _part_type[1], _smax, _smin, _N, _thresh, p[1] );

    char namep0[50]; sprintf(namep0, "orig0_%d.pcd", i );
    char namep1[50]; sprintf(namep1, "orig1_%d.pcd", i );
    pcl::io::savePCDFile( namep0, *part_0 );
    pcl::io::savePCDFile( namep1, *part_1 );

    if( err_0 > err_1 && i == 0 ) { step_size = step_size*-1; }
    
    e0.push_back( err_0 );
    e1.push_back( err_1 );
    et.push_back( err_0*part_0->points.size() + err_1*part_1->points.size() );
    p0s.push_back( p[0] );
    p1s.push_back( p[1] );

    printf("Iter[%d] error 0: %f error 1: %f \n", i, err_0, err_1);
    
  }

  double emin = et[0]; int eind = 0;
    PointCloudPtr output0( new pcl::PointCloud<PointT>() );
    PointCloudPtr output1( new pcl::PointCloud<PointT>() );
    output0 = sampleSQ_uniform<PointT>( p0s[0], true );
    output1 = sampleSQ_uniform<PointT>( p1s[0], true );

    char name0[50]; sprintf(name0,"part0_%d.pcd", 0 );
    char name1[50]; sprintf(name1,"part1_%d.pcd", 0 );
    pcl::io::savePCDFile( name0, *output0 );
    pcl::io::savePCDFile( name1, *output1 );

  for( int i = 1; i < num_steps; ++i ) {
    if( et[i] < emin ) { emin = et[i]; eind = i; }

    // To test, store them both
    PointCloudPtr output0( new pcl::PointCloud<PointT>() );
    PointCloudPtr output1( new pcl::PointCloud<PointT>() );
    output0 = sampleSQ_uniform<PointT>( p0s[i], true );
    output1 = sampleSQ_uniform<PointT>( p1s[i], true );
    char name0[50]; sprintf(name0,"part0_%d.pcd", i );
    char name1[50]; sprintf(name1,"part1_%d.pcd", i );
    pcl::io::savePCDFile( name0, *output0 );
    pcl::io::savePCDFile( name1, *output1 );


  }
  printf("Best apparently: %d \n", eind);
  pars_out_.resize(2);
  pars_out_[0] = p0s[eind];
  pars_out_[1] = p1s[eind];

}

/**
 * @function fit_part
 */
template<typename PointT>
double SQ_fitter_m<PointT>::fit_part( PointCloudPtr _part,
				      int _type, 
				      double _smax,
				      double _smin,
				      int _N,
				      double _thresh,
				      SQ_parameters &_p ) {
  
  // Fit superquadric
  switch( _type ) {
  case REGULAR: {
    //printf("Type: regular \n");
    SQ_fitter<PointT> fitter;
    fitter.setInputCloud( _part );
    fitter.fit( _type, _smax, _smin, _N, _thresh );
    fitter.getFinalParams( _p );
    return fitter.getFinalError();
    break;
  }
  case TAMPERED: {
    //printf("Type: tampered \n");
    SQ_fitter_t<PointT> fitter;
    fitter.setInputCloud( _part );
    fitter.fit( _type, _smax, _smin, _N, _thresh );
    fitter.getFinalParams( _p );
    return fitter.getFinalError();
    break;
  }
  case BENT: {
    //printf("Type: bent \n");
    SQ_fitter_b<PointT> fitter;
    fitter.setInputCloud( _part );
    fitter.fit( _type, _smax, _smin, _N, _thresh );
    fitter.getFinalParams( _p );
    return fitter.getFinalError();
    break;
  }
  }


}

/** Move up from plane (in same direction as N) */
template<typename PointT>
Eigen::Vector4d SQ_fitter_m<PointT>::move_plane( Eigen::Vector4d _plane, 
						 int _i, 
						 double _stepSize ) {
  Eigen::Vector4d new_plane;
  // Put the plane in this point
  new_plane << _plane(0), _plane(1), _plane(2),_plane(3) - _stepSize*_i;
  return new_plane;
}

template<typename PointT>
void SQ_fitter_m<PointT>::divide_cluster( PointCloudPtr &_cloud_1,
					  PointCloudPtr &_cloud_2,
					  Eigen::Vector4d _plane ) {

  // Clear clouds, in case they are not clean
  _cloud_1->points.clear();
  _cloud_2->points.clear();

  for( PointCloudIter it = this->cloud_->begin(); it != this->cloud_->end(); ++it ) {
    // Above
    PointT p = *it;
    if( _plane(0)*(p.x) + _plane(1)*(p.y) + _plane(2)*(p.z) + _plane(3) > 0 ) {
      _cloud_2->points.push_back(*it);
    } else {
      _cloud_1->points.push_back(*it);
    }
  }

  _cloud_2->width = 1; _cloud_2->height = _cloud_2->points.size();
  _cloud_1->width = 1; _cloud_1->height = _cloud_1->points.size();

}

template<typename PointT>
void SQ_fitter_m<PointT>::getPlaneAndDirection( int _hint_search, 
						double _dim[3], 
						double _trans[3], 
						double _rot[3],
						Eigen::Vector4d &_plane, 
						Eigen::Vector3d &_direction,
						double &_maxDist ) {

  // Get the transformation
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd( _rot[2], Eigen::Vector3d::UnitZ() )*
    Eigen::AngleAxisd( _rot[1], Eigen::Vector3d::UnitY() )*
    Eigen::AngleAxisd( _rot[0], Eigen::Vector3d::UnitX() );
  Eigen::Vector3d z_dir = rot.col(2);
  Eigen::Vector3d center( _trans[0], _trans[1], _trans[2] );

  switch( _hint_search ) {
  case PERPENDICULAR_TO_Z : {

    // Start in the middle of the bounding box
    // ax + by +cz + d = 0;
    _direction = z_dir;
    _plane(0) = z_dir(0); _plane(1) = z_dir(1); _plane(2) = z_dir(2);
    _plane(3) = -z_dir.dot(center);
    _maxDist = _dim[2];

  } break;
  case CONTAINING_Z : {
    printf("WEIRD!\n");
  } break;
  
  }


}
