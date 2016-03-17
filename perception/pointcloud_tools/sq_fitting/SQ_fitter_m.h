/**
 * 
 */
#pragma once

#include <perception/pointcloud_tools/sq_fitting/SQ_fitter.h>
#include <perception/pointcloud_tools/sq_fitting/SQ_fitter_t.h>
#include <perception/pointcloud_tools/sq_fitting/SQ_fitter_b.h>

enum MULTIPLE_SEARCH_HINT {
  PERPENDICULAR_TO_Z=0,
  CONTAINING_Z=1
};


template<typename PointT>
class SQ_fitter_m : public SQ_fitter<PointT> {

 public:
  SQ_fitter_m();
  ~SQ_fitter_m();

  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::iterator PointCloudIter;

  bool fit( const int &_type,
	    std::vector<int> _part_type,
	    int _num_parts = 2,
	    int _hint_search = PERPENDICULAR_TO_Z,	    
	    const double &_smax = 0.05,
	    const double &_smin = 0.01,
	    const int &_N = 5,
	    const double &_thresh = 0.005 );

  void getPlaneAndDirection( int _hint_search, 
			     double _dim[3], 
			     double _trans[3], 
			     double _rot[3],
			     Eigen::Vector4d &_plane, 
			     Eigen::Vector3d &_direction,
			     double &_num_steps );

  Eigen::Vector4d move_plane( Eigen::Vector4d _plane, 
			      int _i, 
			      double _stepSize );

  void divide_cluster( PointCloudPtr &_cloud_1,
		       PointCloudPtr &_cloud_2,
		       Eigen::Vector4d _plane );
  
  double fit_part( PointCloudPtr _part,
		   int _type, 
		   double _smax,
		   double _smin,
		   int _N,
		   double _thresh,
		   SQ_parameters &_p );

  void getFinalParams( std::vector<SQ_parameters> &_par ) { _par = pars_out_;}

  std::vector<SQ_parameters> pars_out_;

};

#include "perception/pointcloud_tools/sq_fitting/impl/SQ_fitter_m.hpp"
