/**
 * @brief Fitter for bending deformation
 */

#pragma once

#include <perception/pointcloud_tools/sq_fitting/SQ_fitter.h>

template <typename PointT>
class SQ_fitter_b : public SQ_fitter<PointT> {

 public:
  SQ_fitter_b();
  ~SQ_fitter_b();

  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

  bool fit( const int &_type = SQ_FX_RADIAL, 
	    const double &_smax = 0.05,
	    const double &_smin = 0.01,
	    const int &_N = 5,
	    const double &_thresh = 0.005 );

  
  bool minimize( const int &_type, 
		 const PointCloudPtr &_cloud, 
		 const SQ_parameters &_in,
		 SQ_parameters &_out,
		 double &_error );

  void get_error( SQ_parameters _par,
		  const PointCloudPtr &_cloud,
		  double &_errA, double &_errorB, double &_errorC );


 protected:
  double mLowerLim_k; double mUpperLim_k; // inverse of curvature radius
  double mLowerLim_alpha; double mUpperLim_alpha;
};



#include "perception/pointcloud_tools/sq_fitting/impl/SQ_fitter_b.hpp"
