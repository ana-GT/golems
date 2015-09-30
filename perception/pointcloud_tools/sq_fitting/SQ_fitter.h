/**
 * @file SQ_fitter.h
 */
#pragma once


#include "SQ_parameters.h"
#include <pcl/io/pcd_io.h>

#include "evaluated_eqs.h"

/**
 * @class SQ_fitter
 */
template <typename PointT>
class SQ_fitter {

 public:

  SQ_fitter();
  ~SQ_fitter();

  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

  void setInputCloud( const PointCloudPtr &_cloud );
  void setInitialApprox( const Eigen::Isometry3d &_Tsymm,
			 const Eigen::Vector3d &_Bb );
  void getBoundingBox(const PointCloudPtr &_cloud,
		      double _dim[3],
		      double _trans[3],
		      double _rot[3],
		      bool _debug = false );
  bool fit( const int &_type = SQ_FX_RADIAL, 
	    const double &_smax = 0.05,
	    const double &_smin = 0.01,
	    const int &_N = 5,
	    const double &_thresh = 0.005 ); // used to be 0.005

  // UNIQUE TO ORIGINAL FIT
  bool fit_tampering( const double &_smax = 0.05,
		      const double &_smin = 0.01,
		      const int &_N = 5,
		      const double &_thresh = 0.1 );

  bool minimize( const PointCloudPtr &_cloud, 
		 const SQ_parameters &_in,
		 SQ_parameters &_out,
		 double &_error );
  bool minimize_tampering( const PointCloudPtr &_cloud, 
			   const SQ_parameters &_in,
			   SQ_parameters &_out,
			   double &_error );


  bool minimize_levmar( const PointCloudPtr &_cloud, 
			const SQ_parameters &_in,
			SQ_parameters &_out,
			double &_error );


  double error_metric_tampering( SQ_parameters _par,
				 const PointCloudPtr &_cloud );


  void printResults();
  PointCloudPtr getSampledOutput();
  
  void getFinalParams( SQ_parameters &_par ) { 
    _par = par_out_; 
  }


    bool minimize( const int &_type,
		 const PointCloudPtr &_cloud, 
		 const SQ_parameters &_in,
		 SQ_parameters &_out,
		 double &_error );
  

 private:
  SQ_parameters par_in_;
  SQ_parameters par_out_;

  PointCloudPtr cloud_;

  double smax_; /**< Maximum voxel size */
  double smin_; /**< Minimum voxel size */
  int N_; /**< Number of scales */
  double thresh_; /**< Error threshold */
  bool mGotInitApprox;
  double mInitDim[3]; double mInitRot[3]; double mInitTrans[3];
  double mLowerLim_dim[3]; double mUpperLim_dim[3]; 
  double mLowerLim_rot[3]; double mUpperLim_rot[3]; 
  double mLowerLim_trans[3]; double mUpperLim_trans[3]; 
  double mLowerLim_e; double mUpperLim_e; 
  double mLowerLim_tamp; double mUpperLim_tamp;
};


#include "impl/SQ_fitter.hpp"
