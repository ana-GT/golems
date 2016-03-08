/**
 * @file SQ_fitter.h
 */
#pragma once


#include "perception/pointcloud_tools/sq_fitting/SQ_parameters.h"
#include <pcl/io/pcd_io.h>

#include "perception/pointcloud_tools/sq_fitting/evaluated_eqs.h"

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

  virtual bool fit( const int &_type = SQ_FX_RADIAL, 
		    const double &_smax = 0.05,
		    const double &_smin = 0.01,
		    const int &_N = 5,
		    const double &_thresh = 0.005 );

  virtual bool minimize( const int &_type,
			 const PointCloudPtr &_cloud, 
			 const SQ_parameters &_in,
			 SQ_parameters &_out,
			 double &_error );


  virtual void get_error( SQ_parameters _par,
			  const PointCloudPtr &_cloud,
			  double &_errA, double &_errorB, double &_errorC );

  PointCloudPtr getSampledOutput();

  void printResults();
  
  void getFinalParams( SQ_parameters &_par ) { 
    _par = par_out_; 
  }

  double getFinalError() { return final_error_; }


  

 protected:
    
    double final_error_;
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

};


#include "perception/pointcloud_tools/sq_fitting/impl/SQ_fitter.hpp"
