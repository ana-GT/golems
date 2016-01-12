

#include "fast_tabletop_segmentation.h"

template<typename PointT> 
Fast_Tabletop_Segmentation<PointT>::Fast_Tabletop_Segmentation() :
  mNe( new pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>() ),
  mNe_output_normals(0),
  mMps( pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label>() ),
  mMin_plane_inliers(1000),
  mPlane_ang_threshold(2.0*M_PI/180.0),
  mPlane_dist_threshold(0.02)
  {

    // Set normal estimator parameters
    mNe->setNormalEstimationMethod( mNe->COVARIANCE_MATRIX );
    mNe->setMaxDepthChangeFactor( 0.02f );
    mNe->setNormalSmoothingSize( 20.0f );

    // Set up plane segmentation
    mMps.setMinInliers( mMin_plane_inliers );
    mMps.setAngularThreshold( mPlane_ang_threshold );
    mMps.setDistanceThreshold( mPlane_dist_threshold );
    mMps.setProjectPoints(true);

    pcl::PlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal>::Ptr plane_compare (new pcl::PlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal>());
    plane_compare->setAngularThreshold (2.0*M_PI/180.0);
    plane_compare->setDistanceThreshold (0.01, true);
    mMps.setComparator( plane_compare );

    pcl::PlaneRefinementComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::Ptr refine_compare (new pcl::PlaneRefinementComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> ());
    refine_compare->setDistanceThreshold (0.005, false);//0.01, true//0.0025
    mMps.setRefinementComparator( refine_compare );
    
}


template<typename PointT> 
void Fast_Tabletop_Segmentation<PointT>::compute( CloudConstPtr _cloud ) {

  computeNormals( _cloud );
  computePlanes( _cloud, mNe_output_normals );
  computeClusters();
  printf("Computed \n");

}

template<typename PointT> 
void Fast_Tabletop_Segmentation<PointT>::computeNormals( CloudConstPtr _cloud) {

  mNe->setInputCloud( _cloud );
  NormalCloudPtr normals( new NormalCloud() );
  
  mNe->compute(*normals);
  mNe_output_normals = normals;

}

template<typename PointT> 
void Fast_Tabletop_Segmentation<PointT>::computePlanes( CloudConstPtr _cloud,
							NormalCloudPtr _normals ) {
  
  mMps.setInputNormals( _normals );
  mMps.setInputCloud( _cloud );
  
  mMps_output_labels = LabelCloudPtr(new LabelCloud ());
  mMps_output_regions = std::vector<pcl::PlanarRegion<PointT>, 
				    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >();

  mMps.segmentAndRefine( (mMps_output_regions),
			  (mMps_output_model_coefficients),
			  (mMps_output_inlier_indices),
			  (mMps_output_labels),
			  (mMps_output_label_indices),
			  (mMps_output_boundary_indices) );
}

template<typename PointT> 
void Fast_Tabletop_Segmentation<PointT>::computeClusters() {

}


//Instantiate
template class Fast_Tabletop_Segmentation<pcl::PointXYZRGBA>;
