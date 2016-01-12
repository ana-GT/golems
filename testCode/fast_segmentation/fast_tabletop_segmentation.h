/**
 * @file fast_tabletop_segmentation.h
 */

#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>


/**
 * @class Fast_Tabletop_Segmentation
 */
template <typename PointT>
class Fast_Tabletop_Segmentation  {

  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef pcl::PointCloud<pcl::Normal> NormalCloud;
  typedef typename NormalCloud::Ptr NormalCloudPtr;
  typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;
  typedef pcl::PointCloud<pcl::Label> LabelCloud;
  typedef typename LabelCloud::Ptr LabelCloudPtr;
  typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;
  
 public:

  /**< Constructor */
  Fast_Tabletop_Segmentation();

  /**< Compute plane detection and clustering */
  void compute( CloudConstPtr _cloud );

 private:

  // Functions
  void computeNormals( CloudConstPtr _cloud );
  void computePlanes( CloudConstPtr _cloud,
		      NormalCloudPtr _normals );
  void computeClusters();


  // Normal estimation
  typename pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::Ptr mNe;
  // Plane segmentation
  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mMps;
  
  // Parameters
  NormalCloudPtr mNe_output_normals;
  int mMin_plane_inliers;
  int mMin_cluster_inliers;
  double mPlane_dist_threshold;
  double mPlane_ang_threshold;
  
  // Plane 
  std::vector<pcl::PlanarRegion<PointT>, 
    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > mMps_output_regions;
  LabelCloudPtr mMps_output_labels;
  std::vector<pcl::ModelCoefficients> mMps_output_model_coefficients;
  std::vector<pcl::PointIndices>  mMps_output_inlier_indices;
  std::vector<pcl::PointIndices> mMps_output_label_indices;
  std::vector<pcl::PointIndices> mMps_output_boundary_indices;
  
  
};

