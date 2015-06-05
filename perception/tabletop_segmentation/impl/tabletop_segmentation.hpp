/**
* @file tabletop_segmentation.hpp
*/
#pragma once
/**
* @file tabletop_segmentation.hpp
* @brief Based on code from Marius Muja and Matei Ciocarlie
*/
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/surface/convex_hull.h>

// NOTES
// READ WHAT RADU SAID!
//http://www.pcl-users.org/Point-Cloud-processing-in-grabber-callback-td3928466.html
/*************** HELPERS ***********************/

/**
 * @function getClustersFromPointCloud2
 */
template <typename PointT>
void getClustersFromPointCloud2( const pcl::PointCloud<PointT> &_cloud_objects,
				 const std::vector<pcl::PointIndices> &_clusters2,
				 std::vector<pcl::PointCloud<PointT> > &_clusters ) {
  // Resize
  _clusters.resize( _clusters2.size() );
  // Fill the cluster with the indices
  for (size_t i = 0; i < _clusters2.size (); ++i) {
    pcl::PointCloud<PointT> cloud_cluster;
    pcl::copyPointCloud( _cloud_objects, _clusters2[i], cloud_cluster );
    _clusters[i] = cloud_cluster;
  }
}

/**
 * @function getPlaneTransform
 * @brief Assumes plane coefficients are of the form ax+by+cz+d=0, normalized
*/
Eigen::Matrix4d getPlaneTransform ( pcl::ModelCoefficients coeffs,
				    double up_direction,
				    bool flatten_plane ) {
  Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
  if( coeffs.values.size() <= 3 ) {
    std::cout << "[ERROR] Coefficient size less than 3. I will output nonsense values"<<std::endl;
    return tf;
  }
  double a = coeffs.values[0];
  double b = coeffs.values[1];
  double c = coeffs.values[2];
  double d = coeffs.values[3];
  //asume plane coefficients are normalized
  Eigen::Vector3d position(-a*d, -b*d, -c*d);
  Eigen::Vector3d z(a, b, c);
  //if we are flattening the plane, make z just be (0,0,up_direction)
  if(flatten_plane) {
    z << 0, 0, up_direction;
  }
  else {
    //make sure z points "up"
    if ( z.dot( Eigen::Vector3d(0, 0, up_direction) ) < 0) {
      z = -1.0 * z;
    }
  }
  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  Eigen::Vector3d x; x << 1, 0, 0;
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = Eigen::Vector3d(0, 1, 0);
  Eigen::Vector3d y = z.cross(x); y.normalized();
  x = y.cross(z); x.normalized();
  Eigen::Matrix3d rotation;
  rotation.block(0,0,3,1) = x;
  rotation.block(0,1,3,1) = y;
  rotation.block(0,2,3,1) = z;
  Eigen::Quaterniond orientation( rotation );
  tf.block(0,0,3,3) = orientation.matrix();
  tf.block(0,3,3,1) = position;
  return tf;
}

/*********** CLASS FUNCTIONS ***************/
/**
 * @function TabletopSegmentor
 * @brief Constructor
 */
template<typename PointT>
TabletopSegmentor<PointT>::TabletopSegmentor() {
//initialize operational flags
  inlier_threshold_ = 300;
  plane_detection_voxel_size_ = 0.01;
  clustering_voxel_size_ = 0.0015; // 1.5mm

  z_filter_min_ = 0.3; z_filter_max_ = 1.0; 
  y_filter_min_ = -0.6;  y_filter_max_ = 0.6;
  x_filter_min_ = -0.6; x_filter_max_ = 0.6;

  table_obj_height_filter_min_= 0.03;
  table_obj_height_filter_max_= 0.50;
  cluster_distance_ = 0.03;
  min_cluster_size_ = 300;
  processing_frame_ = "";
  up_direction_ = -1.0;
  table_padding_ = 0.0;
}

/**
 * @function set_filter_minMax
 */
template<typename PointT>
void TabletopSegmentor<PointT>::set_filter_minMax( const double &_xmin, const double &_xmax,
						   const double &_ymin, const double &_ymax,
						   const double &_zmin, const double &_zmax ) {

  x_filter_min_ = _xmin; x_filter_max_ = _xmax;
  y_filter_min_ = _ymin;  y_filter_max_ = _ymax;
  z_filter_min_ = _zmin; z_filter_max_ = _zmax;  
}

/**
 * @function processCloud
 */
template<typename PointT>
bool TabletopSegmentor<PointT>::processCloud(const PointCloudConstPtr &_cloud ) {

  // PCL objects
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::VoxelGrid<PointT> grid_, grid_objects_;
  pcl::PassThrough<PointT> pass_;
  pcl::NormalEstimation<PointT, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_;
  pcl::ProjectInliers<PointT> proj_;
  pcl::ConvexHull<PointT> hull_;
  pcl::ExtractPolygonalPrismData<PointT> prism_;
  pcl::EuclideanClusterExtraction<PointT> pcl_cluster_;
  PointCloudPtr table_hull_ptr (new PointCloud);

  /************ STEP 0: Parameter Settings **************/
  // Filtering parameters
  grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
  grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
  grid_.setFilterFieldName ("z");
  grid_.setFilterLimits (z_filter_min_, z_filter_max_);
  grid_.setDownsampleAllData (false);
  grid_objects_.setDownsampleAllData (false);
  normals_tree_ = boost::make_shared<pcl::search::KdTree<PointT> > ();
  clusters_tree_ = boost::make_shared<pcl::search::KdTree<PointT> > ();
  
  // Normal estimation parameters
  n3d_.setKSearch (10);
  n3d_.setSearchMethod (normals_tree_);

  // Table model fitting parameters
  seg_.setDistanceThreshold (0.03);
  seg_.setMaxIterations (10000);
  seg_.setNormalDistanceWeight (0.1);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  //seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  //seg_.setModelType( pcl::SACMODEL_PERPENDICULAR_PLANE );
  //seg_.setEpsAngle( 45*M_PI/180.0 );
  //seg_.setAxis( Eigen::Vector3f(0,1,0) );
  
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setProbability (0.99);
  
  //proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  proj_.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  
  
  // Clustering parameters
  pcl_cluster_.setClusterTolerance (cluster_distance_);
  pcl_cluster_.setMinClusterSize (min_cluster_size_);
  pcl_cluster_.setSearchMethod (clusters_tree_);

  /******** Step 1 : Filter, remove NaNs and downsample ***********/
  // Filter in X, Y and Z directions: output= cloud_filtered_ptr
  pass_.setInputCloud (_cloud);
  pass_.setFilterFieldName ("z");
  pass_.setFilterLimits (z_filter_min_, z_filter_max_);
  PointCloudPtr z_cloud_filtered_ptr (new PointCloud);
  pass_.filter (*z_cloud_filtered_ptr);
  pass_.setInputCloud (z_cloud_filtered_ptr);
  pass_.setFilterFieldName ("y");
  pass_.setFilterLimits (y_filter_min_, y_filter_max_);
  PointCloudPtr y_cloud_filtered_ptr (new PointCloud);
  pass_.filter (*y_cloud_filtered_ptr);
  pass_.setInputCloud (y_cloud_filtered_ptr);
  pass_.setFilterFieldName ("x");
  pass_.setFilterLimits (x_filter_min_, x_filter_max_);
  PointCloudPtr cloud_filtered_ptr (new PointCloud);
  pass_.filter (*cloud_filtered_ptr);
  // Check that the points filtered at least are of a minimum size
  if (cloud_filtered_ptr->points.size() < (unsigned int)min_cluster_size_) {
    printf("\t [ERROR] Filtered cloud only has %d points. \n",
	   (int)cloud_filtered_ptr->points.size() );
    return false;
  }

  pcl::io::savePCDFile( "filtered_cloud.pcd", *cloud_filtered_ptr, true );
  
  // Downsample the filtered cloud: output = cloud_downsampled_ptr
  PointCloudPtr cloud_downsampled_ptr (new PointCloud);
  grid_.setInputCloud (cloud_filtered_ptr);
  grid_.filter (*cloud_downsampled_ptr);
  if (cloud_downsampled_ptr->points.size() < (unsigned int)min_cluster_size_) {
    printf( "\t [ERROR] Downsampled cloud only has %d points \n",
	    (int)cloud_downsampled_ptr->points.size() );
    return false;
  }

  printf( "\t Downsampled cloud only has %d points / %d. \n",
	  (int)cloud_downsampled_ptr->points.size(), (int)cloud_filtered_ptr->points.size() );


  /***************** Step 2 : Estimate normals ******************/
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  n3d_.setInputCloud (cloud_downsampled_ptr);
  n3d_.compute (*cloud_normals_ptr);

  /************* Step 3 : Perform planar segmentation, **********/
  /** if table is not given, otherwise use given table */
  Eigen::Matrix4d table_plane_trans;
  Eigen::Matrix4d table_plane_trans_flat;
  pcl::PointIndices::Ptr table_inliers_ptr (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr table_coefficients_ptr (new pcl::ModelCoefficients);
  seg_.setInputCloud (cloud_downsampled_ptr);
  seg_.setInputNormals (cloud_normals_ptr);
  seg_.segment( *table_inliers_ptr,
		*table_coefficients_ptr );
  // Check the coefficients and inliers are above threshold values
  if (table_coefficients_ptr->values.size () <=3 ) {
    printf( "\t [ERROR] Failed to detect table in scan \n" );
    return false;
  }
  if ( table_inliers_ptr->indices.size() < (unsigned int)inlier_threshold_) {
    printf( "\t [ERROR] Plane detection has %d below min thresh of %d points \n",
	    (int)table_inliers_ptr->indices.size(),
	    inlier_threshold_ );
    return false;
  }
  
  // Store table's plane coefficients
  mTableCoeffs.resize(4);
  for( int i = 0; i < 4; ++i ) { mTableCoeffs[i] = table_coefficients_ptr->values[i]; }

  /********** Step 4 : Project the table inliers on the table *********/
  PointCloudPtr table_projected_ptr (new PointCloud);
  proj_.setInputCloud (cloud_downsampled_ptr);
  proj_.setIndices (table_inliers_ptr);
  proj_.setModelCoefficients (table_coefficients_ptr);
  proj_.filter (*table_projected_ptr);
  // Get the table transformation w.r.t. camera
  table_plane_trans = getPlaneTransform (*table_coefficients_ptr, up_direction_, false);
  mTableTf.matrix() = table_plane_trans;
  // ---[ Estimate the convex hull (not in table frame)
  hull_.setInputCloud (table_projected_ptr);
  hull_.reconstruct (*table_hull_ptr);

  pcl::io::savePCDFile( "tableProjected.pcd", *table_projected_ptr, true );

  // Save points in the hull with some points down
  mTable_Points.points.resize(0);
  for( int i = 0; i < table_hull_ptr->points.size(); ++i ) {
    mTable_Points.points.push_back(table_hull_ptr->points[i]);
  }
  mTable_Points.width = 1; mTable_Points.height = mTable_Points.points.size();
  
  /******* Step 5 : Get the objects on top of the table ******/
  pcl::PointIndices cloud_object_indices;
  prism_.setInputCloud (cloud_filtered_ptr);
  prism_.setInputPlanarHull (table_hull_ptr);
  prism_.setHeightLimits (table_obj_height_filter_min_, table_obj_height_filter_max_);
  prism_.segment (cloud_object_indices);
  PointCloudPtr cloud_objects_ptr (new PointCloud);
  pcl::ExtractIndices<PointT> extract_object_indices;
  extract_object_indices.setInputCloud (cloud_filtered_ptr);
  extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
  extract_object_indices.filter (*cloud_objects_ptr);
  if (cloud_objects_ptr->points.empty ()) {
    std::cout<<"\t [ERROR] No object points on table" << std::endl;
    return false;
  }

  pcl::io::savePCDFile( "cloudObjects.pcd", *cloud_objects_ptr, true );    

  // Downsample the points
  PointCloudPtr cloud_objects_downsampled_ptr (new PointCloud);
  grid_objects_.setInputCloud (cloud_objects_ptr);
  grid_objects_.filter (*cloud_objects_downsampled_ptr);
  printf("Object clouds from %d to %d \n", cloud_objects_ptr->points.size(),
	 cloud_objects_downsampled_ptr->points.size() );
  // Step 6: Split the objects into Euclidean clusters
  std::vector<pcl::PointIndices> clusters2;
  pcl_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
  pcl_cluster_.extract (clusters2);
  mClusterInds.resize( clusters2.size() );
  for( int i = 0; i < clusters2.size(); ++i ) {
    mClusterInds[i] = clusters2[i];
    printf("Cluster %d size: %d \n", i, clusters2[i].indices.size()   );
  }

  mClusters.resize( clusters2.size() );
  int i = 0;
  for( std::vector<pcl::PointIndices>::const_iterator it = 
	 clusters2.begin (); it != clusters2.end (); ++it ) {
    PointCloud cloud_cluster;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      cloud_cluster.points.push_back (cloud_objects_downsampled_ptr->points[*pit]); 
    }
    cloud_cluster.width = cloud_cluster.points.size ();
    cloud_cluster.height = 1;
    cloud_cluster.is_dense = true;
    
    mClusters[i] = cloud_cluster;
    i++;
  }
  
  return true;
}

