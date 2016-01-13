
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

template<typename PointT>
class Fast_Tabletop_Segmentation {

  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef pcl::PointCloud<pcl::Label> LabelCloud;
  typedef typename LabelCloud::Ptr LabelCloudPtr;
public:
  Fast_Tabletop_Segmentation();
  void process( CloudPtr _cloud );
  
private:

  CloudPtr mCloud;
  float mThreshold;
  bool mDepthDependent;
  bool mPolygonRefinement;
  int mMinPlaneInliers;
  
  // Operators
  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> mNe;
  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mMps;
  typename pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr mEcc;
  
public:
  pcl::PointCloud<pcl::Normal>::Ptr mNormalCloud;
  pcl::PointIndices mPlaneIndices;
  std::vector<pcl::PointIndices> mClustersIndices;
};

/**
 * @function constructor
 */
template<typename PointT>
Fast_Tabletop_Segmentation<PointT>::Fast_Tabletop_Segmentation() :
  mThreshold( 0.02f ),
  mDepthDependent( true ),
  mPolygonRefinement( false ),
  mNormalCloud( new pcl::PointCloud<pcl::Normal>() ),
  mMinPlaneInliers(5000),
  mEcc( new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>() ){  

  mNe.setNormalEstimationMethod( mNe.COVARIANCE_MATRIX );
  mNe.setMaxDepthChangeFactor( 0.02f );
  mNe.setNormalSmoothingSize( 20.0f );


  typename pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare( new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>() );
  refinement_compare->setDistanceThreshold( mThreshold, mDepthDependent );
  
  mMps.setMinInliers( mMinPlaneInliers );
  mMps.setAngularThreshold(3.1416/180.0*2.0);
  mMps.setDistanceThreshold(0.02);
  mMps.setProjectPoints(true);
  mMps.setRefinementComparator( refinement_compare );

  
}

/**
 * @function process
 */
template<typename PointT>
void Fast_Tabletop_Segmentation<PointT>::process( CloudPtr _cloud ) {

  // 1. Estimate normals
  mNe.setInputCloud( _cloud );
  mNe.compute( *mNormalCloud );

  // 2. Estimate planes
  std::vector<pcl::PlanarRegion<PointT>,
	      Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;

  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels( new pcl::PointCloud<pcl::Label>() );
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  
  mMps.setInputNormals( mNormalCloud );
  mMps.setInputCloud( _cloud );
  mMps.segmentAndRefine( regions,
			 model_coefficients,
			 inlier_indices,
			 labels,
			 label_indices,
			 boundary_indices );

  
  // 3. Get the biggest region and let it be the table plane
  bool foundPlane = false;
  int planeIndex; int planeSize;
  
  if( inlier_indices.size() == 0 ) { foundPlane = false; }
  else {
    foundPlane = true;
    planeIndex = 0; planeSize = inlier_indices[planeIndex].indices.size();
    
    for( int i = 1; i < inlier_indices.size(); ++i ) {
      if( inlier_indices[i].indices.size() > planeSize ) {
	planeIndex = i; planeSize = inlier_indices[planeIndex].indices.size();
      }
    } // end for

    mPlaneIndices = inlier_indices[planeIndex];
  } // end else

  // 4. Compute clusters
  printf("Input to ecc, labels size: %d. Label indices: %d \n",
	 labels->points.size(),
	 label_indices.size() );

  std::vector<CloudPtr> output_clusters;
  std::vector<pcl::PointIndices> output_cluster_indices;
  LabelCloudPtr output_labels(new LabelCloud ());

  std::vector<bool> plane_labels;
  plane_labels.resize ( label_indices.size (), false);
    
  if( regions.size () > 0) {
    
    for (size_t i = 0; i < label_indices.size (); i++) {
      if( label_indices[i].indices.size () > mMinPlaneInliers ) {
	plane_labels[i] = true;
      }
    }  
  }
    
  mEcc->setInputCloud (_cloud);
  mEcc->setLabels (labels);
  mEcc->setExcludeLabels (plane_labels);
  mEcc->setDistanceThreshold (0.01f, false);
      
  pcl::PointCloud<pcl::Label> euclidean_labels;
  std::vector<pcl::PointIndices> euclidean_label_indices;
  pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGBA,pcl::Label> euclidean_segmentation (mEcc);
  euclidean_segmentation.setInputCloud (_cloud);
  euclidean_segmentation.segment ( *output_labels, euclidean_label_indices);

  for (size_t i = 0; i < euclidean_label_indices.size (); i++) {
    if (euclidean_label_indices[i].indices.size () > 1000) {
      CloudPtr cluster (new Cloud ());
      pcl::copyPointCloud ( *_cloud,
			    euclidean_label_indices[i].indices,
			    *cluster );
      
      output_clusters.push_back (cluster);
      output_cluster_indices.push_back(euclidean_label_indices[i]);     
    }    
  }
  
  mClustersIndices = output_cluster_indices;
  printf("Number of clusters: %d \n", mClustersIndices.size());
}


/**
 * @function main
 */
int main( int argc, char* argv[] ) {


  srand( time(NULL) );
  Fast_Tabletop_Segmentation<pcl::PointXYZRGBA> ftts;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA>() );
  if( pcl::io::loadPCDFile<pcl::PointXYZRGBA>( argv[1], *cloud ) == -1 ) {
    printf("Error loading pcd %s \n", argv[1]);
    return 1;
  }

  double dt; clock_t ts, tf;
  printf("Start \n");
  ts = clock();
  ftts.process( cloud );
  tf = clock();
  dt = (tf - ts)/ (double)CLOCKS_PER_SEC;
  printf("Done in %f seconds.\n", dt);
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  unsigned char red [6] = {0, 255, 255,   0,   0, 255};
  unsigned char green [6] = {255, 0, 0, 255,   0, 255};
  unsigned char blue [6] = {255, 255, 0,   0, 255,   0};

  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr showCloud( new pcl::PointCloud<pcl::PointXYZRGBA>() );
  *showCloud = *cloud;
  int k;
  for( int i = 0; i < ftts.mPlaneIndices.indices.size(); ++i ) {
    k = ftts.mPlaneIndices.indices[i];
    showCloud->points[k].r = ( showCloud->points[k].r + red[0] ) / 2;
    showCloud->points[k].g = ( showCloud->points[k].g + green[0] ) / 2;
    showCloud->points[k].b = ( showCloud->points[k].b + blue[0] ) / 2;
  }

  unsigned char ri, gi, bi;
  for( int i = 0; i < ftts.mClustersIndices.size(); ++i ) {
    ri = rand() % 255;
    gi = rand() % 255;
    bi = rand() % 255;
    for( int j = 0; j < ftts.mClustersIndices[i].indices.size(); ++j ) { 
      k = ftts.mClustersIndices[i].indices[j];
      showCloud->points[k].r = ri; //( showCloud->points[k].r + ri ) / 2;
      showCloud->points[k].g = gi; //( showCloud->points[k].g + gi ) / 2;
      showCloud->points[k].b = bi; //( showCloud->points[k].b + bi ) / 2;
    }
  }
  
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(showCloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (showCloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

   while (!viewer->wasStopped ())
     {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
     }
   
   
   return 0;
}
 
