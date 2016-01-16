
#include "fast_tabletop_segmentation.h"

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

  srand( time(NULL) );
  
  mNe.setNormalEstimationMethod( mNe.COVARIANCE_MATRIX );
  mNe.setMaxDepthChangeFactor( 0.02f );
  mNe.setNormalSmoothingSize( 20.0f );

  typename pcl::PlaneCoefficientComparator<PointT, pcl::Normal>::Ptr plane_compare (new pcl::PlaneCoefficientComparator<PointT, pcl::Normal>());
  plane_compare->setAngularThreshold (3.1416/180.0*2.0);
  plane_compare->setDistanceThreshold (0.01, true);
  mMps.setComparator (plane_compare);


  typename pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare( new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>() );
  refinement_compare->setDistanceThreshold( 0.0075, false );
  mMps.setRefinementComparator( refinement_compare );
  
  mMps.setMinInliers( mMinPlaneInliers );
  mMps.setAngularThreshold(3.1416/180.0*2.0);
  mMps.setDistanceThreshold(0.02);
  mMps.setProjectPoints(true);
  mMps.setRefinementComparator( refinement_compare );
}


template<typename PointT>
void Fast_Tabletop_Segmentation<PointT>::getSegmentedImg( CloudConstPtr _cloud,
							  bool _showSegmentation ) {
  
  // Store pointcloud in image
  int data_size_ = _cloud->width * _cloud->height*3;
  unsigned char* data_ = new unsigned char[data_size_];
  float* float_data_ = new float[data_size_];
  int ind = 0; int k;
  
  int n = mClusters.size();
  uchar ri[n],gi[n],bi[n];
  for( int i = 0; i <= n; ++i ) {
    ri[i] = rand() % 255;
    gi[i] = rand() % 255;
    bi[i] = rand() % 255;
    }

  typename pcl::PointCloud<PointT>::const_iterator it;
  for( k =0, it = _cloud->begin();
       it != _cloud->end(); ++it, k++ ) {

    if( _showSegmentation ) {
    
      if( mLabels[k] > 0 ) {
	float_data_[ind] = (*it).x; data_[ind++] = ( (*it).b + bi[mLabels[k]] ) / 2; 
	float_data_[ind] = (*it).y; data_[ind++] = ( (*it).g + gi[mLabels[k]] ) / 2; 
	float_data_[ind] = (*it).z; data_[ind++] = ( (*it).r + ri[mLabels[k]] ) / 2; 
      } else {
	float_data_[ind] = (*it).x; data_[ind++] = (*it).b; 
	float_data_[ind] = (*it).y; data_[ind++] = (*it).g; 
	float_data_[ind] = (*it).z; data_[ind++] = (*it).r;     
      }
    } else {
      float_data_[ind] = (*it).x; data_[ind++] = (*it).b; 
      float_data_[ind] = (*it).y; data_[ind++] = (*it).g; 
      float_data_[ind] = (*it).z; data_[ind++] = (*it).r;          
    }
    
  } // for

  
  // Store RGB image
  mRgbImg = cv::Mat( _cloud->height, _cloud->width, CV_8UC3, data_ );
  mXyzImg = cv::Mat( _cloud->height, _cloud->width, CV_32FC3, float_data_ );
}

/**
 * @function getBoundingBox
 */
template<typename PointT>
bool Fast_Tabletop_Segmentation<PointT>::getClusterBB( int _index,
						       int &_xmin, int &_ymin,
						       int &_xmax, int &_ymax ) {

  // From cluster indices
  int width = 640; int height = 480;
  int x, y;
  _xmin = width; _ymin = height;
  _xmax = 0; _ymax = 0;
  
  for( int j = 0; j < mClustersIndices[_index].indices.size();  ++j ) {
    y = mClustersIndices[_index].indices[j] / width;
    x = mClustersIndices[_index].indices[j] % width;

    if( x < _xmin ) { _xmin = x; }
    if( x > _xmax ) { _xmax = x; }
    if( y < _ymin ) { _ymin = y; }
    if( y > _ymax ) { _ymax = y; }    
  }
}

/**
 * @function process
 */
template<typename PointT>
void Fast_Tabletop_Segmentation<PointT>::process( CloudConstPtr _cloud,
						  bool _showSegmentation ) {


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
  int planeIndex; int planeSize;

  
  // If table detected
  if( inlier_indices.size() > 0 ) {
    planeIndex = 0; planeSize = inlier_indices[planeIndex].indices.size();
    for( int i = 1; i < inlier_indices.size(); ++i ) {
      if( inlier_indices[i].indices.size() > planeSize ) {
	planeIndex = i; planeSize = inlier_indices[planeIndex].indices.size();
      }
    } // end for
    
    mPlaneIndices = inlier_indices[planeIndex];

  ////////////////////////////////////
  float a, b, c, d;
  a = model_coefficients[planeIndex].values[0];
  b = model_coefficients[planeIndex].values[1];
  c = model_coefficients[planeIndex].values[2];
  d = model_coefficients[planeIndex].values[3];

  pcl::PointIndices outPoints;
  typename pcl::PointCloud<PointT>::const_iterator it;
  int i = 0;
  for( it = _cloud->begin(); it != _cloud->end(); ++it, ++i ) {
   
      if( (*it).z < 0.35 || (*it).z > 1.4 ) {
	outPoints.indices.push_back(i);
	labels->points[i].label = label_indices.size();
      }
      // Points below the table
      else if( a*((*it).x) + b*((*it).y) + c*((*it).z) + d < -0.005 ) {
	outPoints.indices.push_back(i);
	labels->points[i].label = label_indices.size();
      }    
  }
  label_indices.push_back( outPoints );
  ////////////////////////////

    
  } // end table detected
  
  // 4. Compute clusters
  LabelCloudPtr output_labels(new LabelCloud ());

  std::vector<bool> plane_labels;
  plane_labels.resize ( label_indices.size (), false);  

  if( regions.size () > 0) {
    
    for (size_t i = 0; i < label_indices.size (); i++) {
      if( label_indices[i].indices.size () > mMinPlaneInliers
	  || i == label_indices.size() - 1 ) {
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

  mClusters.clear();
  mClustersIndices.clear();
  for (size_t i = 0; i < euclidean_label_indices.size (); i++) {
    if (euclidean_label_indices[i].indices.size () > 1000) {
      CloudPtr cluster (new Cloud ());
      pcl::copyPointCloud ( *_cloud,
			    euclidean_label_indices[i].indices,
			    *cluster );

      mClusters.push_back (cluster);
      mClustersIndices.push_back(euclidean_label_indices[i]);     
    }    
  }

  // If segmented should show
  mLabels.clear();
  mLabels.resize(_cloud->points.size(), -1);
  
  int k;
  for( int i = 0; i < mClustersIndices.size(); ++i ) {
    for( int j = 0; j < mClustersIndices[i].indices.size(); ++j ) {
      k = mClustersIndices[i].indices[j];
      mLabels[k] = i+1;
    }
    
  }

  this->getSegmentedImg(_cloud, _showSegmentation );

}



// Instantiate
template class Fast_Tabletop_Segmentation<pcl::PointXYZRGBA>;
