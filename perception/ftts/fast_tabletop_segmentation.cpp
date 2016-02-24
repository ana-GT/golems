
#include "fast_tabletop_segmentation.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>

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
  mClusterMinSize(1000),
  mClusterDistThreshold(0.015f),
  mMinZ(0.35),
  mMaxZ(1.4),
  mThresh_dist2Table(0.015),
  mThresh_smallestBBdim(0.015),
  mEcc( new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>() ){  
  srand( time(NULL) );
  
  mNe.setNormalEstimationMethod( mNe.COVARIANCE_MATRIX );
  mNe.setMaxDepthChangeFactor( 0.02f );
  mNe.setNormalSmoothingSize( 20.0f );

  typename pcl::PlaneCoefficientComparator<PointT, pcl::Normal>::Ptr plane_compare (new pcl::PlaneCoefficientComparator<PointT, pcl::Normal>());
  plane_compare->setAngularThreshold (3.1416/180.0*2.0);
  plane_compare->setDistanceThreshold (0.005, true);
  mMps.setComparator (plane_compare);


  typename pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare( new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>() );
  refinement_compare->setDistanceThreshold( 0.01, true );
  mMps.setRefinementComparator( refinement_compare );
  
  mMps.setMinInliers( mMinPlaneInliers );
  mMps.setAngularThreshold(3.1416/180.0*2.0);
  mMps.setDistanceThreshold(0.01);
  mMps.setProjectPoints(false);
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
  
  int n = 1; if( mClusters.size() > n ) { n = mClusters.size(); }
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


  for( k =0; k < mPlaneIndices.indices.size();
       k++ ) {

    if( _showSegmentation ) {
       data_[3*mPlaneIndices.indices[k] + 0] = 255;
       data_[3*mPlaneIndices.indices[k] + 1] = 255;
       data_[3*mPlaneIndices.indices[k] + 2] = 0;
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
  int planeIndex; int planeSize; int table_label;
  bool tableDetected = false;

  // If table detected
    planeIndex = -1; planeSize = 0;
  if( inlier_indices.size() > 0 ) {
    for( int i = 0; i < inlier_indices.size(); ++i ) {
      if( regions[i].getCentroid()(2) > mMaxZ ) { continue; }
      if( inlier_indices[i].indices.size() > planeSize ) {
	planeIndex = i; planeSize = inlier_indices[planeIndex].indices.size();
      }
    } // end for
  }

  if( planeIndex >= 0 ) { tableDetected = true; }

    float a, b, c, d;
  if( tableDetected ) {
    
    mPlaneIndices = inlier_indices[planeIndex];
    
    table_label = labels->points[inlier_indices[planeIndex].indices[0]].label;

    // Take points below the table and too far out of consideration

    a = model_coefficients[planeIndex].values[0];
    b = model_coefficients[planeIndex].values[1];
    c = model_coefficients[planeIndex].values[2];
    d = model_coefficients[planeIndex].values[3];
    
    // Check if the normal is pointing on the right direction 
    // (for details, see notes on my logbook on Friday, January 22nd, 2016)
    PointT p = _cloud->points[mPlaneIndices.indices[0]];
    if( a*(p.x) + b*(p.y) + c*(p.z) > 0 ) { a = -a; b = -b; c = -c; d = -d; }
    
    pcl::PointIndices outPoints;
    typename pcl::PointCloud<PointT>::const_iterator it;
    int i = 0;
    for( it = _cloud->begin(); it != _cloud->end(); ++it, ++i ) {
      
      if( (*it).z < mMinZ || (*it).z > mMaxZ ) {
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
/*
  // Get clean pointcloud
    pcl::PointIndices::Ptr pc(new pcl::PointIndices() ); 
    pc->indices = outPoints.indices;
    pc->indices.insert( pc->indices.end(), 
			mPlaneIndices.indices.begin(), 
			mPlaneIndices.indices.end() );
    // Get in points
    pcl::ExtractIndices<PointT> extract;
    CloudPtr output( new Cloud() );
    extract.setInputCloud( _cloud );
    extract.setIndices( pc );
    extract.setNegative( true );
    extract.filter( *output );
    pcl::io::savePCDFileASCII("output.pcd", *output );
*/
  } // end table detected

  // 4. Compute clusters
  LabelCloudPtr output_labels(new LabelCloud ());

  std::vector<bool> plane_labels;
  plane_labels.resize ( label_indices.size (), false);  

  if( tableDetected ) {
    
    // Plane labels: Exlucde table:
    plane_labels[table_label] = true;
    // Last label added
    plane_labels[label_indices.size() -1] = true;

  }
  mEcc->setInputCloud (_cloud);
  mEcc->setLabels (labels);
  mEcc->setExcludeLabels (plane_labels);
  mEcc->setDistanceThreshold (mClusterDistThreshold, false);

  pcl::PointCloud<pcl::Label> euclidean_labels;
  std::vector<pcl::PointIndices> euclidean_label_indices;
  pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGBA,pcl::Label> euclidean_segmentation (mEcc);
  euclidean_segmentation.setInputCloud (_cloud);
  euclidean_segmentation.segment ( *output_labels, euclidean_label_indices);

  mClusters.clear();

  mClustersIndices.clear();

  pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
  PointT min_OBB, max_OBB, pos_OBB; Eigen::Matrix3f rot_OBB;
  double dx, dy, dz; 
  double smallestBBdim;
  Eigen::Vector3f mc;
  float dist2table;

  for (size_t i = 0; i < euclidean_label_indices.size (); i++) {
    if (euclidean_label_indices[i].indices.size () > mClusterMinSize) {
      CloudPtr cluster (new Cloud ());
      pcl::copyPointCloud ( *_cloud,
			    euclidean_label_indices[i].indices,
			    *cluster );

      // Check the bounding boxes of these clusters. If one looks too flat, then it is likely spurious points from the table so leave them out
      feature_extractor.setInputCloud(cluster);
      feature_extractor.compute();
      feature_extractor.getOBB( min_OBB, max_OBB, pos_OBB, rot_OBB );
      dx = fabs(max_OBB.x - min_OBB.x);
      dy = fabs(max_OBB.y - min_OBB.y);
      dz = fabs(max_OBB.z - min_OBB.z);
      if( dx <= dy && dx <= dz ) { smallestBBdim = dx; }
      else if( dy <= dx && dy <= dz ) { smallestBBdim = dy; }
      else if( dz <= dx && dz <= dy ) { smallestBBdim = dz; }
      feature_extractor.getMassCenter(mc);
      // Distance to table
      dist2table = fabs(a*mc(0) + b*mc(1) + c*mc(2) + d);
      if( dist2table < mThresh_dist2Table || smallestBBdim < mThresh_smallestBBdim ) { }
      else { 
        mClusters.push_back (cluster);
        mClustersIndices.push_back(euclidean_label_indices[i]);     
      }
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
