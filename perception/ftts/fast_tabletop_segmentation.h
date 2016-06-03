/**
 * @file fast_tabletop_segmentation.h
 * @brief Implements segmentation using multiplanar stuff from Alex Trevor
 * @date January 22nd, 2016
 */
#pragma once

#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/core.hpp>


/**
 * @class Fast_Tabletop_Segmentation
 */
template<typename PointT>
class Fast_Tabletop_Segmentation {

  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef pcl::PointCloud<pcl::Label> LabelCloud;
  typedef typename LabelCloud::Ptr LabelCloudPtr;

 public:
  Fast_Tabletop_Segmentation();
  void setMinMaxFilter( double _xmin, double _xmax, double _ymin,
			double _ymax, double _zmin, double _zmax );
  void process( CloudConstPtr _cloud, bool _showSegmentation = true );
  cv::Mat getRgbImg() { return mRgbImg; }
  cv::Mat getXyzImg() { return mXyzImg; }

  void setMinClusterSize( int _size ) { mClusterMinSize = _size; }
  int getNumClusters() { return mClusters.size(); }
  bool getClusterBB( int _i, int &_xmin, int &_ymin,
		     int &_xmax, int &_ymax );
  CloudPtr getCluster( int _i ) {
    if( _i >= 0 && _i < mClusters.size() ) {return mClusters[_i]; }
    else { return NULL; }
  }
  pcl::PointIndices getClusterIndices( int _i ) {
    return mClustersIndices[_i]; 
  }
  
  CloudPtr getTable() { return mTable; }


private:

  void getSegmentedImg( CloudConstPtr _cloud, bool _showSegmentation );
  std::vector<CloudPtr> getClusters() { return mClusters; }
  
  double mMinZ, mMaxZ;
  double mMinY, mMaxY;
  double mMinX, mMaxX;
  CloudPtr mCloud;
  float mThreshold;
  float mThresh_dist2Table;
  double mThresh_smallestBBdim;
  int mClusterMinSize;
  float mClusterDistThreshold;
  bool mDepthDependent;
  bool mPolygonRefinement;
  int mMinPlaneInliers;
  cv::Mat mRgbImg;
  cv::Mat mXyzImg;
  
  // Operators
  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> mNe;
  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mMps;
  typename pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr mEcc;
  
public:
  pcl::PointCloud<pcl::Normal>::Ptr mNormalCloud;
  pcl::PointIndices mPlaneIndices;
  std::vector<pcl::PointIndices> mClustersIndices;
  std::vector<CloudPtr> mClusters;
  CloudPtr mTable;
  float mTableCoeffs[4];
  std::vector<int> mLabels; // 0: table numbers > 0: label (-): No label
  
};
