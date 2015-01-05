/**
 * @file tabletop_segmentation.h
 */
#pragma once

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>


/**
 * @class TabletopSegmentor
 */
template<typename PointT>
class TabletopSegmentor {
      
public:

    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;


    /** Constructor */
    TabletopSegmentor(); 
    
    /** Destructor */
    ~TabletopSegmentor() {}
    
    
    //------------------- Complete processing -----
    
    //! Complete processing for new style point cloud
    bool processCloud(const PointCloudConstPtr &_cloud );
    
    int getNumClusters() { return mClusters.size(); }
    PointCloud getCluster( int _ind ) { 
      return mClusters[_ind];	  
    }
    PointCloud getTable() { return mTable_Points; }
    std::vector<double> getTableCoeffs() { return mTableCoeffs; }
    
 private:
    
    //! Min number of inliers for reliable plane detection
    int inlier_threshold_;
    //! Size of downsampling grid before performing plane detection
    double plane_detection_voxel_size_;
    //! Size of downsampling grid before performing clustering
    double clustering_voxel_size_;
    //! Filtering of original point cloud along the z, y, and x axes
    double z_filter_min_, z_filter_max_;
    double y_filter_min_, y_filter_max_;
    double x_filter_min_, x_filter_max_;
    //! Filtering of point cloud in table frame after table detection
    double table_z_filter_min_, table_z_filter_max_;
    //! Min distance between two clusters
    double cluster_distance_;
    //! Min number of points for a cluster
    int min_cluster_size_;
    //! Clouds are transformed into this frame before processing; leave empty if clouds
    //! are to be processed in their original frame
    std::string processing_frame_;
    //! Positive or negative z is closer to the "up" direction in the processing frame?
    double up_direction_;
    //! How much the table gets padded in the horizontal direction
    double table_padding_;
    
    /** Info that will be messages */
    std::vector<PointCloud> mClusters;
    PointCloud mTable_Points;
    PointCloud mTableHull_Points;
    std::vector<double> mTableCoeffs; 
    
    /** Debugging variables */
    PointCloud dDownsampledFilteredCloud;
    PointCloud dTableInliers;
    PointCloud dTableProjected;
   
};

#include "impl/tabletop_segmentation.hpp"
