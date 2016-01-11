
#pragma once

#include "fast_segmentation.h"


template <typename PointT>
class Demo {
  
 public:
  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef pcl::PointCloud<pcl::Label> LabelCloud;
  typedef typename LabelCloud::Ptr LabelCloudPtr;
  typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

 protected:
  boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;
  boost::shared_ptr<pcl::visualization::ImageViewer> plane_image_viewer_;
  
  bool updated_;
  bool plane_updated_;
  CloudConstPtr prev_cloud_;
  LabelCloudConstPtr prev_labels_;
  CloudConstPtr prev_plane_cloud_;
  LabelCloudConstPtr prev_plane_labels_;
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > prev_regions_;
    boost::mutex cloud_mutex_;
    boost::mutex plane_cloud_mutex_;
    boost::condition_variable clust_cloud_cond_;
    boost::condition_variable plane_cloud_cond_;

    // Full Planes
    boost::mutex full_planes_mutex_;
    boost::condition_variable full_planes_cv_;
    bool full_planes_updated_;
    CloudConstPtr full_planes_cloud_;
    std::vector<pcl::PointIndices> full_planes_inliers_;

    // Full Clusters
    boost::mutex full_cluster_mutex_;
    boost::condition_variable full_cluster_cv_;
    bool full_cluster_updated_;
    CloudConstPtr full_cluster_cloud_;
    std::vector<pcl::PointIndices> full_clusters_indices_;

  public:
    Demo ()
      : image_viewer_ (new pcl::visualization::ImageViewer ("Segmented Clusters")),
        plane_image_viewer_ (new pcl::visualization::ImageViewer ("Segmented Planes")),
        updated_ (false),
        plane_updated_ (false),
        prev_cloud_ (new Cloud ()),
        prev_labels_ (new LabelCloud ()),
        full_planes_cloud_ (new Cloud ()),
        full_cluster_cloud_ (new Cloud ())
    {
      image_viewer_->setPosition (0, 0);
      plane_image_viewer_->setPosition (640, 0);
    }
    
    void
    planarRegionsCallback (const CloudConstPtr cloud, boost::posix_time::ptime t, std::vector<pcl::ModelCoefficients> models, 
                           std::vector<pcl::PointIndices> inlier_indices, std::vector<pcl::PointIndices> label_indices,
                           std::vector<pcl::PointIndices> boudnary_indices)
    {
      boost::mutex::scoped_lock lock (full_planes_mutex_);
      full_planes_cloud_ = cloud;
      full_planes_inliers_ = inlier_indices;
      full_planes_updated_ = true;
      full_planes_cv_.notify_one ();
    }

    void
    fullClusterCallback (const CloudConstPtr& cloud, boost::posix_time::ptime t,
                         std::vector<pcl::PointIndices> indices)
    {
      boost::mutex::scoped_lock lock (full_cluster_mutex_);
      full_cluster_cloud_ = cloud;
      full_clusters_indices_ = indices;
      full_cluster_updated_ = true;
      full_cluster_cv_.notify_one ();
    }

    void
    clusterLabelsCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr& labels)
    {
      boost::mutex::scoped_lock lock (cloud_mutex_);
      prev_cloud_ = cloud;
      prev_labels_ = labels;
      updated_ = true;
      clust_cloud_cond_.notify_one ();
    }

    void
    planeLabelsCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr& labels)
    {
      boost::mutex::scoped_lock lock (plane_cloud_mutex_);
      prev_plane_cloud_ = cloud;
      prev_plane_labels_ = labels;
      plane_updated_ = true;
      plane_cloud_cond_.notify_one ();
    }
    
    // Displays segmented planes
    void
    spinVisFullPlanes ()
    {
      CloudConstPtr cloud (new Cloud ());
      LabelCloudConstPtr labels (new LabelCloud ());
      std::vector<pcl::PointIndices> inlier_inds;

      {  
        boost::mutex::scoped_lock lock (full_planes_mutex_);
        while (!full_planes_updated_)
        {
          full_planes_cv_.wait (lock);
        }
        full_planes_cloud_.swap (cloud);
        inlier_inds = full_planes_inliers_;
        full_planes_updated_ = false;
      }
      
      CloudPtr color_cloud (new Cloud (*cloud));
      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
      
      for (size_t i = 0; i < inlier_inds.size (); i++)
      {
        for (size_t j = 0; j < inlier_inds[i].indices.size (); j++)
        {
          color_cloud->points[inlier_inds[i].indices[j]].r = (cloud->points[inlier_inds[i].indices[j]].r + red[i%6]) / 2;
          color_cloud->points[inlier_inds[i].indices[j]].g = (cloud->points[inlier_inds[i].indices[j]].g + grn[i%6]) / 2;
          color_cloud->points[inlier_inds[i].indices[j]].b = (cloud->points[inlier_inds[i].indices[j]].b + blu[i%6]) / 2;
        }
      }

      if (color_cloud->points.size () > 200)
        plane_image_viewer_->addRGBImage<PointT>(color_cloud, "label_image", 0.2);
      
      plane_image_viewer_->spinOnce ();
    }

    // Displays Raw Plane Labels
    void
    spinVisPlanes ()
    {
      CloudConstPtr cloud (new Cloud ());
      LabelCloudConstPtr labels (new LabelCloud ());

      {  
        boost::mutex::scoped_lock lock (plane_cloud_mutex_);
        while (!plane_updated_)
        {
          plane_cloud_cond_.wait (lock);
        }
        prev_plane_cloud_.swap (cloud);
        prev_plane_labels_.swap (labels);
        plane_updated_ = false;
      }
      
      CloudPtr color_cloud (new Cloud (*cloud));
      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
      
      for (size_t i = 0; i < cloud->points.size (); i++)
      {
        if (labels->points[i].label == std::numeric_limits<unsigned>::max ())
        {
          // Do nothing
        }
        else
        {
          color_cloud->points[i].r = (cloud->points[i].r + red[labels->points[i].label%6]) / 2;
          color_cloud->points[i].g = (cloud->points[i].g + grn[labels->points[i].label%6]) / 2;
          color_cloud->points[i].b = (cloud->points[i].b + blu[labels->points[i].label%6]) / 2;
        }
        
      } 
      if (color_cloud->points.size () > 200)
        plane_image_viewer_->addRGBImage<PointT>(color_cloud, "label_image", 0.2);
      
      plane_image_viewer_->spinOnce ();
    }

    // Displays Raw Cluster Labels
    void
    spinVisClusters ()
    {
      CloudConstPtr cloud (new Cloud ());
      LabelCloudConstPtr labels (new LabelCloud ());

      {  
        boost::mutex::scoped_lock lock (cloud_mutex_);
        while (!updated_)
        {
          clust_cloud_cond_.wait (lock);
        }
        prev_cloud_.swap (cloud);
        prev_labels_.swap (labels);
        updated_ = false;
      }
      
      CloudPtr color_cloud (new Cloud (*cloud));
      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
      
      for (size_t i = 0; i < cloud->points.size (); i++)
      {
        if (labels->points[i].label == std::numeric_limits<unsigned>::max ())
        {
          // Do nothing
        }
        else
        {
          color_cloud->points[i].r = (cloud->points[i].r + red[labels->points[i].label%6]) / 2;
          color_cloud->points[i].g = (cloud->points[i].g + grn[labels->points[i].label%6]) / 2;
          color_cloud->points[i].b = (cloud->points[i].b + blu[labels->points[i].label%6]) / 2;
        }
        
      } 
      if (color_cloud->points.size () > 200)
        image_viewer_->addRGBImage<PointT>(color_cloud, "label_image", 0.2);
      
      image_viewer_->spinOnce ();
    }

    // Display Full Clusters
    void
    spinVisFullClusters ()
    {
      CloudConstPtr cloud (new Cloud ());
      LabelCloudConstPtr labels (new LabelCloud ());
      std::vector<pcl::PointIndices> cluster_inds;
      
      {  
        boost::mutex::scoped_lock lock (full_cluster_mutex_);
        while (!full_cluster_updated_)
        {
          full_cluster_cv_.wait (lock);
        }
        cloud.swap (full_cluster_cloud_);
        cluster_inds = full_clusters_indices_;
        full_cluster_updated_ = false;
      }

      CloudPtr color_cloud (new Cloud (*cloud));
      unsigned char red [6] = {255,   0,   0, 255, 255,   0};
      unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
      unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

      for (size_t i = 0; i < cluster_inds.size (); i++)
      {
        for (size_t j = 0; j < cluster_inds[i].indices.size (); j++)
        {
          color_cloud->points[cluster_inds[i].indices[j]].r = (cloud->points[cluster_inds[i].indices[j]].r + red[i%6]) / 2;
          color_cloud->points[cluster_inds[i].indices[j]].g = (cloud->points[cluster_inds[i].indices[j]].g + grn[i%6]) / 2;
          color_cloud->points[cluster_inds[i].indices[j]].b = (cloud->points[cluster_inds[i].indices[j]].b + blu[i%6]) / 2;          
        }
      }
      
      if (color_cloud->points.size () > 200)
        image_viewer_->addRGBImage<PointT>(color_cloud, "label_image", 0.2);
      
      image_viewer_->spinOnce ();
    }
};
