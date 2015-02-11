#pragma once

#include <iostream>
#include <limits>

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/features/integral_image_normal.h>             // Normal estimation

#include <pcl/segmentation/organized_multi_plane_segmentation.h>       // Planes
#include <pcl/segmentation/edge_aware_plane_comparator.h>

#include <pcl/filters/crop_box.h>                                    // GetCloud

#include "mvobbox/mvobbox.h"

#include <pcl/filters/statistical_outlier_removal.h>            // Noise removal
#include <pcl/filters/extract_indices.h>                         // Downsampling

#include <pcl/kdtree/kdtree_flann.h>                 // Distance for orth planes

#include <pcl/visualization/pcl_visualizer.h>                     // Fancy stuff

#define meshCloudViz 0 
#define meshThroughHoleBox 1

#define boxFixZ 1

#define boxViz 0
#define segmentViz 0

#if segmentViz
// Planes colors:
//                           0    1    2    3    4    5
const unsigned char c[6] = {'R', 'G', 'B', 'Y', 'P', 'C'};
const unsigned char r[6] = {255,   0,   0, 255, 255,   0};
const unsigned char g[6] = {  0, 255,   0, 255,   0, 255};
const unsigned char b[6] = {  0,   0, 255,   0, 255, 255};
#endif



#include <pcl/segmentation/organized_multi_plane_segmentation.h>       // Planes

namespace Eigen
{
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<float, 6, 1> Vector6f;
  typedef Eigen::Matrix<double, 7, 1> Vector7d;
  typedef Eigen::Matrix<float, 7, 1> Vector7f;
}

template<typename PointT>
class ContainerDetection 
{
  // PointCloud of PointT
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;
  
  // Vector of PointT
  typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > PointVector;
  typedef typename PointVector::iterator PointVectorIt;
  typedef typename PointVector::const_iterator PointVectorConstIt;
  
  // Vector of PointCloud
  typedef std::vector<PointCloud> PointCloudVector;
  typedef typename PointCloudVector::iterator PointCloudVectorIt;
  typedef typename PointCloudVector::const_iterator PointCloudVectorConstIt;

  typedef std::vector<PointCloudPtr> PointCloudPtrVector;
  typedef typename PointCloudPtrVector::iterator PointCloudPtrVectorIt;
  //~ typedef typename PointCloudPtrVector::const_iterator PointCloudPtrVectorConstIt;
  
  // PlanarRegion of PointT, Vector of PlanarRegion
  typedef pcl::PlanarRegion<PointT> PlanarRegion;
  typedef std::vector<PlanarRegion, Eigen::aligned_allocator<PlanarRegion> > 
                                                             PlanarRegionVector;
  typedef typename PlanarRegionVector::iterator PlanarRegionVectorIt;
  typedef typename PlanarRegionVector::const_iterator PlanarRegionVectorConstIt;

  /** @struct estim
   *  @brief Holds the info about a candidate for the plane estimation
   *  @var estim::para_dev
   *  Average deviation between the two parallel planes
   *  @var estim::para_dist
   *  Average distance between the two parallel planes
   *  @var estim::perp_dev
   *  Average deviation between the two parallel and the perpendicular planes
   *  @var estim::perp_dist
   *  Total "distance" between the perpendicular plane and the parallel ones
   *  @var estim::indices
   *  Vector holding the indices of the planes
   */
  typedef struct 
  {
    float para_dev;
    float para_dist;
    float perp_dev;
    float perp_dist;
    std::vector<int> indices;
  } estim_t;

  /** @struct box_t
   * @brief Holds the info about a bounding box
   * @var box_t::q_
   * Quaternion used for the orientation
   * @var box_t::t_
   * Vector used for the position
   * @var box_t s_
   * Vector used for the size/dimensions of the box
   */
  typedef struct
  {
    Eigen::Quaternionf q_;
    Eigen::Vector3f t_;
    Eigen::Vector3f s_;
    Eigen::Vector3i z_;
  } box_t;

  public:
    typedef struct
    {
      unsigned int  seg_min_inliers;
      double        seg_ang_thres;
      double        seg_dist_thres;
      double        seg_max_curv;
      
      bool          est_mode_para;
      double        est_para_max_dev;
      double        est_para_min_dist;
      double        est_perp_max_dev;
    } params_t;
    
    ContainerDetection();
    ~ContainerDetection();

    void setInputCloud(const PointCloudPtr &c);
    void setInputCloud(const PointCloudConstPtr &c);
  
    void setParams(const params_t &p);
    void getParams(params_t &p);
    
    int search();
    
    void getCloud(PointCloudPtr c,const int &i=0);
    void getCloud(const Eigen::Vector3d &t_, 
                  const Eigen::Quaterniond &r_,
                  const Eigen::Vector4d &s_, 
                  PointCloudPtr c);
    Eigen::Vector4d getSize(const int &i=0);
    Eigen::Vector3d getPosition(const int &i=0);
    Eigen::Quaterniond getRotation(const int &i=0);
    Eigen::Affine3d getTF(const Eigen::Vector3d &t_,
                          const Eigen::Quaterniond &r_);
    
    std::string getOFFMeshStr(const bool &world=false);
    std::string getOFFMeshStr(const Eigen::Vector4d &s_,
                    const Eigen::Quaterniond &r_=Eigen::Quaterniond::Identity(),
                    const Eigen::Vector3d &t_=Eigen::Vector3d::Zero());
    
    static bool compareCandidates(const estim_t &c1, const estim_t &c2);
  
    // Parameters
    /** @struct params_t
     * @brief Runtime parameters.
     *
     * TODO: Doc parameters
     */

  private:
    params_t _params;
    PointCloudConstPtr _cloud;                                    // Input cloud
    pcl::PointCloud<pcl::Normal>::Ptr _normals;                 // Cloud normals
    PointCloudPtrVector _sp_clouds;
    PlanarRegionVector _sp_regions;
    std::vector<estim_t> _est_cand;
    // Containers
    std::vector<box_t> _boxes;
  
    int segmentPlanes();
    void findParaCandidates(void);
    void findPerpCandidates(void);

};


#include "impl/container_detection.hpp"

