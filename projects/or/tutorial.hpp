/**
 * @file tutorial.hpp
 * @brief Implementation of tutorial class
 */

/**
 * @function ICCVTutorial
 * @brief Constructor
 */
template<typename FeatureType>
ICCVTutorial<FeatureType>::ICCVTutorial(boost::shared_ptr<pcl::Keypoint<PointT, pcl::PointXYZI> >keypoint_detector,
                                        typename pcl::Feature<PointT, FeatureType>::Ptr feature_extractor,
                                        boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointNormal> > surface_reconstructor,
                                        typename pcl::PointCloud<PointT>::ConstPtr source,
                                        typename pcl::PointCloud<PointT>::ConstPtr target )
  : source_keypoints_ (new pcl::PointCloud<pcl::PointXYZI> ())
  , target_keypoints_ (new pcl::PointCloud<pcl::PointXYZI> ())
  , keypoint_detector_ (keypoint_detector)
  , feature_extractor_ (feature_extractor)
  , surface_reconstructor_ (surface_reconstructor)
  , source_ (source)
  , target_ (target)
  , source_segmented_ (new pcl::PointCloud<PointT>)
  , target_segmented_ (new pcl::PointCloud<PointT>)
  , source_transformed_ (new pcl::PointCloud<PointT>)
  , source_registered_ (new pcl::PointCloud<PointT>)
  , source_features_ (new pcl::PointCloud<FeatureType>)
  , target_features_ (new pcl::PointCloud<FeatureType>)
  , correspondences_ (new pcl::Correspondences)
  , show_source2target_ (false)
  , show_target2source_ (false)
  , show_correspondences (false) {

  visualizer_.registerKeyboardCallback( &ICCVTutorial::keyboard_callback, *this, 0 );
  
  // PIPELINE FOR GLOBAL DESCRIPTORS
  // 1. Segment
  printf("Segment source \n");
  //segmentation (source_, source_segmented_);
  *source_segmented_ = *source_;
  printf("Segment target \n");
  segmentation (target_, target_segmented_);  
  // 2. Detect Keypoints
  printf("Detect keypoints source \n");
  detectKeypoints (source_segmented_, source_keypoints_);
  printf("Detect keypoints target \n");
  detectKeypoints (target_segmented_, target_keypoints_);
  /*
  // 3. Extract Descriptors
  printf("Extract descriptors source \n");
  extractDescriptors (source_segmented_, source_keypoints_, source_features_);
  printf("Extract descriptors target \n");
  extractDescriptors (target_segmented_, target_keypoints_, target_features_);
  // 4. Find correspondences
  printf("Correspondences source target  \n");
  findCorrespondences (source_features_, target_features_, source2target_);
  printf("Correspondences target source  \n");
  findCorrespondences (target_features_, source_features_, target2source_);
  // 5. Filter correspondences
  printf("Filter correspondences \n");
  filterCorrespondences ();
  printf("Initial transformation \n");
  determineInitialTransformation ();
  printf("Final transformation \n");
  determineFinalTransformation ();
  // 7. Reconstruct surface
  printf("Reconstruct surface \n");
  reconstructSurface ();
  */
}

/**
 * @brief remove plane and select largest cluster as input object
 * @param input the input point cloud
 * @param segmented the resulting segmented point cloud containing only points of the largest cluster
 */
template<typename FeatureType>
void ICCVTutorial<FeatureType>::segmentation ( typename pcl::PointCloud<PointT>::ConstPtr source, 
					       typename pcl::PointCloud<PointT>::Ptr segmented ) const {

  std::cout << "Segmentation step starts... " << std::flush;
  // fit plane and keep points above that plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);

  seg.setInputCloud (source);
  seg.segment (*inliers, *coefficients);
  
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (source);
  extract.setIndices (inliers);
  extract.setNegative (true);

  extract.filter (*segmented);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);
  std::cout << "OK" << endl;
  
  cout << "\t Clustering points above table..." << std::flush;
  // Euclidean clustering
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (segmented);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> clustering;
  clustering.setClusterTolerance (0.02); // 2cm
  clustering.setMinClusterSize (1000);
  clustering.setMaxClusterSize (250000);
  clustering.setSearchMethod (tree);
  clustering.setInputCloud(segmented);
  clustering.extract (cluster_indices);
  
  if (cluster_indices.size() > 0)//use largest cluster
  {
    std::cout << cluster_indices.size() << " clusters found";
    if (cluster_indices.size() > 1)
      std::cout <<" Using largest one...";
    std::cout << endl;
    typename pcl::IndicesPtr indices (new std::vector<int>);
    *indices = cluster_indices[0].indices;
    extract.setInputCloud (segmented);
    extract.setIndices (indices);
    extract.setNegative (false);

    extract.filter (*segmented);
  }

  std::cout << "... Segmentation is over" << std::endl;
}

/**
 * @function detectKeypoints
 */
template<typename FeatureType>
void ICCVTutorial<FeatureType>::detectKeypoints (typename pcl::PointCloud<PointT>::ConstPtr input, 
						 pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const {

  std::cout << "Keypoint detection..." << std::flush;
  keypoint_detector_->setInputCloud(input);
  //keypoint_detector_->setSearchSurface(input);
  keypoint_detector_->compute(*keypoints);
  std::cout << "OK. keypoints found: " << keypoints->points.size() << endl;
}

/**
 * @function extractDescriptors
 */
template<typename FeatureType>
void ICCVTutorial<FeatureType>::extractDescriptors (typename pcl::PointCloud<PointT>::ConstPtr input, 
						    typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, 
						    typename pcl::PointCloud<FeatureType>::Ptr features ) {

  typename pcl::PointCloud<PointT>::Ptr kpts(new pcl::PointCloud<PointT>);
  kpts->points.resize(keypoints->points.size());
  
  pcl::copyPointCloud(*keypoints, *kpts);
          
  typename pcl::FeatureFromNormals<PointT, pcl::Normal, FeatureType>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<PointT, pcl::Normal, FeatureType> > (feature_extractor_);
  
  feature_extractor_->setSearchSurface(input);
  feature_extractor_->setInputCloud(kpts);
  
  if (feature_from_normals) {
    cout << "normal estimation..." << std::flush;
    typename pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
    normal_estimation.setRadiusSearch (0.01);
    normal_estimation.setInputCloud (input);
    normal_estimation.compute (*normals);
    feature_from_normals->setInputNormals(normals);
    std::cout << "OK" << std::endl;
  }

  std::cout << "Descriptor extraction..." << std::flush;
  feature_extractor_->compute (*features);
  std::cout << "OK" << std::endl;
}

/**
 * @function findCorrespondences
 */
template<typename FeatureType>
void ICCVTutorial<FeatureType>::findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, 
						     typename pcl::PointCloud<FeatureType>::Ptr target, 
						     std::vector<int>& correspondences) const {

  std::cout << "Correspondence assignment..." << std::flush;
  correspondences.resize (source->size());
  
  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<FeatureType> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
    correspondences[i] = k_indices[0];
  }
  std::cout << "OK" << std::endl;
}

/**
 * @function filterCorrespondences
 */
template<typename FeatureType>
void ICCVTutorial<FeatureType>::filterCorrespondences ()
{
  cout << "correspondence rejection..." << std::flush;
  std::vector<std::pair<unsigned, unsigned> > correspondences;
  for (unsigned cIdx = 0; cIdx < source2target_.size (); ++cIdx)
    if (target2source_[source2target_[cIdx]] == cIdx)
      correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));
  
  correspondences_->resize (correspondences.size());
  for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
  {
    (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
    (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
  }
  
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
  rejector.setInputSource(source_keypoints_);
  rejector.setInputTarget(target_keypoints_);
  rejector.setInputCorrespondences(correspondences_);
  rejector.getCorrespondences(*correspondences_);
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::determineInitialTransformation ()
{
  cout << "initial alignment..." << std::flush;
  pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);
  
  transformation_estimation->estimateRigidTransformation (*source_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);
  
  pcl::transformPointCloud(*source_segmented_, *source_transformed_, initial_transformation_matrix_);
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::determineFinalTransformation ()
{
  cout << "final registration..." << std::flush;
  pcl::Registration<PointT, PointT>::Ptr registration (new pcl::IterativeClosestPoint<PointT, PointT>);
  registration->setInputSource(source_transformed_);
  //registration->setInputCloud(source_segmented_);
  registration->setInputTarget (target_segmented_);
  registration->setMaxCorrespondenceDistance(0.05);
  registration->setRANSACOutlierRejectionThreshold (0.05);
  registration->setTransformationEpsilon (0.000001);
  registration->setMaximumIterations (1000);
  registration->align(*source_registered_);
  transformation_matrix_ = registration->getFinalTransformation();
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::reconstructSurface ()
{
  cout << "surface reconstruction..." << std::flush;
  // merge the transformed and the target point cloud
  pcl::PointCloud<PointT>::Ptr merged (new pcl::PointCloud<PointT>);
  *merged = *source_transformed_;
  *merged += *target_segmented_;
  
  // apply grid filtering to reduce amount of points as well as to make them uniform distributed
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(merged);
  voxel_grid.setLeafSize(0.002, 0.002, 0.002);
  voxel_grid.setDownsampleAllData(true);
  voxel_grid.filter(*merged);

  pcl::PointCloud<pcl::PointNormal>::Ptr vertices (new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*merged, *vertices);

  pcl::NormalEstimation<PointT, pcl::PointNormal> normal_estimation;
  normal_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  normal_estimation.setRadiusSearch (0.01);
  normal_estimation.setInputCloud (merged);
  normal_estimation.compute (*vertices);
  
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud (vertices);

  surface_reconstructor_->setSearchMethod(tree);
  surface_reconstructor_->setInputCloud(vertices);
  surface_reconstructor_->reconstruct(surface_);
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::run()
{
  visualizer_.spin ();
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if (event.keyUp())
  {
    switch (event.getKeyCode())
    {
      case '1':
        if (!visualizer_.removePointCloud("source_points"))
        {
          visualizer_.addPointCloud(source_, "source_points");
        }
        break;
        
      case '2':
        if (!visualizer_.removePointCloud("target_points"))
        {
          visualizer_.addPointCloud(target_, "target_points");
        }
        break;
      
      case '3':
        if (!visualizer_.removePointCloud("source_segmented"))
        {
          visualizer_.addPointCloud(source_segmented_, "source_segmented");
        }
        break;
        
      case '4':
        if (!visualizer_.removePointCloud("target_segmented"))
        {
          visualizer_.addPointCloud(target_segmented_, "target_segmented");
        }
        break;
        
      case '5':
        if (!visualizer_.removePointCloud("source_keypoints"))
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color (source_keypoints_, 0, 0, 255);
          visualizer_.addPointCloud(source_keypoints_, keypoint_color, "source_keypoints");
        }
        break;
      
      case '6':
        if (!visualizer_.removePointCloud("target_keypoints"))
        {
          //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> keypoint_color (target_keypoints_, "intensity");
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color (target_keypoints_, 255, 0, 0);
          visualizer_.addPointCloud(target_keypoints_, keypoint_color, "target_keypoints");
        }
        break;

      case '7':
        if (!show_source2target_)
          visualizer_.addCorrespondences<pcl::PointXYZI>(source_keypoints_, target_keypoints_, source2target_, "source2target");
        else
          visualizer_.removeCorrespondences("source2target");
          
        show_source2target_ = !show_source2target_;
        break;

      case '8':
        if (!show_target2source_)
          visualizer_.addCorrespondences<pcl::PointXYZI>(target_keypoints_, source_keypoints_, target2source_, "target2source");
        else
          visualizer_.removeCorrespondences("target2source");

        show_target2source_ = !show_target2source_;
        break;
        
      case '9':
        if (!show_correspondences)
          visualizer_.addCorrespondences<pcl::PointXYZI>(source_keypoints_, target_keypoints_, *correspondences_, "correspondences");
        else
          visualizer_.removeCorrespondences("correspondences");
        show_correspondences = !show_correspondences;
        break;
        
      case 'i':
      case 'I':
        if (!visualizer_.removePointCloud("transformed"))
          visualizer_.addPointCloud(source_transformed_, "transformed");
        break;

      case 'r':
      case 'R':
        if (!visualizer_.removePointCloud("registered"))
          visualizer_.addPointCloud(source_registered_, "registered");
        break;
        
      case 't':
      case 'T':
          visualizer_.addPolygonMesh(surface_, "surface");
        break;
    }
  }
}
