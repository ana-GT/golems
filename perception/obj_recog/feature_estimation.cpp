/**
 * @file feature_estimation.cpp
 */
#include <pcl/keypoints/iss_3d.h>
#include "feature_estimation.h"



/* Use NormalEstimation to estimate a cloud's surface normals 
 * Inputs:
 *   input
 *     The input point cloud
 *   radius
 *     The size of the local neighborhood used to estimate the surface
 * Return: A pointer to a SurfaceNormals point cloud
 */
SurfaceNormalsPtr estimateSurfaceNormals( const PointCloudPtr & input,
					  float radius ) {

  pcl::NormalEstimation<PointT, NormalT> normal_estimation;
  normal_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  normal_estimation.setRadiusSearch (radius);
  normal_estimation.setInputCloud (input);
  SurfaceNormalsPtr normals (new SurfaceNormals);
  normal_estimation.compute (*normals);

  return (normals);
}


/* Use SIFTKeypoint to detect a set of keypoints
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 *   min_scale
 *     The smallest scale in the difference-of-Gaussians (DoG) scale-space
 *   nr_octaves
 *     The number of times the scale doubles in the DoG scale-space
 *   nr_scales_per_octave
 *     The number of scales computed for each doubling
 *   min_contrast
 *     The minimum local contrast that must be present for a keypoint to be detected
 * Return: A pointer to a point cloud of keypoints
 */
PointCloudPtr detectKeypoints ( const PointCloudPtr & points,
				const SurfaceNormalsPtr & normals,
				float min_scale,
				int nr_octaves,
				int nr_scales_per_octave,
				float min_contrast ) {

  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
  PointCloudPtr keypoints (new PointCloud);
  
  /*
  printf("Keypoints: min scale: %f, nr octaves: %d nr scales per octave: %d min_contrast: %f \n",
	 min_scale, nr_octaves, nr_scales_per_octave, min_contrast );
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
  sift_detect.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
  sift_detect.setMinimumContrast (min_contrast);
  sift_detect.setInputCloud (points);
  sift_detect.compute (keypoints_temp);
  pcl::copyPointCloud (keypoints_temp, *keypoints);
  */

  // ISS keypoint detector object.
  pcl::ISSKeypoint3D<PointT, PointT> detector;
  detector.setInputCloud(points);
  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
  detector.setSearchMethod(kdtree);
  //double resolution = computeCloudResolution(points);
  double resolution = 0.005;
  // Set the radius of the spherical neighborhood used to compute the scatter matrix.
  detector.setSalientRadius(6 * resolution);
  // Set the radius for the application of the non maxima supression algorithm.
  detector.setNonMaxRadius(4 * resolution);
  // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
  detector.setMinNeighbors(5);
  // Set the upper bound on the ratio between the second and the first eigenvalue.
  detector.setThreshold21(0.975);
  // Set the upper bound on the ratio between the third and the second eigenvalue.
  detector.setThreshold32(0.975);
  // Set the number of prpcessing threads to use. 0 sets it to automatic.
  detector.setNumberOfThreads(4);
  
  detector.compute(*keypoints);
  
  return (keypoints);
}

/* Use FPFHEstimation to compute local feature descriptors around each keypoint
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 *   keypoints
 *     A cloud of keypoints specifying the positions at which the descriptors should be computed
 *   feature_radius
 *     The size of the neighborhood from which the local descriptors will be computed 
 * Return: A pointer to a LocalDescriptors (a cloud of LocalDescriptorT points)
 */
LocalDescriptorsPtr computeLocalDescriptors (const PointCloudPtr & points,
					     const SurfaceNormalsPtr & normals, 
					     const PointCloudPtr & keypoints,
					     float feature_radius) {
  pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
  fpfh_estimation.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  fpfh_estimation.setRadiusSearch (feature_radius);
  fpfh_estimation.setSearchSurface (points);  
  fpfh_estimation.setInputNormals (normals);
  fpfh_estimation.setInputCloud (keypoints);
  LocalDescriptorsPtr local_descriptors (new LocalDescriptors);
  fpfh_estimation.compute (*local_descriptors);

  return (local_descriptors);
}

/* Use VFHEstimation to compute a single global descriptor for the entire input cloud
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 * Return: A pointer to a GlobalDescriptors point cloud (a cloud containing a single GlobalDescriptorT point)
 */
GlobalDescriptorsPtr computeGlobalDescriptor (const PointCloudPtr & points,
					      const SurfaceNormalsPtr & normals) {
  
  pcl::FPFHEstimation<PointT, NormalT, GlobalDescriptorT> fpfh_est;
  fpfh_est.setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  fpfh_est.setInputCloud (points);
  fpfh_est.setInputNormals (normals);
  fpfh_est.setRadiusSearch(0.08); // normal radius=0.04, This radius has to be bigger
  GlobalDescriptorsPtr global_descriptor (new GlobalDescriptors);
  printf("Computing global \n");
  fpfh_est.compute (*global_descriptor);
  
  return (global_descriptor);
}


/* Estimate normals, detect keypoints, and compute local and global descriptors 
 * Return: An ObjectFeatures struct containing all the features
 */
ObjectFeatures computeFeatures (const PointCloudPtr & input ) {
  ObjectFeatures features;
  features.points = input;
  features.normals = estimateSurfaceNormals (input, 0.05);
  features.keypoints = detectKeypoints (input, features.normals, 0.005, 10, 8, 1.5);
  features.local_descriptors = computeLocalDescriptors (input, features.normals, features.keypoints, 0.1);
  features.global_descriptor = computeGlobalDescriptor (input, features.normals);

  return (features);
}