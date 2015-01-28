/**
 * @file tutorial.cpp
 */
#include "tutorial.h"

int main (int argc, char ** argv)
{
  if (argc < 6) 
  {
    pcl::console::print_info ("Syntax is: %s <source-pcd-file> <target-pcd-file> <keypoint-method> <descriptor-type> <surface-reconstruction-method>\n", argv[0]);
    pcl::console::print_info ("available <keypoint-methods>: 1 = Sift3D\n");
    pcl::console::print_info ("                              2 = Harris3D\n");
    pcl::console::print_info ("                              3 = Tomasi3D\n");
    pcl::console::print_info ("                              4 = Noble3D\n");
    pcl::console::print_info ("                              5 = Lowe3D\n");
    pcl::console::print_info ("                              6 = Curvature3D\n\n");
    pcl::console::print_info ("available <descriptor-types>: 1 = FPFH\n");
    pcl::console::print_info ("                              2 = SHOTRGB\n");
    pcl::console::print_info ("                              3 = PFH\n");
    pcl::console::print_info ("                              4 = PFHRGB\n\n");
    pcl::console::print_info ("available <surface-methods>:  1 = Greedy Projection\n");
    pcl::console::print_info ("                              2 = Marching Cubes\n");    
    
    return (1);
  }
  pcl::console::print_info ("== MENU ==\n");
  pcl::console::print_info ("1 - show/hide source point cloud\n");
  pcl::console::print_info ("2 - show/hide target point cloud\n");
  pcl::console::print_info ("3 - show/hide segmented source point cloud\n");
  pcl::console::print_info ("4 - show/hide segmented target point cloud\n");
  pcl::console::print_info ("5 - show/hide source key points\n");
  pcl::console::print_info ("6 - show/hide target key points\n");
  pcl::console::print_info ("7 - show/hide source2target correspondences\n");
  pcl::console::print_info ("8 - show/hide target2source correspondences\n");
  pcl::console::print_info ("9 - show/hide consistent correspondences\n");
  pcl::console::print_info ("i - show/hide initial alignment\n");
  pcl::console::print_info ("r - show/hide final registration\n");
  pcl::console::print_info ("t - show/hide triangulation (surface reconstruction)\n");
  pcl::console::print_info ("h - show visualizer options\n");
  pcl::console::print_info ("q - quit\n");
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile (argv[1], *source);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile (argv[2], *target);
  
  int keypoint_type   = atoi (argv[3]);
  int descriptor_type = atoi (argv[4]);
  int surface_type    = atoi (argv[5]);
  
  boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector;
  
  if (keypoint_type == 1)
  {
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
    sift3D->setScales(0.01, 3, 2);
    sift3D->setMinimumContrast(0.0);
    keypoint_detector.reset(sift3D);
  }
  else
  {
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>* harris3D = new pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI> (pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
    harris3D->setNonMaxSupression(true);
    harris3D->setRadius (0.01);
    harris3D->setRadiusSearch (0.01);
    keypoint_detector.reset(harris3D);
    switch (keypoint_type)
    {
      case 2:
        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
      break;

      case 3:
        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::TOMASI);
      break;

      case 4:
        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::NOBLE);
      break;
      
      case 5:
        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::LOWE);
      break;

      case 6:
        harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::CURVATURE);
      break;
      default:
        pcl::console::print_error("unknown key point detection method %d\n expecting values between 1 and 6", keypoint_type);
        exit (1);
        break;
    }
    
  }
  
  boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstruction;
  
  if (surface_type == 1)
  {
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>* gp3 = new pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>;

    // Set the maximum distance between connected points (maximum edge length)
    gp3->setSearchRadius (0.025);

    // Set typical values for the parameters
    gp3->setMu (2.5);
    gp3->setMaximumNearestNeighbors (100);
    gp3->setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3->setMinimumAngle(M_PI/18); // 10 degrees
    gp3->setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3->setNormalConsistency(false);
    surface_reconstruction.reset(gp3);
  }
  else if (surface_type == 2)
  {
    pcl::MarchingCubes<pcl::PointXYZRGBNormal>* mc = new pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal>;
    mc->setIsoLevel(0.001);
    mc->setGridResolution(50, 50, 50);
    surface_reconstruction.reset(mc);
  }
  else
  {
    pcl::console::print_error("unknown surface reconstruction method %d\n expecting values between 1 and 2", surface_type);
    exit (1);
  }
  
  switch (descriptor_type)
  {
    case 1:
    {
      pcl::Feature<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr feature_extractor (new pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>); 
      feature_extractor->setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
      feature_extractor->setRadiusSearch (0.05);
      ICCVTutorial<pcl::FPFHSignature33> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, source, target);
      tutorial.run ();
    }
    break;
    
    case 2:
    {
      pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344>* shot = new pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344>;
      shot->setRadiusSearch (0.04);
      pcl::Feature<pcl::PointXYZRGB, pcl::SHOT1344>::Ptr feature_extractor (shot);
      ICCVTutorial<pcl::SHOT1344> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, source, target);
      tutorial.run ();
    }
    break;
    
    case 3:
    {
      pcl::Feature<pcl::PointXYZRGB, pcl::PFHSignature125>::Ptr feature_extractor (new pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125>);
      feature_extractor->setKSearch(50);
      ICCVTutorial<pcl::PFHSignature125> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, source, target);
      tutorial.run ();
    }
    break;
    
    case 4:
    {
      pcl::Feature<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr feature_extractor (new pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>);
      feature_extractor->setKSearch(50);
      ICCVTutorial<pcl::PFHRGBSignature250> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, source, target);
      tutorial.run ();
    }
    break;
    
    default:
      pcl::console::print_error("unknown descriptor type %d\n expecting values between 1 and 4", descriptor_type);
      exit (1);
      break;
  }  
  
  return (0);
}
