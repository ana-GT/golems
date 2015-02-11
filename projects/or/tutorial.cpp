/**
 * @file tutorial.cpp
 */
#include "tutorial.h"

typedef pcl::PointXYZ PointT;

void help( char* argv0 );
void showMenu();


/**
 * @function main
 */
int main (int argc, char *argv[] ) {

  if (argc < 6) {
    help( argv[0]);
    return (1);
  }

  showMenu();

  // Load model to recognize
  pcl::PointCloud<PointT>::Ptr source (new pcl::PointCloud<PointT>);
  pcl::io::loadPCDFile (argv[1], *source);
  
  // Load model to fit
  pcl::PointCloud<PointT>::Ptr target (new pcl::PointCloud<PointT>);
  pcl::io::loadPCDFile (argv[2], *target);

  // Get the keypoint, descriptor and surface (?) types
  int keypoint_type   = atoi (argv[3]);
  int descriptor_type = atoi (argv[4]);
  int surface_type    = atoi (argv[5]);
  
  // Keypoint
  boost::shared_ptr<pcl::Keypoint<PointT, pcl::PointXYZI> > keypoint_detector;
  
  if( keypoint_type < 6 ) {
    pcl::HarrisKeypoint3D<PointT,pcl::PointXYZI>* harris3D = new pcl::HarrisKeypoint3D<PointT,pcl::PointXYZI> (pcl::HarrisKeypoint3D<PointT,pcl::PointXYZI>::HARRIS);
    harris3D->setNonMaxSupression(true);
    harris3D->setRadius (0.01);
    harris3D->setRadiusSearch (0.01);
    keypoint_detector.reset(harris3D);
    switch (keypoint_type) {
    case 1:
      harris3D->setMethod(pcl::HarrisKeypoint3D<PointT,pcl::PointXYZI>::HARRIS);
      break;
      
    case 2:
      harris3D->setMethod(pcl::HarrisKeypoint3D<PointT,pcl::PointXYZI>::TOMASI);
      break;
      
    case 3:
      harris3D->setMethod(pcl::HarrisKeypoint3D<PointT,pcl::PointXYZI>::NOBLE);
      break;
      
    case 4:
      harris3D->setMethod(pcl::HarrisKeypoint3D<PointT,pcl::PointXYZI>::LOWE);
      break;
      
    case 5:
      harris3D->setMethod(pcl::HarrisKeypoint3D<PointT,pcl::PointXYZI>::CURVATURE);
      break;
    }
    
  }
    /*
  else if( keypoint_type == 6 ) {
  pcl::SIFTKeypoint<PointT, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<PointT, pcl::PointXYZI>;
  sift3D->setScales(0.01, 3, 2);
  sift3D->setMinimumContrast(0.0);
  keypoint_detector.reset(sift3D);
  }*/

  
  
  boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointNormal> > surface_reconstruction;
  
  if (surface_type == 1) {
    pcl::GreedyProjectionTriangulation<pcl::PointNormal>* gp3 = new pcl::GreedyProjectionTriangulation<pcl::PointNormal>;
    
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
    pcl::MarchingCubes<pcl::PointNormal>* mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>;
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
      pcl::Feature<PointT, pcl::FPFHSignature33>::Ptr feature_extractor (new pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33>); 
      feature_extractor->setSearchMethod (pcl::search::Search<PointT>::Ptr (new pcl::search::KdTree<PointT>));
      feature_extractor->setRadiusSearch (0.05);
      ICCVTutorial<pcl::FPFHSignature33> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, source, target);
      tutorial.run ();
    }
    break;
    
    
    case 2:
    {
      pcl::Feature<PointT, pcl::PFHSignature125>::Ptr feature_extractor (new pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125>);
      feature_extractor->setKSearch(50);
      ICCVTutorial<pcl::PFHSignature125> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, source, target);
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

/**
 * @function help
 */
void help( char* argv0 ) {

  printf("Syntax is: %s <source-pcd-file> <target-pcd-file> <keypoint-method> <descriptor-type> <surface-reconstruction-method>\n", argv0);
  printf("available <keypoint-methods>: \n" );
  printf("                              1 = Harris3D\n");
  printf("                              2 = Tomasi3D\n");
  printf("                              3 = Noble3D\n");
  printf("                              4 = Lowe3D\n");
  printf("                              5 = Curvature3D\n");
  printf("                              6 = Sift3D\n");
  printf("available <descriptor-types>: 1 = FPFH\n");
  printf("                              2 = PFH\n");
  printf("available <surface-methods>:  1 = Greedy Projection\n");
  printf("                              2 = Marching Cubes\n");    
}

/**
 * @function showMenu
 */
void showMenu() {
  printf("== MENU ==\n");
  printf("1 - show/hide source point cloud\n");
  printf("2 - show/hide target point cloud\n");
  printf("3 - show/hide segmented source point cloud\n");
  printf("4 - show/hide segmented target point cloud\n");
  printf("5 - show/hide source key points\n");
  printf("6 - show/hide target key points\n");
  printf("7 - show/hide source2target correspondences\n");
  printf("8 - show/hide target2source correspondences\n");
  printf("9 - show/hide consistent correspondences\n");
  printf("i - show/hide initial alignment\n");
  printf("r - show/hide final registration\n");
  printf("t - show/hide triangulation (surface reconstruction)\n");
  printf("h - show visualizer options\n");
  printf("q - quit\n");
}
