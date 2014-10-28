/**
 * @brief Templates always go in headers
 */


/**
* @function getOFFMesh
* @brief Make an OFF mesh from the box
*/
template<typename PointT>
std::string ContainerDetection<PointT>::getOFFMeshStr(const bool &world)
{
  if(world)
    return getOFFMeshStr(getSize(),getRotation(),getPosition());
  return getOFFMeshStr(getSize());
}
/**
* @function getOFFMesh
* @brief Make an OFF mesh from the box
*        The box need to have the hole in the Z direction.
*/
template<typename PointT>
std::string ContainerDetection<PointT>::getOFFMeshStr(
                                                  const Eigen::Vector4d &s_, 
                                                  const Eigen::Quaterniond &r_,
                                                  const Eigen::Vector3d &t_
                                                  )
{
  PointCloudPtr cloudb(new PointCloud);

  // Tweak size for External box
  Eigen::Vector3d ss_(s_.head<3>());
  ss_ += Eigen::Vector3d::Ones()*s_.w()/2;
  for(size_t i=0;i<8;i++)
  {
    cloudb->points.push_back(PointT(
                                        (-1+2*((i%2)>0))*ss_.x()/2,
                                        (-1+2*((i%4)>1))*ss_.y()/2,
                                        (-1+2*((i%8)>3))*ss_.z()/2
                             ));
  }
  // Tweak size for Internal box
  #if meshThroughHoleBox
  ss_.head<2>() -= Eigen::Vector2d::Ones()*s_.w();
  Eigen::Vector3d o_ = Eigen::Vector3d::Zero();
  #else
  ss_ -= Eigen::Vector3d::Ones()*s_.w();
  Eigen::Vector3d o_ = Eigen::Vector3d::UnitZ()*(s_.w()/2);
  #endif
  for(size_t i=0;i<8;i++)
  {
    cloudb->points.push_back(PointT(
                                        (-1+2*((i%2)>0))*ss_.x()/2+o_.x(),
                                        (-1+2*((i%4)>1))*ss_.y()/2+o_.y(),
                                        (-1+2*((i%8)>3))*ss_.z()/2+o_.z()
                             ));
  }
  cloudb->width = cloudb->points.size();
  cloudb->height = 1;
  
  // Move it into the real world !
  Eigen::Affine3d tf=Eigen::Affine3d::Identity();
  tf.linear() = r_.toRotationMatrix();
  tf.translation() = t_;
  pcl::transformPointCloud (*cloudb, *cloudb, tf.cast<float>());
  
  #if meshCloudViz
  pcl::visualization::PCLVisualizer vlocal;
  vlocal.addPointCloud(cloudb);
  vlocal.spin();
  #endif
  
  std::stringstream ss;
  // Header
  ss << "OFF" << std::endl;
  // nVertices     nFaces       nEdges
  #if meshThroughHoleBox
  ss << 16 << " " << 16 << " " << 32 << std::endl;
  #else
  ss << 16 << " " << 14 << " " << 28 << std::endl;
  #endif
  // Add vertices from PointCloud
  for(PointVectorIt it=cloudb->points.begin(); it != cloudb->points.end(); it++)
  {
    ss << it->x << " " << it->y << " " << it->z << std::endl;
  }
  // Add faces
    // External Sides
  ss << "4  0  1  5  4" << std::endl; // Ext Left
  ss << "4  2  3  7  6" << std::endl; // Ext Right
  ss << "4  1  3  7  5" << std::endl; // Ext Front
  ss << "4  0  2  6  4" << std::endl; // Ext Back
    // Internal Sides
  ss << "4  8  9 13 12" << std::endl; // Int Left 
  ss << "4 10 11 15 14" << std::endl; // Int Right
  ss << "4  9 11 15 13" << std::endl; // Int Front
  ss << "4  8 10 14 12" << std::endl; // Int Back  
      // Top
  ss << "4  4  5 13 12" << std::endl; // Top left
  ss << "4  6  7 15 14" << std::endl; // Top right
  ss << "4  5  7 15 13" << std::endl; // Top front
  ss << "4  4  6 14 12" << std::endl; // Top back
  
  #if meshThroughHoleBox
    // Bottom
  ss << "4  0  1  9  8" << std::endl; // Bottom left
  ss << "4  2  3 11 10" << std::endl; // Bottom right
  ss << "4  1  3 11  9" << std::endl; // Bottom front
  ss << "4  0  2 10  8" << std::endl; // Bottom back
  #else
    // External Sides
  ss << "4  0  1  3  2" << std::endl; // Ext Bottom
    // Internal Sides
  ss << "4  8  9 11 10" << std::endl; // Int Bottom
  #endif
  return ss.str();
}

/**
* @function setParams
* @brief Quick access to the parameters
*/
template<typename PointT>
void ContainerDetection<PointT>::setParams(const params_t &p)
{
  _params = p;
}

/**
* @function getParams
* @brief Quick access to the parameters
*/
template<typename PointT>
void ContainerDetection<PointT>::getParams(params_t &p)
{
  p = _params;
}

/**
* @function search
* @brief Try to locate a container in the pointcloud
*/
template<typename PointT>
int ContainerDetection<PointT>::search(void)
{
  // Estimate normals for the PointCloud
  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud (_cloud);
  ne.compute (*_normals);
  
  // Segment
  if(segmentPlanes() > 0)
  {
    // Find candidates
    if(_params.est_mode_para) // Two parallel planes, one perpendicular
      findParaCandidates();
    else                      // Two perpendicular planes
      findPerpCandidates();
    // Sort it out !
    std::sort(_est_cand.begin(), _est_cand.end(), compareCandidates);
    typedef typename std::vector<estim_t>::iterator estim_it;
    for(estim_it it=_est_cand.begin();it!=_est_cand.end();it++)
    {
      // Drop the candidate if it doesn't have three planes. Shouldn't happen...
      if(it->indices.size() < 3) {  // TODO: Do better =)
        continue;
      }
      // Get all the planes belonging to the Container
      PointCloudPtr allp(new PointCloud);
      for(std::vector<int>::iterator itt=it->indices.begin();
                                     itt<it->indices.end();
                                     itt++)
      {
        *allp += *_sp_clouds[*itt].get();
      }
      // Clean up a littl' bit
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud (allp);
      sor.setMeanK (15);              // TODO: Parameters !
      sor.setStddevMulThresh (1.0);
      sor.filter (*allp);

      // "Real-life" CS of the BBox
      Eigen::Vector3f v1_ = 
                  Eigen::Vector4f(_sp_regions[it->indices[0]].getCoefficients())
                                  .head<3>().normalized();
      Eigen::Vector3f v2_ = 
                  Eigen::Vector4f(_sp_regions[it->indices[2]].getCoefficients())
                                  .head<3>().normalized();
      Eigen::Vector3f v_(Eigen::Vector3f((v1_).cross(v2_)).normalized());
      
      // Find the BBox
      box_t box;
      MVOBBox<PointT> bbox;
      bbox.setInputCloud(allp);
      //~ bbox.setInitialVector(v_);
      bbox.setInitialBase(v1_,v2_);      
      bbox.findMVOBBox(box.q_,box.t_,box.s_);
      
      // Fix the Zaxis of the BBox so  it matches the expected RealLife CS
        /* Note: This is really not clean, assuming large enough values of the 
         * dot-product to cast correctly to Int. Do that better with index of 
         * the max value of the dotprod, then map it to the correct axis (X,Y,Z)
         */
      #if boxFixZ
        #if 0
          pcl::console::print_value("Origin:\n"
                              "\trotation (x,y,z,w):  % .3f % .3f % .3f % .3f\n"
                              "\ttranslation (x,y,z): % .3f % .3f % .3f (m)\n"
                              "\tsize (x,y,z):        % .3f % .3f % .3f (m)\n",
                                box.q_.x(),box.q_.y(), box.q_.z(),box.q_.w(),
                                box.t_.x(),box.t_.y(),box.t_.z(),
                                box.s_.x(),box.s_.y(),box.s_.z());
        #endif
        Eigen::Vector3i zBox_ = Eigen::Vector3f(
                    (box.q_.toRotationMatrix().transpose()*v_)*1.5).cast<int>();
        Eigen::Quaternionf fix;
        /* Okay, there's an assumption here: The rotation will be compatible 
         * with the quaternion of the box, meaning it won't screw things up.
         * It went fine on all my tests though...
         */
        fix.setFromTwoVectors(Eigen::Vector3f::UnitZ(),
                              Eigen::Vector3i(zBox_).cast<float>());
        box.q_ = box.q_*fix;
        box.s_ = (fix*box.s_).cwiseAbs();
        #if 0
          pcl::console::print_value("Fixed:\n"
                              "\trotation (x,y,z,w):  % .3f % .3f % .3f % .3f\n"
                              "\ttranslation (x,y,z): % .3f % .3f % .3f (m)\n"
                              "\tsize (x,y,z):        % .3f % .3f % .3f (m)\n",
                                box.q_.x(),box.q_.y(), box.q_.z(),box.q_.w(),
                                box.t_.x(),box.t_.y(),box.t_.z(),
                                box.s_.x(),box.s_.y(),box.s_.z());
        #endif
      #endif 
      #if boxViz
      pcl::visualization::PCLVisualizer vlocal;
      vlocal.addCoordinateSystem(0.1);
      Eigen::Affine3f tf=Eigen::Affine3f::Identity();
      tf.linear() = box.q_.toRotationMatrix();
      tf.translation() = box.t_;
      vlocal.addCoordinateSystem(0.1,tf);
      vlocal.addPointCloud(_cloud);
      {
      Eigen::Vector3f centroid = _sp_regions[it->indices[0]].getCentroid();
      Eigen::Vector3f model = v1_;
      PointT pt1 = PointT (centroid[0], centroid[1], centroid[2]);
      PointT pt2 = PointT (centroid[0] + (0.2f * model[0]),
                                         centroid[1] + (0.2f * model[1]),
                                         centroid[2] + (0.2f * model[2]));

      vlocal.addArrow (pt2, pt1, 1, 0, 0, false, "norm0");
      }
      {
      Eigen::Vector3f centroid = _sp_regions[it->indices[2]].getCentroid();
      Eigen::Vector3f model = v2_;
      PointT pt1 = PointT (centroid[0], centroid[1], centroid[2]);
      PointT pt2 = PointT (centroid[0] + (0.2f * model[0]),
                                         centroid[1] + (0.2f * model[1]),
                                         centroid[2] + (0.2f * model[2]));

      vlocal.addArrow (pt2, pt1, 0, 1, 0, false, "norm1");
      }
      {
      Eigen::Vector3f centroid = box.t_;
      Eigen::Vector3f model = v_;
      PointT pt1 = PointT (centroid[0], centroid[1], centroid[2]);
      PointT pt2 = PointT (centroid[0] + (0.2f * model[0]),
                                         centroid[1] + (0.2f * model[1]),
                                         centroid[2] + (0.2f * model[2]));

      vlocal.addArrow (pt2, pt1, 0, 0, 1, false, "norm3");
      }
      vlocal.addCube(box.t_,box.q_, box.s_.x(),box.s_.y(), box.s_.z(),"cubeA");
      Eigen::Vector3f tt_(0,0,0);
      Eigen::Quaternionf qt_(0,0,0,1);
      vlocal.addCube(tt_,box.q_, box.s_.x(),box.s_.y(), box.s_.z(),"cube0");
      vlocal.addCube(tt_,box.q_, box.s_.x(),box.s_.y(), box.s_.z(),"cube2");
      vlocal.addCube(tt_,qt_, box.s_.x(),box.s_.y(), box.s_.z(),"cube1");
      vlocal.spin();
      #endif
      _boxes.push_back(box);
    }
  }
  
  return _boxes.size();
}

/**
* @function compareCandidates
* @brief Compare two candidates, returns true if c1 is better than c2
*         Currently uses only the perpendicular distance, make it smarter ;)
*/
template<typename PointT>
bool ContainerDetection<PointT>::compareCandidates(const estim_t &c1, 
                                                   const estim_t &c2)
{
  return c1.perp_dist < c2.perp_dist;
}

/**
* @function findParaCandidates
* @brief Make groups of planes that could be a container, using three planes
*/
template<typename PointT>
void ContainerDetection<PointT>::findParaCandidates(void)
{
  for(size_t i=0;i<_sp_regions.size();i++)
  {
    for(size_t j=0;j<i;j++)
    {
      // Find two parallel planes
      Eigen::Vector3f v1 = Eigen::Vector4f(_sp_regions[i].getCoefficients())
                                                        .head<3>().normalized();
      Eigen::Vector3f v2 = Eigen::Vector4f(_sp_regions[j].getCoefficients())
                                                        .head<3>().normalized();
      // Avg deviation (v1,v2) and (v2,v1)
      float dev = (acos(v1.dot (v2))+acos(v2.dot (v1)))/2;
      if(dev < _params.est_para_max_dev)
      {
        Eigen::Vector3f p1 = _sp_regions[i].getCentroid();
        Eigen::Vector3f p2 = _sp_regions[j].getCentroid();
        //~ Eigen::Vector3f v = p2-p1;
        // Proj p2 on (p1,v1) name it p2'
        Eigen::Vector3f p2p = p1 + ((p2-p1).dot(v1)/(v1.dot(v1)))*v1;
        // Proj p1 on (p2,v2) name it p1'
        Eigen::Vector3f p1p = p2 + ((p1-p2).dot(v2)/(v2.dot(v2)))*v2;
        // Get avg of (|p2'_p1|,|p1'_p2|), known as |p1_p2|
        double p1p2 = ((p2p-p1).norm() + (p1p-p2).norm())/2;
        if(p1p2 > _params.est_para_min_dist)
        {
          /* pcl::console::print_value("Planes %2d(%c) and %2d(%c):\t"
                                    "Avg distance: %fm\n",
                                    i,c[i%6],j,c[j%6],p1p2);
          //*/
          estim_t est;
          est.para_dev = dev;
          est.para_dist = p1p2;
          est.perp_dev = std::numeric_limits<float>::infinity();
          est.perp_dist = std::numeric_limits<float>::infinity();
          est.indices.push_back(i);
          est.indices.push_back(j);
          //~ _est_cand.push_back(est);
            // Find a perpendicular one now..
          for(size_t k=0;k<_sp_regions.size();k++)
          {
            if(k != i && k != j)
            {
              Eigen::Vector3f v3 = Eigen::Vector4f(
                       _sp_regions[k].getCoefficients()).head<3>().normalized();
              double dev2 = (fabs(acos(v1.dot (v3))-M_PI/2) +
                            fabs(acos(v2.dot (v3))-M_PI/2))/2;
              // Allow some deviation,
              if(dev2 < _params.est_perp_max_dev)
              {
                est.perp_dev = dev2;
                est.indices.push_back(k);
                //~ _est_cand.push_back(est);
                Eigen::Vector3f p3 = _sp_regions[k].getCentroid();
                // Proj p3 on (p1,v1) name it p3'
                Eigen::Vector3f p3p = p1 + ((p3-p1).dot(v1)/(v1.dot(v1)))*v1;
                // Proj p3 on (p2,v2) name it p3"
                Eigen::Vector3f p3s = p2 + ((p3-p2).dot(v2)/(v2.dot(v2)))*v2;
                // Proj p1 on (p3,v3), name it p1"
                Eigen::Vector3f p1s = p3 + ((p1-p3).dot(v3)/(v3.dot(v3)))*v3;
                // Proj p2 on (p3,v3), name it p2"
                Eigen::Vector3f p2s = p3 + ((p2-p3).dot(v3)/(v3.dot(v3)))*v3;
                /* Check:
                 *  - |p3'_p1| < |p1_p2|
                 *  - |p3"_p2| < |p1_p2|
                 *  - sg(p3'_p1) != sg(p3"_p2) // Should be trivial
                 *  - sg(p3'_p3) = sg(p3"_p3)
                 */
                if((p3p-p1).norm() < p1p2 && (p3s-p2).norm() < p1p2 && 
                    (p3p-p1).dot(p3s-p2) < 0 && (p1s-p3).dot(p2s-p3) > 0)
                    
                {
                  typedef pcl::KdTree<PointT> KdTree;
                  typedef pcl::KdTreeFLANN<PointT> KdTreeFlann;
                  typedef typename KdTree::Ptr KdTreePtr;
                  KdTreePtr tree_ (new KdTreeFlann);
                  std::vector<int> nn_indices (1);
                  std::vector<float> nn_dists (1);
              
                  // Proj p1 on (p3,v1), name it p1"'
                  Eigen::Vector3f p1t = p3+((p1-p3).dot(v1)/v1.dot(v1))*v1;
                  tree_->setInputCloud(_sp_clouds[i]);
                  tree_->nearestKSearch(PointT(p1t[0], p1t[1], p1t[2]),
                                        1, nn_indices, nn_dists);
                  double dist = nn_dists.front();
                  // Proj p3 on (p1,v3), name it p3"'
                  Eigen::Vector3f p3t = p1+((p3-p1).dot(v3)/v3.dot(v3))*v3;
                  tree_->setInputCloud(_sp_clouds[k]);
                  tree_->nearestKSearch(PointT(p3t[0], p3t[1], p3t[2]),
                                        1, nn_indices, nn_dists);
                  dist += nn_dists.front();
                  // Proj p2 on (p3,v2), name it p2"'
                  Eigen::Vector3f p2t = p3+((p2-p3).dot(v2)/v2.dot(v2))*v2;
                  tree_->setInputCloud(_sp_clouds[j]);
                  tree_->nearestKSearch(PointT(p2t[0], p2t[1], p2t[2]),
                                        1, nn_indices, nn_dists);
                  dist += nn_dists.front();
                  // Proj p3 on (p2,v3), name it p3""
                  Eigen::Vector3f p3q = p2+((p3-p2).dot(v3)/v3.dot(v3))*v3;
                  tree_->setInputCloud(_sp_clouds[k]);
                  tree_->nearestKSearch(PointT(p3q[0], p3q[1], p3q[2]),
                                        1, nn_indices, nn_dists);
                  dist += nn_dists.front();

                  est.perp_dist = dist;
                  _est_cand.push_back(est);
                }
                est.indices.pop_back();
              }
            }
          }
        }
      }
    }
  }
}

/**
* @function findPerpCandidates
* @brief Make groups of planes that could be a container, using only two planes
*/
template<typename PointT>
void ContainerDetection<PointT>::findPerpCandidates(void)
{
  for(size_t i=0;i<_sp_regions.size();i++)
  {
    for(size_t j=0;j<i;j++)
    {
      // Find two perpendicular planes
      Eigen::Vector3f v1 = Eigen::Vector4f(_sp_regions[i].getCoefficients())
                                                        .head<3>().normalized();
      Eigen::Vector3f v3 = Eigen::Vector4f(_sp_regions[j].getCoefficients())
                                                        .head<3>().normalized();
      double dev = fabs(acos(v1.dot (v3))-M_PI/2);
      if(dev < _params.est_perp_max_dev)
      {
        estim_t est;
        est.perp_dev = dev;
        est.perp_dist = std::numeric_limits<float>::infinity();
        est.indices.push_back(i);
        est.indices.push_back(i); // Héhé :p Deal with the // case
        est.indices.push_back(j);
        _est_cand.push_back(est);

        Eigen::Vector3f p1 = _sp_regions[i].getCentroid();
        Eigen::Vector3f p3 = _sp_regions[j].getCentroid();

        typedef pcl::KdTree<PointT> KdTree;
        typedef pcl::KdTreeFLANN<PointT> KdTreeFlann;
        typedef typename KdTree::Ptr KdTreePtr;
        KdTreePtr tree_ (new KdTreeFlann);
        std::vector<int> nn_indices (1);
        std::vector<float> nn_dists (1);

        // Proj ortho de P1 sur (P3,v1)
        Eigen::Vector3f pp1p3v1 = p3+((p1-p3).dot(v1)/v1.dot(v1))*v1;
        tree_->setInputCloud(_sp_clouds[i]);
        tree_->nearestKSearch(
                            PointT(pp1p3v1[0], pp1p3v1[1], pp1p3v1[2]),
                            1, nn_indices, nn_dists);
        float dist1 = *nn_dists.begin();
        // Proj ortho de P3 sur (P1,v3)
        Eigen::Vector3f pp3p1v3 = p1+((p3-p1).dot(v3)/v3.dot(v3))*v3;
        tree_->setInputCloud(_sp_clouds[j]);
        tree_->nearestKSearch(
                            PointT(pp3p1v3[0], pp3p1v3[1], pp3p1v3[2]),
                            1, nn_indices, nn_dists);
        float dist2 = *nn_dists.begin();
        float dist = dist1+dist2;
        est.perp_dist = dist;
        _est_cand.push_back(est);
      }
    }
  }
}


/**
* @function segmentPlanes
* @brief Wrapper for pcl::OrganizedMultiPlaneSegmentation, with parameters and 
*        storage of the results
*/
template<typename PointT>
int ContainerDetection<PointT>::segmentPlanes(void)
{
  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
  mps.setMinInliers (_params.seg_min_inliers);
  mps.setAngularThreshold (_params.seg_ang_thres);    // rad
  mps.setDistanceThreshold (_params.seg_dist_thres);  // m
  mps.setMaximumCurvature(_params.seg_max_curv);      // m^-1
  mps.setInputNormals (_normals);
  mps.setInputCloud (_cloud);
  std::vector<pcl::PointIndices> inliers;
    // We don't care 'bout that :/
  std::vector<pcl::ModelCoefficients> m;
  pcl::PointCloud<pcl::Label>::Ptr l (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> li;
  std::vector<pcl::PointIndices> bi;

  mps.segmentAndRefine (_sp_regions, m, inliers, l, li, bi);
  //~ mps.segmentAndRefine (_sp_regions);
  // Prcess the segmented planes ///////////////////////////////////////////////
  //pcl::console::print_highlight ("Processing the segmentation results...\n");
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (_cloud);
  
  for (std::vector<pcl::PointIndices>::iterator it=inliers.begin();
                                                it!=inliers.end();
                                                it++)
  {
    _sp_clouds.push_back(PointCloudPtr(new PointCloud));
    extract.setIndices(pcl::IndicesPtr(new std::vector<int>(it->indices)));
    extract.filter (*(_sp_clouds.back()));
  }
  #if segmentViz
  #define ptSize 1
  pcl::visualization::PCLVisualizer vlocal;
  vlocal.addPointCloud(_cloud);
  for (size_t i = 0; i < _sp_regions.size (); i++)
  {
    std::stringstream ss;
    if(1)
    {
      Eigen::Vector3f centroid = _sp_regions[i].getCentroid ();
      Eigen::Vector4f model = _sp_regions[i].getCoefficients ();
      PointT pt1 = PointT (centroid[0], centroid[1], centroid[2]);
      PointT pt2 = PointT (centroid[0] + (0.2f * model[0]),
                                         centroid[1] + (0.2f * model[1]),
                                         centroid[2] + (0.2f * model[2]));

      ss << "normal_" << i;
      vlocal.addArrow (pt2, pt1, ((double)r[i%6])/255,
                                  ((double)g[i%6])/255,
                                  ((double)b[i%6])/255, false, ss.str());
    }
    pcl::PointCloud<PointT>::Ptr
                                    contour(new pcl::PointCloud<PointT>);
    contour->points = _sp_regions[i].getContour ();

    pcl::visualization::PointCloudColorHandlerCustom<PointT>
                                        color (contour, r[i%6], g[i%6], b[i%6]);
    ss.str("cplane_"); ss << i;
    vlocal.addPointCloud(contour, color, ss.str());
                                                                // Bigger points
    vlocal.setPointCloudRenderingProperties(
                                  pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                  ptSize*3, ss.str());
    ss.str("plane_"); ss << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointT>
                                 color2 (_sp_clouds[i], r[i%6], g[i%6], b[i%6]);
    vlocal.addPointCloud(_sp_clouds[i], color2,ss.str());          // Add colors
  }
  vlocal.spin();
  #endif
  return _sp_clouds.size();
}

/**
* @function getContainerPosition
* @brief return the pose x,y,z
*/
template<typename PointT>
Eigen::Vector3d ContainerDetection<PointT>::getPosition(const int &i)
{
  if(i < _boxes.size()) 
  {
    return Eigen::Vector3f(_boxes[i].t_).cast<double>();
  }
  return Eigen::Vector3d::Zero();
}

/**
* @function getRotation
* @brief return the pose rx,ry,rz,rw
*/
template<typename PointT>
Eigen::Quaterniond ContainerDetection<PointT>::getRotation(const int &i)
{
  if(i < _boxes.size()) 
  {
    return Eigen::Quaternionf(_boxes[i].q_).cast<double>();
  }
  return Eigen::Quaterniond::Identity();
}

/**
* @function getSize
* @brief return the size: x,y,z,w
*/
template<typename PointT>
Eigen::Vector4d ContainerDetection<PointT>::getSize(const int &i)
{
  if(i < _boxes.size())
  {
    Eigen::Vector4d s_;
    s_.head<3>() = Eigen::Vector3f(_boxes[i].s_).cast<double>();
    s_.w() = 0.05;    // ToDo: Parameter or auto-detection
    return s_;
  }
  return Eigen::Vector4d::Zero();
}

/**
* @function getCloud
* @brief returns the PointCloud of the container
*/
template<typename PointT>
void ContainerDetection<PointT>::getCloud(const Eigen::Vector3d &t_,
                                          const Eigen::Quaterniond &r_,
                                          const Eigen::Vector4d &s_,
                                          PointCloudPtr ocloud)
{
  
  Eigen::Vector3f ss_(s_.cast<float>().head<3>());
  ss_ += Eigen::Vector3f::Ones()*s_.w()/2;
  // Get the corners of the Bounding Box
  PointCloudPtr cloudbb(new PointCloud);
  for(size_t i=0;i<8;i++)
  {
    cloudbb->points.push_back(PointT( (-1+2*((i%2)>0))*ss_.x()/2,
                                      (-1+2*((i%4)>1))*ss_.y()/2,
                                      (-1+2*((i%8)>3))*ss_.z()/2
                                    ));
  }
  cloudbb->width = cloudbb->points.size();
  cloudbb->height = 1;
  // Move it into the real world !
  Eigen::Affine3d tf=Eigen::Affine3d::Identity();
  tf.linear() = r_.toRotationMatrix();
  tf.translation() = t_;
  pcl::transformPointCloud (*cloudbb, *cloudbb, tf.cast<float>());
  Eigen::Vector4f minpt(0,0,0,1),maxpt(0,0,0,1);
  minpt.head<3>() = -(ss_/2);
  maxpt.head<3>() =  (ss_/2);

  // Fetch all the segmented planes
  PointCloudPtr worldp(new PointCloud);
  for(PointCloudPtrVectorIt it=_sp_clouds.begin(); it<_sp_clouds.end(); it++)
  {
    *worldp += *(it->get());
  }
  // Get the Points 
  pcl::CropBox<PointT> cropFilter;
  cropFilter.setInputCloud (worldp);
  cropFilter.setMin(minpt);
  cropFilter.setMax(maxpt);
  cropFilter.setTransform(tf.inverse().cast<float>());
  cropFilter.filter (*ocloud);
  // Move  back to origin
  pcl::transformPointCloud (*ocloud, *ocloud, tf.inverse().cast<float>());
  
  // Cropbox to remove all points inside the BBox_in
  if(s_.w()) 
  {
    ss_ -= Eigen::Vector3f::Ones()*s_.w();
    unsigned int i=0;
    for(PointVectorIt it=cloudbb->points.begin();it!=cloudbb->points.end();it++,
                                                                           i++)
    {
      *it = PointT((-1+2*((i%2)>0))*ss_.x()/2,
                                  (-1+2*((i%4)>1))*ss_.y()/2,
                                  (-1+2*((i%8)>3))*ss_.z()/2
                                 );
    }
    minpt.head<3>() = -(ss_/2);
    maxpt.head<3>() =  (ss_/2);
    cropFilter.setInputCloud (ocloud);
    cropFilter.setMin(minpt);
    cropFilter.setMax(maxpt);
    cropFilter.setNegative(true);
    cropFilter.filter (*ocloud);
  }
}

/**
* @function getCloud
* @brief returns the PointCloud of the container
*/
template<typename PointT>
void ContainerDetection<PointT>::getCloud(PointCloudPtr c,const int &i)
{
  if(i < _boxes.size())
    getCloud(getPosition(i),getRotation(i),getSize(i),c);
}

/**
 * @function ContainerDetection
 * @brief Constructor. Init to default values
 */
template<typename PointT>
ContainerDetection<PointT>::ContainerDetection() :
  _cloud(new PointCloud()),
  _normals(new pcl::PointCloud<pcl::Normal>)
{
  _params.seg_min_inliers   = 300;  // 
  _params.seg_ang_thres     = 3.00*M_PI/180; // rad
  _params.seg_dist_thres    = 0.05; // m 
  _params.seg_max_curv      = 0.10; // m^-1
    
  _params.est_mode_para     = true; //
  _params.est_para_max_dev  = 6*M_PI/180;    // rad
  _params.est_para_min_dist = 0.10; // m
  _params.est_perp_max_dev  = 6*M_PI/180;    // rad
}

/**
 * @function ~ContainerDetection
 * @brief Destructor
 */
template<typename PointT>
ContainerDetection<PointT>::~ContainerDetection() 
{
}

/**
 * @function setInputCloud
 * @brief Set cloud to process
 */
template<typename PointT>
void ContainerDetection<PointT>::setInputCloud(const PointCloudConstPtr &cloud)
{
  _cloud = cloud;
}

/**
 * @function setInputCloud
 * @brief Set cloud to process
 */
template<typename PointT>
void ContainerDetection<PointT>::setInputCloud(const PointCloudPtr &cloud)
{
  _cloud = cloud;
}

template class ContainerDetection<pcl::PointXYZ>;
template class ContainerDetection<pcl::PointXYZRGB>;
//~ template class ContainerDetection<pcl::PointXYZRGBA>;
