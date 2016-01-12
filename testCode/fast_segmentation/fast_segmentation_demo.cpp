/*
 * Software License Agreement (BSD License)
 *
 *  OmniMapper
 *  Copyright (c) 2012-, Georgia Tech Research Corporation,
 *  Atlanta, Georgia 30332-0415
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "fast_segmentation.h"
#include "fast_segmentation_demo.h"

/** Alex Trevor's header */

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;
typedef pcl::PointCloud<pcl::Label> LabelCloud;
typedef typename LabelCloud::Ptr LabelCloudPtr;
typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;


int main (int argc, char** argv) {
  
  // Create our segmentation class
  FastSegmentation<PointT> seg;

  // Create a grabber, and hook it up to the feature extraction
  pcl::io::OpenNI2Grabber ni_grabber ("#1");
  boost::function<void (const CloudConstPtr&)> f = boost::bind (&FastSegmentation<PointT>::cloudCallback, &seg, _1);
  boost::signals2::connection c = ni_grabber.registerCallback (f);

  // Hook up our demo app callbacks, which will visualize the segmentation results
  Demo<PointT> demo;
  boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)> cluster_label_callback = boost::bind (&Demo<PointT>::clusterLabelsCallback, &demo, _1, _2);
  //  if (raw_labels)
  //    seg.setClusterLabelsCallback (cluster_label_callback);
  boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)> plane_label_callback = boost::bind (&Demo<PointT>::planeLabelsCallback, &demo, _1, _2);
  //  if (raw_labels)
  //    seg.setPlaneLabelsCallback (plane_label_callback);

  boost::function<void(const CloudConstPtr, boost::posix_time::ptime, std::vector<pcl::ModelCoefficients>, std::vector<pcl::PointIndices>, std::vector<pcl::PointIndices>, std::vector<pcl::PointIndices>)> full_plane_callback = boost::bind (&Demo<PointT>::planarRegionsCallback, &demo, _1, _2, _3, _4, _5, _6);
  //if (!raw_labels)
    seg.setFullPlanarRegionCallback (full_plane_callback);

  boost::function<void(const CloudConstPtr, boost::posix_time::ptime, std::vector<pcl::PointIndices>)> full_cluster_callback = boost::bind (&Demo<PointT>::fullClusterCallback, &demo, _1, _2, _3);
  //  if (!raw_labels)
      seg.setFullClusterCallback (full_cluster_callback);

  // Start spinning
  ni_grabber.start ();
  seg.spin ();

  while (true)
  {
    boost::this_thread::sleep (boost::posix_time::milliseconds (10));
    /*    if (raw_labels)
    {  
      demo.spinVisClusters ();
      demo.spinVisPlanes ();
      }*/
    //else
    {
      demo.spinVisFullPlanes ();
      demo.spinVisFullClusters ();
    }//
  }


  return (0);
}
