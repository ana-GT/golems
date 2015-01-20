/**
 * @file openni2_capture.h
 * @brief Adapted from the PCL tutorial from ICCV 2011 
 */
#pragma once

#include <pcl/pcl_config.h>

#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "typedefs.h"

/* A simple class for capturing data from an OpenNI2 camera (PrimeSense 1.09) */
class OpenNI2Capture
{
  public:
    OpenNI2Capture (const std::string& device_id = "");
    ~OpenNI2Capture ();
    
    void setTriggerMode (bool use_trigger);
    const PointCloudPtr snap ();
    const PointCloudPtr snapAndSave (const std::string & filename);

  protected:
    void onNewFrame (const PointCloudConstPtr &cloud);
    void onKeyboardEvent (const pcl::visualization::KeyboardEvent & event);

    void waitForTrigger ();
          
    pcl::io::OpenNI2Grabber grabber_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> preview_;
    int frame_counter_;
    PointCloudPtr most_recent_frame_;
    bool use_trigger_, trigger_;
    boost::mutex mutex_;
};

