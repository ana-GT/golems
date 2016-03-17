
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>

#include <global/crichton_global.h>
#include <global/fsa_data.h>

#include "perception/ftts/fast_tabletop_segmentation.h"
#include "perception/msgs/perception_msgs.h"
#include "perception/pointcloud_tools/tabletop_symmetry/mindGapper.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter_t.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter_b.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter_m.h"
#include "object_recognition/ObjectsDatabase.h"
#include <pcl/filters/statistical_outlier_removal.h>

#include <mutex>

#include <ach.h>
#include <sns.h>


template <typename PointT>
class CrichtonView {

  public:

  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef pcl::PointCloud<pcl::Label> LabelCloud;
  typedef typename LabelCloud::Ptr LabelCloudPtr;

  CrichtonView();

  void mirrorState(int state, void* data);
  void startComm( int state, void* userdata );
  void storeCloud( char* _name, int _index, CloudPtr _cloud );
  void setGrabber();
  void setGUI();
  void grabber_callback( const CloudConstPtr& _cloud );
  void drawBoundingBox();

  void onMouse( int event, int x, int y, int flags, void* userdata ); // static

  void send( int state, void* userData );

void fit_SQ( CloudPtr _cluster, int _index,
	       std::vector<SQ_parameters> &_p,
	       ObjectEntry _oe );
  void create_table_mesh( pcl::PolygonMesh &_table_mesh,
			  char _table_name_mesh[50] );




  // Bounding box
  std::vector<Eigen::VectorXd> mClustersBB;
  int mSelectedCluster;
  bool mShowSegmentation;
  bool mMirror;

  // Global variables
  std::string mWindowName;
  cv::Mat mRgbImg, mPclMap;
  double mF;

  pcl::io::OpenNI2Grabber* mGrabber;
  Fast_Tabletop_Segmentation<PointT> mTts;
  std::mutex  mMutex;

  // Communication
  bool mChanReady;
  ach_channel_t mObj_param_chan; // Channel to send object param to planner
  ach_channel_t mServer2Module_chan; // Channel to receive commands from server
  ach_channel_t mModule2Server_chan; // Channel to send responses to server

  ObjectsDatabase mOd;
};


#include "CrichtonView.hpp"
