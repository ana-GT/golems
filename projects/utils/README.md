Description of the projects here included:

* grab_pointcloud.cpp
  --------------------
  - Added on January 22nd, 2016
  - Uses OpenNI2 grabber to get pointclouds from the Asus sensor. When "s" is pressed,
    a cloud is stored under the name grabbed_full_pcd.pcd

* ftts_segmentation_demo_1
  -------------------------
  - Added on January 22nd, 2016.	
  - Segments objects in a tabletop and show them colored.

* ftts_segmentation_demo_2
  ------------------------
  - Added on January 22nd, 2016
  - Same as demo_1 but with bounding boxes

* ftts_get_object_data_4
  -----------------------
  - Added on January 22nd, 2016.
  - Each time the user clicks on top of a bounding box, the program stores the RGB, depth,
    and overlays of the selected cluster. This is input for our object recognition system.

* perception_pick_proposal: Get one-view pointcloud (original color) + mirror pointcloud (random color) + mesh for proposal presentation
* perception_pick: Standard perception executable used to grab data
