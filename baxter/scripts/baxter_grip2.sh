#!/bin/sh

BAXTER_SCENES_PATH="${HOME}/Research/commonData/scenes/baxter"
BAXTER_CONFIG_PATH="${HOME}/Research/Code/GM/data/gripconfig_files"

# Functions
simple_sim() {
  grip -c ${BAXTER_CONFIG_PATH}/baxter_graspv1.gripconfig 
}


unimanual_calib() {
 grip -c ${BAXTER_CONFIG_PATH}/baxter_graspv1.gripconfig
}

###################
# Script options
###################
case "$1" in
 0)
  simple_sim
  ;;
 calib)
  unimanual_calib
;;
 *)
  echo "[ERROR] Invalid command. Options are: 0/calib/shake"
  exit 1
  ;;
esac	
