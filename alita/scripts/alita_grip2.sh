#!/bin/sh

ALITA_SCENES_PATH="/home/ana/Research/commonData/scenes/alita"
ALITA_CONFIG_PATH="/home/ana/Research/Code/GM/data/gripconfig_files"

# Functions
simple_sim() {
 grip -c ${ALITA_CONFIG_PATH}/alita_graspv1.gripconfig 
}

unimanual_calib() {
 grip -c ${ALITA_CONFIG_PATH}/alita_calib.gripconfig
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
  echo "[ERROR] Invalid command. Options are: 0 / calib"
  exit 1
  ;;
esac	
