#!/bin/sh

CRICHTON_SCENES_PATH="/home/ana/Research/commonData/scenes/crichton"
CRICHTON_CONFIG_PATH="/home/ana/Research/Code/GM/data/gripconfig_files"

# Functions
simple_sim() {
  grip -c ${CRICHTON_CONFIG_PATH}/crichton_graspv1.gripconfig 
}

shake_sim() { 
  grip -c ${CRICHTON_CONFIG_PATH}/crichton_shake.gripconfig 
}

unimanual_calib() {
 grip -c ${CRICHTON_CONFIG_PATH}/crichton_calib.gripconfig
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
shake)
 shake_sim
;; 
 *)
  echo "[ERROR] Invalid command. Options are: 0/calib/shake"
  exit 1
  ;;
esac	
