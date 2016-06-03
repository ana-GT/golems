#!/bin/sh

CRICHTON_SCENES_PATH="/home/ana/Research/commonData/scenes/crichton"
CRICHTON_CONFIG_PATH="/home/ana/Research/Code/GM/data/gripconfig_files"

# Functions
simple_sim() {
  grip -c ${CRICHTON_CONFIG_PATH}/crichton_graspv1.gripconfig 
}

pap_sim() {
  grip -c ${CRICHTON_CONFIG_PATH}/crichton_graspv2.gripconfig 
}

papr_sim() {
  grip -c ${CRICHTON_CONFIG_PATH}/crichton_grasp_papr.gripconfig 
}
pouring_sim() {
  grip -c ${CRICHTON_CONFIG_PATH}/crichton_grasp_pouring.gripconfig 
}
handover_sim() {
  grip -c ${CRICHTON_CONFIG_PATH}/crichton_grasp_double_handover.gripconfig 
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
 1)
  simple_sim
  ;;
 calib)
  unimanual_calib
;;
shake)
 shake_sim
;; 
2)
 pap_sim
;;
3)
 papr_sim
;;
4)
 pouring_sim
;;
5)
 handover_sim
;;
 *)
  echo "[ERROR] Invalid command. Options are: 1/2/3/4/5/calib/shake"
  exit 1
  ;;
esac	
