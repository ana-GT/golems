#!/bin/sh

# Locations
CURRENT_LOC=`pwd`
DATASET_LOC="/home/ana/sq_dataset"
EXEC_LOC="/home/ana/Research/Code/GM/aoi/refresher/bin"
EXEC="./test_mirror"
       

# Run it
cd ${EXEC_LOC}
${EXEC} -p "${DATASET_LOC}/$1_$2.pcd" -m "${DATASET_LOC}/$1_$2.txt" 
cd ${CURRENT_LOC}
