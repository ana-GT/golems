#!/bin/bash

FOLDER_NAME="ch3_9"
URDF_DIR="/home/ana/Research/commonData/objects/${FOLDER_NAME}/urdf"
SCRIPT="/home/ana/Research/golems/scripts/./write_world.bash"

cd ${URDF_DIR}

for URDF_FILE in `find . -type f -name "*.urdf"`; do
  # If you want to get rid of the ./ in wrl_file, add the line below
  URDF_FILE2=$(echo "${URDF_FILE}" | cut -f 2 -d '/' );
  ${SCRIPT} ${URDF_FILE2}  ${FOLDER_NAME}
done


