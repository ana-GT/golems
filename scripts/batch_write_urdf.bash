#!/bin/bash

PLY_DIR="/home/ana/Research/commonData/objects/ch3_shapes/meshes"
SCRIPT="/home/ana/Research/golems/scripts/./write_urdf.bash"

cd ${PLY_DIR}

for PLY_FILE in `find . -type f -name "*.ply"`; do
  # If you want to get rid of the ./ in wrl_file, add the line below
  PLY_FILE2=$(echo "${PLY_FILE}" | cut -f 2 -d '/' );
  ${SCRIPT} ${PLY_FILE2}  
done


