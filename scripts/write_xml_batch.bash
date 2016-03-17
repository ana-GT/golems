#!/bin/bash

WRL_DIR="/home/ana/Software/graspit/models/objects/ch3_9"
SCRIPT="/home/ana/Research/golems/scripts/./write_xml.bash"
MATERIAL="plastic"

cd ${WRL_DIR}

for WRL_FILE in `find . -type f -name "*.wrl"`; do
  # If you want to get rid of the ./ in wrl_file, add the line below
  WRL_FILE2=$(echo "${WRL_FILE}" | cut -f 2 -d '/' );
  ${SCRIPT} ${WRL_FILE2} ${MATERIAL} 
done


