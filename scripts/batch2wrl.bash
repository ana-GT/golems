#!/bin/bash


DIR="/home/ana/Research/golems/scripts/ch3_shapes"
PLY2WRL_SCRIPT="/home/ana/Research/golems/scripts/ply2wrl.py"

cd $DIR;
# Use script to wrl
for PLY_FILE in `find . -type f -name "*.ply"`; do
  blender --background --python ${PLY2WRL_SCRIPT} -- ${PLY_FILE}
done
