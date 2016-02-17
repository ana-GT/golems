#!/bin/bash

elem=( 'cubic' 'cubic_cylinder' 'sphere'  'cylinder' 'disk' 'irregular' 'ellipse_A' 'ellipse_B' 'ellipse_C' )
WRITE_WORLD_SCRIPT="/home/ana/Research/golems/scripts/./write_xml_world.bash"


for e in ${elem[@]}; do
  ${WRITE_WORLD_SCRIPT} ${e}
done
