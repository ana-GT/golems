#!/bin/bash

#elem=( 'cubic_6_6_6' 'cubic_9_9_9' 'cubic_cylinder_7_7_10'  'cubic_cylinder_10_10_14' 'sphere_7_7_7' 'sphere_9_9_9' 'sphere_11_11_11' 'cylinder_5_5_12' 'cylinder_8_8_24' 'ellipsoid_5_5_6' 'ellipsoid_7_7_8' 'ellipsoid_9_9_12_a' 'ellipsoid_9_9_12_b' )

elem=('banana_mirrored' 'pringles_mirrored' 'brine_ball_mirrored' 'cheezit_mirrored' 'master_chef_coffee_mirrored' 'mike_monster_mirrored' 'soft_scrub_mirrored' 'sun_maid_raisins_mirrored' 'white_cup_mirrored' 'yellow_cone_mirrored' )

WRITE_WORLD_SCRIPT="/home/ana/Research/golems/scripts/./write_xml_world.bash"


for e in ${elem[@]}; do
  ${WRITE_WORLD_SCRIPT} ${e}
done
