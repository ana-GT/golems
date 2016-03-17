#!/bin/bash

FOLDER_NAME=${2}
FULL_URDF_FILE=${1}
URDF_FILE=$(echo "${FULL_URDF_FILE}" | cut -f 2 -d '/');
URDF_NAME=$(echo "${URDF_FILE}" | cut -f 1 -d '.');
FILENAME="${URDF_NAME}_world.urdf"

if [ -e ${FILENAME} ]
 then
 # Remove it
 echo "** Removing existing ${FILENAME}!"
 rm ${FILENAME}
fi 


printf '  <world name="golems_ch3_world">\n' >> ${FILENAME}
printf '  <!-- Crichton urdf --> \n' >> ${FILENAME}
printf '  <include filename="../../robots/lwa4d/urdf/lwa4d_left_phys_box.urdf" model_name="lwa4d_left"/> \n' >> ${FILENAME} 
printf '  <include filename="../../robots/lwa4d/urdf/lwa4d_right_phys_box.urdf" model_name="lwa4d_right"/> \n' >> ${FILENAME}  
printf '  <include filename="../../robots/sdh/urdf/sdh.urdf" model_name="sdh"/> \n' >> ${FILENAME} 
printf '  <include filename="../../objects/LWA4_table/urdf/LWA4_verticalSupport.urdf" model_name="base"/> \n' >> ${FILENAME}
printf '  <include filename="../../objects/LWA4_table/urdf/LWA4_middleSupport_mod.urdf" model_name="mid"/> \n' >> ${FILENAME} 
printf '  <!-- Objects --> \n' >> ${FILENAME}
printf '  <include filename="../../objects/%s/urdf/%s.urdf" model_name="%s"/> \n' ${FOLDER_NAME} ${URDF_NAME} ${URDF_NAME} >> ${FILENAME}
printf '  <!-- Table urdf --> \n' >> ${FILENAME}
printf '  <include filename="../../objects/LWA4_table/urdf/table_alone.urdf" model_name="table"/> \n' >> ${FILENAME}
printf '  <!-- Crichton --> \n' >> ${FILENAME}
printf '  <entity model="base" name="base"> \n' >> ${FILENAME}
printf '\t <origin xyz="-0.45 -0.45 0.01" rpy="+0 0 0"/> \n' >> ${FILENAME}
printf '  </entity> \n' >> ${FILENAME}
printf '  <entity model="mid" name="mid"> \n' >> ${FILENAME}
printf '\t <origin xyz="0.01 0.01 1.23" rpy="0 0 0"/> \n' >> ${FILENAME}
printf '  </entity> \n' >> ${FILENAME}
printf '  <entity model="lwa4d_left" name="crichtonLeftArm"> \n' >> ${FILENAME}
printf '\t <origin xyz="0.0 0.0 +1.23" rpy="0 0 0"/> \n' >> ${FILENAME}
printf '  </entity> \n' >> ${FILENAME}
printf '  <entity model="lwa4d_right" name="crichtonRightArm"> \n' >> ${FILENAME}
printf '\t <origin xyz="0.0 0.0 +1.23" rpy="0 0 3.14157"/> \n' >> ${FILENAME}
printf '  </entity> \n' >> ${FILENAME}
printf '  <entity model="sdh" name="crichtonLeftHand"> \n' >> ${FILENAME}
printf '\t    <origin xyz="0.6 0.6625 +3.0" rpy="0 0 0"/> \n' >> ${FILENAME}
printf '  </entity> \n' >> ${FILENAME}
printf '  <entity model="sdh" name="crichtonRightHand"> \n' >> ${FILENAME}
printf '\t    <origin xyz="0.6 -0.6625 +3.0" rpy="0 0 0"/> \n' >> ${FILENAME}
printf '  </entity> \n' >> ${FILENAME}
printf '  <!-- Balls --> \n' >> ${FILENAME}
printf '  <entity model="%s" name="%s"> \n' ${URDF_NAME} ${URDF_NAME} >> ${FILENAME}
printf '\t    <origin xyz="0.5 0.5 0.72" rpy="0 0 0"/> \n' >> ${FILENAME}
printf '  </entity> \n' >> ${FILENAME}
printf '  <!-- Table --> \n' >> ${FILENAME}
printf ' <entity model="table" name="table"> \n' >> ${FILENAME}
printf '\t    <origin xyz="0.5 0.1016 0.0" rpy="0 0 0"/> \n' >> ${FILENAME}
printf '  </entity> \n' >> ${FILENAME}
printf '</world> \n' >> ${FILENAME}

