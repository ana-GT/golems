#!/bin/bash

FULL_MESHFILE=${1}
MESHFILE=$(echo "${FULL_MESHFILE}" | cut -f 2 -d '/');
MESHNAME=$(echo "${MESHFILE}" | cut -f 1 -d '.');
FILENAME="${MESHNAME}.urdf"

if [ -e ${FILENAME} ]
 then
 # Remove it
 echo "** Removing existing ${FILENAME}!"
 rm ${FILENAME}
fi 

printf '<robot \n' >> ${FILENAME}

printf 'name=\"%s\"> \n' ${MESHNAME} >> ${FILENAME}
printf '\t <link name=\"%s\"> \n' ${MESHNAME} >> ${FILENAME}
printf '\t\t <inertial> \n' >> ${FILENAME}
printf '\t\t\t <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/> \n' >> ${FILENAME}
printf '\t\t\t <mass value=\"0.5\"/> \n' >> ${FILENAME}
printf '\t\t\t <inertia ixx=\"1\" ixy=\"0\" ixz=\"0\" iyy=\"1\" iyz=\"0\" izz=\"1\" /> \n' >> ${FILENAME}
printf '\t\t </inertial> \n' >> ${FILENAME}
printf '\t\t <visual> \n' >> ${FILENAME}
printf '\t\t\t <geometry> \n' >> ${FILENAME}
printf '\t\t\t\t <mesh filename=\"../meshes/%s\" scale=\"1 1 1\" /> \n' ${MESHFILE} >> ${FILENAME}
printf '\t\t\t </geometry> \n' >> ${FILENAME}
printf '\t\t </visual> \n' >> ${FILENAME}
printf '\t\t <collision> \n' >> ${FILENAME}
printf '\t\t\t <geometry> \n' >> ${FILENAME}
printf '\t\t\t\t <mesh filename=\"../meshes/%s\" scale=\"1 1 1\" /> \n' ${MESHFILE} >> ${FILENAME}
printf '\t\t\t </geometry> \n' >> ${FILENAME}
printf '\t\t </collision> \n' >> ${FILENAME}
printf '\t </link> \n' >> ${FILENAME}
printf '</robot> \n' >> ${FILENAME}

