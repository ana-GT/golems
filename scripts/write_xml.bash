#!/bin/bash

MESHNAME=${1}
MATERIAL=${2}
FILENAME=$(echo "${MESHNAME}" | cut -f 1 -d '.');
FILENAME=$(echo "${FILENAME}" | cut -f 2 -d '/');
FILENAME="${FILENAME}.xml"

if [ -e ${FILENAME} ]
 then
 # Remove it
 echo "** Removing existing ${FILENAME}!"
 rm ${FILENAME}
fi 

echo "<?xml version=\"1.0\" ?>" >> ${FILENAME}
echo "<root>" >> ${FILENAME}
echo "<material>${MATERIAL}</material>" >> ${FILENAME}
echo "<mass>500</mass>" >> ${FILENAME}
echo "<cog>0 0 0</cog>" >> ${FILENAME}
echo "<inertia_matrix>1 0 0 0 1 0 0 0 1</inertia_matrix>" >> ${FILENAME}
echo "<geometryFile type=\"Inventor\">${MESHNAME}</geometryFile>" >> ${FILENAME}
echo "</root>" >> ${FILENAME}
