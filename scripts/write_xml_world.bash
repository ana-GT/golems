#!/bin/bash

OBJECT=${1}
OBJECT_XML="${OBJECT}.xml"
FILENAME="${OBJECT}_world.xml"
HAND_FILENAME="models/robots/Schunk/Schunk.xml"

# Remove file if it exists
if [ -e ${FILENAME} ]
  then
  rm ${FILENAME}
fi

printf '<?xml version=\"1.0\" ?>\n' >> ${FILENAME}
printf '<world>\n' >> ${FILENAME}
printf '\t <graspableBody>\n' >> ${FILENAME}
printf '\t\t<filename>models/objects/ch3_shapes/%s</filename> \n' ${OBJECT_XML} >> ${FILENAME}
printf '\t\t<transform> \n' >> ${FILENAME}
printf '\t\t\t<fullTransform>(+1 0 0 0)[0 0 0]</fullTransform> \n' >> ${FILENAME}
printf '\t\t</transform> \n' >> ${FILENAME}
printf '\t </graspableBody> \n' >> ${FILENAME}
printf '\t <robot>\n' >> ${FILENAME}
printf '\t\t <filename>%s</filename> \n' ${HAND_FILENAME} >> ${FILENAME}
printf '\t\t <dofValues>0 0 0 0 0 0 0 0</dofValues> \n' >> ${FILENAME}
printf '\t\t <transform> \n' >> ${FILENAME}
printf '\t\t\t<fullTransform>(+1 0 0 0)[0 0 400]</fullTransform> \n' >> ${FILENAME}
printf '\t\t </transform> \n' >> ${FILENAME}
printf '\t </robot>\n' >> ${FILENAME}
printf '\t <camera>\n' >> ${FILENAME}
printf '\t\t <position>+512 +11 +92</position> \n' >> ${FILENAME}
printf '\t\t <orientation>0.49 0.48 0.51 0.51</orientation> \n' >> ${FILENAME}
printf '\t\t <focalDistance>501</focalDistance> \n' >> ${FILENAME}
printf '\t </camera>\n' >> ${FILENAME}
printf '</world>\n' >> ${FILENAME}
