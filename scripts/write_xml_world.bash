#!/bin/bash

OBJECT=${1}
OBJECT_XML="${OBJECT}.xml"
FILENAME="${OBJECT}_world.xml"
HAND_FILENAME="models/robots/Schunk/Schunk.xml"
TABLE_FILENAME="models/obstacles/ch3_shapes/LWA4_table.xml"
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
printf '\t\t\t<fullTransform>(+1 0 0 0)[700 -200 900]</fullTransform> \n' >> ${FILENAME}
printf '\t\t</transform> \n' >> ${FILENAME}
printf '\t </graspableBody> \n' >> ${FILENAME}


printf '\t <obstacle>\n' >> ${FILENAME}
printf '\t\t<filename>%s</filename>\n' ${TABLE_FILENAME} >> ${FILENAME}
printf '\t\t<transform> \n' >> ${FILENAME}
printf '\t\t\t<fullTransform>(+1 0 0 0)[500 101.6 0]</fullTransform> \n' >> ${FILENAME}
printf '\t\t</transform> \n' >> ${FILENAME}
printf '\t </obstacle> \n' >> ${FILENAME}

printf '\t <robot>\n' >> ${FILENAME}
printf '\t\t <filename>%s</filename> \n' ${HAND_FILENAME} >> ${FILENAME}
printf '\t\t <dofValues>0 0 0 0 0 0 0 0</dofValues> \n' >> ${FILENAME}
printf '\t\t <transform> \n' >> ${FILENAME}
printf '\t\t\t<fullTransform>(+1 0 0 0)[700 -200 1200]</fullTransform> \n' >> ${FILENAME}
printf '\t\t </transform> \n' >> ${FILENAME}
printf '\t </robot>\n' >> ${FILENAME}

printf '\t <camera>\n' >> ${FILENAME}
printf '\t\t <position>+3178 +334 +1822</position> \n' >> ${FILENAME}
printf '\t\t <orientation>0.36 0.418 0.60 0.57</orientation> \n' >> ${FILENAME}
printf '\t\t <focalDistance>2903</focalDistance> \n' >> ${FILENAME}
printf '\t </camera>\n' >> ${FILENAME}

printf '</world>\n' >> ${FILENAME}
