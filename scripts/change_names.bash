#!/bin/bash


# Go to folder
DIR="/home/ana/Desktop/Crichton_data/brush_celester"


cd $DIR;
echo `pwd`

for i in $( ls ); do
   NEW_NAME=`echo ${i} | sed -e "s/brush_celester/light_blue_brush/"`
   echo ${NEW_NAME}
   mv ${i} ${NEW_NAME}
done

