#!/bin/bash

DIR="/home/ana/Research/dataset_asu"

cd $DIR;
echo `pwd`
echo `ls`
OUTPUT_FILE="$DIR/training_images.txt"
OUTPUT_LABEL="$DIR/training_labels.txt"

COUNTER=0;

for i in $( ls ); do

    if [ -d "${i}" ]; then 

      FOLDER="$DIR/$i"
      echo "Folder: $FOLDER"
      cd $FOLDER;
      for file in `find . -type f -name "${i}_cropped_rgb*"`; do
	  each_rgb=$(echo "$file" | cut -f 2 -d '/');
	  each_rgb_fullpath="$FOLDER/$each_rgb";
	  echo "$each_rgb_fullpath   $COUNTER" >> $OUTPUT_FILE
      done
      echo "$COUNTER $i" >> $OUTPUT_LABEL
      let COUNTER=COUNTER+1
      cd ..
    fi
done
