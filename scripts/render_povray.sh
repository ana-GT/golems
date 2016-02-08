#!/bin/bash

# Go to folder
filename=test_render

mkdir $filename
sleep 1

for file in `find . -type f -name "${filename}*.pov"`; do

   pov_file=$(echo "$file" | cut -f 2 -d '/');

   each_name=$(echo "$file" | cut -f 2 -d '.');
   each_name=$(echo "$each_name" | cut -f 2 -d '/');
   povray ${pov_file} Output_File_Name=${filename}/${each_name}.png
   echo $each_rgb
done
