#!/bin/bash

var='output_Sunday_14_24_54.txt'

draw_pos() {
 cmd="$cmd plot '$2' using 1:2,'$2' using 1:9," 
 for i in {3..7}; do
   cmd="$cmd '$2' using 1:$i,'$2' using 1:$[$i+7],"
 done
   cmd="$cmd '$2' using 1:8,'$2' using 1:15"

 gnuplot -persist -e "$cmd"
}


draw_vel() {
 cmd="$cmd plot '$2' using 1:16,'$2' using 1:23," 
 for i in {17..21}; do
   cmd="$cmd '$2' using 1:$i,'$2' using 1:$[$i+7],"
 done
   cmd="$cmd '$2' using 1:22,'$2' using 1:29"

 gnuplot -persist -e "$cmd"
}

draw_acc() {
 cmd="$cmd plot '$2' using 1:30," 
 for i in {31..35}; do
   cmd="$cmd '$2' using 1:$i,"
 done
   cmd="$cmd '$2' using 1:36"

 gnuplot -persist -e "$cmd"
}


case "$1" in
  pos)
    echo "Second argument: $2"
   draw_pos $@
  ;;
  vel)
   draw_vel $@
  ;;
  acc)
   draw_acc $@
  ;;
  *)
   echo "[ERROR] Only options are pos/vel"
  ;;
esac
