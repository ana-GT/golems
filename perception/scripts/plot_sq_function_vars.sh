#!/bin/bash

BIN_PATH="/home/ana/Research/golems/bin"

titles=('Radial' 'Solina' 'Ichim' 'Chevalier' 'F5' 'F6')

draw_t() {
 cmd="$cmd plot '${BIN_PATH}/t_$2.txt' using 1 with lines title '${titles[0]}'," 
 for i in {2..5}; do
   cmd="$cmd '${BIN_PATH}/t_$2.txt' using $i with lines title '${titles[$i-1]}',"
 done
   cmd="$cmd '${BIN_PATH}/t_$2.txt' using 6 with lines  title '${titles[5]}'"

 gnuplot -persist -e "$cmd"
}

draw_er1() {
 cmd="$cmd plot '${BIN_PATH}/er1_$2.txt' using 1 with lines title '${titles[0]}'," 
 for i in {2..5}; do
   cmd="$cmd '${BIN_PATH}/er1_$2.txt' using $i with lines title '${titles[$i-1]}',"
 done
   cmd="$cmd '${BIN_PATH}/er1_$2.txt' using 6 with lines  title '${titles[5]}'"

 gnuplot -persist -e "$cmd"
}

draw_er2() {
 cmd="$cmd plot '${BIN_PATH}/er2_$2.txt' using 1 with lines title '${titles[0]}'," 
 for i in {2..5}; do
   cmd="$cmd '${BIN_PATH}/er2_$2.txt' using $i with lines title '${titles[$i-1]}',"
 done
   cmd="$cmd '${BIN_PATH}/er2_$2.txt' using 6 with lines  title '${titles[5]}'"

 gnuplot -persist -e "$cmd"
}

draw_er4() {
 cmd="$cmd plot '${BIN_PATH}/er4_$2.txt' using 1 with lines title '${titles[0]}'," 
 for i in {2..5}; do
   cmd="$cmd '${BIN_PATH}/er4_$2.txt' using $i with lines title '${titles[$i-1]}',"
 done
   cmd="$cmd '${BIN_PATH}/er4_$2.txt' using 6 with lines  title '${titles[5]}'"

 gnuplot -persist -e "$cmd"
}

draw_e1() {
 cmd="$cmd plot '${BIN_PATH}/e1_$2.txt' using 2 with lines title '${titles[0]}'," 
 for i in {3..6}; do
   cmd="$cmd '${BIN_PATH}/e1_$2.txt' using $i with lines title '${titles[$i-2]}',"
 done
   cmd="$cmd '${BIN_PATH}/e1_$2.txt' using 7 with lines  title '${titles[5]}',"

 cmd="$cmd '${BIN_PATH}/e1_$2.txt' using 1 with lines title 'Baseline' lw 2" 


 gnuplot -persist -e "$cmd"
}

draw_e2() {

 cmd="$cmd plot '${BIN_PATH}/e2_$2.txt' using 1 with lines title 'Baseline' lw 2," 

 cmd="$cmd '${BIN_PATH}/e2_$2.txt' using 2 with lines title '${titles[0]}'," 
 for i in {3..6}; do
   cmd="$cmd '${BIN_PATH}/e2_$2.txt' using $i with lines title '${titles[$i-2]}',"
 done
   cmd="$cmd '${BIN_PATH}/e2_$2.txt' using 7 with lines  title '${titles[5]}'"

 gnuplot -persist -e "$cmd"
}


case "$1" in
  t)
   draw_t $@
  ;;
  er1)
   draw_er1 $@
  ;;
  er2)
   draw_er2 $@
  ;;
  er4)
   draw_er4 $@
  ;;
  e1)
   draw_e1 $@
  ;;
  e2)
   draw_e2 $@
  ;;
  *)
   echo "[ERROR] Only options are t/er1/er2/er4 with argument 0/1/2/3 (clean/noisy/partial clean/partial noisy. You gave me $1"
  ;;
esac
