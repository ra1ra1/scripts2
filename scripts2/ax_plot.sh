#!/bin/sh

# values.csvを描画
gnuplot -e "
  set terminal vttek;
  set datafile separator ',';
  plot [0:25][-4:4] 'data.csv' using 1:2  with line, 'data.csv' using 1:3  with line
  "
