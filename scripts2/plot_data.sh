#!/bin/sh

# values.csvを描画
gnuplot -e "
  set terminal vttek;
  set datafile separator ',';
  plot [0:20][-2:2] 'data.csv' using 1:2  with line, 'data.csv' using 1:3  with line
  "
