#!/bin/sh

# values.csvを描画
gnuplot -e "
  set terminal vttek;
  set datafile separator ',';
  plot [0:20][-2:2] 'log.csv' using 1:2  with line, 'log.csv' using 4:5  with line
  "
