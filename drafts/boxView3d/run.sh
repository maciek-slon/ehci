#!/bin/sh
echo 'Make sure you run "make" before running this script'
LD_LIBRARY_PATH=/usr/local/lib/; export LD_LIBRARY_PATH      #linux path for opencv dynamic link libraries
DYLD_LIBRARY_PATH=/usr/local/lib/; export DYLD_LIBRARY_PATH  #osx   path for opencv dynamic link libraries
./boxView3d
