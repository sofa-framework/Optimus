#!/bin/sh


FILENAME=$1

awk '{if (NF == 4) print $2" "$3" "$4 }' $FILENAME > nodes.mat
awk '{if (NF == 8 && $2 == "2" ) print $6" "$7" "$8}' $FILENAME > faces.mat
awk '{if (NF == 9 && $2 == "4" ) print $6" "$7" "$8" "$9}' $FILENAME > elems.mat

awk '{  POSZ=$3;
	if (POSZ < 0.001) print NR}' nodes.mat > X0_nodes.mat

awk ' BEGIN {PX=0.0; PY=0.0; PZ=0.20; minDist=1000; } \
	{ \
		X=$1; Y=$2; Z=$3;  \
		distQ=(X-PX)*(X-PX) + (Y-PY)*(Y-PY) + (Z-PZ)*(Z-PZ);
		if (distQ < minDist ) { \
			minDist = distQ; \
			minIndex = NR; \
			#print "Index " NR " distance " distQ  \
		}  \
	} \
	END {print "Minimal distance is "minDist" with index = "minIndex};' nodes.mat
	

