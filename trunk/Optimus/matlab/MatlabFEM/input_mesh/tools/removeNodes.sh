#!/bin/sh


FILENAME=elems.mat;
awk '  {A=$1; B=$2; C=$3; D=$4; \
	A=(A>7) ? A-2 : A-1;\
	B=(B>7) ? B-2 : B-1;\
	C=(C>7) ? C-2 : C-1;\
	D=(D>7) ? D-2 : D-1;\
	print A" "B" "C" "D }'  $FILENAME > x
mv x elems.mat


FILENAME=faces.mat;
awk '  {A=$1; B=$2; C=$3; \
	A=(A>7) ? A-2 : A-1;\
	B=(B>7) ? B-2 : B-1;\
	C=(C>7) ? C-2 : C-1;\
	print A" "B" "C}'  $FILENAME > x
mv x faces.mat
			
