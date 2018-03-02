#!/bin/bash

echo "Generating observations..."
runSofa -g batch -n 301 assimBC_synthBrick_GenObs.py &> genObsOut
echo "... done"

echo "Running data assimilation..."
runSofa -g batch -n 300 assimBC_synthBrick_SDA.py &> sdaOut
echo "... done"

echo "Comparing state w.r.t. benchmark:"
python compArraysPerLine.py outSDA/state.txt outSDA/benchmarked/state.txt 

echo "Comparing variance w.r.t. benchmark:"
python compArraysPerLine.py outSDA/variance.txt outSDA/benchmarked/variance.txt 

echo "Comparing covariance w.r.t. benchmark:"
python compArraysPerLine.py outSDA/covariance.txt outSDA/benchmarked/covariance.txt 
