#!/bin/bash

echo "Generating observations..."
runSofa -g batch -n 301 assimBC_synthBrick_GenObs.py &> genObsOut
echo "... done"

echo "Running data assimilation..."
runSofa -g batch -n 300 assimBC_synthBrick_ROUKF.py &> sdaOut
echo "... done"

echo "Comparing state w.r.t. benchmark:"
python compArraysPerLine.py outROUKF/state.txt outROUKF/benchmarked/state.txt 

echo "Comparing variance w.r.t. benchmark:"
python compArraysPerLine.py outROUKF/variance.txt outROUKF/benchmarked/variance.txt 

echo "Comparing covariance w.r.t. benchmark:"
python compArraysPerLine.py outROUKF/covariance.txt outROUKF/benchmarked/covariance.txt 
