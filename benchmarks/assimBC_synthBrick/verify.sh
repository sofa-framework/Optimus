#!/bin/bash

mkdir -p roukf_testing

echo "Generating observations..."
#runSofa -g batch -n 301 assimBC_synthBrick_GenObs.py &> genObsOut
echo "... done"

echo "Running data assimilation..."
#runSofa -g batch -n 300 assimBC_synthBrick_ROUKF.py &> sdaOut
echo "... done"

echo "Comparing state w.r.t. benchmark:"
python compArraysPerLine.py roukf_testing/state.txt roukf_benchmarked/state.txt 

echo "Comparing variance w.r.t. benchmark:"
python compArraysPerLine.py roukf_testing/variance.txt roukf_benchmarked/variance.txt 

echo "Comparing covariance w.r.t. benchmark:"
python compArraysPerLine.py roukf_testing/covariance.txt roukf_benchmarked/covariance.txt 
