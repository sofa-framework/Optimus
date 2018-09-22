#!/bin/bash

if [ $# == "1" ]; then
        SOFA_EXEC=$1
else
        SOFA_EXEC=runSofa
fi
echo "Using SOFA executable: " $SOFA_EXEC

rm -rf obs_testing
mkdir -p obs_testing

rm -rf roukf_testing
mkdir -p roukf_testing

numItObs=501
numItSDA=500

echo "Generating observations..."
$SOFA_EXEC -g batch -n $numItObs cyl10Gravity_GenObs.py  &> genObsOut
echo "... done"

echo "Running data assimilation..."
$SOFA_EXEC -g batch -n $numItSDA cyl10Gravity_SDA.py &> sdaOut
echo "... done"

echo "Comparing state w.r.t. benchmark:"
python compArraysPerLine.py roukf_testing/state.txt roukf_benchmarked/state.txt 

echo "Comparing variance w.r.t. benchmark:"
python compArraysPerLine.py roukf_testing/variance.txt roukf_benchmarked/variance.txt 

echo "Comparing covariance w.r.t. benchmark:"
python compArraysPerLine.py roukf_testing/covariance.txt roukf_benchmarked/covariance.txt 
