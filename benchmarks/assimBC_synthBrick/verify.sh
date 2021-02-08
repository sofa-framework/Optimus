#!/bin/bash

if [ $# == "2" ]; then
        SOFA_EXEC="$1 -l $2/libSofaPython3.so"
elif [ $# == "1" ]; then
        SOFA_EXEC="$1 -l libSofaPython3.so"
else
        SOFA_EXEC="runSofa -l libSofaPython3.so"
fi
echo "Using SOFA executable: " $SOFA_EXEC

numItSDA=300
goScene=assimBC_synthBrick_GenObs.py
sdaScene=assimBC_synthBrick_ROUKF.py
compareScript=../helper/compareData.py

#==============================================================

numItObs=$[$numItSDA+1]

rm -rf obs_testing
mkdir -p obs_testing

rm -rf roukf_testing
mkdir -p roukf_testing

echo "Generating observations..."
$SOFA_EXEC -g batch -n $numItObs $goScene &> genObsOut
echo "... done"

echo "Running data assimilation..."
$SOFA_EXEC -g batch -n $numItSDA $sdaScene &> sdaOut
echo "... done"

echo "Comparing state w.r.t. benchmark:"
python $compareScript roukf_testing/state.txt roukf_benchmarked/state.txt $numItSDA

echo "Comparing variance w.r.t. benchmark:"
python $compareScript roukf_testing/variance.txt roukf_benchmarked/variance.txt $numItSDA

echo "Comparing covariance w.r.t. benchmark:"
python $compareScript roukf_testing/covariance.txt roukf_benchmarked/covariance.txt $numItSDA
