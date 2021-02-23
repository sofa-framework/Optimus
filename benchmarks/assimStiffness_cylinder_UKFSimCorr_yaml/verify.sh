#!/bin/bash

if [ $# == "2" ]; then
        SOFA_EXEC="$1 -l $2/libSofaPython3.so"
elif [ $# == "1" ]; then
        SOFA_EXEC="$1 -l libSofaPython3.so"
else
        SOFA_EXEC="runSofa -l libSofaPython3.so"
fi
echo "Using SOFA executable: " $SOFA_EXEC

numItSDA=2500
goScene=cylConstForce_GenObs_bench.py
sdaScene=cylConstForce_SDA_bench.py
yamlConfig=cylConstForce_scene_config_bench.yml
compareScript=../helper/compareData.py

#==============================================================

numItObs=$[$numItSDA+1]

rm -rf obs_testing
mkdir -p obs_testing

rm -rf ukf_sim_corr_testing
mkdir -p ukf_sim_corr_testing

echo "Generating observations..."
$SOFA_EXEC -g batch -n $numItObs $goScene --argv $yamlConfig &> genObsOut 2>&1
echo "... done"

echo "Running data assimilation..."
$SOFA_EXEC -g batch -n $numItSDA $sdaScene --argv $yamlConfig &> sdaOut 2>&1
echo "... done"

echo "Comparing state w.r.t. benchmark:"
python $compareScript ukf_sim_corr_bench/state_test.txt ukf_sim_corr_testing/state_test.txt $numItSDA

echo "Comparing variance w.r.t. benchmark:"
python $compareScript ukf_sim_corr_bench/variance_test.txt ukf_sim_corr_testing/variance_test.txt $numItSDA

echo "Comparing covariance w.r.t. benchmark:"
python $compareScript ukf_sim_corr_bench/covariance_test.txt ukf_sim_corr_testing/covariance_test.txt $numItSDA

echo "Comparing innovation w.r.t. benchmark:"
python $compareScript ukf_sim_corr_bench/innovation_test.txt ukf_sim_corr_testing/innovation_test.txt $numItSDA
