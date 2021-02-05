#!/bin/bash

echo "This is a test script for Optimus plugin"

##########################################
### compile and verify the latest updates
##########################################
HOME_DIRECTORY=/home/sergei
GENERAL_DIRECTORY=$HOME_DIRECTORY/Optimus_test
if ! [ -d "$GENERAL_DIRECTORY" ]; then
    mkdir $GENERAL_DIRECTORY
fi
SOFA_DIRECTORY=$GENERAL_DIRECTORY/sofa
BUILD_DIRECTORY=$GENERAL_DIRECTORY/sofa/build_release
SOFACONFIG_DIRECTORY=$GENERAL_DIRECTORY/sofaconfig
PARDISO_SOLVER_DIRECTORY=$GENERAL_DIRECTORY/SofaPardisoSolver
OPTIMUS_DIRECTORY=$GENERAL_DIRECTORY/Optimus
OPTIMUS_BUILD_DIRECTORY=$GENERAL_DIRECTORY/Optimus/build_release
SOFA_PYTHON3_DIRECTORY=$GENERAL_DIRECTORY/SofaPython3
SOFA_PYTHON3_BUILD_DIRECTORY=$SOFA_PYTHON3_DIRECTORY/build_release

### export pardiso license
export PARDISO_LIC_PATH=$HOME_DIRECTORY/External_libraries/Pardiso
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME_DIRECTORY/External_libraries/Pardiso
export PYTHONPATH=$SOFA_PYTHON3_BUILD_DIRECTORY/lib/site-packages
export SOFA_PLUGIN_PATH=$OPTIMUS_BUILD_DIRECTORY


### verify benchmark tests
echo "Execute benchmark tests"
OPTIMUS_BENCHMARK_TESTS_DIRECTORY=$OPTIMUS_DIRECTORY/benchmarks
TEST_FOLDERS=$OPTIMUS_BENCHMARK_TESTS_DIRECTORY/*
for FOLDER in $TEST_FOLDERS
do
    cd $FOLDER
    if [ -f $FOLDER/verify.sh ]; then
        echo "Perform test: $FOLDER"
        $FOLDER/verify.sh $BUILD_DIRECTORY/bin/runSofa $SOFA_PYTHON3_BUILD_DIRECTORY/lib >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    fi
done
echo "All tests have been executed"

### send a notification about the fact that process is finished
if [ 1 ]
then
    /usr/bin/zenity --info --text="All tests have been passed, verify the log please" --display=:0.0 2>/dev/null &
else
    /usr/bin/zenity --error --text="Error during tests execution" --display=:0.0 2>/dev/null &
fi

