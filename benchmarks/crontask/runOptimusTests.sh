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
OPTIMUS_DIRECTORY=$GENERAL_DIRECTORY/Optimus
OPTIMUS_BUILD_DIRECTORY=$OPTIMUS_DIRECTORY/build_release
SOFA_PYTHON3_DIRECTORY=$GENERAL_DIRECTORY/SofaPython3
SOFA_PYTHON3_BUILD_DIRECTORY=$SOFA_PYTHON3_DIRECTORY/build_release


### export pardiso license
export PARDISO_LIC_PATH=$HOME_DIRECTORY/External_libraries/Pardiso
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME_DIRECTORY/External_libraries/Pardiso
export PYTHONPATH=$SOFA_PYTHON3_BUILD_DIRECTORY/lib/site-packages
export SOFA_PLUGIN_PATH=$OPTIMUS_BUILD_DIRECTORY


### checkout source code
### checkout sofa
echo $SOFA_DIRECTORY
if [ -d "$SOFA_DIRECTORY" ]; then
    echo "Update sofa repository"
    cd $SOFA_DIRECTORY
    /usr/bin/git pull --progress https://github.com/sofa-framework/sofa.git 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
else
    /usr/bin/git clone --progress https://github.com/sofa-framework/sofa.git $SOFA_DIRECTORY 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    echo "Clone data from sofa repository"
fi
if ! [ -d "$BUILD_DIRECTORY" ]; then
    mkdir $BUILD_DIRECTORY
fi

### checkout Python3
if [ -d "$SOFA_PYTHON3_DIRECTORY" ]; then
    echo "Update Python3 repository"
    cd $SOFA_PYTHON3_DIRECTORY
    /usr/bin/git pull --progress https://github.com/sofa-framework/SofaPython3.git 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
else
    /usr/bin/git clone --progress https://github.com/sofa-framework/SofaPython3.git $SOFA_PYTHON3_DIRECTORY 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    echo "Clone data from Python3 repository"
fi

### checkout Optimus
if [ -d "$OPTIMUS_DIRECTORY" ]; then
    echo "Update Optimus repository"
    cd $OPTIMUS_DIRECTORY
    /usr/bin/git pull --progress https://github.com/sofa-framework/Optimus.git 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
else
    /usr/bin/git clone --progress https://github.com/sofa-framework/Optimus.git $OPTIMUS_DIRECTORY 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    echo "Clone data from Optimus repository"
fi


### configure and make the system
echo "Recompile sofa sources"
cd $BUILD_DIRECTORY
/usr/bin/make clean
/usr/local/bin/cmake -DSOFA_BUILD_TESTS=ON -DSOFAGUI_BUILD_TESTS=ON -DSOFA_EXTERNAL_DIRECTORIES=/home/sergei/Optimus_test/sofaconfig/sergei .. 2>&1 >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
/usr/bin/make -B -j 8 2>&1 >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
/usr/bin/make install 2>&1 >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt

### remove SofaPython plugin with python2 from the plugins default list (use local version of list)
cp $OPTIMUS_DIRECTORY/benchmarks/crontask/plugin_list.conf.default $BUILD_DIRECTORY/lib/plugin_list.conf.default
cp $OPTIMUS_DIRECTORY/benchmarks/crontask/plugin_list.conf.default $BUILD_DIRECTORY/install/lib/plugin_list.conf.default


echo "Recompile python3 plugin"
if ! [ -d "$SOFA_PYTHON3_BUILD_DIRECTORY" ]; then
    mkdir $SOFA_PYTHON3_BUILD_DIRECTORY
fi
cd $SOFA_PYTHON3_BUILD_DIRECTORY
/usr/bin/make clean
/usr/local/bin/cmake -DSP3_BUILD_TEST=OFF -DCMAKE_PREFIX_PATH=$BUILD_DIRECTORY/install .. 2>&1 >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
/usr/bin/make -B -j 8 2>&1 >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
/usr/bin/make install 2>&1 >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt

echo "Recompile Optimus plugin"
if ! [ -d "$OPTIMUS_BUILD_DIRECTORY" ]; then
    mkdir $OPTIMUS_BUILD_DIRECTORY
fi
cd $OPTIMUS_BUILD_DIRECTORY
/usr/bin/make clean
/usr/local/bin/cmake -DCMAKE_PREFIX_PATH=$BUILD_DIRECTORY/install .. 2>&1 >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
/usr/bin/make -B -j 8 2>&1 >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
/usr/bin/make install 2>&1 >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt



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

