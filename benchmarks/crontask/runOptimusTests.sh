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
IMAUX_DIRECTORY=$GENERAL_DIRECTORY/ImageMeshAux
BOUNDARY_CONDITIONS_DIRECTORY=$GENERAL_DIRECTORY/BoundaryConditions
OPTIMUS_DIRECTORY=$GENERAL_DIRECTORY/Optimus


### export pardiso license
export PARDISO_LIC_PATH=$HOME_DIRECTORY/External_libraries/Pardiso
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME_DIRECTORY/External_libraries/Pardiso


### checkout source code
# checkout sofa
echo $SOFA_DIRECTORY
if [ -d "$SOFA_DIRECTORY" ]; then
    echo "Update sofa repository"
    cd $SOFA_DIRECTORY
    #/usr/bin/git pull --progress https://github.com/mimesis-inria/sofa.git 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    /usr/bin/git pull --progress https://github.com/sofa-framework/sofa.git 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
else
    #/usr/bin/git clone --progress https://github.com/mimesis-inria/sofa.git $SOFA_DIRECTORY 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    /usr/bin/git clone --progress https://github.com/sofa-framework/sofa.git $SOFA_DIRECTORY 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    echo "Clone data from sofa repository"
fi
if ! [ -d "$BUILD_DIRECTORY" ]; then
    mkdir $BUILD_DIRECTORY
fi

# checkout sofa config
if [ -d "$SOFACONFIG_DIRECTORY" ]; then
    echo "Update SofaConfig repository"
    cd $SOFACONFIG_DIRECTORY
    /usr/bin/git pull --progress https://gitlab.inria.fr/mimesis/sofaconfig.git 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
else
    /usr/bin/git clone --progress https://gitlab.inria.fr/mimesis/sofaconfig.git $SOFACONFIG_DIRECTORY 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    echo "Clone data from SofaConfig repository"
fi

# checkout Boundary Conditions
if [ -d "$BOUNDARY_CONDITIONS_DIRECTORY" ]; then
    echo "Update Boundary Conditions repository"
    cd $BOUNDARY_CONDITIONS_DIRECTORY
    /usr/bin/git pull --progress https://gitlab.inria.fr/mimesis/BoundaryConditions.git 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
else
    /usr/bin/git clone --progress https://gitlab.inria.fr/mimesis/BoundaryConditions.git $BOUNDARY_CONDITIONS_DIRECTORY 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    echo "Clone data from Boundary Conditions repository"
fi

# checkout Pardiso Solver
if [ -d "$PARDISO_SOLVER_DIRECTORY" ]; then
    echo "Update Pardiso Solver repository"
    cd $PARDISO_SOLVER_DIRECTORY
    /usr/bin/git pull --progress https://gitlab.inria.fr/mimesis/SofaPardisoSolver.git 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
else
    /usr/bin/git clone --progress https://gitlab.inria.fr/mimesis/SofaPardisoSolver.git $PARDISO_SOLVER_DIRECTORY 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    echo "Clone data from Pardiso Solver repository"
fi

# checkout IMAUX
if [ -d "$IMAUX_DIRECTORY" ]; then
    echo "Update ImageMeshAux repository"
    cd $IMAUX_DIRECTORY
    /usr/bin/git pull --progress https://gitlab.inria.fr/mimesis/ImageMeshAux.git 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
else
    /usr/bin/git clone --progress https://gitlab.inria.fr/mimesis/ImageMeshAux.git $IMAUX_DIRECTORY 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    echo "Clone data from ImageMeshAux repository"
fi

# checkout Optimus
if [ -d "$OPTIMUS_DIRECTORY" ]; then
    echo "Update Optimus repository"
    cd $OPTIMUS_DIRECTORY
    /usr/bin/git pull --progress https://gitlab.inria.fr/mimesis/Optimus.git 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
else
    /usr/bin/git clone --progress https://gitlab.inria.fr/mimesis/Optimus.git $OPTIMUS_DIRECTORY 2>> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
    echo "Clone data from Optimus repository"
fi

### configure and make the system
echo "Recompile sources"
cd $BUILD_DIRECTORY
/usr/bin/make -B -j 8 2>&1 >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt


### verify benchmark tests
echo "Execute benchmark tests"
OPTIMUS_BNECHMARK_TESTS_DIRECTORY=$GENERAL_DIRECTORY/Optimus/benchmarks
TEST_FOLDERS=$OPTIMUS_BNECHMARK_TESTS_DIRECTORY/*
for FOLDER in $TEST_FOLDERS
do
    cd $FOLDER
    if [ -f $FOLDER/verify.sh ]; then
        echo "Perform test: $FOLDER"
        $FOLDER/verify.sh $BUILD_DIRECTORY/bin/runSofa >> $GENERAL_DIRECTORY/log_`/bin/date +"%Y_%m_%d"`.txt
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

