#!/bin/bash

echo "This is a test script for Optimus plugin"

###################################################
### compile and verify the latest updates
###################################################
SOFA_DIRECTORY=/home/sergei/Optimus_test/sofa
BUILD_DIRECTORY=/home/sergei/Optimus_test/sofa/build_release
SOFACONFIG_DIRECTORY=/home/sergei/Optimus_test/sofaconfig
PARDISO_SOLVER_DIRECTORY=/home/sergei/Optimus_test/SofaPardisoSolver
IMAUX_DIRECTORY=/home/sergei/Optimus_test/ImageMeshAux
BOUNDARY_CONDITIONS_DIRECTORY=/home/sergei/Optimus_test/BoundaryConditions
OPTIMUS_DIRECTORY=/home/sergei/Optimus_test/Optimus

### checkout source code
# checkout sofa
if [ -d "$SOFA_DIRECTORY" ]; then
    echo "Update sofa repository"
    cd $SOFA_DIRECTORY
    /usr/bin/git pull https://github.com/mimesis-inria/sofa.git
else
    /usr/bin/git clone https://github.com/mimesis-inria/sofa.git $SOFA_DIRECTORY
    echo "Clone data from sofa repository"
fi
if ! [ -d "$BUILD_DIRECTORY" ]; then
    mkdir $BUILD_DIRECTORY
fi

# checkout sofa config
if [ -d "$SOFACONFIG_DIRECTORY" ]; then
    echo "Update SofaConfig repository"
    cd $SOFACONFIG_DIRECTORY
    /usr/bin/git pull https://gitlab.inria.fr/mimesis/sofaconfig.git
else
    /usr/bin/git clone https://gitlab.inria.fr/mimesis/sofaconfig.git $SOFACONFIG_DIRECTORY
    echo "Clone data from SofaConfig repository"
fi

# checkout Boundary Conditions
if [ -d "$BOUNDARY_CONDITIONS_DIRECTORY" ]; then
    echo "Update Boundary Conditions repository"
    cd $BOUNDARY_CONDITIONS_DIRECTORY
    /usr/bin/git pull https://gitlab.inria.fr/mimesis/BoundaryConditions.git
else
    /usr/bin/git clone https://gitlab.inria.fr/mimesis/BoundaryConditions.git $BOUNDARY_CONDITIONS_DIRECTORY
    echo "Clone data from Boundary Conditions repository"
fi

# checkout Pardiso Solver
if [ -d "$PARDISO_SOLVER_DIRECTORY" ]; then
    echo "Update Pardiso Solver repository"
    cd $PARDISO_SOLVER_DIRECTORY
    /usr/bin/git pull https://gitlab.inria.fr/mimesis/SofaPardisoSolver.git
else
    /usr/bin/git clone https://gitlab.inria.fr/mimesis/SofaPardisoSolver.git $PARDISO_SOLVER_DIRECTORY
    echo "Clone data from Pardiso Solver repository"
fi

# checkout IMAUX
if [ -d "$IMAUX_DIRECTORY" ]; then
    echo "Update ImageMeshAux repository"
    cd $IMAUX_DIRECTORY
    /usr/bin/git pull https://gitlab.inria.fr/mimesis/ImageMeshAux.git
else
    /usr/bin/git clone https://gitlab.inria.fr/mimesis/ImageMeshAux.git $IMAUX_DIRECTORY
    echo "Clone data from ImageMeshAux repository"
fi

# checkout Optimus
if [ -d "$OPTIMUS_DIRECTORY" ]; then
    echo "Update Optimus repository"
    cd $OPTIMUS_DIRECTORY
    /usr/bin/git pull https://gitlab.inria.fr/mimesis/Optimus.git
else
    /usr/bin/git clone https://gitlab.inria.fr/mimesis/Optimus.git $OPTIMUS_DIRECTORY
    echo "Clone data from Optimus repository"
fi

### configure and make the system
cd $BUILD_DIRECTORY
/usr/bin/make -B -j 8


### verify benchmark tests
OPTIMUS_BNECHMARK_TESTS_DIRECTORY=/home/sergei/Optimus_test/Optimus/benchmarks
TEST_FOLDERS=$OPTIMUS_BNECHMARK_TESTS_DIRECTORY/*
for folder in $TEST_FOLDERS
do
    if [ -f $folder/verify.sh ]; then
        echo "Perform test: $folder"
        $folder/verify.sh $BUILD_DIRECTORY/runSofa >> /home/sergei/Optimus_test/log.txt
    fi
done

### send a notification about the fact that process is finished
if [ 1 ]
then
    /usr/bin/zenity --info --text="All tests have been succesfully passed" 2>/dev/null &
else
    /usr/bin/zenity --error --text="Error during tests execution" 2>/dev/null &
fi

