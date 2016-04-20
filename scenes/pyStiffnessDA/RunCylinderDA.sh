#!/bin/sh
# ********************************************************************
# ***** START environment variable definitions                   *****
# ********************************************************************

#SofapythonData=2
#export SofapythonData 


# ********************************************************************
# ***** END environment variable definitions                    
# ********************************************************************

#if [ $# -eq 0 ]
# then
#   echo "No arguments supplied."
#fi

#2

if [ ! "$#" -eq 2 ]; then
	echo Two parameters needed, e.g.   CreateSceneCyl10.py  simplex_8_0
	exit
fi
	
echo "Executing version $1 with parameters $2" 

DO_RUN=0
if [ -L CreateScene.py ]; then
	rm CreateScene.py
	DO_RUN=1
fi

if [ ! -f CreateScene.py ]; then
	DO_RUN=1
fi

if [ "$DO_RUN" -eq 0 ]; then
 	echo "Cannot execute scene, CreateScene.py is not a symbolic link"
	exit
fi

ln -s $1 CreateScene.py 
export SofaPythonData=$2
export OMP_NUM_THREADS=1
#valgrind --log-file=val_par1.out runSofa Test_python.scn
runSofa pythonDA.scn
#runSofa -g batch -n 10 pythonDA.scn

