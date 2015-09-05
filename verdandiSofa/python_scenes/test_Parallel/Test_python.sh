#!/bin/sh
# ********************************************************************
# ***** START environment variable definitions                   *****
# ********************************************************************

SofapythonData=2
export SofapythonData 


# ********************************************************************
# ***** END environment variable definitions                    
# ********************************************************************

#if [ $# -eq 0 ]
# then
#   echo "No arguments supplied."
#fi

for i in "$@"
do
 SofapythonData=$i
 #valgrind --log-file=val_par1.out runSofa Test_python.scn
 runSofa Test_python.scn
 #runSofa -g batch -n 1 Test_python.scn

done
