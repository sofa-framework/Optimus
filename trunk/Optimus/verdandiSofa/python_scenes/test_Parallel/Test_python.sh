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
 runSofa Test_python.scn

done
