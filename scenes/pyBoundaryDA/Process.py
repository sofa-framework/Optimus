import Sofa # Required in every python scene

import os # To accesss environment variables
import time # For execution time

import numpy as np # for maths
from decimal import Decimal # for maths
import math # for maths

from subprocess import call # To launch terminal command
from subprocess import check_output  # To launch terminal command


import ModifyScene # Recalage is a subclass of ModifyScene

############################################################################################
# Class definition (subclass of ModifyScene)
############################################################################################

class Process(ModifyScene.ModifyScene):
	
############################"  
# Parameters definitions :
############################"	
	

############################"  
# Class function :
############################"	
	
	# Constructor
	def __init__(self): 
		print "Instance of process created"

	# First process (called in onBeginAnimationStep)
	def process1(self,step,total_time):
                print "p1"


	# Second process (called in onEndAnimationStep)
	def process2(self,step,total_time):
                print "p2"
