import Sofa

import numpy as np
############################################################################################

class Kalman(Sofa.PythonScriptController):

    
    # called once the script is loaded
    def onLoaded(self,node):
        print 'Python.onLoaded called from node '+node.name
        self.rootNode = node
        self.simDOFs=node.getChild('SimuSphere').getObject('simDOFs')
        self.realDOFs=node.getChild('RealSphere').getObject('realDOFs')
    
    # called on each animation step
    def onBeginAnimationStep(self,dt):
        self.posSimu = self.simDOFs.findData('position').value
        string = str(self.posSimu)
        print 'Simu sphere Position '+ string
        
        self.velSimu = self.simDOFs.findData('velocity').value
        string = str(self.velSimu)
        print 'Simu sphere velocity '+ string
        
        self.posReal = self.realDOFs.findData('position').value
        string = str(self.posReal)
        print 'Real sphere Position '+ string



	#M = (np.array(self.posSimu) + np.array(self.posReal)) / 2
        ##c=np.average(data, axis=0)
	#M=c.split(",")
	N=np.mean( np.array([ self.posSimu, self.posReal]), axis=0 )
	self.M= N.tolist()
        print 'Mean sphere position' +str(self.M)
        #print 'Mean sphere Position '+ str(self.transpose)
        
        return 0

    def onEndAnimationStep(self,dt):
	#for i in range(len(self.posSimu)):
	  #self.posSimu[i]=self.M[i]
	  #self.simDOFs.findData('velocity').value = self.M
	  return 0
        
  
