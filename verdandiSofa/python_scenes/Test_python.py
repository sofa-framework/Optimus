import Sofa
import math
import os
import Process


from decimal import Decimal

#
import inspect
#  
    
############################################################################################
# 
############################################################################################



class Test_python(Sofa.PythonScriptController):

    # Graph creation 
    def createGraph(self,node):
        print 'createGraph called (python side)'

                
        # self.process is an attribute of the class Compliance_only
        # self.process is an insance of the class Process
        self.process=Process.Process()
        

        

        # child2 = node.createChild('test2')  

        # Nodes creation

        self.process.createScene(node)

        
        #print dir(Sofa.Node)
        #print dir(Sofa)

        # Save rootnode in an attribute
        self.rootNode = node.getRoot()
        
        return 0



    def initGraph(self,node):
        print 'initGraph called (python side)'
        self.step    =     0
        self.total_time =     0
        
        self.process.initializationObjects(node)
        return 0


    def reset(self):
        print 'reset called (python side)'
        return 0
'''
    
    # called at each animation step
    def onBeginAnimationStep(self,dt):        
      
        self.step += 1
        self.total_time += dt

        # print "Start process 1"
        self.process.process1(self.step,self.total_time)
        # print "End process 1"
        return 0

    # called at each animation step
    def onEndAnimationStep(self,deltaTime) :
        # print "Start process 2"
        self.process.process2(self.step,self.total_time)
        # print "End process 2"
        return 0;
'''
