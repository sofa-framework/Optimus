import Sofa
import math
import os

#import numpy as np
from decimal import Decimal

#from CreateParallelizableScene import createParallelizableScene

# Class definition 
class CreateScene:   

############################"  
# Parameters definitions :
############################"
   
    # counts    
    m_sigmaPointType = 'simplex'
    m_slaveSceneCount = 0
    m_saveToFile = 0
    m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method
    #################################################################################
       
    #Get environment variable
    sofapythonData= os.environ["SofaPythonData"] # command line parameters 

    token=sofapythonData.split("_")
    if len(token)>32:
        print 'Python: Too many parameters'
        print "Usage : sh Test_python.sh sigmaPointType_threadCount_saveToFile[0/1]"
        print "Example : sh Test_python.sh simplex_4_1 "
        print " "
        print " Note: Use threadCount:=0 for serialized version."
        print " "
        print " "
    if len(token)<3:
        print 'Python: Save to file not specified. Not saving.'
    else:
        m_saveToFile=int(token[2])

    if len(token)<2:
        print 'Python: Thread count not given, creating serialized scene.'
        m_slaveSceneCount = 0
    else:
        m_slaveSceneCount = int(token[1])

    if len(token)<1:
        print 'Python: Sigma count not given, using simplex.'
    else:
        if (token[0] == 'simplex') or (token[0] == 'canonical') or (token[0] == 'star'):
            m_sigmaPointType = token[0]
            print m_sigmaPointType
    

    ########################################################################################################################################
    #                            Node creation                                                   
    ########################################################################################################################################     
        
    def createGlobalComponents(self, node):
        # scene global stuff
        node.createObject('RequiredPlugin', pluginName='Optimus', name='Optimus')
        node.createObject('RequiredPlugin', pluginName='SofaPardisoSolver', name='SofaPardisoSolver')
        
        node.findData('gravity').value="0 -9.81 0"
        node.findData('dt').value="0.01"
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1400 1000', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")        
        node.createObject('ROUKFilter', name="ROUKF", sigmaPointType="simplex", verbose="1")        
        node.createObject('MeshVTKLoader', filename="../../data/cylinder3_770.vtk", name="loader")



    def createCommontComponents(self, node):
        node.createObject('OptimParams', name="paramE", optimize="1", template="Vector", initValue="6000", stdev="2000", transformParams="1", loader="@/loader")                
        node.createObject('StaticSolver', applyIncrementFactor="1")
        node.createObject('SparsePARDISOSolver')        
        node.createObject('MechanicalObject', src="@/loader", name="Volume")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo", template="Vec3d")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
        node.createObject('UniformMass', totalMass="0.2513")
        node.createObject('BoxROI', name="fixedBox1", box="-0.05 -0.05 -0.002   0.05 0.05 0.002")
        node.createObject('BoxROI', name="fixedBox2", box="-0.05 -0.05  0.238   0.05 0.05 0.242")
        node.createObject('MergeSets', name="mergeIndices", in1="@fixedBox1.indices", in2="@fixedBox2.indices")
        node.createObject('FixedConstraint', indices="@mergeIndices.out")
        # node.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="@paramE.value", poissonRatio="0.45", method="large", computeVonMisesStress="0", drawHeterogeneousTetra="1")


    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        
        self.createCommontComponents(node)

        obsNode = node.createChild('obsNode')        
        obsNode.createObject('MechanicalObject', name='SourceMO', position="0.02 0 0.08    0.02 0 0.16    0.0141 0.0141 0.08    0.0141 -0.0141 0.08    0.0141 0.0141 0.16    0.0141 -0.0141 0.16    0.02 0 0.0533    0.02 0 0.107   \
		    0.02 0 0.133    0.02 0 0.187    0.02 0 0.213    0.0175 0.00961 0.0649    0.00925 0.0177 0.0647    0.0139 0.0144 0.0398    0.00961 -0.0175 0.0649    0.0177 -0.00925 0.0647  \
		    0.0144 -0.0139 0.0402    0.0177 0.00936 0.145    0.0095 0.0176 0.145    0.0175 0.00961 0.0951    0.00925 0.0177 0.0953    0.0139 0.0144 0.12    0.00937 -0.0177 0.145   \
		    0.0176 -0.00949 0.145    0.00935 -0.0177 0.0953    0.0176 -0.00949 0.095    0.0142 -0.0141 0.12    0.0177 0.00937 0.175    0.00949 0.0176 0.175    0.014 0.0143 0.2   \
		    0.00959 -0.0175 0.175    0.0177 -0.00924 0.175    0.0143 -0.014 0.2")
        obsNode.createObject('Sphere', radius="0.002", color="1 0 0 1")
        obsNode.createObject('BarycentricMapping')
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev="2e-3", noiseStdev="0.0", listening="1", stateWrapper="@../StateWrapper", verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix="../obsStiffnessDA/cylinder3_770_YMStat")

        obsVisuNode = node.createChild('SourceNode')
        obsVisuNode.createObject('MechanicalObject', name="aux_Source", position="@../obsNode/MOBS.mappedObservations")
        obsVisuNode.createObject('Sphere', radius="0.002", color="0 0 0.3 1")

 
    def createSlaveScene(self, node):
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='hideBehaviorModels hideForceFields hideCollisionModels')
        wrapper=node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        wrapper.findData("name").value = "StochasticWrapperSlave"
        wrapper.findData("slave").value = 1;        
        
        self.createCommontComponents(node)
        
        self.m_slaveScenesCreated+=1

    ########################################################################################################################################
    #                            Scene creation                                                   
    ########################################################################################################################################
    def createScene(self,node):
        r_slaves = [] # list of created auxiliary nodes
        self.createGlobalComponents(node)
                
        masterNode=node.createChild('MasterScene')
        self.createMasterScene(masterNode)        
        masterNode.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels')
        
        slaveSubordinate=node.createChild('SlaveSubordinate')
        for i in range(1,self.m_slaveSceneCount):
            slave=slaveSubordinate.createChild('SlaveScene_'+str(i))
            #slave.createObject('VisualStyle', name='VisualStyle', displayFlags='hideAll')
            self.createSlaveScene(slave)
            r_slaves.append(slave)        
            
        return 0

    #############################################################################################################
    # Initialization functions
    #
    ########################################################################################################################################
               
    def initializationObjects(self,node):          
        self.cameraReactivated=False

        self.rootNode=node
      
        #if(self.runFromScript):
        #    self.rootNode.getRootContext().animate = False
           
         
        print  "Init Graph called (Python side)"
        print " \n \n \n"
        return 0

