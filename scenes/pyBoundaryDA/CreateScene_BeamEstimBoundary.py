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
        
    # components in the scene header: required plugins, viewer setting, animation loop, filter, mesh loaders 
    def createGlobalComponents(self, node):
        # scene global stuff
        node.createObject('RequiredPlugin', pluginName='Optimus', name='Optimus')
        node.createObject('RequiredPlugin', pluginName='SofaPardisoSolver', name='SofaPardisoSolver')
        node.createObject('RequiredPlugin', pluginName='ImageMeshAux', name='ImageMeshAux')
        
        node.findData('gravity').value="0 0 0"
        node.findData('dt').value="0.01"
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")        
        node.createObject('ROUKFilter', name="ROUKF", sigmaPointType="simplex", verbose="1")        
        node.createObject('MeshVTKLoader', name="loader", filename="../../data/beamA1_1635.vtk")
        node.createObject('MeshSTLLoader', name="sloader",filename="../../data/beamA1_1635.stl")
        

    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createCommonComponents(self, node):  
        node.createObject('OptimParams', name="springStiffness", template="Vector", numParams="30",initValue="50", stdev="10", transformParams="1", optimize="1", printLog="1")
                        
        node.createObject('StaticSolver', applyIncrementFactor="0")
        node.createObject('SparsePARDISOSolver')        
        node.createObject('MechanicalObject', src="@/loader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass="0.2513")

        node.createObject('BoxROI', name="fixedBox1", box="-0.001 -0.001 -0.001   0.05 0.001 0.08")        
        node.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points="@fixedBox1.indices", angularStiffness="0", stiffness="@springStiffness.value", springThickness="1", listening="1", updateStiffness="1", printLog="0")
        node.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        node.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large")

        toolEmu = node.createChild('toolEmu')
        toolEmu.createObject('MechanicalObject',name="MO",position="0.16 0.06 0.01  0.16 0.06 0.025  0.16 0.06 0.040  0.16 0.06 0.055   0.16 0.06 0.070   0.175 0.06 0.01  0.175 0.06 0.025  0.175 0.06 0.040  0.175 0.06 0.055   0.175 0.06 0.070")
        toolEmu.createObject('LinearMotionStateController',name="SlaveController",indices="0 1 2 3 4 5 6 7 8 9",keyDisplacements="0 0 0    0.05 0.05 0", keyTimes="0 2",printLog="1")
        toolEmu.createObject('Sphere',radius="0.002")
        
        node.createObject('Mapped3DoFForceField', mappedFEM="mappedTool/toolSpring", mappedMechObject="mappedTool/MO", mapping="mappedTool/baryMapping", printLog="0")        
        toolMapped = node.createChild('mappedTool');
        toolMapped.createObject('MechanicalObject',name="MO", position="0.16 0.06 0.01  0.16 0.06 0.025  0.16 0.06 0.040  0.16 0.06 0.055   0.16 0.06 0.070   0.175 0.06 0.01  0.175 0.06 0.025  0.175 0.06 0.040  0.175 0.06 0.055   0.175 0.06 0.070")        
        toolMapped.createObject('ExtendedRestShapeSpringForceField', name="toolSpring", angularStiffness="0", stiffness="1e5", external_rest_shape="../toolEmu/MO", springThickness="1", listening="1", updateStiffness="1", springColor="0 1 0 1",startTimeSpringOn="0",numStepsSpringOn="10000")

        toolMapped.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        toolMapped.createObject('Sphere',radius="0.002",color="0 0 1 1") 
        toolMapped.createObject('BarycentricMapping',name="baryMapping")




    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        
        self.createCommonComponents(node)

        obsNode = node.createChild('obsNode')
        obsNode.createObject('MechanicalObject', name='MO', position="0.01 0.06 0.01  0.01 0.06 0.04  0.01 0.06 0.07  0.03 0.06 0.01  0.03 0.06 0.04  0.03 0.06 0.07  0.05 0.06 0.01  0.05 0.06 0.04  0.05 0.06 0.07  0.07 0.06 0.01  0.07 0.06 0.04  0.07 0.06 0.07  0.09 0.06 0.01  0.09 0.06 0.04  0.09 0.06 0.07  0.11 0.06 0.01  0.11 0.06 0.04  0.11 0.06 0.07  0.13 0.06 0.01  0.13 0.06 0.04  0.13 0.06 0.07  0.15 0.06 0.01  0.15 0.06 0.04  0.15 0.06 0.07")
        #obsNode.createObject('MechanicalObject', name='MO', position="0.01 0.06 0.01  0.01 0.06 0.04  0.01 0.06 0.07  0.03 0.06 0.01  0.03 0.06 0.04  0.03 0.06 0.07  0.05 0.06 0.01  0.05 0.06 0.04  0.05 0.06 0.07  0.07 0.06 0.01  0.07 0.06 0.04  0.07 0.06 0.07  0.09 0.06 0.01  0.09 0.06 0.04  0.09 0.06 0.07  0.11 0.06 0.01  0.11 0.06 0.04  0.11 0.06 0.07  0.13 0.06 0.01  0.13 0.06 0.04  0.13 0.06 0.07  0.15 0.06 0.01  0.15 0.06 0.04  0.15 0.06 0.07   0.18 0.01 0.01  0.18 0.01 0.04  0.18 0.01 0.07  0.18 0.03 0.01  0.18 0.03 0.04  0.18 0.03 0.07  0.18 0.05 0.01  0.18 0.05 0.04  0.18 0.05 0.07")
        obsNode.createObject('Sphere', radius="0.001", color="0.2 0.8 0.2 1")
        obsNode.createObject('BarycentricMapping')
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev="1e-4", noiseStdev="0.0", listening="1", stateWrapper="@../StateWrapper", verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix="../obsBoundaryDA/beam1635_A")
    
        # visNode = node.createChild('BeamVisualization')
        # visNode.createObject('MechanicalObject',src="@/sloader",name="Surface")
        # visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@/sloader", tags=" ")
        # visNode.createObject('TriangleSetTopologyModifier', name="Modifier")        
        # visNode.createObject('TriangleSetTopologyAlgorithms', name="TopoAlgo")
        # visNode.createObject('TriangleSetGeometryAlgorithms', name="GeomAlgo")        
        # visNode.createObject('Line',color="0 0 0 1")
        # visNode.createObject('Triangle',color="1 0 0 1")
        # visNode.createObject('BarycentricMapping')
        #visNode.createObject('VTKExporter',filename="vtkExp/beam",XMLformat="true",listening="true",edges="0",triangles="1",quads="0",tetras="0",exportAtBegin="1",exportAtEnd="0",exportEveryNumberOfSteps="1")

        obsVisuNode = node.createChild('ObservationVisualization')
        obsVisuNode.createObject('MechanicalObject', name="aux_Source", position="@../obsNode/MOBS.mappedObservations")
        obsVisuNode.createObject('Sphere', radius="0.002", color="0.2 0.8 0.2 1")
 
 
    def createSlaveScene(self, node):
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='hideBehaviorModels hideForceFields hideCollisionModels')
        wrapper=node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        wrapper.findData("name").value = "StochasticWrapperSlave"
        wrapper.findData("slave").value = 1;        
        
        self.createCommonComponents(node)
        
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

