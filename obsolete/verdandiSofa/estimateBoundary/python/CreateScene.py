import Sofa
import math
import os

#import numpy as np
from decimal import Decimal

from CreateParallelizableScene import createParallelizableScene

# Class definition 
class CreateScene:   

############################"  
# Parameters definitions :
############################"

    # Set to true if you want to run batch of simulation
    runFromScript=True    

    # counts    
    m_sigmaPointType = 'simplex'
    m_slaveSceneCount = 0
    m_saveToFile = 0

    m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method

    #############################################################################################################################   
    
    if(runFromScript):
        #Get environment variable
        sofapythonData= os.environ["SofapythonData"] # command line parameters 
    
        token=sofapythonData.split("_")
        numTokens = len(token)        
        print 'Tokens: %d' % numTokens
        if len(token)>3:
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
        node.createObject('RequiredPlugin', pluginName='ImageMeshAux', name='ImageMeshAux')
        node.findData('gravity').value="0 0 0"
        node.findData('dt').value="0.01"
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1400 1000', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('VerdandiAnimationLoop', name="verdAnimLoop", verbose="0")

        #simplex, canonical, star
        ROUKF = node.createObject('SofaReducedOrderUKFParallel', name="sofaROUKF", sigmaPointType="simplex", paramFileName="../estimationPython/exp1.out", paramVarFileName="../estimationPython/exp1_vars.out")
        ROUKF.findData('sigmaPointType').value=self.m_sigmaPointType        
        node.createObject('MeshVTKLoader', filename="../../Data/beamA1_1635.vtk", name="loader")
        


    def createMasterScene(self, node):
        
        node.createObject('OptimParams', name="springStiffness", template="Vector", numParams="96",initValue="50", stdev="10", transformParams="1")
                                        
        bcs = node.createChild('bcScene')
        bcs.findData('activated').value="1"

        bcs.createObject('StaticSolver', applyIncrementFactor="0")

        PARDISO = bcs.createObject('SparsePARDISOSolver')
        if (self.m_saveToFile != 0):
            pardisoLabel = "Master"
            
        bcs.createObject('MechanicalObject', src="@/loader", name="Volume")
        bcs.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        bcs.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        bcs.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo", template="Vec3d")
        bcs.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
        bcs.createObject('UniformMass', totalMass="0.2513")
                
        bcs.createObject('BoxROI', name="fixedBox1", box="-0.001 -0.001 -0.001   0.18 0.001 0.08")        
        bcs.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points="@fixedBox1.indices", stiffness="@../springStiffness.value", drawSpring="1", listening="1", forceDir="springForces", updateStiffness="1", printLog="1")
        bcs.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        bcs.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large", computeVonMisesStress="0", drawHeterogeneousTetra="0")
                                
        toolEmu = bcs.createChild('toolEmu')
        toolEmu.createObject('MechanicalObject',name="MO",position="0.16 0.06 0.01  0.16 0.06 0.025  0.16 0.06 0.040  0.16 0.06 0.055   0.16 0.06 0.070   0.175 0.06 0.01  0.175 0.06 0.025  0.175 0.06 0.040  0.175 0.06 0.055   0.175 0.06 0.070")
        toolEmu.createObject('LinearMotionStateController',indices="0 1 2 3 4 5 6 7 8 9",keyDisplacements="0 0 0    0.05 0.05 0", keyTimes="0 2")
        toolEmu.createObject('Sphere',radius="0.002")
        
        bcs.createObject('Mapped3DoFForceField', mappedFEM="mappedTool/toolSpring", mappedMechObject="mappedTool/MO", mapping="mappedTool/baryMapping", printLog="1")        
        tool = bcs.createChild('mappedTool');
        tool.createObject('MechanicalObject',name="MO", position="0.16 0.06 0.01  0.16 0.06 0.025  0.16 0.06 0.040  0.16 0.06 0.055   0.16 0.06 0.070   0.175 0.06 0.01  0.175 0.06 0.025  0.175 0.06 0.040  0.175 0.06 0.055   0.175 0.06 0.070")
        tool.createObject('ExtendedRestShapeSpringForceField', name="toolSpring", angularStiffness="0", stiffness="1e5", external_rest_shape="../toolEmu/MO", drawSpring="1", listening="1", updateStiffness="1", springColor="0 1 0 1")
        tool.createObject('Sphere',radius="0.002",color="0 0 1 1") 
        tool.createObject('BarycentricMapping',name="baryMapping")
 
        Obs = bcs.createChild('obsNode')
        Obs.createObject('MechanicalObject', name='MO', position="0.01 0.06 0.01  0.01 0.06 0.04  0.01 0.06 0.07  0.03 0.06 0.01  0.03 0.06 0.04  0.03 0.06 0.07  0.05 0.06 0.01  0.05 0.06 0.04  0.05 0.06 0.07  0.07 0.06 0.01  0.07 0.06 0.04  0.07 0.06 0.07  0.09 0.06 0.01  0.09 0.06 0.04  0.09 0.06 0.07  0.11 0.06 0.01  0.11 0.06 0.04  0.11 0.06 0.07  0.13 0.06 0.01  0.13 0.06 0.04  0.13 0.06 0.07  0.15 0.06 0.01  0.15 0.06 0.04  0.15 0.06 0.07   0.18 0.01 0.01  0.18 0.01 0.04  0.18 0.01 0.07  0.18 0.03 0.01  0.18 0.03 0.04  0.18 0.03 0.07  0.18 0.05 0.01  0.18 0.05 0.04  0.18 0.05 0.07")
        Obs.createObject('Sphere', radius="0.001", color="0.2 0.8 0.2 1")
        Obs.createObject('BarycentricMapping')
        Obs.createObject('MappedPointsObservationManagerParallel', name="MOBS", observationStdev="1e-4", noiseStdev="0.0", listening="1")
        Obs.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix="../observations/beam1635_A",printLog="1")
 
        #visobs = bcs.createChild('SourceNode')
        #visobs.createObject('MechanicalObject', name="MO", position="@../obsNode/MOBS.mappedObservations", rest_position="@../obsNode/MO.position")
        #visobs.createObject('Sphere', radius="0.001", color="0.2 0.8 0.2 1")
        
        vis = bcs.createChild('visualization')
        vis.createObject('MeshSTLLoader',name="loader",filename="../../Data/beam1_1635.stl")
        vis.createObject('TriangleSetTopologyModifier', name="Modifier")
        vis.createObject('TriangleSetTopologyContainer', name="Container", src="@loader", tags=" ")
        vis.createObject('TriangleSetTopologyAlgorithms', name="TopoAlgo", template="Vec3d")
        vis.createObject('TriangleSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
        vis.createObject('MechanicalObject',src="@loader",name="Surface")
        vis.createObject('Line',color="0 0 0 1")
        vis.createObject('Triangle',color="1 0 0 1")
        vis.createObject('BarycentricMapping')
        vis.createObject('VTKExporter',filename="vtkExp/beam",XMLformat="true",listening="true",edges="0",triangles="1",quads="0",tetras="0",exportAtBegin="1",exportAtEnd="0",exportEveryNumberOfSteps="1")
 
    def createSlaveScene(self, node):    
        
        node.createObject('OptimParams', name="springStiffness", template="Vector", numParams="96",initValue="50", stdev="10", transformParams="1")
                                        
        bcs = node.createChild('bcScene')
        bcs.findData('activated').value="1"

        bcs.createObject('StaticSolver', applyIncrementFactor="0")

        PARDISO = bcs.createObject('SparsePARDISOSolver')
        if (self.m_saveToFile != 0):
            pardisoLabel = "Master"
            
        bcs.createObject('MechanicalObject', src="@/loader", name="VolumeMaster")
        bcs.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        bcs.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        bcs.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo", template="Vec3d")
        bcs.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
        bcs.createObject('UniformMass', totalMass="0.2513")
                
        bcs.createObject('BoxROI', name="fixedBox1", box="-0.001 -0.001 -0.001   0.18 0.001 0.08")        
        bcs.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points="@fixedBox1.indices", angularStiffness="0", stiffness="@../springStiffness.value", drawSpring="1", listening="1", forceDir="springForces", updateStiffness="1", printLog="1")
        bcs.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        bcs.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large", computeVonMisesStress="0", drawHeterogeneousTetra="0")
        
        toolEmu = bcs.createChild('toolEmu')
        toolEmu.createObject('MechanicalObject',name="MO",position="0.16 0.06 0.01  0.16 0.06 0.025  0.16 0.06 0.040  0.16 0.06 0.055   0.16 0.06 0.070   0.175 0.06 0.01  0.175 0.06 0.025  0.175 0.06 0.040  0.175 0.06 0.055   0.175 0.06 0.070")
        toolEmu.createObject('LinearMotionStateController',name="SlaveController",indices="0 1 2 3 4 5 6 7 8 9",keyDisplacements="0 0 0    0.05 0.05 0", keyTimes="0 2",printLog="1")
        toolEmu.createObject('Sphere',radius="0.002")
        
        bcs.createObject('Mapped3DoFForceField', mappedFEM="mappedTool/toolSpring", mappedMechObject="mappedTool/MO", mapping="mappedTool/baryMapping", printLog="1")        
        tool = bcs.createChild('mappedTool');
        tool.createObject('MechanicalObject',name="MO", position="0.16 0.06 0.01  0.16 0.06 0.025  0.16 0.06 0.040  0.16 0.06 0.055   0.16 0.06 0.070   0.175 0.06 0.01  0.175 0.06 0.025  0.175 0.06 0.040  0.175 0.06 0.055   0.175 0.06 0.070")
        tool.createObject('ExtendedRestShapeSpringForceField', name="toolSpring", angularStiffness="0", stiffness="1e5", external_rest_shape="../toolEmu/MO", drawSpring="1", listening="1", updateStiffness="1", springColor="0 1 0 1")
        tool.createObject('Sphere',radius="0.002",color="0 0 1 1") 
        tool.createObject('BarycentricMapping',name="baryMapping")

        self.m_slaveScenesCreated+=1

    ########################################################################################################################################
    #                            Scene creation                                                   
    ########################################################################################################################################
    def createScene(self,node):        
        print 'Python: Creating parallelized scene:', self.m_sigmaPointType,',', self.m_slaveSceneCount, 'threads.'
        createParallelizableScene(self, node, self.m_slaveSceneCount)
    
        return 0


    #############################################################################################################
    # Initialization functions
    #
   
               
    def initializationObjects(self,node):          
        self.cameraReactivated=False

        self.rootNode=node
      
        if(self.runFromScript):
            self.rootNode.getRootContext().animate = False
           
         
        print  "Init Graph called (Python side)"
        print " \n \n \n"
        return 0

