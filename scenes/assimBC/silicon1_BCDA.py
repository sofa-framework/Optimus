import Sofa
import math
import os

# Class definition 
class silicon1_BCDA(Sofa.PythonScriptController):
    
    def createGlobalComponents(self, node):
        # scene global stuff
        node.createObject('RequiredPlugin', pluginName='Optimus', name='Optimus')
        node.createObject('RequiredPlugin', pluginName='SofaPardisoSolver', name='SofaPardisoSolver')
        node.createObject('RequiredPlugin', pluginName='ImageMeshAux', name='ImageMeshAux')
        node.createObject('RequiredPlugin', pluginName='image', name='image')
        
        node.findData('gravity').value="0 0 0"
        node.findData('dt').value="0.01"
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")        
        node.createObject('ROUKFilter', name="ROUKF", sigmaPointType="simplex", verbose="1")        

        node.createObject('MeshVTKLoader', name="objectLoader", filename="../../data/brickC/brickC_1216.vtk")    
        node.createObject('MeshSTLLoader', name="objectSLoader",filename="../../data/brickC/brickC_1216.stl")
        node.createObject('MeshVTKLoader', name="obsLoader", filename="../obsBoundaryDA/brickC_150316/obsRestA.vtk")
        node.createObject('MeshVTKLoader', name="toolLoader", filename="../obsBoundaryDA/brickC_150316/toolRestA.vtk")

        return 0
        


    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createCommonComponents(self, node):                           
        #node.createObject('StaticSolver', applyIncrementFactor="0")
        #node.createObject('SparsePARDISOSolver')        

        node.createObject('NewtonStaticSolver', name="NewtonStatic", printLog="1", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt="1")   
        node.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        node.createObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToDir="", iterativeSolverNumbering="0")

        node.createObject('MechanicalObject', src="@/objectLoader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/objectLoader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass="0.2513")

        node.createObject('BoxROI', name="fixedBox1", box="-0.001 0.052 -0.001   0.1631 0.054 0.0131")
        node.createObject('OptimParams', name="springStiffness", template="Vector", numParams="@fixedBox1.nbIndices",initValue="50", stdev="10", transformParams="1", optimize="1", printLog="1")

        node.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points="@fixedBox1.indices", stiffness="@springStiffness.value", springThickness="3", listening="1", updateStiffness="1", printLog="0")
        node.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        node.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large")

        toolEmu = node.createChild('toolEmu')
        toolEmu.createObject('MechanicalObject',name="MO",src="@/toolLoader")
        toolEmu.createObject('SimulatedStateObservationSource', name="ToolA",printLog="1", monitorPrefix="../obsBoundaryDA/brickC_150316/toolA",drawSize="0.0015",controllerMode="1")
        
        node.createObject('Mapped3DoFForceField', mappedFEM="mappedTool/toolSpring", mappedMechObject="mappedTool/MO", mapping="mappedTool/baryMapping", printLog="0")        
        toolMapped = node.createChild('mappedTool');
        toolMapped.createObject('MechanicalObject',name="MO",src="@/toolLoader")
        toolMapped.createObject('ExtendedRestShapeSpringForceField', name="toolSpring", stiffness="1e5", external_rest_shape="../toolEmu/MO", springThickness="1", listening="1", updateStiffness="1", springColor="0 1 0 1",startTimeSpringOn="0",numStepsSpringOn="10000")
        toolMapped.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        toolMapped.createObject('Sphere',radius="0.002",color="0 0 1 1") 
        toolMapped.createObject('BarycentricMapping',name="baryMapping")

        return 0



    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        
        self.createCommonComponents(node)

        obsNode = node.createChild('obsNode')
        obsNode.createObject('MechanicalObject', name='MO', src="@/obsLoader")
        #obsNode.createObject('MechanicalObject', name='MO', position="0.01 0.06 0.01  0.01 0.06 0.04  0.01 0.06 0.07  0.03 0.06 0.01  0.03 0.06 0.04  0.03 0.06 0.07  0.05 0.06 0.01  0.05 0.06 0.04  0.05 0.06 0.07  0.07 0.06 0.01  0.07 0.06 0.04  0.07 0.06 0.07  0.09 0.06 0.01  0.09 0.06 0.04  0.09 0.06 0.07  0.11 0.06 0.01  0.11 0.06 0.04  0.11 0.06 0.07  0.13 0.06 0.01  0.13 0.06 0.04  0.13 0.06 0.07  0.15 0.06 0.01  0.15 0.06 0.04  0.15 0.06 0.07   0.18 0.01 0.01  0.18 0.01 0.04  0.18 0.01 0.07  0.18 0.03 0.01  0.18 0.03 0.04  0.18 0.03 0.07  0.18 0.05 0.01  0.18 0.05 0.04  0.18 0.05 0.07")
        obsNode.createObject('Sphere', radius="0.001", color="0.2 0.8 0.2 1")
        obsNode.createObject('BarycentricMapping')                   
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev="1e-5", noiseStdev="0.0", listening="1", stateWrapper="@../StateWrapper",doNotMapObservations="1",verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix="../obsBoundaryDA/brickC_150316/obsA")
    
        visNode = node.createChild('ObjectVisualization')
        visNode.createObject('MechanicalObject',src="@/objectSLoader",name="Surface")
        visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@/objectSLoader", tags=" ")
        visNode.createObject('TriangleSetTopologyModifier', name="Modifier")        
        visNode.createObject('TriangleSetTopologyAlgorithms', name="TopoAlgo")
        visNode.createObject('TriangleSetGeometryAlgorithms', name="GeomAlgo")        
        visNode.createObject('Line',color="0 0 0 1")
        visNode.createObject('Triangle',color="1 0 0 1")
        visNode.createObject('BarycentricMapping')
        #visNode.createObject('VTKExporter',filename="vtkExp/beam",XMLformat="true",listening="true",edges="0",triangles="1",quads="0",tetras="0",exportAtBegin="1",exportAtEnd="0",exportEveryNumberOfSteps="1")

        obsVisuNode = node.createChild('ObservationVisualization')
        obsVisuNode.createObject('MechanicalObject', name="aux_Source", position="@../obsNode/MOBS.mappedObservations")
        obsVisuNode.createObject('Sphere', radius="0.002", color="0.2 0.8 0.2 1")

        return 0
 
 

    def createSlaveScene(self, node):
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='hideBehaviorModels hideForceFields hideCollisionModels')
        wrapper=node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        wrapper.findData("name").value = "StochasticWrapperSlave"
        wrapper.findData("slave").value = 1;        
        
        self.createCommonComponents(node)        
        self.m_slaveScenesCreated+=1

        return 0

    

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


               
    def createGraph(self,node):          
        self.cameraReactivated=False
        self.rootNode=node              
         
        print  "Create graph called (Python side)\n"

        self.m_sigmaPointType = 'simplex'
        self.m_slaveSceneCount = 2
        self.m_saveToFile = 0
        self.m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method

        self.createScene(node)
        
        return 0


    def initGraph(self,node):
        print 'Init graph called (python side)'
        self.step    =     0
        self.total_time =     0

        print self.fixedROI.findData("indices").value
        
        
        # self.process.initializationObjects(node)
        return 0

