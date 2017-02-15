import Sofa
import math
import os

# Class definition 
class pigLiver1_BCDA(Sofa.PythonScriptController):
    
    def createGlobalComponents(self, node):
        # scene global stuff
        node.createObject('RequiredPlugin', pluginName='Optimus', name='Optimus')
        node.createObject('RequiredPlugin', pluginName='SofaPardisoSolver', name='SofaPardisoSolver')
        node.createObject('RequiredPlugin', pluginName='ImageMeshAux', name='ImageMeshAux')
        node.createObject('RequiredPlugin', pluginName='image', name='image')
        node.createObject('RequiredPlugin', pluginName='OpticalFlow', name='OpticalFlow')
        node.createObject('RequiredPlugin', pluginName='SofaMJEDFEM')
        
        node.findData('gravity').value="0 0 0"
        node.findData('dt').value="0.01"
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels hideForceFields showCollisionModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")        
        node.createObject('ROUKFilter', name="ROUKF", sigmaPointType="simplex", verbose="1")        

        node.createObject('MeshVTKLoader', name="objectLoader", filename="../../Data/pigLiver/liverLobe0_598.vtk", scale3d='2900 2900 2900', translation='450 215 0', rotation='-15 180 188')    
        node.createObject('MeshSTLLoader', name="objectSLoader",filename="../../Data/pigLiver/liverLobe0_3914.stl", scale3d='2900 2900 2900', translation='450 215 0', rotation='-15 180 188')
        # node.createObject('MeshVTKLoader', name="obsLoader", filename="../obsBoundaryDA/brickC_150316/obsRestA.vtk")
        # node.createObject('MeshVTKLoader', name="toolLoader", filename="../obsBoundaryDA/brickC_150316/toolRestA.vtk")

        features = node.createChild('features')
        features.createObject('PreStochasticWrapper')
        features.createObject('LKOpticalFlowTrackerSimple', maskName='../../Data/pigLiver/porcineLiverMask.png', vidName='../../Data/pigLiver/porcineLiverCut1.avi', name='LK', winSize='31', detectorThresh='190', scaleImg='1', displayFeatures='0', view='1')
        features.createObject('MechanicalObject', scale3d='1 1 1', position='@LK.outputFeatures', name='DOFs')
        # Vision.createObject('Sphere', color='0.0 0.0 1.0 1', radius='5', template='Vec3d')
        #original video start: Vision.createObject('BoxROI', name="ToolIx", box='480 160 -10 580 180 10', drawBoxes='1')
        features.createObject('BoxROI', name="ToolIx", box='520 140 -10 620 160 10', drawBoxes='1')
        features.createObject('PointsFromIndices', template='Vec3d', name='ToolPoints', indices='@ToolIx.indices')
        #original video start: Vision.createObject('BoxROI', name="ObsIx", box='320 120 -10 370 160 10   220 200 -10 280 280 10    510 260 -10   550 300  10', drawBoxes='1')
        #trimmed video with lower feature: features.createObject('BoxROI', name="ObsIx", box='320 120 -10 390 160 10   280 200 -10 320 280 10    510 260 -10   550 280  10    330 320 -10  350 340 10', drawBoxes='1')
        features.createObject('BoxROI', name="ObsIx", box='320 120 -10 390 160 10   280 200 -10 320 280 10    510 260 -10   550 280  10 ', drawBoxes='1')
        features.createObject('PointsFromIndices', template='Vec3d', name='ObsPoints', indices='@ObsIx.indices')

        features.createObject('BoxROI', name="LandmarkIX", box='330 320 -10  350 350 10', drawBoxes='1')
        features.createObject('PointsFromIndices', template='Vec3d', name='LandPoints', indices='@LandmarkIX.indices')

        toolNode = node.createChild('tool')
        toolNode.createObject('MechanicalObject', scale3d='1 1 1', position='@/features/ToolPoints.indices_position', name='MO')
        toolNode.createObject('Sphere', color='1.0 0.0 1.0 1', radius='5', template='Vec3d')

        obsNode = node.createChild('obs')
        obsNode.createObject('MechanicalObject', scale3d='1 1 1', position='@/features/ObsPoints.indices_position', name='MO')
        obsNode.createObject('Sphere', color='1.0 1.0 0.0 1', radius='5', template='Vec3d')

        landNode = node.createChild('land')
        landNode.createObject('MechanicalObject', scale3d='1 1 1', position='@/features/LandPoints.indices_position', name='MO')
        landNode.createObject('Sphere', color='1.0 0.0 0.8 1', radius='5', template='Vec3d')

        return 0
        


    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createCommonComponents(self, node):    
        nu=0.45
        E=1e5
        node.findData('dt').value="1"                       
        #node.createObject('StaticSolver', applyIncrementFactor="0")
        #node.createObject('SparsePARDISOSolver')        

        node.createObject('NewtonStaticSolver', name="NewtonStatic", printLog="1", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt="3")   
        # node.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        node.createObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToDir="", iterativeSolverNumbering="0")

        node.createObject('MechanicalObject', src="@/objectLoader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/objectLoader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass="0.2513")

        # node.createObject('BoxROI', name="fixedBox1", box="-0.001 0.052 -0.001   0.1631 0.054 0.0131")
        node.createObject('BoxROI', name="fixedBox1", box='100 400 -100 700 500 100', drawBoxes='1')
        node.createObject('OptimParams', name="springStiffness", template="Vector", numParams="@fixedBox1.nbIndices",initValue="0", stdev="4", transformParams="1", optimize="1", printLog="1")

        # node.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points="@fixedBox1.indices", springColor='0 1 0 1', stiffness="1e3", springThickness="3", listening="1", updateStiffness="1", printLog="0")
        node.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points="@fixedBox1.indices", stiffness="@springStiffness.value", springThickness="3", listening="1", updateStiffness="1", printLog="0")
        node.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        # node.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large")

        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        materialParams='{} {}'.format(mu,lamb)
        node.createObject('MJEDTetrahedralForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)

        # toolEmu = node.createChild('toolEmu')
        # toolEmu.createObject('MechanicalObject',name="MO",src="@/toolLoader")
        # toolEmu.createObject('SimulatedStateObservationSource', name="ToolA",printLog="1", monitorPrefix="../obsBoundaryDA/brickC_150316/toolA",drawSize="0.0015",controllerMode="1")
        
        node.createObject('Mapped3DoFForceField', mappedFEM="mappedTool/toolSpring", mappedMechObject="mappedTool/MO", mapping="mappedTool/baryMapping", printLog="0")        
        mappedTool = node.createChild('mappedTool')
        mappedTool.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@/features/ToolPoints.indices_position', name='MO')
        mappedTool.createObject('Sphere', color='0.0 1.0 1.0 1', radius='5', template='Vec3d')
        self.toolSprings=mappedTool.createObject('ExtendedRestShapeSpringForceField', external_rest_shape='/features/MO', numStepsSpringOn='10000', stiffness='1e8', name='toolSpring', springColor='0 1 0 1', drawSpring='1', updateStiffness='1', printLog='0', listening='1', angularStiffness='0', startTimeSpringOn='0')
        mappedTool.createObject('BarycentricMapping', name="baryMapping")


        return 0



    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        
        self.createCommonComponents(node)
        
        obsNode = node.createChild('obsNode')
        obsNode.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@/features/ObsPoints.indices_position', name='MO')
        obsNode.createObject('Sphere', color='0.8 0.5 1.0 1', radius='3', template='Vec3d')
        obsNode.createObject('BarycentricMapping', name="baryMapping")
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev="1e-6", noiseStdev="0.0", listening="1", stateWrapper="@../StateWrapper",doNotMapObservations="1",verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", trackedObservations='@/features/ObsPoints.indices_position', monitorPrefix="../obsBoundaryDA/brickC_150316/obsA")

        landNode = node.createChild('landNode')
        landNode.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@/features/LandPoints.indices_position', name='MO')
        landNode.createObject('Sphere', color='0.5 0.6 1.0 1', radius='3', template='Vec3d')
        landNode.createObject('BarycentricMapping', name="baryMapping")

        

        # obsNode = node.createChild('obsNode')
        # obsNode.createObject('MechanicalObject', name='MO', src="@/obsLoader")
        #obsNode.createObject('MechanicalObject', name='MO', position="0.01 0.06 0.01  0.01 0.06 0.04  0.01 0.06 0.07  0.03 0.06 0.01  0.03 0.06 0.04  0.03 0.06 0.07  0.05 0.06 0.01  0.05 0.06 0.04  0.05 0.06 0.07  0.07 0.06 0.01  0.07 0.06 0.04  0.07 0.06 0.07  0.09 0.06 0.01  0.09 0.06 0.04  0.09 0.06 0.07  0.11 0.06 0.01  0.11 0.06 0.04  0.11 0.06 0.07  0.13 0.06 0.01  0.13 0.06 0.04  0.13 0.06 0.07  0.15 0.06 0.01  0.15 0.06 0.04  0.15 0.06 0.07   0.18 0.01 0.01  0.18 0.01 0.04  0.18 0.01 0.07  0.18 0.03 0.01  0.18 0.03 0.04  0.18 0.03 0.07  0.18 0.05 0.01  0.18 0.05 0.04  0.18 0.05 0.07")
        # obsNode.createObject('Sphere', radius="0.001", color="0.2 0.8 0.2 1")
        # obsNode.createObject('BarycentricMapping')                   
        # obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev="1e-5", noiseStdev="0.0", listening="1", stateWrapper="@../StateWrapper",doNotMapObservations="1",verbose="1")
        # obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix="../obsBoundaryDA/brickC_150316/obsA")
    
        visNode = node.createChild('ObjectVisualization')
        visNode.createObject('VisualStyle', name='VisualStyle', displayFlags='showWireframe showBehaviorModels showForceFields showCollisionModels')
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
        # masterNode.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels')
        
        # slaveSubordinate=node.createChild('SlaveSubordinate')
        # for i in range(1,self.m_slaveSceneCount):
        #     slave=slaveSubordinate.createChild('SlaveScene_'+str(i))
        #     #slave.createObject('VisualStyle', name='VisualStyle', displayFlags='hideAll')
        #     self.createSlaveScene(slave)
        #     r_slaves.append(slave)        
            
        return 0


               
    def createGraph(self,node):          
        self.cameraReactivated=False
        self.rootNode=node              
         
        print  "Create graph called (Python side)\n"

        self.m_sigmaPointType = 'simplex'
        self.m_slaveSceneCount = 4
        self.m_saveToFile = 0
        self.m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method

        self.createScene(node)
        
        return 0


    def initGraph(self,node):
        print 'Init graph called (python side)'
        self.step    =     0
        self.total_time =     0

        # print self.fixedROI.findData("indices").value
        
        
        # self.process.initializationObjects(node)
        return 0

