import Sofa
import math
import os
import sys
import csv

__file = __file__.replace('\\', '/') # windows

def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', pluginName='SofaPardisoSolver')
    rootNode.createObject('RequiredPlugin', name='BoundaryConditions', pluginName="BoundaryConditionsPlugin")
    
    rootNode.createObject('PythonScriptController', name='SynthBCDA', filename=__file, classname='synth1_BCDA')



# Class definition 
class synth1_BCDA(Sofa.PythonScriptController):


    def createGraph(self,node):          
        self.cameraReactivated=False
        self.rootNode=node              
         
        print  "Create graph called (Python side)\n"

        E=5000
        nu=0.45
        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        self.materialParams='{} {}'.format(mu,lamb)
        
        self.ogridID=4
        inputDir='obs_testing'
        outDir='roukf_testing'
        self.volumeVTK=inputDir+'/object_0.vtk'
        self.surfaceSTL='../../data/brickD/brickD_536.stl'
        self.obsVTK=inputDir+'/observations_0.vtk'
        self.toolVTK=inputDir+'/tool_0.vtk'

        self.obsMonitorPrefix=inputDir+'/observations'
        self.toolMonitorPrefix=inputDir+'/tool'
        
        self.m_slaveSceneCount = 0
        self.m_saveToFile = 0
        self.m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method

        self.saveState = 1
        self.saveToolForces=0
        self.saveAssess=0

        self.paramInitExp = 0.0
        self.paramInitSD = 5
        self.obsInitSD= 1e-4

        # self.suffix='psd'+str(self.paramInitSD)+'#osd'+str(self.obsInitSD)+'#ogrid'+str(self.ogridID)
        self.suffix=''

        if self.saveState:
            self.stateExpFile=outDir+'/state'+self.suffix+'.txt'
            self.stateVarFile=outDir+'/variance'+self.suffix+'.txt'
            self.stateCovarFile=outDir+'/covariance'+self.suffix+'.txt'
            os.system('rm '+self.stateExpFile)
            os.system('rm '+self.stateVarFile)
            os.system('rm '+self.stateCovarFile)
      
        if self.saveToolForces:
            self.toolForceFile=outDir+'/toolForce_'+self.suffix+'.txt'
            os.system('rm '+self.toolForceFile);


        if self.saveAssess:
            self.assessFile=outDir+'/assess_'+self.suffix+'.txt'
            os.system('rm '+self.assessFile);            

        self.createScene(node)
        
        return 0

    
    def createGlobalComponents(self, node):
        # scene global stuff                
        node.findData('gravity').value="0 0 0"
        node.findData('dt').value="1"
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")        
        self.filter = node.createObject('ROUKFilter', name="ROUKF", sigmaTopology="Simplex", verbose="1", useUnbiasedVariance='0')        
            
        node.createObject('MeshVTKLoader', name='objectLoader', filename=self.volumeVTK)
        node.createObject('MeshSTLLoader', name='objectSLoader', filename=self.surfaceSTL)
        node.createObject('MeshVTKLoader', name='obsLoader', filename=self.obsVTK)
        node.createObject('MeshVTKLoader', name='toolLoader', filename=self.toolVTK)                

        return 0
        


    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createCommonComponents(self, node):                                  
        #node.createObject('StaticSolver', applyIncrementFactor="0")
        #node.createObject('SparsePARDISOSolver')        

        # node.createObject('EulerImplicitSolver', rayleighStiffness='0.1', rayleighMass='0.1')
        node.createObject('NewtonStaticSolver', name="NewtonStatic", printLog="0", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt="2")   
        # node.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        node.createObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToFolder="", iterativeSolverNumbering="0")

        node.createObject('MechanicalObject', src="@/objectLoader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/objectLoader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass="0.2513")

        # node.createObject('BoxROI', name='fixedBox1', box='-0.001 -0.001 -0.011 0.011 0.001 0.001', drawBoxes='1')
        node.createObject('BoxROI', name='fixedBox1', box='-0.001 -0.001 -0.011 0.101 0.001 0.001', drawBoxes='1', doUpdate='0')
        # node.createObject('BoxROI', name="fixedBox1", box="-0.001 0.052 -0.001   0.1631 0.054 0.0131")        
        self.optimParams = node.createObject('OptimParams', name="springStiffness", template="Vector", numParams="@fixedBox1.nbIndices",
            initValue=self.paramInitExp, stdev=self.paramInitSD, transformParams="absolute", optimize="1", printLog="1")
        node.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points="@fixedBox1.indices", stiffness="@springStiffness.value",
            showIndicesScale='0', springThickness="3", listening="1", updateStiffness="1", printLog="0")
        node.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        #node.createObject('MJEDTetrahedralForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=self.materialParams)
        node.createObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=self.materialParams)
        # node.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large")

        #node.createObject('BoxROI', name='fixedBoxA', box='-0.001 -0.001 -0.011 0.003 0.001 0.001', drawBoxes='0')
        #node.createObject('PointsFromIndices', template='Vec3d', name='fixedA', indices='@fixedBoxA.indices', position="@Volume.rest_position")
        #node.createObject('BoxROI', name='fixedBoxB', box='0.013 -0.001 -0.011 0.016 0.001 0.001', drawBoxes='0')
        #node.createObject('PointsFromIndices', template='Vec3d', name='fixedB', indices='@fixedBoxB.indices', position="@Volume.rest_position")
        #node.createObject('BoxROI', name='fixedBoxC', box='0.025 -0.001 -0.011 0.030 0.001 0.001', drawBoxes='0')
        #node.createObject('PointsFromIndices', template='Vec3d', name='fixedC', indices='@fixedBoxC.indices', position="@Volume.rest_position")
        #node.createObject('BoxROI', name='fixedBoxD', box='0.035 -0.001 -0.011 0.101 0.001 0.001', drawBoxes='0')
        #node.createObject('PointsFromIndices', template='Vec3d', name='fixedD', indices='@fixedBoxD.indices', position="@Volume.rest_position")

        #fixedA = node.createChild('fixedNA')
        #fixedA.createObject('MechanicalObject',name='MO', position='@../fixedA.indices_position')
        #fixedA.createObject('Sphere', color='1.0 0.0 0.0 1', radius="0.0019", template='Vec3d')

        #fixedB = node.createChild('fixedNB')
        #fixedB.createObject('MechanicalObject',name='MO', position='@../fixedB.indices_position')
        #fixedB.createObject('Sphere', color='0.0 0.8 0.0 1', radius="0.0019", template='Vec3d')

        #fixedC = node.createChild('fixedNC')
        #fixedC.createObject('MechanicalObject',name='MO', position='@../fixedC.indices_position')
        #fixedC.createObject('Sphere', color='1.0 0.64 0.0 1', radius="0.0019", template='Vec3d')

        #fixedD = node.createChild('fixedND')
        #fixedD.createObject('MechanicalObject',name='MO', position='@../fixedD.indices_position')
        #fixedD.createObject('Sphere', color='0.0 1.0 1.0 1', radius="0.0019", template='Vec3d')

        toolEmu = node.createChild('toolEmu')        
        toolEmu.createObject('MechanicalObject',name="MO",src="@/toolLoader")
        print self.toolMonitorPrefix
        toolEmu.createObject('SimulatedStateObservationSource', name="ToolA",printLog="1", monitorPrefix=self.toolMonitorPrefix,drawSize="0.0015",controllerMode="1")
        
        node.createObject('Mapped3DoFForceField', mappedFEM="mappedTool/toolSpring", mappedMechObject="mappedTool/MO", mapping="mappedTool/baryMapping", printLog="0")        
        toolMapped = node.createChild('mappedTool');
        toolMapped.createObject('MechanicalObject',name="MO",src="@/toolLoader")
        self.toolSprings=toolMapped.createObject('ExtendedRestShapeSpringForceField', name="toolSpring", stiffness="1e5", external_rest_shape="../toolEmu/MO", springThickness="1", listening="1", updateStiffness="1", springColor="0 1 0 1",startTimeSpringOn="0",numStepsSpringOn="10000")
        toolMapped.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        toolMapped.createObject('Sphere',radius="0.002",color="0 0 1 1") 
        toolMapped.createObject('BarycentricMapping',name="baryMapping")

        return 0



    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose='1', estimatePosition='1')
        
        self.createCommonComponents(node)

        obsNode = node.createChild('obsNode')
        obsNode.createObject('MechanicalObject', name='MO', src="@/obsLoader")                
        obsNode.createObject('Sphere', color='0.0 0.5 0.0 1', radius="0.0014", template='Vec3d')

        obsNode.createObject('BarycentricMapping')                   
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev=self.obsInitSD, listening="1", stateWrapper="@../StateWrapper",doNotMapObservations="1",verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.obsMonitorPrefix, drawSize="0.000")
        obsNode.createObject('ShowSpheres', position='@MOBS.observations',color='1.0 0.0 1.0 1', radius="0.0012")
    
        visNode = node.createChild('ObjectVisualization')
        visNode.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping showWireframe hideNormals')
        visNode.createObject('MechanicalObject',src="@/objectSLoader",name="Surface")
        visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@/objectSLoader", tags=" ")        
        visNode.createObject('Line',color="0 0 0 1")
        visNode.createObject('Triangle',color="1 0 0 1")
        visNode.createObject('BarycentricMapping')

        #visNode.createObject('VTKExporter',filename="vtkExp/beam",XMLformat="true",listening="true",edges="0",triangles="1",quads="0",tetras="0",exportAtBegin="1",exportAtEnd="0",exportEveryNumberOfSteps="1")

        #visNode2 = node.createChild('ObjectVisualization2')
        #visNode2.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping hideWireframe hideNormals')
        #visNode2.createObject('MechanicalObject',src="@/objectSLoader",name="Surface")
        #visNode2.createObject('TriangleSetTopologyContainer', name="Container", src="@/objectSLoader", tags=" ")                
        #visNode2.createObject('Triangle',color="1 0 0 0.2")
        #visNode2.createObject('BarycentricMapping')

        node.createObject('BoxROI', box='-0.001 -0.001 -0.011 0.105 0.001 0.001', drawBoxes='0', name='baseROI', doUpdate='0')
        self.basePoints=node.createObject('PointsFromIndices', template='Vec3d', name='fixedA', indices='@baseROI.indices', position="@Volume.position")


        # visNode2 = node.createChild('ObjectVisualization2')
        # visNode2.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping hideWireframe hideNormals')        
        # visNode.createObject('MechanicalObject',src="@/objectSLoader",name="Surface")
        # visNode2.createObject('TriangleSetTopologyContainer', name="Container", src="@objectSLoader", tags=" ")                
        # visNode2.createObject('Triangle',color="1 0 0 0.2")
        # visNode2.createObject('BarycentricMapping')

        #obsVisuNode = node.createChild('ObservationVisualization')
        #obsVisuNode.createObject('MechanicalObject', name="aux_Source", position="@../obsNode/MOBS.observations")
        #obsVisuNode.createObject('Sphere', radius="0.002", color="0.2 0.8 0.2 1")

        #asNode = node.createChild('assessNode')
        #asNode.createObject('RegularGrid', name="grid", min='0.01 0.005 -0.005', max='0.09 0.07 -0.005', n='8 5 1')  # obs. grid4
        #self.asMO=asNode.createObject('MechanicalObject', src='@grid', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='1')
        #asNode.createObject('Sphere', color='1 0 1 1', radius="0.001", template='Vec3d')
        #asNode.createObject('BarycentricMapping')

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



    def initGraph(self,node):
        print 'Init graph called (python side)'
        self.step    =     0
        self.total_time =     0
        
        # self.process.initializationObjects(node)
        return 0

    def onEndAnimationStep(self, deltaTime):  

        if self.saveState:
            rs=self.filter.findData('reducedState').value
            reducedState = [val for sublist in rs for val in sublist]
            #print 'Reduced state:'
            #print reducedState

            f1 = open(self.stateExpFile, "a")        
            f1.write(" ".join(map(lambda x: str(x), reducedState)))
            f1.write('\n')
            f1.close()    
                    
            rv=self.filter.findData('reducedVariance').value
            reducedVariance = [val for sublist in rv for val in sublist]
            #print 'Reduced variance:'
            #print reducedVariance

            f2 = open(self.stateVarFile, "a")        
            f2.write(" ".join(map(lambda x: str(x), reducedVariance)))
            f2.write('\n')
            f2.close()

            rcv=self.filter.findData('reducedCovariance').value
            reducedCovariance = [val for sublist in rcv for val in sublist]
            #print 'Reduced Covariance:'
            #print reducedCovariance

            f3 = open(self.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), reducedCovariance)))
            f3.write('\n')
            f3.close()

        if self.saveToolForces:
            tsp=self.toolSprings.findData('totalForce').value
            f4 = open(self.toolForceFile, "a")
            f4.write(" ".join(map(lambda x: str(x), tsp[0])))
            f4.write('\n')
            f4.close()            
            #print 'Tool forces:'
            #print tsp

        if self.saveAssess:
            tsp=self.asMO.findData('position').value
            f5 = open(self.assessFile, "a")
            f5.write(" ".join(map(lambda x: str(x), tsp)))
            f5.write('\n')
            f5.close()                        

        #print self.basePoints.findData('indices_position').value

        return 0

    def onScriptEvent(self, senderNode, eventName,data):        
        return 0;

