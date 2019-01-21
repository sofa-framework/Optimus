import Sofa
import math
import os
import sys
import csv
import time
import numpy as np

__file = __file__.replace('\\', '/') # windows

def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', pluginName='SofaPardisoSolver')
    rootNode.createObject('RequiredPlugin', pluginName='ImageMeshAux')
    #rootNode.createObject('RequiredPlugin', pluginName='SofaMJEDFEM')
    
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
               
        self.geometry = 'brickD'
        self.fixedPoints = 'L1'          #L1, L4        
        self.fixingMethod = 'proj'       #proj, penal        

        # self.obsPoints = 'GT3x10'    # observations in a grid 3x10 top part of the object
        # self.obsPoints = 'GC2x2'    # observations in a grid 2x2, corners
        self.obsPoints = 'PLB'

        # self.integration = 'Euler'
        # self.numIter = 1
        self.integration = 'Newton'
        self.numIter = 3
        # self.integration = 'VarSym'
        # self.numIter = 3

        self.toolTrajectory = 1
        
        #self.filterKind = 'ROUKF'
        self.filterKind = 'UKFSimCorr'

        self.simulationDir=self.geometry+'_fix-'+self.fixedPoints+self.fixingMethod+'_obs-'+self.obsPoints+'_int-'+self.integration+str(self.numIter)+'_TR'+str(self.toolTrajectory)        
        self.inputDir=self.simulationDir+'/observations'
        self.volumeVTK=self.inputDir+'/object_0.vtk'
        self.surfaceSTL='../../data/brickD/brickD_536.stl'
        self.obsVTK=self.inputDir+'/observations_0.vtk'
        self.toolVTK=self.inputDir+'/tool_0.vtk'

        self.obsMonitorPrefix=self.inputDir+'/observations'
        self.toolMonitorPrefix=self.inputDir+'/tool'
        
        self.m_slaveSceneCount = 0
        self.m_saveToFile = 0
        self.m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method

        self.saveState = 1
        self.saveToolForces=1
        self.saveAssess=0


        self.estimQuantity = 'forces'
        self.sdaSuffix = 'ForcesN1'
        self.paramInitExp = [0.0, 0.0, 0.0]
        self.paramInitSD = [10,10,0.0001]
        self.obsInitSD= 1e-5
        self.transformParams='none'
                
        # self.estimQuantity = 'springs'
        # self.sdaSuffix = 'SpringsN4'
        # self.paramInitExp = [0.0]
        # self.paramInitSD = [1e1]
        # self.obsInitSD= 1e-5
        # self.transformParams='absolute'

        self.outDir=self.simulationDir+'/'+self.filterKind +'_Ep' + str(self.paramInitExp[0]) + '_SDp' + str(self.paramInitSD[0]) + '_SDon' + str(self.obsInitSD) + '_' + self.sdaSuffix
        os.system('mkdir -p '+self.simulationDir+'/arch')        
        stamp='_'+str(int(time.time()))
        os.system('mv --backup -S '+stamp+' '+self.outDir+' '+self.simulationDir+'/arch')
        os.system('mkdir -p '+self.outDir)
        self.suffix=''

        if self.saveState:
            self.stateExpFile=self.outDir+'/state'+self.suffix+'.txt'
            self.stateVarFile=self.outDir+'/variance'+self.suffix+'.txt'
            self.stateCovarFile=self.outDir+'/covariance'+self.suffix+'.txt'
            self.innovationFile=self.outDir+'/innovation_test.txt'
            self.computationTimeFile=self.outDir+'/computationTime.txt'            
      
        if self.saveToolForces:
            self.toolForceFile=self.outDir+'/toolForce.txt'


        if self.saveAssess:
            self.assessFile=self.outDir+'/assess.txt'
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

        if (self.filterKind == 'ROUKF'):
            self.filter = node.createObject('ROUKFilter', name="ROUKF", sigmaTopology="Simplex", verbose="1", useUnbiasedVariance='0')        
            self.estimatePosition = 1
        elif (self.filterKind == 'UKFSimCorr'):
            self.filter = node.createObject('UKFilterSimCorr', name="UKF", verbose="1", sigmaTopology="Simplex", useUnbiasedVariance="0",  lambdaScale="1", boundFilterState="0")
            self.estimatePosition = 0
            
        node.createObject('MeshVTKLoader', name='objectLoader', filename=self.volumeVTK)
        node.createObject('MeshSTLLoader', name='objectSLoader', filename=self.surfaceSTL)
        node.createObject('MeshVTKLoader', name='obsLoader', filename=self.obsVTK)
        node.createObject('MeshVTKLoader', name='toolLoader', filename=self.toolVTK)                

        return 0
        


    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createCommonComponents(self, node):                                  
        #node.createObject('StaticSolver', applyIncrementFactor="0")
        #node.createObject('SparsePARDISOSolver')        

        if self.integration == 'Euler':
            node.createObject('EulerImplicitSolver', rayleighStiffness='0.1', rayleighMass='0.1')
        elif self.integration == 'Newton':
            node.createObject('NewtonStaticSolver', maxIt=self.numIter, name='NewtonStatic', correctionTolerance='1e-8', convergeOnResidual='1', residualTolerance='1e-8', printLog='0')
        elif self.integration == 'VarSym':
            node.createObject('VariationalSymplecticSolver', rayleighStiffness='1', rayleighMass='1',
             newtonError='1e-12', steps=self.numIter, verbose='0', useIncrementalPotentialEnergy='1')

        
        # node.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        node.createObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToFolder="", iterativeSolverNumbering="0")

        node.createObject('MechanicalObject', src="@/objectLoader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/objectLoader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass="0.2513")

        # node.createObject('BoxROI', name='fixedBox1', box='-0.001 -0.001 -0.011 0.011 0.001 0.001', drawBoxes='1')
        if self.fixedPoints  == 'L4':
            # entire bottom face:  (N16)
        	# node.createObject('BoxROI', name='fixedBox1', box='-0.001 -0.001 -0.011 0.101 0.001 0.001', drawBoxes='1', doUpdate='0')
            # only four points on the left (N4)
            node.createObject('BoxROI', name='fixedBox1', box='-0.001 -0.001 -0.011 0.015 0.001 0.001', drawBoxes='1', doUpdate='0')
        elif self.fixedPoints == 'L1':
            node.createObject('BoxROI', name='fixedBox1', box='-0.001 -0.001 -0.001 0.001 0.001 0.001', drawBoxes='1', doUpdate='0')

        	    

        if self.estimQuantity == 'springs':
            self.optimParams = node.createObject('OptimParams', name="StochasticParams", template="Vector", numParams="@fixedBox1.nbIndices",
            initValue=self.paramInitExp, stdev=self.paramInitSD, transformParams=self.transformParams, optimize="1", printLog="1")
        
            node.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points="@fixedBox1.indices", stiffness="@StochasticParams.value",
            showIndicesScale='0.05', springThickness="3", listening="1", updateStiffness="1", printLog="0")
            node.createObject('ColorMap',colorScheme="Blue to Red")
        
        elif self.estimQuantity == 'forces':
            self.optimParams = node.createObject('OptimParams', name="StochasticParams", template="VecDeriv3d", numParams="@fixedBox1.nbIndices",        
            initValue=self.paramInitExp, stdev=self.paramInitSD, transformParams=self.transformParams, optimize="1", printLog="1")
            
            node.createObject('ConstantForceField', indices='@fixedBox1.indices', forces='@StochasticParams.value')



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

        node.createObject('VTKExporterDA', filename=self.outDir+'/DAobject.vtk', XMLformat='0',listening='1',edges="0",triangles="0",quads="0",tetras="1",
        	exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1")

        return 0



    def createMasterScene(self, node):        
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose='1', estimatePosition=self.estimatePosition)
        
        self.createCommonComponents(node)

        obsNode = node.createChild('obsNode')
        obsNode.createObject('MechanicalObject', name='MO', src="@/obsLoader")                
        obsNode.createObject('Sphere', color='0.0 0.5 0.0 1', radius="0.0014", template='Vec3d')

        obsNode.createObject('BarycentricMapping')                   
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev=self.obsInitSD, listening="1", stateWrapper="@../StateWrapper",doNotMapObservations="1",verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.obsMonitorPrefix, drawSize="0.000")
        obsNode.createObject('ShowSpheres', position='@MOBS.observations',color='1 1 0 1', radius="0.0012")
    
        visNode = node.createChild('ObjectVisualization')
        visNode.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping showWireframe hideNormals')
        visNode.createObject('MechanicalObject',src="@/objectSLoader",name="Surface")
        visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@/objectSLoader", tags=" ")        
        visNode.createObject('Line',color="0 0 0 1")
        visNode.createObject('Triangle',color="1 0 0 1")
        visNode.createObject('BarycentricMapping')
        

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
            if self.filterKind == 'ROUKF':
                rs=self.filter.findData('reducedState').value
            else:
                rs=self.filter.findData('state').value
            reducedState = [val for sublist in rs for val in sublist]
            #print 'Reduced state:'
            #print reducedState

            print 'State:',reducedState

            f1 = open(self.stateExpFile, "a")        
            f1.write(" ".join(map(lambda x: str(x), reducedState)))
            f1.write('\n')
            f1.close()    
            
            if self.filterKind == 'ROUKF':
                rv=self.filter.findData('reducedVariance').value
            else:
                rv=self.filter.findData('variance').value   

            std1=np.sqrt(rv)/self.paramInitSD[0]
            print 'Stdev: ', std1.tolist()

            
            reducedVariance = [val for sublist in rv for val in sublist]
            #print 'Reduced variance:'
            #print reducedVariance

            f2 = open(self.stateVarFile, "a")        
            f2.write(" ".join(map(lambda x: str(x), reducedVariance)))
            f2.write('\n')
            f2.close()

            # rcv=self.filter.findData('reducedCovariance').value
            # reducedCovariance = [val for sublist in rcv for val in sublist]
            # #print 'Reduced Covariance:'
            # #print reducedCovariance

            # f3 = open(self.stateCovarFile, "a")
            # f3.write(" ".join(map(lambda x: str(x), reducedCovariance)))
            # f3.write('\n')
            # f3.close()

            # if (self.filterKind == 'ROUKF'):
            #     innov=self.filter.findData('reducedInnovation').value
            # elif (self.filterKind == 'UKFSimCorr' or self.options.filter.kind == 'UKFClassic'):
            #     innov=self.filter.findData('innovation').value

            # innovation = [val for sublist in innov for val in sublist]            

            # f3b = open(self.innovationFile, "a")        
            # f3b.write(" ".join(map(lambda x: str(x), innovation)))
            # f3b.write('\n')
            # f3b.close()   

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

