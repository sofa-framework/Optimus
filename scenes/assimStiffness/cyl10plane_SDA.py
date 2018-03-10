import Sofa
import math
import os
import sys
import csv

__file = __file__.replace('\\', '/') # windows

def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='Optimus', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')
    rootNode.createObject('RequiredPlugin', name='IMAUX', pluginName='ImageMeshAux')
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

        self.geometry = 'cyl10'
        self.volumeFileName='../../data/cylinder/cylinder10_4245.vtk'
        self.surfaceFileName='../../data/cylinder/cylinder10_4245.stl'        
        self.dt='0.01'
        self.gravity='0 -9.81 0'
        self.totalMass='0.2'
        #self.totalMass='0.3769'
        self.rayleighMass=0.1
        self.rayleighStiffness=3

        self.filterKind = 'ROUKF'
        self.estimatePosition = 1

        # self.filterKind = 'UKFSimCorr'
        # self.estimatePosition = 0

        self.planeCollision = 0
                          
        self.integration = 'Euler'
        self.numIter = 1

        #self.integration = 'Newton'
        #self.numIter = 3                 

        self.observationDir=self.geometry+'gravity_INT'+self.integration+str(self.numIter)+'/observations'
        self.outDir=self.geometry+'gravity_INT'+self.integration+str(self.numIter)+'/'+self.filterKind
        if self.planeCollision:
            self.observationDir= self.observationDir + '_plane'
            self.outDir= self.outDir + '_plane'

        self.observationFileName=self.observationDir+'/pos'

        os.system('mv -rf '+self.outDir+' arch')
        os.system('mkdir -p '+self.outDir)
        
        self.saveState = 1
        self.suffix=''   #psd'+str(self.paramInitSD)+'#osd'+str(self.obsInitSD)+'#ogrid'+str(self.ogridID)
        if self.saveState:
            self.stateExpFile=self.outDir+'/state'+self.suffix+'.txt'
            self.stateVarFile=self.outDir+'/variance'+self.suffix+'.txt'
            self.stateCovarFile=self.outDir+'/covariance'+self.suffix+'.txt'
            os.system('rm '+self.stateExpFile)
            os.system('rm '+self.stateVarFile)
            os.system('rm '+self.stateCovarFile)
          
        # self.m_slaveSceneCount = 0
        # self.m_saveToFile = 0
        # self.m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method

                
        # self.paramInitExp = 0.0
        # self.paramInitSD = 5
        # self.obsInitSD= 1e-4

        os.system('mkdir -p outCyl10plane')    
        self.createScene(node)
        
        return 0

    
    def createGlobalComponents(self, node):
        # scene global stuff                
        node.findData('gravity').value=self.gravity
        node.findData('dt').value=self.dt
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisualModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")        
        if self.filterKind == 'ROUKF':
            self.filter = node.createObject('ROUKFilter', name="ROUKF", verbose="1")
        elif self.filterKind == 'UKFSimCorr':
            self.filter = node.createObject('UKFilterSimCorr', name="UKFSC", verbose="1")
            
        node.createObject('MeshVTKLoader', name='loader', filename=self.volumeFileName)
        node.createObject('MeshSTLLoader', name='sloader', filename=self.surfaceFileName)        
        
        return 0        


    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createDeformableBody(self, parentNode):                                  
        #node.createObject('StaticSolver', applyIncrementFactor="0")  
        node=parentNode.createChild('cylinder')      
        node.createObject('EulerImplicitSolver', rayleighStiffness=self.rayleighStiffness, rayleighMass=self.rayleighMass)
        # node.createObject('NewtonStaticSolver', name="NewtonStatic", printLog="0", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt="2")   
        # node.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        node.createObject('SparsePARDISOSolver', name="lsolver", symmetric="1", exportDataToFolder="", iterativeSolverNumbering="0", pardisoSchurComplement='1')

        node.createObject('MechanicalObject', src="@/loader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass=self.totalMass)

        node.createObject('BoxROI', box='-0.05 -0.05 -0.002   0.05 0.05 0.002', name='fixedBox1', doUpdate='0')
        node.createObject('BoxROI', box='-0.05 -0.05  0.298   0.05 0.05 0.302', name='fixedBox2', doUpdate='0')
        node.createObject('MergeSets', name='mergeIndices', in2='@fixedBox2.indices', in1='@fixedBox1.indices')
        node.createObject('FixedConstraint', indices='@mergeIndices.out')
                    
        node.createObject('OptimParams', name="paramE", optimize="1", numParams='10', template="Vector", initValue="6000", stdev="1000", transformParams="absolute")
        node.createObject('Indices2ValuesMapper', name='youngMapper', indices='1 2 3 4 5 6 7 8 9 10', values='@paramE.value', inputValues='@/loader.dataset')
        node.createObject('TetrahedronFEMForceField', name='FEM', updateStiffness='1', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')

        if self.planeCollision == 1:
            # node.createObject('LinearSolverConstraintCorrection')
            node.createObject('PardisoConstraintCorrection', solverName='lsolver', schurSolverName='lsolver')

        # create collision model
        if self.planeCollision == 1:
            surface=node.createChild('collision')        
            surface.createObject('TriangleSetTopologyContainer', position='@/sloader.position', name='TriangleContainer', triangles='@/sloader.triangles')
            surface.createObject('TriangleSetTopologyModifier', name='Modifier')
            surface.createObject('MechanicalObject', showIndices='false', name='mstate')
            surface.createObject('Triangle', color='1 0 0 1', group=0)
            surface.createObject('Line', color='1 0 0 1', group=0)
            surface.createObject('Point', color='1 0 0 1', group=0)
            surface.createObject('BarycentricMapping', name='bpmapping')

        # create observation node
        obsNode = node.createChild('observations')
        obsNode.createObject('MechanicalObject', name='SourceMO', position='0.02 0 0.08 0.02 0 0.16    0.0141 0.0141 0.08    0.0141 -0.0141 0.08    0.0141 0.0141 0.16    0.0141 -0.0141 0.16    0.02 0 0.0533    0.02 0 0.107   \
            0.02 0 0.133    0.02 0 0.187    0.02 0 0.213    0.0175 0.00961 0.0649    0.00925 0.0177 0.0647    0.0139 0.0144 0.0398    0.00961 -0.0175 0.0649    0.0177 -0.00925 0.0647  \
            0.0144 -0.0139 0.0402    0.0177 0.00936 0.145    0.0095 0.0176 0.145    0.0175 0.00961 0.0951    0.00925 0.0177 0.0953    0.0139 0.0144 0.12    0.00937 -0.0177 0.145   \
            0.0176 -0.00949 0.145    0.00935 -0.0177 0.0953    0.0176 -0.00949 0.095    0.0142 -0.0141 0.12    0.0177 0.00937 0.175    0.00949 0.0176 0.175    0.014 0.0143 0.2   \
            0.00959 -0.0175 0.175    0.0177 -0.00924 0.175    0.0143 -0.014 0.2')        
        obsNode.createObject('BarycentricMapping')
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev="2e-3", noiseStdev="0.0", listening="1", stateWrapper="@../../StateWrapper", verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.observationFileName)
        obsNode.createObject('ShowSpheres', radius="0.002", color="1 0 0 1", position='@SourceMO.position')
        obsNode.createObject('ShowSpheres', radius="0.0015", color="1 1 0 1", position='@MOBS.mappedObservations')


                # rootNode/simuNode/oglNode
        oglNode = node.createChild('visualization')
        self.oglNode = oglNode
        oglNode.createObject('OglModel', color='1 0 0 1')
        oglNode.createObject('BarycentricMapping')
        # node.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large")
                
        return 0

    def createObstacle(self, node): 
        if self.planeCollision == 1:
            floor = node.createChild('floor')
            floor.createObject('RegularGrid', nx="2", ny="2", nz="2", xmin="-0.1", xmax="0.1",  ymin="-0.059", ymax="-0.061", zmin="0.0", zmax="0.3")
            floor.createObject('MechanicalObject', template="Vec3d")
            floor.createObject('Triangle',simulated="false", bothSide="true", contactFriction="0.00", color="1 1 0 1")
            floor.createObject('Line', simulated="false", bothSide="true", contactFriction="0.0", color="1 1 0 1")
            floor.createObject('Point', simulated="false", bothSide="true", contactFriction="0.0", color="1 1 0 1")                                 
        return


    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose='1', langrangeMultipliers=self.planeCollision, estimatePosition=self.estimatePosition)

        if self.planeCollision == 1:
            node.createObject('GenericConstraintSolver', maxIterations='1000', tolerance='1e-6', printLog='0', allVerified='0')
            node.createObject('CollisionPipeline', depth="6", verbose="0", draw="0")
            node.createObject('BruteForceDetection', name="N2")
            node.createObject('LocalMinDistance', name="Proximity",  alarmDistance='0.002', contactDistance='0.001',  angleCone='90.0', filterIntersection='0')
            node.createObject('DefaultContactManager', name="Response", response="FrictionContact", responseParams='mu=0')

        self.createDeformableBody(node)
        self.createObstacle(node)

        return 0
  

    # def createSlaveScene(self, node):
    #     node.createObject('VisualStyle', name='VisualStyle', displayFlags='hideBehaviorModels hideForceFields hideCollisionModels')
    #     wrapper=node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
    #     wrapper.findData("name").value = "StochasticWrapperSlave"
    #     wrapper.findData("slave").value = 1;        
        
    #     self.createCommonComponents(node)        
    #     self.m_slaveScenesCreated+=1

    #     return 0

    

    def createScene(self,node):
        # r_slaves = [] # list of created auxiliary nodes
        self.createGlobalComponents(node)
                
        masterNode=node.createChild('SimulationScene')
        self.createMasterScene(masterNode)        
        # masterNode.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels')
        
        # slaveSubordinate=node.createChild('SlaveSubordinate')
        # for i in range(1,self.m_slaveSceneCount):
        #     slave=slaveSubordinate.createChild('SlaveScene_'+str(i))
        #     #slave.createObject('VisualStyle', name='VisualStyle', displayFlags='hideAll')
        #     self.createSlaveScene(slave)
        #     r_slaves.append(slave)        
            
        return 0


    def initGraph(self,node):
        print 'Init graph called (python side)'
        self.step    =     0
        self.total_time =     0
        
        # self.process.initializationObjects(node)
        return 0

    def onEndAnimationStep(self, deltaTime): 
        
        stateName = 'reducedState' if self.filterKind == 'ROUKF' else 'state'
        varName = 'reducedVariance' if self.filterKind == 'ROUKF' else 'variance'
        covarName = 'reducedCovariance' if self.filterKind == 'ROUKF' else 'covariance'

        if self.saveState:              
            rs=self.filter.findData(stateName).value
            state = [val for sublist in rs for val in sublist]
            #print 'Reduced state:'
            #print reducedState

            f1 = open(self.stateExpFile, "a")        
            f1.write(" ".join(map(lambda x: str(x), state)))
            f1.write('\n')
            f1.close()    
                    
            rv=self.filter.findData(varName).value
            variance = [val for sublist in rv for val in sublist]
            #print 'Reduced variance:'
            #print reducedVariance

            f2 = open(self.stateVarFile, "a")        
            f2.write(" ".join(map(lambda x: str(x), variance)))
            f2.write('\n')
            f2.close()

            rcv=self.filter.findData(covarName).value
            covariance = [val for sublist in rcv for val in sublist]
            #print 'Reduced Covariance:'
            #print reducedCovariance

            f3 = open(self.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), covariance)))
            f3.write('\n')
            f3.close()    

        # print self.basePoints.findData('indices_position').value

        return 0

    def onScriptEvent(self, senderNode, eventName,data):        
        return 0;

