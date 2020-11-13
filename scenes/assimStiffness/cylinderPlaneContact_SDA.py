import Sofa
import math
import os
import sys
import csv

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='Optimus', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', name='Python', pluginName='SofaPython')
    rootNode.createObject('PythonScriptController', name='SynthBCDA', filename=__file, classname='synth1_BCDA')




class synth1_BCDA(Sofa.PythonScriptController):

    def createGraph(self, node):
        self.cameraReactivated = False
        self.rootNode = node

        print "Create graph called (Python side)\n"

        E=5000
        nu=0.45
        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        self.materialParams='{} {}'.format(mu,lamb)

        self.geometry = 'cyl10'
        self.volumeFileName='../../data/cylinder/cylinder10_4245.vtk'
        self.surfaceFileName='../../data/cylinder/cylinder10_4245.stl'
        self.observationPointsFileName="../../data/cylinder/cyl10_4245_obs33.vtk"
        self.dt='0.01'
        self.gravity='0 -9.81 0'
        self.totalMass='0.2' # '0.3769'
        self.rayleighMass=0.1
        self.rayleighStiffness=3

        self.filterKind = 'ROUKF' # options are "ROUKF", "UKFSimCorr", and "UKFClassic"
        self.estimatePosition = 1 # 0

        self.planeCollision = 1

        self.integration = 'Euler' # options are 'Euler' and 'Newton'
        self.linearSolver = 'CG'  # options are 'CG' and 'Pardiso'
        self.numIter = 1 # 3

        sdaSuffix = '_precond_s1_pcg15'
        self.observationDir=self.geometry+'gravity_INT'+self.integration+str(self.numIter)+'/observations'
        self.outDir=self.geometry+'gravity_INT'+self.integration+str(self.numIter)+'/'+self.filterKind+sdaSuffix
        if self.planeCollision:
            self.observationDir= self.observationDir + '_plane'
            self.outDir= self.outDir + '_plane'

        self.observationFileName=self.observationDir+'/pos'

        os.system('mv -rf '+self.outDir+' arch')
        os.system('mkdir -p '+self.outDir)

        self.saveState = 1
        self.suffix=''
        if self.saveState:
            self.stateExpFile=self.outDir+'/state'+self.suffix+'.txt'
            self.stateVarFile=self.outDir+'/variance'+self.suffix+'.txt'
            self.stateCovarFile=self.outDir+'/covariance'+self.suffix+'.txt'
            os.system('rm '+self.stateExpFile)
            os.system('rm '+self.stateVarFile)
            os.system('rm '+self.stateCovarFile)

        # self.paramInitExp = 0.0
        # self.paramInitSD = 5
        # self.obsInitSD = 1e-4

        os.system('mkdir -p outCyl10plane')
        self.createGlobalComponents(node)
        masterNode = node.createChild('SimulationScene')
        self.createMasterScene(masterNode)

        return 0



    def createGlobalComponents(self, node):
        ### scene global stuff
        node.findData('gravity').value=self.gravity
        node.findData('dt').value=self.dt

        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisualModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")

        ### filter data
        if self.filterKind == 'ROUKF':
            self.filter = node.createObject('ROUKFilter', name="ROUKF", verbose="1")
        elif self.filterKind == 'UKFSimCorr':
            self.filter = node.createObject('UKFilterSimCorr', name="UKFSC", verbose="1")

        ### object loader
        node.createObject('MeshVTKLoader', name='loader', filename=self.volumeFileName)
        node.createObject('MeshSTLLoader', name='sloader', filename=self.surfaceFileName)

        return 0


    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose='1', langrangeMultipliers=self.planeCollision, estimatePosition=self.estimatePosition)

        if self.planeCollision == 1:
            node.createObject('GenericConstraintSolver', maxIterations='1000', tolerance='1e-6', printLog='0', allVerified='0')
            node.createObject('DefaultPipeline', depth="6", verbose="0", draw="0")
            node.createObject('BruteForceDetection', name="N2")
            node.createObject('LocalMinDistance', name="Proximity",  alarmDistance='0.002', contactDistance='0.001',  angleCone='90.0', filterIntersection='0')
            node.createObject('DefaultContactManager', name="Response", response="FrictionContact", responseParams='mu=0')

        self.createDeformableBody(node)
        self.createObstacle(node)

        return 0


    ### common components for simulation
    def createDeformableBody(self, parentNode):
        node=parentNode.createChild('cylinder')
        ### solvers
        if self.integration == 'Euler':
            node.createObject('EulerImplicitSolver', rayleighStiffness=self.rayleighStiffness, rayleighMass=self.rayleighMass)
        elif self.integration == 'Newton':
            node.createObject('StaticSolver', name="NewtonStatic", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1",  newton_iterations="2")
        else:
            print 'Unknown solver type!'

        if self.linearSolver == 'Pardiso':
            node.createObject('SparsePARDISOSolver', name="lsolver", symmetric="1", exportDataToFolder="", iterativeSolverNumbering="0", pardisoSchurComplement='0')
        elif self.linearSolver == 'CG':
            node.createObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')
            #node.createObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='1', use_precond='1', tolerance='1e-15', iterations='500', verbose='0', update_step='10', listening='1', preconditioners='lsolver')

        ### mechanical object
        node.createObject('MechanicalObject', src="@/loader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass=self.totalMass)

        ### boundary conditions
        node.createObject('BoxROI', box='-0.05 -0.05 -0.002 0.05 0.05 0.002  -0.05 -0.05  0.298 0.05 0.05 0.302', name='fixedBox', doUpdate='0')
        node.createObject('FixedConstraint', indices='@fixedBox.indices')

        ### stiffness estimation
        node.createObject('OptimParams', name="paramE", optimize="1", numParams='10', template="Vector", initValue="6000", stdev="1000", transformParams="absolute")
        node.createObject('Indices2ValuesMapper', name='youngMapper', indices='1 2 3 4 5 6 7 8 9 10', values='@paramE.value', inputValues='@/loader.dataset')
        node.createObject('TetrahedronFEMForceField', name='FEM', updateStiffness='1', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')

        if self.planeCollision == 1:
            node.createObject('PardisoConstraintCorrection', solverName='lsolver', schurSolverName='lsolver')
            # node.createObject('LinearSolverConstraintCorrection')

            ### create collision model
            surface=node.createChild('collision')
            surface.createObject('TriangleSetTopologyContainer', position='@/sloader.position', name='TriangleContainer', triangles='@/sloader.triangles')
            surface.createObject('TriangleSetTopologyModifier', name='Modifier')
            surface.createObject('MechanicalObject', showIndices='false', name='mstate')
            surface.createObject('TriangleCollisionModel', color='1 0 0 1', group=0)
            surface.createObject('LineCollisionModel', color='1 0 0 1', group=0)
            surface.createObject('PointCollisionModel', color='1 0 0 1', group=0)
            surface.createObject('BarycentricMapping', name='bpmapping')

        ### node with groundtruth observations
        obsNode = node.createChild('observations')
        obsNode.createObject('MeshVTKLoader', name='obsLoader', filename=self.observationPointsFileName)
        obsNode.createObject('MechanicalObject', name='SourceMO', src="@obsLoader")
        obsNode.createObject('BarycentricMapping')
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev="2e-3", noiseStdev="0.0", listening="1", stateWrapper="@../../StateWrapper", verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.observationFileName)
        obsNode.createObject('ShowSpheres', name="estimated", radius="0.002", color="1 0 0 1", position='@SourceMO.position')
        obsNode.createObject('ShowSpheres', name="groundTruth", radius="0.0015", color="1 1 0 1", position='@MOBS.mappedObservations')

        ### visual mode
        oglNode = node.createChild('visualization')
        oglNode.createObject('OglModel', color='1 0 0 1')
        oglNode.createObject('BarycentricMapping')

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



    def initGraph(self, node):
        print 'Init graph called (python side)'
        self.step = 0
        self.total_time = 0
        # self.process.initializationObjects(node)

        return 0


    ### save filtering data to files
    def onEndAnimationStep(self, deltaTime):

        stateName = 'reducedState' if self.filterKind == 'ROUKF' else 'state'
        varName = 'reducedVariance' if self.filterKind == 'ROUKF' else 'variance'
        covarName = 'reducedCovariance' if self.filterKind == 'ROUKF' else 'covariance'

        if self.saveState:
            rs=self.filter.findData(stateName).value
            state = [val for sublist in rs for val in sublist]
            # print 'Reduced state:'
            # print reducedState

            f1 = open(self.stateExpFile, "a")
            f1.write(" ".join(map(lambda x: str(x), state)))
            f1.write('\n')
            f1.close()

            rv=self.filter.findData(varName).value
            variance = [val for sublist in rv for val in sublist]
            # print 'Reduced variance:'
            # print reducedVariance

            f2 = open(self.stateVarFile, "a")
            f2.write(" ".join(map(lambda x: str(x), variance)))
            f2.write('\n')
            f2.close()

            rcv=self.filter.findData(covarName).value
            covariance = [val for sublist in rcv for val in sublist]
            # print 'Reduced Covariance:'
            # print reducedCovariance

            f3 = open(self.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), covariance)))
            f3.write('\n')
            f3.close()

        # print self.basePoints.findData('indices_position').value

        return 0


    def onScriptEvent(self, senderNode, eventName, data):
        return 0;

