import Sofa
import math
import os
import sys
import csv

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):

    try:
        sys.argv[0]
    except:
        commandLineArguments = []
    else:
        commandLineArguments = sys.argv
    mycyl10_GenObs = cyl10_GenObs(rootNode, commandLineArguments)
    return 0;




class cyl10_GenObs(Sofa.PythonScriptController):

    def __init__(self, rootNode, commandLineArguments):
        self.geometry = 'cyl10'

        self.volumeFileName = '../../data/cylinder/cylinder10_4245.vtk'
        self.surfaceFileName = '../../data/cylinder/cylinder10_4245.stl'
        self.dt = '0.01'
        self.gravity = '0 -9.81 0'
        self.totalMass = '0.2' # '0.3769'
        self.rayleighMass = 0.1
        self.rayleighStiffness = 3
        self.youngModuli = '3500 4000 1000 6000 2000 7000 2500 8000 3000 1500'

        self.saveObservations = 1
        self.planeCollision = 1

        rootNode.findData('dt').value = self.dt
        rootNode.findData('gravity').value = self.gravity

        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : " + str(commandLineArguments)

        self.integration = 'Euler' # options are 'Euler' and 'Newton'
        self.linearSolver = 'Pardiso'  # options are 'CG' and 'Pardiso'
        self.numIter = 1 # 3

        self.observationDir=self.geometry+'gravity_INT'+self.integration+str(self.numIter)+'/observations_s1_test'
        if self.planeCollision:
            self.observationDir= self.observationDir + '_plane'

        if self.saveObservations:
            os.system('mv ' + self.observationDir+' arch')
            os.system('mkdir -p ' + self.observationDir)

        self.createGraph(rootNode)

        return None;



    def createGraph(self, rootNode):
        ### rootNode
        rootNode.createObject('RequiredPlugin', name='Optimus', pluginName='Optimus')
        rootNode.createObject('RequiredPlugin', name='Python', pluginName='SofaPython')
        if self.linearSolver=='Pardiso':
            rootNode.createObject('RequiredPlugin', pluginName='SofaPardisoSolver')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')

        ### collisions handler
        if self.planeCollision == 1:
            rootNode.createObject('FreeMotionAnimationLoop')
            rootNode.createObject('GenericConstraintSolver', maxIterations='1000', tolerance='1e-6', printLog='0', allVerified='0')
            rootNode.createObject('DefaultPipeline', depth="6", verbose="0", draw="0")
            rootNode.createObject('BruteForceDetection', name="N2")
            rootNode.createObject('LocalMinDistance', name="Proximity",  alarmDistance='0.002', contactDistance='0.001',  angleCone='90.0', filterIntersection='0')
            rootNode.createObject('DefaultContactManager', name="Response", response="FrictionContact", responseParams='mu=0')

        ### general node
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode

        ### solvers
        if self.integration == 'Euler':
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness=self.rayleighStiffness, rayleighMass=self.rayleighMass)
        elif self.integration == 'Newton':
            simuNode.createObject('StaticSolver', name="NewtonStatic", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1",  newton_iterations="1")
        else:
            print 'Unknown solver type!'

        if self.linearSolver == 'Pardiso':
            simuNode.createObject('SparsePARDISOSolver', name='lsolver', verbose='0', pardisoSchurComplement='1', symmetric='1')
        elif self.linearSolver == 'CG':
            simuNode.createObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')
            # simuNode.createObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='0', use_precond='1', tolerance='1e-10', iterations='500', verbose='0', update_step='10', listening='1', preconditioners='lsolver')
            # simuNode.createObject('ShewchukPCGLinearSolver', name='lsolverit', iterations='500', use_precond='1', tolerance='1e-10', preconditioners='lsolver')
        else:
            print 'Unknown linear solver type!'

        ### mechanical object
        simuNode.createObject('MeshVTKLoader', name='loader', filename=self.volumeFileName)
        simuNode.createObject('MechanicalObject', src='@loader', name='Volume')
        simuNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        simuNode.createObject('UniformMass', totalMass=self.totalMass)

        simuNode.createObject('Indices2ValuesMapper', indices='1 2 3 4 5 6 7 8 9 10', values=self.youngModuli, name='youngMapper', inputValues='@loader.dataset')
        simuNode.createObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')

        ### boundary conditions
        simuNode.createObject('BoxROI', box='-0.05 -0.05 -0.002 0.05 0.05 0.002  -0.05 -0.05  0.298 0.05 0.05 0.302', name='fixedBox')
        simuNode.createObject('FixedConstraint', indices='@fixedBox.indices')        

        ### saving generated observations
        if self.saveObservations:
            simuNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1')
            simuNode.createObject('Monitor', name='ObservationMonitor', indices='@observationBox.indices', fileName=self.observationDir+'/pos', ExportPositions='1', ExportVelocities='0', ExportForces='0')

        ### solver for constraint movement
        if self.planeCollision:
            simuNode.createObject('PardisoConstraintCorrection', solverName='lsolver', schurSolverName='lsolver')
            # simuNode.createObject('LinearSolverConstraintCorrection')

            ### collision plane surface
            surface=simuNode.createChild('collision')
            surface.createObject('MeshSTLLoader', name='sloader', filename=self.surfaceFileName)
            surface.createObject('TriangleSetTopologyContainer', position='@sloader.position', name='TriangleContainer', triangles='@sloader.triangles')
            surface.createObject('TriangleSetTopologyModifier', name='Modifier')
            surface.createObject('MechanicalObject', showIndices='false', name='mstate')
            surface.createObject('TriangleCollisionModel', color='1 0 0 1', group=0)
            surface.createObject('LineCollisionModel', color='1 0 0 1', group=0)
            surface.createObject('PointCollisionModel', color='1 0 0 1', group=0)
            surface.createObject('BarycentricMapping', name='bpmapping')

        ### visual object
        oglNode = simuNode.createChild('oglNode')
        self.oglNode = oglNode
        oglNode.createObject('OglModel')

        ### collision plane surface
        if self.planeCollision:
            floor = simuNode.createChild('floor')
            floor.createObject('RegularGrid', nx="2", ny="2", nz="2", xmin="-0.1", xmax="0.1",  ymin="-0.059", ymax="-0.061", zmin="0.0", zmax="0.3")
            floor.createObject('MechanicalObject', template="Vec3d")
            floor.createObject('TriangleCollisionModel',simulated="false", bothSide="true", contactFriction="0.00", color="1 0 0 1")
            floor.createObject('LineCollisionModel', simulated="false", bothSide="true", contactFriction="0.0")
            floor.createObject('PointCollisionModel', simulated="false", bothSide="true", contactFriction="0.0")

        return 0;



    def onMouseButtonLeft(self, mouseX, mouseY, isPressed):
        return 0;

    def onKeyReleased(self, c):
        return 0;

    def initGraph(self, node):
        return 0;

    def onKeyPressed(self, c):
        return 0;

    def onMouseWheel(self, mouseX, mouseY, wheelDelta):
        return 0;

    def storeResetState(self):
        return 0;

    def cleanup(self):
        return 0;

    def onGUIEvent(self, strControlID, valueName, strValue):
        return 0;

    def onEndAnimationStep(self, deltaTime):
        return 0;

    def onLoaded(self, node):
        return 0;

    def reset(self):
        return 0;

    def onMouseButtonMiddle(self, mouseX, mouseY, isPressed):
        return 0;

    def bwdInitGraph(self, node):
        return 0;

    def onScriptEvent(self, senderNode, eventName, data):
        return 0;

    def onMouseButtonRight(self, mouseX, mouseY, isPressed):
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        return 0;

