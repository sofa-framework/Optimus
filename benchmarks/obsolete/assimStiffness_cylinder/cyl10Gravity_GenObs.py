import Sofa
import math
import os
import sys
import csv

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='Python', pluginName='SofaPython')
    rootNode.createObject('RequiredPlugin', name='Optimus', pluginName='Optimus')

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
        ### configuration
        self.volumeFileName='../../data/cylinder/cylinder10_4245.vtk'
        self.surfaceFileName='../../data/cylinder/cylinder10_4245.stl'
        self.observationFileName='obs_testing/cylinder10plane_4245'
        self.dt='0.01'
        self.gravity='0 -9.81 0'
        self.totalMass='0.2' # '0.3769'
        self.rayleighMass=0.1
        self.rayleighStiffness=3
        self.linearSolver='CG'  # options are 'CG' and 'Pardiso'
        self.youngModuli='3500 4000 1000 6000 2000 7000 2500 8000 3000 1500'

        self.saveObservations=1

        rootNode.findData('dt').value = self.dt
        rootNode.findData('gravity').value = self.gravity

        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        os.system('mkdir -p obs_testing')
        self.createGraph(rootNode)

        return None;



    def createGraph(self, rootNode):
        ### rootNode
        if self.linearSolver=='Pardiso':
            rootNode.createObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')

        ### collision handling
        # rootNode.createObject('FreeMotionAnimationLoop')
        # rootNode.createObject('GenericConstraintSolver', maxIterations='1000', tolerance='1e-6', printLog='0', allVerified='0')
        # rootNode.createObject('CollisionPipeline', depth="6", verbose="0", draw="0")
        # rootNode.createObject('BruteForceDetection', name="N2")
        # rootNode.createObject('LocalMinDistance', name="Proximity",  alarmDistance='0.002', contactDistance='0.001',  angleCone='90.0', filterIntersection='0')
        # rootNode.createObject('DefaultContactManager', name="Response", response="FrictionContact", responseParams='mu=0')

        ### object node
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode

        ### solver
        simuNode.createObject('EulerImplicitSolver', rayleighStiffness=self.rayleighStiffness, rayleighMass=self.rayleighMass)
        if self.linearSolver=='CG':
            simuNode.createObject('CGLinearSolver', iterations="20", tolerance="1e-12", threshold="1e-12")
        elif self.linearSolver=='Pardiso':
            simuNode.createObject('SparsePARDISOSolver', name='lsolver', verbose='0')

        ### mechanical object
        simuNode.createObject('MeshVTKLoader', name='loader', filename=self.volumeFileName)
        simuNode.createObject('MechanicalObject', src='@loader', name='Volume')
        simuNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        simuNode.createObject('UniformMass', totalMass=self.totalMass)

        ### stiffness
        simuNode.createObject('Indices2ValuesMapper', indices='1 2 3 4 5 6 7 8 9 10', values=self.youngModuli, name='youngMapper', inputValues='@loader.dataset')
        simuNode.createObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')

        ### fixed boundary conditions
        simuNode.createObject('BoxROI', box='-0.05 -0.05 -0.002 0.05 0.05 0.002  -0.05 -0.05  0.298 0.05 0.05 0.302', name='fixedBox')
        simuNode.createObject('FixedConstraint', indices='@fixedBox.indices')

        ### save observations
        if self.saveObservations:
            simuNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1')
            simuNode.createObject('Monitor', name='ObservationMonitor', indices='@observationBox.indices', fileName=self.observationFileName, ExportPositions='1', ExportVelocities='0', ExportForces='0')

        ### collision handling
        # simuNode.createObject('PardisoConstraintCorrection', solverName='lsolver', schurSolverName='lsolver')
        # simuNode.createObject('LinearSolverConstraintCorrection')

        # surface=simuNode.createChild('collision')
        # surface.createObject('MeshSTLLoader', name='sloader', filename=self.surfaceFileName)
        # surface.createObject('TriangleSetTopologyContainer', position='@sloader.position', name='TriangleContainer', triangles='@sloader.triangles')
        # surface.createObject('TriangleSetTopologyModifier', name='Modifier')
        # surface.createObject('MechanicalObject', showIndices='false', name='mstate')
        # surface.createObject('Triangle', color='1 0 0 1', group=0)
        # surface.createObject('Line', color='1 0 0 1', group=0)
        # surface.createObject('Point', color='1 0 0 1', group=0)
        # surface.createObject('BarycentricMapping', name='bpmapping')

        # floor = simuNode.createChild('floor')
        # floor.createObject('RegularGrid', nx="2", ny="2", nz="2", xmin="-0.1", xmax="0.1",  ymin="-0.059", ymax="-0.061", zmin="0.0", zmax="0.3")
        # floor.createObject('MechanicalObject', template="Vec3d")
        # floor.createObject('Triangle',simulated="false", bothSide="true", contactFriction="0.00", color="1 0 0 1")
        # floor.createObject('Line', simulated="false", bothSide="true", contactFriction="0.0")
        # floor.createObject('Point', simulated="false", bothSide="true", contactFriction="0.0")

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

