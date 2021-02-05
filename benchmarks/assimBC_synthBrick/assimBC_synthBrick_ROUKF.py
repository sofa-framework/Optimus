import Sofa
import sys
import math
import os
import csv

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='MiscFem', pluginName='SofaMiscFem')
    rootNode.addObject('RequiredPlugin', name='GeneralEngine', pluginName='SofaGeneralEngine')
    rootNode.addObject('RequiredPlugin', name='Engine', pluginName='SofaEngine')
    rootNode.addObject('RequiredPlugin', name='BoundaryCondition', pluginName='SofaBoundaryCondition')
    rootNode.addObject('RequiredPlugin', name='ImplicitOdeSolver', pluginName='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='Deformable', pluginName='SofaDeformable')
    rootNode.addObject('RequiredPlugin', name='MeshCollision', pluginName='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='Loader', pluginName='SofaLoader')
    rootNode.addObject('RequiredPlugin', name='Visual', pluginName='SofaOpenglVisual')
    rootNode.addObject('RequiredPlugin', name='GraphComponent', pluginName='SofaGraphComponent')
    # rootNode.addObject('RequiredPlugin', name='Python3', pluginName='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='Optimus', pluginName='Optimus')

    rootNode.addObject(SyntheticSDA_Controller(name="Synthetic_SDA", node=rootNode))
    return 0




class SyntheticSDA_Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rootNode = kwargs["node"]
        self.cameraReactivated = False
        print("Create graph called (Python side)\n")

        ### configuration
        self.ogridID = 4
        inputDir = 'obs_testing'
        outDir = 'roukf_testing'
        self.volumeVTK = inputDir+'/object_0.vtk'
        self.surfaceSTL = '../../data/brickD/brickD_536.stl'
        self.obsVTK = inputDir+'/observations_0.vtk'
        self.toolVTK = inputDir+'/tool_0.vtk'

        self.obsMonitorPrefix = inputDir+'/observations'
        self.toolMonitorPrefix = inputDir+'/tool'

        self.m_saveToFile = 0
        self.saveState = 1
        self.saveToolForces = 0
        self.saveAssess = 0

        self.paramInitExp = 0.0
        self.paramInitSD = 5
        self.obsInitSD = 1e-4

        self.solver = 'Newton' # options are 'Newton' and 'Euler'
        self.linearSolver = 'CG' # options are 'Pardiso' and 'CG'

        self.suffix = ''

        ### generate directories to export data
        if self.saveState:
            self.stateExpFile = outDir+'/state'+self.suffix+'.txt'
            self.stateVarFile = outDir+'/variance'+self.suffix+'.txt'
            self.stateCovarFile = outDir+'/covariance'+self.suffix+'.txt'
            if os.path.isfile(self.stateExpFile):
                os.system('rm '+self.stateExpFile)
            if os.path.isfile(self.stateVarFile):
                os.system('rm '+self.stateVarFile)
            if os.path.isfile(self.stateCovarFile):
                os.system('rm '+self.stateCovarFile)

        if self.saveToolForces:
            self.toolForceFile = outDir+'/toolForce_'+self.suffix+'.txt'
            if os.path.isfile(self.toolForceFile):
                os.system('rm '+self.toolForceFile)

        if self.saveAssess:
            self.assessFile = outDir+'/assess_'+self.suffix+'.txt'
            if os.path.isfile(self.assessFile):
                os.system('rm '+self.assessFile)

        if self.linearSolver == 'Pardiso':
            self.rootNode.addObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')

        self.createGlobalComponents(self.rootNode)
        masterNode = self.rootNode.addChild('MasterScene')
        self.createMasterScene(masterNode)

        return None;


    def createGlobalComponents(self, node):
        ### scene global stuff
        node.findData('gravity').value = [0.0, 0.0, 0.0]
        node.findData('dt').value = 1.0

        node.addObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.addObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        ### filter data
        node.addObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")
        self.filter = node.addObject('ROUKFilter', name="ROUKF", sigmaTopology="Simplex", verbose="1", useUnbiasedVariance='0')

        ### models loaders
        node.addObject('MeshVTKLoader', name='objectLoader', filename=self.volumeVTK)
        node.addObject('MeshSTLLoader', name='objectSLoader', filename=self.surfaceSTL)
        node.addObject('MeshVTKLoader', name='obsLoader', filename=self.obsVTK)
        node.addObject('MeshVTKLoader', name='toolLoader', filename=self.toolVTK)

        return 0



    ### common components for the simulation itself
    def createCommonComponents(self, node):
        ### solvers
        if self.solver == 'Euler':
            node.addObject('EulerImplicitSolver', rayleighStiffness='0.1', rayleighMass='0.1')
        elif self.solver == 'Newton':
            node.addObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="3")
        else:
            print('Unknown solver type')

        if self.linearSolver == 'CG':
            node.addObject('CGLinearSolver', iterations="100", tolerance="1e-20", threshold="1e-20")
            # node.addObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        elif self.linearSolver == 'Pardiso':
            node.addObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToFolder="", iterativeSolverNumbering="0")
        else:
            print('Unknown linear solver type')

        ### mechanical object
        node.addObject('MechanicalObject', src="@/objectLoader", name="Volume")
        node.addObject('TetrahedronSetTopologyContainer', name="Container", src="@/objectLoader", tags=" ")
        node.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        node.addObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.addObject('UniformMass', totalMass="0.2513")

        ### material stiffness
        E = 5000
        nu = 0.45
        lamb = (E*nu)/((1+nu)*(1-2*nu))
        mu = E/(2+2*nu)
        self.materialParams = '{} {}'.format(mu,lamb)
        node.addObject('TetrahedronHyperelasticityFEMForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=self.materialParams)
        # node.addObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large")

        ### boundary conditions
        node.addObject('BoxROI', name='fixedBox1', box='-0.001 -0.001 -0.011 0.101 0.001 0.001', drawBoxes='1', doUpdate='0')
        self.optimParams = node.addObject('OptimParams', name="springStiffness", template="Vector", numParams="@fixedBox1.nbIndices", initValue=self.paramInitExp, stdev=self.paramInitSD, transformParams="absolute", optimize="1", printLog="1")
        node.addObject('RestShapeSpringsForceField', name="fixedSpring", points="@fixedBox1.indices", stiffness="@springStiffness.value", listening="1", drawSpring='1', printLog="0")
        node.addObject('OglColorMap', colorScheme="Blue to Red")

        ### impact emulator
        toolEmu = node.addChild('toolEmu')
        toolEmu.addObject('MechanicalObject', name="MO", src="@/toolLoader")
        toolEmu.addObject('SimulatedStateObservationSource', name="ToolA", printLog="1", monitorPrefix=self.toolMonitorPrefix, drawSize="0.0015", controllerMode="1")

        toolMapped = node.addChild('mappedTool')
        toolMapped.addObject('MechanicalObject', name="MO",src="@/toolLoader")
        self.toolSprings = toolMapped.addObject('RestShapeSpringsForceField', name="toolSpring", stiffness="1e5", external_rest_shape="@../toolEmu/MO", listening="1", springColor="0 1 0 1")
        toolMapped.addObject('OglColorMap', colorScheme="Blue to Red")
        toolMapped.addObject('SphereCollisionModel', radius="0.002", color="0 0 1 1")
        toolMapped.addObject('BarycentricMapping', name="baryMapping")

        return 0



    def createMasterScene(self, node):
        node.addObject('StochasticStateWrapper',name="StateWrapper",verbose='1', estimatePosition='1')
        self.createCommonComponents(node)
        ### node with groundtruth observations
        obsNode = node.addChild('obsNode')
        obsNode.addObject('MechanicalObject', name='MO', src="@/obsLoader")
        obsNode.addObject('SphereCollisionModel', color='0.0 0.5 0.0 1', radius="0.0014", template='Vec3d')
        obsNode.addObject('BarycentricMapping')
        obsNode.addObject('MappedStateObservationManager', name="MOBS", observationStdev=self.obsInitSD, listening="1", stateWrapper="@../StateWrapper", doNotMapObservations="1", verbose="1")
        obsNode.addObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.obsMonitorPrefix, drawSize="0.000")
        obsNode.addObject('ShowSpheres', position='@MOBS.observations', color='1.0 0.0 1.0 1', radius="0.0012")

        ### visual node
        visNode = node.addChild('ObjectVisualization')
        visNode.addObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping showWireframe hideNormals')
        visNode.addObject('MechanicalObject',src="@/objectSLoader", name="Surface")
        visNode.addObject('TriangleSetTopologyContainer', name="Container", src="@/objectSLoader", tags=" ")
        visNode.addObject('LineCollisionModel', color="0 0 0 1")
        visNode.addObject('TriangleCollisionModel', color="1 0 0 1")
        visNode.addObject('BarycentricMapping')

        node.addObject('BoxROI', box='-0.001 -0.001 -0.011 0.105 0.001 0.001', drawBoxes='0', name='baseROI', doUpdate='0')
        self.basePoints = node.addObject('PointsFromIndices', template='Vec3d', name='fixedA', indices='@baseROI.indices', position="@Volume.position")

        return 0



    def initGraph(self, node):
        print('Init graph called (python side)')
        self.step = 0
        self.total_time = 0
        return 0


    ### save filtering data to files
    def onAnimateEndEvent(self, deltaTime):

        if self.saveState:
            rs = self.filter.findData('reducedState').value
            reducedState = [val for val in rs]
            # print('Reduced state:')
            # print(reducedState)

            f1 = open(self.stateExpFile, "a")
            f1.write(" ".join(map(lambda x: str(x), reducedState)))
            f1.write('\n')
            f1.close()

            rv = self.filter.findData('reducedVariance').value
            reducedVariance = [val for val in rv]
            # print('Reduced variance:')
            # print(reducedVariance)

            f2 = open(self.stateVarFile, "a")
            f2.write(" ".join(map(lambda x: str(x), reducedVariance)))
            f2.write('\n')
            f2.close()

            rcv = self.filter.findData('reducedCovariance').value
            reducedCovariance = [val for val in rcv]
            # print('Reduced Covariance:')
            # print(reducedCovariance)

            f3 = open(self.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), reducedCovariance)))
            f3.write('\n')
            f3.close()

        if self.saveToolForces:
            tsp = self.toolSprings.findData('totalForce').value
            f4 = open(self.toolForceFile, "a")
            f4.write(" ".join(map(lambda x: str(x), tsp[0])))
            f4.write('\n')
            f4.close()
            # print('Tool forces:')
            # print(tsp)

        # print(self.basePoints.findData('indices_position').value)

        return 0

    def onScriptEvent(self, senderNode, eventName, data):
        return 0;

