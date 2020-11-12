import Sofa
import math
import os
import sys
import csv

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='SofaMiscFem')
    rootNode.createObject('RequiredPlugin', name='SofaGeneralEngine')
    rootNode.createObject('RequiredPlugin', name='Exporter', pluginName='SofaExporter')
    rootNode.createObject('RequiredPlugin', name='Visual', pluginName='SofaOpenglVisual')
    rootNode.createObject('RequiredPlugin', name='Python', pluginName='SofaPython')
    rootNode.createObject('RequiredPlugin', name='Optimus', pluginName='Optimus')

    rootNode.createObject('PythonScriptController', name='SynthBCDA', filename=__file, classname='synth1_BCDA')




class synth1_BCDA(Sofa.PythonScriptController):

    def createGraph(self, node):
        self.cameraReactivated = False
        self.rootNode = node

        print "Create graph called (Python side)\n"

        ### configuration
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

        self.m_saveToFile = 0
        self.saveState = 1
        self.saveToolForces = 0
        self.saveAssess = 0

        self.paramInitExp = 0.0
        self.paramInitSD = 5
        self.obsInitSD= 1e-4

        self.solver = 'Newton' # options are 'Newton' and 'Euler'
        self.linearSolver = 'CG' # options are 'Pardiso' and 'CG'

        self.suffix=''

        ### generate directories to export data
        if self.saveState:
            self.stateExpFile=outDir+'/state'+self.suffix+'.txt'
            self.stateVarFile=outDir+'/variance'+self.suffix+'.txt'
            self.stateCovarFile=outDir+'/covariance'+self.suffix+'.txt'
            if os.path.isfile(self.stateExpFile):
                os.system('rm '+self.stateExpFile)
            if os.path.isfile(self.stateVarFile):
                os.system('rm '+self.stateVarFile)
            if os.path.isfile(self.stateCovarFile):
                os.system('rm '+self.stateCovarFile)

        if self.saveToolForces:
            self.toolForceFile=outDir+'/toolForce_'+self.suffix+'.txt'
            if os.path.isfile(self.toolForceFile):
                os.system('rm '+self.toolForceFile)

        if self.saveAssess:
            self.assessFile=outDir+'/assess_'+self.suffix+'.txt'
            if os.path.isfile(self.assessFile):
                os.system('rm '+self.assessFile)


        self.createGlobalComponents(node)
        masterNode = node.createChild('MasterScene')
        self.createMasterScene(masterNode)

        return 0


    def createGlobalComponents(self, node):
        ### scene global stuff
        node.findData('gravity').value="0 0 0"
        node.findData('dt').value="1"

        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        ### filter data
        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")
        self.filter = node.createObject('ROUKFilter', name="ROUKF", sigmaTopology="Simplex", verbose="1", useUnbiasedVariance='0')

        ### models loaders
        node.createObject('MeshVTKLoader', name='objectLoader', filename=self.volumeVTK)
        node.createObject('MeshSTLLoader', name='objectSLoader', filename=self.surfaceSTL)
        node.createObject('MeshVTKLoader', name='obsLoader', filename=self.obsVTK)
        node.createObject('MeshVTKLoader', name='toolLoader', filename=self.toolVTK)

        return 0



    ### common components for the simulation itself
    def createCommonComponents(self, node):
        ### solvers
        if self.solver == 'Euler':
            node.createObject('EulerImplicitSolver', rayleighStiffness='0.1', rayleighMass='0.1')
        elif self.solver == 'Newton':
            node.createObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="3")
        else:
            print 'Unknown solver type'

        if self.linearSolver == 'CG':
            node.createObject('CGLinearSolver', iterations="100", tolerance="1e-20", threshold="1e-20")
            # node.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        elif self.linearSolver == 'Pardiso':
            node.createObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToFolder="", iterativeSolverNumbering="0")
        else:
            print 'Unknown linear solver type'

        ### mechanical object
        node.createObject('MechanicalObject', src="@/objectLoader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/objectLoader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass="0.2513")

        ### material stiffness
        node.createObject('TetrahedronHyperelasticityFEMForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=self.materialParams)
        # node.createObject('MJEDTetrahedralForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=self.materialParams)
        # node.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large")

        ### boundary conditions
        node.createObject('BoxROI', name='fixedBox1', box='-0.001 -0.001 -0.011 0.101 0.001 0.001', drawBoxes='1', doUpdate='0')
        self.optimParams = node.createObject('OptimParams', name="springStiffness", template="Vector", numParams="@fixedBox1.nbIndices", initValue=self.paramInitExp, stdev=self.paramInitSD, transformParams="absolute", optimize="1", printLog="1")
        node.createObject('RestShapeSpringsForceField', name="fixedSpring", points="@fixedBox1.indices", stiffness="@springStiffness.value", listening="1", drawSpring='1', printLog="0")
        node.createObject('OglColorMap',colorScheme="Blue to Red")

        ### impact emulator
        toolEmu = node.createChild('toolEmu')
        toolEmu.createObject('MechanicalObject', name="MO", src="@/toolLoader")
        print self.toolMonitorPrefix
        toolEmu.createObject('SimulatedStateObservationSource', name="ToolA", printLog="1", monitorPrefix=self.toolMonitorPrefix, drawSize="0.0015", controllerMode="1")

        toolMapped = node.createChild('mappedTool')
        toolMapped.createObject('MechanicalObject', name="MO",src="@/toolLoader")
        self.toolSprings=toolMapped.createObject('RestShapeSpringsForceField', name="toolSpring", stiffness="1e5", external_rest_shape="@../toolEmu/MO", listening="1", springColor="0 1 0 1")
        toolMapped.createObject('OglColorMap', colorScheme="Blue to Red")
        toolMapped.createObject('SphereCollisionModel', radius="0.002", color="0 0 1 1")
        toolMapped.createObject('BarycentricMapping', name="baryMapping")

        return 0



    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose='1', estimatePosition='1')
        self.createCommonComponents(node)
        ### node with groundtruth observations
        obsNode = node.createChild('obsNode')
        obsNode.createObject('MechanicalObject', name='MO', src="@/obsLoader")
        obsNode.createObject('SphereCollisionModel', color='0.0 0.5 0.0 1', radius="0.0014", template='Vec3d')
        obsNode.createObject('BarycentricMapping')
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev=self.obsInitSD, listening="1", stateWrapper="@../StateWrapper", doNotMapObservations="1", verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.obsMonitorPrefix, drawSize="0.000")
        obsNode.createObject('ShowSpheres', position='@MOBS.observations', color='1.0 0.0 1.0 1', radius="0.0012")

        ### visual node
        visNode = node.createChild('ObjectVisualization')
        visNode.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping showWireframe hideNormals')
        visNode.createObject('MechanicalObject',src="@/objectSLoader", name="Surface")
        visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@/objectSLoader", tags=" ")
        visNode.createObject('LineCollisionModel', color="0 0 0 1")
        visNode.createObject('TriangleCollisionModel', color="1 0 0 1")
        visNode.createObject('BarycentricMapping')

        node.createObject('BoxROI', box='-0.001 -0.001 -0.011 0.105 0.001 0.001', drawBoxes='0', name='baseROI', doUpdate='0')
        self.basePoints=node.createObject('PointsFromIndices', template='Vec3d', name='fixedA', indices='@baseROI.indices', position="@Volume.position")

        return 0



    def initGraph(self, node):
        print 'Init graph called (python side)'
        self.step = 0
        self.total_time = 0
        return 0


    ### save filtering data to files
    def onEndAnimationStep(self, deltaTime):

        if self.saveState:
            rs=self.filter.findData('reducedState').value
            reducedState = [val for sublist in rs for val in sublist]
            # print 'Reduced state:'
            # print reducedState

            f1 = open(self.stateExpFile, "a")
            f1.write(" ".join(map(lambda x: str(x), reducedState)))
            f1.write('\n')
            f1.close()

            rv=self.filter.findData('reducedVariance').value
            reducedVariance = [val for sublist in rv for val in sublist]
            # print 'Reduced variance:'
            # print reducedVariance

            f2 = open(self.stateVarFile, "a")
            f2.write(" ".join(map(lambda x: str(x), reducedVariance)))
            f2.write('\n')
            f2.close()

            rcv=self.filter.findData('reducedCovariance').value
            reducedCovariance = [val for sublist in rcv for val in sublist]
            # print 'Reduced Covariance:'
            # print reducedCovariance

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
            # print 'Tool forces:'
            # print tsp

        if self.saveAssess:
            tsp=self.asMO.findData('position').value
            f5 = open(self.assessFile, "a")
            f5.write(" ".join(map(lambda x: str(x), tsp)))
            f5.write('\n')
            f5.close()

        # print self.basePoints.findData('indices_position').value

        return 0

    def onScriptEvent(self, senderNode, eventName, data):
        return 0;

