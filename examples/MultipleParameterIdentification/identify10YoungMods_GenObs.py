import Sofa
import math
import os
import sys
import csv
import yaml
import pprint
import numpy as np

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='Engine', pluginName='SofaEngine')
    rootNode.createObject('RequiredPlugin', name='GeneralEngine', pluginName='SofaGeneralEngine')
    rootNode.createObject('RequiredPlugin', name='ImplicitOdeSolver', pluginName='SofaImplicitOdeSolver')
    rootNode.createObject('RequiredPlugin', name='BoundaryCondition', pluginName='SofaBoundaryCondition')
    rootNode.createObject('RequiredPlugin', name='Loader', pluginName='SofaLoader')
    rootNode.createObject('RequiredPlugin', name='MiscForceField', pluginName='SofaMiscForceField')
    rootNode.createObject('RequiredPlugin', name='SimpleFem', pluginName='SofaSimpleFem')
    rootNode.createObject('RequiredPlugin', name='Exporter', pluginName='SofaExporter')
    rootNode.createObject('RequiredPlugin', name='Visual', pluginName='SofaOpenglVisual')
    rootNode.createObject('RequiredPlugin', name='Python', pluginName='SofaPython')
    rootNode.createObject('RequiredPlugin', name='Optimus', pluginName='Optimus')

    try:
        sys.argv[0]
    except:
        commandLineArguments = []
    else:
        commandLineArguments = sys.argv

    if len(commandLineArguments) > 1:
        configFileName = commandLineArguments[1]
    else:
        print 'ERROR: Must supply a yaml config file as an argument!'
        return

    with open(configFileName, 'r') as stream:
        try:
            options = yaml.load(stream)

        except yaml.YAMLError as exc:
            print(exc)
            return

    if options['model']['int']['lin_type'] == 'Pardiso':
        rootNode.createObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')

    AppliedForces_GenObs(rootNode, options)

    return 0;




class AppliedForces_GenObs(Sofa.PythonScriptController):

    def __init__(self, rootNode, opt):
        self.opt = opt

        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(opt)

        ### set configuration
        self.saveObs = opt['io']['saveObs']
        self.saveGeo = opt["io"]["saveGeo"]
        self.planeCollision = opt['model']['plane_collision']

        ### generate output folders
        if self.saveObs or self.saveGeo:
            prefix = opt['io']['prefix']
            suffix = opt['io']['suffix']

            if self.planeCollision:
                prefix = prefix + 'plane_'

            self.mainFolder = 'observations'

            os.system('mv '+self.mainFolder+' '+self.mainFolder+'_arch')
            os.system('mkdir '+self.mainFolder)

        if self.saveObs:
            self.obsFile = self.mainFolder + '/' + opt['io']['obsFileName']

        if self.saveGeo:
            self.geoFolder = self.mainFolder + '/VTK'
            os.system('mkdir -p '+self.geoFolder)

        self.createGraph(rootNode)

        return None;




    def createGraph(self, rootNode):
        ### set general scene stuff
        rootNode.findData('dt').value = self.opt['model']['dt']
        rootNode.findData('gravity').value = self.opt['model']['gravity']

        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')

        if self.planeCollision == 1:
            rootNode.createObject('FreeMotionAnimationLoop')
            rootNode.createObject('GenericConstraintSolver', maxIterations='1000', tolerance='1e-6', printLog='0', allVerified='0')
            rootNode.createObject('CollisionPipeline', depth="6", verbose="0", draw="0")
            rootNode.createObject('BruteForceDetection', name="N2")
            rootNode.createObject('LocalMinDistance', name="Proximity",  alarmDistance='0.002', contactDistance='0.001',  angleCone='90.0', filterIntersection='0')
            rootNode.createObject('DefaultContactManager', name="Response", response="FrictionContact", responseParams='mu=0')

        ### general node
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode

        ### solvers
        intType = self.opt['model']['int']['type']
        if intType == 'Euler':
            rmass = self.opt['model']['int']['rmass']
            rstiff = self.opt['model']['int']['rstiff']
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness=rstiff, rayleighMass=rmass)
        elif intType == 'Newton':
            intMaxit = self.opt['model']['int']['maxit']
            simuNode.createObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations=intMaxit)

        linType = self.opt['model']['int']['lin_type']
        if linType == 'Pardiso':
            simuNode.createObject('SparsePARDISOSolver', name='lsolver', verbose='0', pardisoSchurComplement=self.planeCollision, symmetric=self.opt['model']['linsol']['pardisoSym'], exportDataToFolder=self.opt['model']['linsol']['pardisoFolder'])
        elif linType == 'CG':
            simuNode.createObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')
            if self.opt['model']['linsol']['usePCG'] == 1:
                lsconf = self.opt['model']['linsol']
                simuNode.createObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='1', use_precond='1', tolerance=lsconf['pcgTol'], iterations=lsconf['pcgIt'], verbose=lsconf['pcgVerb'], update_step=lsconf['pcgUpdateStep'], numIterationsToRefactorize=lsconf['pcgAdaptiveThreshold'], listening='1', preconditioners='lsolver')

        ### mechanical object
        simuNode.createObject('MeshVTKLoader', name='loader', filename=self.opt['model']['vol_mesh'])
        simuNode.createObject('MechanicalObject', src='@loader', name='Volume')
        simuNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")

        if 'total_mass' in self.opt['model'].keys():
            simuNode.createObject('UniformMass', totalMass=self.opt['model']['total_mass'])

        if 'density' in self.opt['model'].keys():
            simuNode.createObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.opt['model']['density'], name='mass')

        ### material and elasticiy properties
        youngModuli=self.opt['model']['fem']['young_moduli']
        poissonRatio = self.opt['model']['fem']['poisson_ratio']
        indices = range(1, len(youngModuli)+1)

        method = self.opt['model']['fem']['method']
        if  method[0:3] == 'Cor':
            simuNode.createObject('Indices2ValuesMapper', indices=indices, values=youngModuli, name='youngMapper', inputValues='@loader.dataset')
            simuNode.createObject('TetrahedronFEMForceField', name='FEM', method=method[3:].lower(), listening='true', drawHeterogeneousTetra='1', poissonRatio=poissonRatio, youngModulus='@youngMapper.outputValues', updateStiffness='1')
        elif method == 'StVenant':
            poissonRatii = poissonRatio * np.ones([1,len(youngModuli)])
            simuNode.createObject('Indices2ValuesTransformer', name='paramMapper', indices=indices, values1=youngModuli, values2=poissonRatii, inputValues='@loader.dataset', transformation='ENu2MuLambda')
            simuNode.createObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet='@paramMapper.outputValues', drawHeterogeneousTetra='1', numOmpThreads=-1)

        ### boundary conditions
        simuNode.createObject('BoxROI', box=self.opt['model']['bc']['boxes'], name='fixedBox', drawBoxes='1')
        simuNode.createObject('FixedConstraint', indices='@fixedBox.indices')

        ### export data
        if self.saveGeo:
            simuNode.createObject('VTKExporter', filename=self.geoFolder+'/object.vtk', XMLformat='0', listening='1', edges="0", triangles="0", quads="0", tetras="1", exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1")

        if self.saveObs:
            simuNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1')
            simuNode.createObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName=self.obsFile, ExportPositions='1', ExportVelocities='0', ExportForces='0')

        ### collision handling
        if self.planeCollision:
            simuNode.createObject('PardisoConstraintCorrection', solverName='lsolver', schurSolverName='lsolver')
            # simuNode.createObject('LinearSolverConstraintCorrection')

        if self.planeCollision:
            surface=simuNode.createChild('collision')
            surface.createObject('MeshSTLLoader', name='sloader', filename=self.opt['model']['surf_mesh'])
            surface.createObject('TriangleSetTopologyContainer', position='@sloader.position', name='TriangleContainer', triangles='@sloader.triangles')
            surface.createObject('TriangleSetTopologyModifier', name='Modifier')
            surface.createObject('MechanicalObject', showIndices='false', name='mstate')
            surface.createObject('TriangleCollisionModel', color='1 0 0 1', group=0)
            surface.createObject('LineCollisionModel', color='1 0 0 1', group=0)
            surface.createObject('PointCollisionModel', color='1 0 0 1', group=0)
            surface.createObject('BarycentricMapping', name='bpmapping')

        if self.planeCollision:
            floor = simuNode.createChild('floor')
            floor.createObject('RegularGrid', nx="2", ny="2", nz="2", xmin="-0.1", xmax="0.1",  ymin="-0.059", ymax="-0.061", zmin="0.0", zmax="0.3")
            floor.createObject('MechanicalObject', template="Vec3d")
            floor.createObject('TriangleCollisionModel',simulated="false", bothSide="true", contactFriction="0.00", color="1 0 0 1")
            floor.createObject('LineCollisionModel', simulated="false", bothSide="true", contactFriction="0.0")
            floor.createObject('PointCollisionModel', simulated="false", bothSide="true", contactFriction="0.0")

        ### visual node
        oglNode = simuNode.createChild('oglNode')
        oglNode.createObject('OglModel')
        oglNode.createObject('BarycentricMapping')

        return 0;




    def initGraph(self, node):
        return 0;

    def storeResetState(self):
        return 0;

    def cleanup(self):
        if self.saveObs or self.saveGeo:
            print 'Observations saved to '+ self.mainFolder
        return 0;

    def onEndAnimationStep(self, deltaTime):
        return 0;

    def onLoaded(self, node):
        return 0;

    def reset(self):
        return 0;

    def bwdInitGraph(self, node):
        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        return 0;

