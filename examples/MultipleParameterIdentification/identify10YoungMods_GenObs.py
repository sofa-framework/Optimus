import Sofa
import sys
import math
import os
import csv
import yaml
import pprint
import numpy as np

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='Engine', pluginName='SofaEngine')
    rootNode.addObject('RequiredPlugin', name='GeneralEngine', pluginName='SofaGeneralEngine')
    rootNode.addObject('RequiredPlugin', name='ImplicitOdeSolver', pluginName='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='BoundaryCondition', pluginName='SofaBoundaryCondition')
    rootNode.addObject('RequiredPlugin', name='Loader', pluginName='SofaLoader')
    rootNode.addObject('RequiredPlugin', name='MiscForceField', pluginName='SofaMiscForceField')
    rootNode.addObject('RequiredPlugin', name='SimpleFem', pluginName='SofaSimpleFem')
    rootNode.addObject('RequiredPlugin', name='Visual', pluginName='SofaOpenglVisual')
    # rootNode.addObject('RequiredPlugin', name='Python3', pluginName='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='Optimus', pluginName='Optimus')

    try:
        sys.argv[0]
    except:
        commandLineArguments = []
    else:
        commandLineArguments = sys.argv

    # if len(commandLineArguments) > 1:
    #    configFileName = commandLineArguments[1]
    configFileName = "cylinder10AppledGravity.yml"
    # else:
    #    print 'ERROR: Must supply a yaml config file as an argument!'
    #    return

    with open(configFileName, 'r') as stream:
        try:
            options = yaml.safe_load(stream)

        except yaml.YAMLError as exc:
            print(exc)
            return

    if options['model']['int']['lin_type'] == 'Pardiso':
        rootNode.addObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')

    rootNode.addObject(AppliedForcesGenObs_Controller(name="AppliedForces_GenObs", node=rootNode, opt=options))

    return 0;




class AppliedForcesGenObs_Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        ### These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rootNode = kwargs["node"]
        self.opt = kwargs["opt"]

        ### print options
        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(self.opt)

        ### set configuration
        self.saveObs = self.opt['io']['saveObs']
        self.saveGeo = self.opt["io"]["saveGeo"]
        self.planeCollision = self.opt['model']['plane_collision']

        ### generate output folders
        if self.saveObs or self.saveGeo:
            prefix = self.opt['io']['prefix']
            suffix = self.opt['io']['suffix']

            if self.planeCollision:
                prefix = prefix + 'plane_'

            self.mainFolder = 'observations'

            os.system('mv '+self.mainFolder+' '+self.mainFolder+'_arch')
            os.system('mkdir '+self.mainFolder)

        if self.saveObs:
            self.obsFile = self.mainFolder + '/' + self.opt['io']['obsFileName']

        if self.saveGeo:
            self.geoFolder = self.mainFolder + '/VTK'
            os.system('mkdir -p '+self.geoFolder)

        self.createGraph(self.rootNode)
        return None




    def createGraph(self, rootNode):
        ### set general scene stuff
        rootNode.findData('dt').value = self.opt['model']['dt']
        rootNode.findData('gravity').value = self.opt['model']['gravity']

        rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')

        if self.planeCollision == 1:
            rootNode.addObject('FreeMotionAnimationLoop')
            rootNode.addObject('GenericConstraintSolver', maxIterations='1000', tolerance='1e-6', printLog='0', allVerified='0')
            rootNode.addObject('CollisionPipeline', depth="6", verbose="0", draw="0")
            rootNode.addObject('BruteForceDetection', name="N2")
            rootNode.addObject('LocalMinDistance', name="Proximity",  alarmDistance='0.002', contactDistance='0.001',  angleCone='90.0', filterIntersection='0')
            rootNode.addObject('DefaultContactManager', name="Response", response="FrictionContact", responseParams='mu=0')

        ### general node
        simuNode = rootNode.addChild('simuNode')
        self.simuNode = simuNode

        ### solvers
        intType = self.opt['model']['int']['type']
        if intType == 'Euler':
            rmass = self.opt['model']['int']['rmass']
            rstiff = self.opt['model']['int']['rstiff']
            simuNode.addObject('EulerImplicitSolver', rayleighStiffness=rstiff, rayleighMass=rmass)
        elif intType == 'Newton':
            intMaxit = self.opt['model']['int']['maxit']
            simuNode.addObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations=intMaxit)

        linType = self.opt['model']['int']['lin_type']
        if linType == 'Pardiso':
            simuNode.addObject('SparsePARDISOSolver', name='lsolver', verbose='0', pardisoSchurComplement=self.planeCollision, symmetric=self.opt['model']['linsol']['pardisoSym'], exportDataToFolder=self.opt['model']['linsol']['pardisoFolder'])
        elif linType == 'CG':
            simuNode.addObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')
            if self.opt['model']['linsol']['usePCG'] == 1:
                lsconf = self.opt['model']['linsol']
                simuNode.addObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='1', use_precond='1', tolerance=lsconf['pcgTol'], iterations=lsconf['pcgIt'], verbose=lsconf['pcgVerb'], update_step=lsconf['pcgUpdateStep'], numIterationsToRefactorize=lsconf['pcgAdaptiveThreshold'], listening='1', preconditioners='lsolver')

        ### mechanical object
        simuNode.addObject('MeshVTKLoader', name='loader', filename=self.opt['model']['vol_mesh'])
        simuNode.addObject('MechanicalObject', src='@loader', name='Volume')
        simuNode.addObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.addObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")

        if 'total_mass' in self.opt['model'].keys():
            simuNode.addObject('UniformMass', totalMass=self.opt['model']['total_mass'])

        if 'density' in self.opt['model'].keys():
            simuNode.addObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.opt['model']['density'], name='mass')

        ### material and elasticiy properties
        youngModuli=self.opt['model']['fem']['young_moduli']
        poissonRatio = self.opt['model']['fem']['poisson_ratio']
        indices = list(range(1, len(youngModuli)+1))

        method = self.opt['model']['fem']['method']
        if  method[0:3] == 'Cor':
            simuNode.addObject('Indices2ValuesMapper', indices=indices, values=youngModuli, name='youngMapper', inputValues='@loader.dataset')
            simuNode.addObject('TetrahedronFEMForceField', name='FEM', method=method[3:].lower(), listening='true', drawHeterogeneousTetra='1', poissonRatio=poissonRatio, youngModulus='@youngMapper.outputValues', updateStiffness='1')
        elif method == 'StVenant':
            poissonRatii = poissonRatio * np.ones([1,len(youngModuli)])
            simuNode.addObject('Indices2ValuesTransformer', name='paramMapper', indices=indices, values1=youngModuli, values2=poissonRatii, inputValues='@loader.dataset', transformation='ENu2MuLambda')
            simuNode.addObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet='@paramMapper.outputValues', drawHeterogeneousTetra='1', numOmpThreads=-1)

        ### boundary conditions
        simuNode.addObject('BoxROI', box=self.opt['model']['bc']['boxes'], name='fixedBox', drawBoxes='1')
        simuNode.addObject('FixedConstraint', indices='@fixedBox.indices')

        ### export data
        if self.saveGeo:
            simuNode.addObject('VTKExporter', filename=self.geoFolder+'/object.vtk', XMLformat='0', listening='1', edges="0", triangles="0", quads="0", tetras="1", exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1")

        if self.saveObs:
            simuNode.addObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1')
            simuNode.addObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName=self.obsFile, ExportPositions='1', ExportVelocities='0', ExportForces='0')

        ### collision handling
        if self.planeCollision:
            simuNode.addObject('PardisoConstraintCorrection', solverName='lsolver', schurSolverName='lsolver')
            # simuNode.addObject('LinearSolverConstraintCorrection')

        if self.planeCollision:
            surface = simuNode.addChild('collision')
            surface.addObject('MeshSTLLoader', name='sloader', filename=self.opt['model']['surf_mesh'])
            surface.addObject('TriangleSetTopologyContainer', position='@sloader.position', name='TriangleContainer', triangles='@sloader.triangles')
            surface.addObject('TriangleSetTopologyModifier', name='Modifier')
            surface.addObject('MechanicalObject', showIndices='false', name='mstate')
            surface.addObject('TriangleCollisionModel', color='1 0 0 1', group=0)
            surface.addObject('LineCollisionModel', color='1 0 0 1', group=0)
            surface.addObject('PointCollisionModel', color='1 0 0 1', group=0)
            surface.addObject('BarycentricMapping', name='bpmapping')

        if self.planeCollision:
            floor = simuNode.addChild('floor')
            floor.addObject('RegularGrid', nx="2", ny="2", nz="2", xmin="-0.1", xmax="0.1",  ymin="-0.059", ymax="-0.061", zmin="0.0", zmax="0.3")
            floor.addObject('MechanicalObject', template="Vec3d")
            floor.addObject('TriangleCollisionModel',simulated="false", bothSide="true", contactFriction="0.00", color="1 0 0 1")
            floor.addObject('LineCollisionModel', simulated="false", bothSide="true", contactFriction="0.0")
            floor.addObject('PointCollisionModel', simulated="false", bothSide="true", contactFriction="0.0")

        ### visual node
        oglNode = simuNode.addChild('oglNode')
        oglNode.addObject('OglModel')
        oglNode.addObject('BarycentricMapping')

        return 0;




    def initGraph(self, node):
        return 0;

    def storeResetState(self):
        return 0;

    def cleanup(self):
        if self.saveObs or self.saveGeo:
            print('Observations saved to '+ self.mainFolder)
        return 0;

    def onAnimateEndEvent(self, deltaTime):
        return 0;

    def onLoaded(self, node):
        return 0;

    def reset(self):
        return 0;

    def bwdInitGraph(self, node):
        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        return 0;

    def onAnimateBeginEvent(self, deltaTime):
        return 0;

