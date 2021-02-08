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
    rootNode.addObject('RequiredPlugin', name='GraphComponent', pluginName='SofaGraphComponent')
    rootNode.addObject('RequiredPlugin', name='Visual', pluginName='SofaOpenglVisual')
    # rootNode.addObject('RequiredPlugin', name='Python3', pluginName='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='Optim', pluginName='Optimus')

    try:
        sys.argv[0]
    except:
        commandLineArguments = []
    else:
        commandLineArguments = sys.argv

    # if len(commandLineArguments) > 1:
    #     configFileName = commandLineArguments[1]
    configFileName = "cylinder10AppledGravity.yml"
    # else:
    #     print 'ERROR: Must supply a yaml config file as an argument!'
    #     return

    with open(configFileName, 'r') as stream:
        try:
            options = yaml.load(stream)

        except yaml.YAMLError as exc:
            print(exc)
            return

    if options['model']['int']['lin_type'] == 'Pardiso':
        rootNode.addObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')

    rootNode.addObject(AppliedForcesSDA_Controller(name="AppliedForces_SDA", node=rootNode, opt=options))
    return 0;




class AppliedForcesSDA_Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        ### These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rootNode = kwargs["node"]
        self.opt = kwargs["opt"]

        ### print options
        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(self.opt)

        ### extract configuration data
        self.planeCollision = self.opt['model']['plane_collision']
        self.saveEst = self.opt['io']['saveEst']
        self.saveGeo = self.opt["io"]["saveGeo"]

        prefix = self.opt['io']['prefix']
        suffix = self.opt['io']['suffix']

        if self.planeCollision:
            prefix = prefix + 'plane_'

        self.obsFile = 'observations/'+ self.opt['io']['obsFileName']
        self.estFolder = 'ROUKF'

        if self.saveEst:
            os.system('mv '+self.estFolder+' '+self.estFolder+'_arch')
            os.system('mkdir '+self.estFolder)

            self.stateExpFile=self.estFolder+'/state.txt'
            self.stateVarFile=self.estFolder+'/variance.txt'
            self.stateCovarFile=self.estFolder+'/covariance.txt'

        if self.saveGeo:
            self.geoFolder = self.estFolder + '/VTK'
            os.system('mkdir -p '+self.geoFolder)

        ### create file with parameters and additional information
        self.opt['visual_parameters'] = {}
        self.stateFileName = 'state.txt'
        self.opt['visual_parameters']['state_file_name'] = self.stateFileName
        self.varianceFileName = 'variance.txt'
        self.opt['visual_parameters']['variance_file_name'] = self.varianceFileName
        self.covarianceFileName = 'covariance.txt'
        self.opt['visual_parameters']['covariance_file_name'] = self.covarianceFileName
        self.innovationFileName = 'innovation.txt'
        self.opt['visual_parameters']['innovation_file_name'] = self.innovationFileName

        self.informationFileName = self.estFolder + '/daconfig.yml'

        with open(self.informationFileName, 'w') as stream:
            try:
                yaml.dump(self.opt, stream, default_flow_style = False)

            except yaml.YAMLError as exc:
                print(exc)
                return

        self.createGraph(self.rootNode)

        return



    def createGraph(self, rootNode):
        self.rootNode = rootNode

        ### scene global stuff
        rootNode.findData('dt').value = self.opt['model']['dt']
        rootNode.findData('gravity').value = self.opt['model']['gravity']
        rootNode.addObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisualModels')

        rootNode.addObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")

        ### filter data
        self.filterKind = self.opt['filter']['kind']
        if self.filterKind == 'ROUKF':
            self.filter = rootNode.addObject('ROUKFilter', name="ROUKF", verbose="1", useBlasToMultiply='0', printLog='1', sigmaTopology=self.opt['filter']['sigma_points_topology'])
            estimatePosition = 1
        elif self.filterKind == 'UKFSimCorr':
            self.filter = rootNode.addObject('UKFilterSimCorr', name="UKFSC", verbose="1")
            estimatePosition = 0

        ### general node
        modelNode = rootNode.addChild('ModelNode')
        modelNode.addObject('StochasticStateWrapper', name="StateWrapper", verbose='1', langrangeMultipliers=self.planeCollision, estimatePosition=estimatePosition)

        if self.planeCollision == 1:
            modelNode.addObject('GenericConstraintSolver', maxIterations='1000', tolerance='1e-6', printLog='0', allVerified='0')
            modelNode.addObject('CollisionPipeline', depth="6", verbose="0", draw="0")
            modelNode.addObject('BruteForceDetection', name="N2")
            modelNode.addObject('LocalMinDistance', name="Proximity",  alarmDistance='0.002', contactDistance='0.001',  angleCone='90.0', filterIntersection='0')
            modelNode.addObject('DefaultContactManager', name="Response", response="FrictionContact", responseParams='mu=0')

        ### object node
        simuNode = modelNode.addChild('cylinder')

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
        lsconf = self.opt['model']['linsol']
        if linType == 'Pardiso':
            simuNode.addObject('SparsePARDISOSolver', name='lsolver', verbose='0', pardisoSchurComplement=self.planeCollision, symmetric=lsconf['pardisoSym'], exportDataToFolder=lsconf['pardisoFolder'])
        elif linType == 'CG':
            simuNode.addObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')
            if lsconf['usePCG'] == 1:
                simuNode.addObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='1', use_precond='1', tolerance=lsconf['pcgTol'], iterations=lsconf['pcgIt'], verbose=lsconf['pcgVerb'], update_step=lsconf['pcgUpdateStep'], numIterationsToRefactorize=lsconf['pcgAdaptiveThreshold'], listening='1', preconditioners='lsolver')

        ### object loader
        simuNode.addObject('MeshVTKLoader', name='loader', filename=self.opt['model']['vol_mesh'])
        simuNode.addObject('MeshSTLLoader', name='sloader', filename=self.opt['model']['surf_mesh'])

        ### mechanical object
        simuNode.addObject('MechanicalObject', src="@loader", name="Volume")
        simuNode.addObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.addObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")

        if 'total_mass' in self.opt['model'].keys():
            simuNode.addObject('UniformMass', totalMass=self.opt['model']['total_mass'])

        if 'density' in self.opt['model'].keys():
            simuNode.addObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.opt['model']['density'], name='mass')

        ### elasticiy properties estimation
        simuNode.addObject('OptimParams', name="paramE", optimize="1", template="Vector", numParams=self.opt['filter']['nparams'], transformParams=self.opt['filter']['param_transform'], initValue=self.opt['filter']['param_init_exval'], stdev=self.opt['filter']['param_init_stdev'])

        youngModuli=self.opt['model']['fem']['young_moduli']
        poissonRatio = self.opt['model']['fem']['poisson_ratio']
        indices = list(range(1, len(youngModuli)+1))
        method = self.opt['model']['fem']['method']
        if  method[0:3] == 'Cor':
            simuNode.addObject('Indices2ValuesMapper', indices=indices, values='@paramE.value', name='youngMapper', inputValues='@loader.dataset')
            simuNode.addObject('TetrahedronFEMForceField', name='FEM', method=method[3:].lower(), listening='true', drawHeterogeneousTetra='1', poissonRatio=poissonRatio, youngModulus='@youngMapper.outputValues', updateStiffness='1')
        elif method == 'StVenant':
            poissonRatii = poissonRatio * np.ones([1,len(youngModuli)])
            simuNode.addObject('Indices2ValuesTransformer', name='paramMapper', indices=indices, values1='@paramE.value', values2=poissonRatii, inputValues='@loader.dataset', transformation='ENu2MuLambda')
            simuNode.addObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet='@paramMapper.outputValues', drawHeterogeneousTetra='1')

        ### boundary conditions
        simuNode.addObject('BoxROI', box=self.opt['model']['bc']['boxes'], name='fixedBox')
        simuNode.addObject('FixedConstraint', indices='@fixedBox.indices')

        ### export estimated mesh
        if self.saveGeo:
            simuNode.addObject('VTKExporterDA', filename=self.geoFolder+'/object.vtk', XMLformat='0', listening='1', edges="0", triangles="0", quads="0", tetras="1",  exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1")


        ### process collision data
        if self.planeCollision == 1:
            # simuNode.createObject('LinearSolverConstraintCorrection')
            simuNode.addObject('PardisoConstraintCorrection', solverName='lsolver', schurSolverName='lsolver')

        if self.planeCollision == 1:
            surface=simuNode.addChild('collision')
            surface.addObject('TriangleSetTopologyContainer', position='@sloader.position', name='TriangleContainer', triangles='@sloader.triangles')
            surface.addObject('TriangleSetTopologyModifier', name='Modifier')
            surface.addObject('MechanicalObject', showIndices='false', name='mstate')
            surface.addObject('TriangleCollisionModel', color='1 0 0 1', group=0)
            surface.addObject('LineCollisionModel', color='1 0 0 1', group=0)
            surface.addObject('PointCollisionModel', color='1 0 0 1', group=0)
            surface.addObject('BarycentricMapping', name='bpmapping')

        ### node with groundtruth observations
        obsNode = simuNode.addChild('observations')
        obsNode.addObject('MeshVTKLoader', name='obsloader', filename=self.opt['filter']['obs_points'])
        obsNode.addObject('MechanicalObject', name='SourceMO', position='@obsloader.position')
        obsNode.addObject('VTKExporter', name='temporaryExporter', filename='tempObs.vtk', XMLformat='0',listening='1',edges="0",triangles="0",quads="0",tetras="0", exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="0", position='@SourceMO.position')
        obsNode.addObject('BarycentricMapping')
        obsNode.addObject('MappedStateObservationManager', name="MOBS", listening="1", stateWrapper="@../../StateWrapper", verbose="1", observationStdev=self.opt['filter']['observ_stdev'], noiseStdev=self.opt['filter']['observ_noise_stdev'])
        obsNode.addObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.obsFile)
        obsNode.addObject('ShowSpheres', name="estimated", radius="0.002", color="1 0 0 1", position='@SourceMO.position')
        obsNode.addObject('ShowSpheres', name="groundTruth", radius="0.0015", color="1 1 0 1", position='@MOBS.mappedObservations')

        ### visual node
        oglNode = simuNode.addChild('visualization')
        oglNode.addObject('OglModel', color='1 0 0 1')
        oglNode.addObject('BarycentricMapping')

        ### collision plane
        if self.planeCollision == 1:
            floor = modelNode.addChild('floor')
            floor.addObject('RegularGrid', nx="2", ny="2", nz="2", xmin="-0.1", xmax="0.1",  ymin="-0.059", ymax="-0.061", zmin="0.0", zmax="0.3")
            floor.addObject('MechanicalObject', template="Vec3d")
            floor.addObject('TriangleCollisionModel',simulated="false", bothSide="true", contactFriction="0.00", color="1 1 0 1")
            floor.addObject('LineCollisionModel', simulated="false", bothSide="true", contactFriction="0.0", color="1 1 0 1")
            floor.addObject('PointCollisionModel', simulated="false", bothSide="true", contactFriction="0.0", color="1 1 0 1")

        return 0


    def initGraph(self, node):
        print('Init graph called (python side)')
        self.step = 0
        self.total_time = 0
        return 0


    ### save filtering data to files
    def onAnimateEndEvent(self, deltaTime):

        if self.saveEst:
            stateName = 'reducedState' if self.filterKind == 'ROUKF' else 'state'
            varName = 'reducedVariance' if self.filterKind == 'ROUKF' else 'variance'
            covarName = 'reducedCovariance' if self.filterKind == 'ROUKF' else 'covariance'

            rs = self.filter.findData(stateName).value
            state = [val for val in rs]
            print('State:', state)
            # print(reducedState)

            f1 = open(self.stateExpFile, "a")
            f1.write(" ".join(map(lambda x: str(x), state)))
            f1.write('\n')
            f1.close()

            rv = self.filter.findData(varName).value
            variance = [val for val in rv]
            print('Stdev: ', np.sqrt(variance))
            # print('Reduced variance:')
            # print(reducedVariance)

            f2 = open(self.stateVarFile, "a")
            f2.write(" ".join(map(lambda x: str(x), variance)))
            f2.write('\n')
            f2.close()

            rcv = self.filter.findData(covarName).value
            covariance = [val for val in rcv]
            # print('Reduced Covariance:')
            # print(reducedCovariance)

            f3 = open(self.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), covariance)))
            f3.write('\n')
            f3.close()

        # print(self.basePoints.findData('indices_position').value)

        return 0

    def cleanup(self):
        if self.saveEst:
            print('Estimations saved to '+self.estFolder)

        if self.saveGeo:
            print('Geometries saved to '+self.geoFolder)

        return 0;

    def onScriptEvent(self, senderNode, eventName, data):
        return 0;

