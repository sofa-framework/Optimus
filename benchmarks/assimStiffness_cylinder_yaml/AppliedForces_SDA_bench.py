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
    rootNode.addObject('RequiredPlugin', name='SparseSolver', pluginName='SofaSparseSolver')
    rootNode.addObject('RequiredPlugin', name='BoundaryCondition', pluginName='SofaBoundaryCondition')
    rootNode.addObject('RequiredPlugin', name='SLoader', pluginName='SofaLoader')
    rootNode.addObject('RequiredPlugin', name='SimpleFem', pluginName='SofaSimpleFem')
    rootNode.addObject('RequiredPlugin', name='GraphComponent', pluginName='SofaGraphComponent')
    # rootNode.addObject('RequiredPlugin', name='Python3', pluginName='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='Optimus', pluginName='Optimus')

    try:
        sys.argv[0]
    except:
        commandLineArguments = []
    else:
        commandLineArguments = sys.argv

    #if len(commandLineArguments) > 1:
    #    configFileName = commandLineArguments[1]
    configFileName = "cyl10_appliedGravity_bench.yml"
    #else:
    #    print('ERROR: Must supply a yaml config file as an argument!')
    #    return


    with open(configFileName, 'r') as stream:
        try:
            options = yaml.safe_load(stream)

        except yaml.YAMLError as exc:
            print(exc)
            return

    if options['model']['int']['linear_type'] == 'Pardiso':
        rootNode.addObject('RequiredPlugin', name='PardisoSolver', pluginName='SofaPardisoSolver')

    rootNode.addObject(AppliedForcesSDA_Controller(name="AppliedForces_SDA", node=rootNode, opt=options))
    return 0




class AppliedForcesSDA_Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.rootNode = kwargs["node"]
        self.opt = kwargs["opt"]

        pp = pprint.PrettyPrinter(indent = 4)
        pp.pprint(self.opt)

        self.planeCollision = self.opt['model']['plane_collision']
        self.saveEst = self.opt['io']['saveEst']
        self.saveGeo = self.opt["io"]["saveGeo"]

        prefix = self.opt['io']['prefix']
        suffix = self.opt['io']['suffix']

        if self.planeCollision:
            prefix = prefix + 'plane_'

        #self.mainFolder =  '' prefix + opt['model']['int']['type'] + str(opt['model']['int']['maxit']) + suffix
        self.obsFile = 'obs_testing/obs' # self.mainFolder + '/' + opt['io']['obsFileName']
        self.estFolder = 'roukf_testing' # self.mainFolder + '/' + opt['filter']['kind'] + '_' + opt['filter']['obs_tag'] + opt['io']['sdaFolderSuffix']

        if self.saveEst:
            #os.system('mv '+self.estFolder+' '+self.estFolder+'_arch')
            if not os.path.isdir(self.estFolder):
                os.system('mkdir ' + self.estFolder)

            self.stateExpFile=self.estFolder+'/state.txt'
            self.stateVarFile=self.estFolder+'/variance.txt'
            self.stateCovarFile=self.estFolder+'/covariance.txt'
            # os.system('rm '+self.stateExpFile)
            # os.system('rm '+self.stateVarFile)
            # os.system('rm '+self.stateCovarFile)

        if self.saveGeo:
            self.geoFolder = self.estFolder + '/VTK'
            os.system('mkdir -p ' + self.geoFolder)

        self.createGraph(self.rootNode)
        return



    def createGraph(self, rootNode):
        ### scene global stuff
        rootNode.findData('dt').value = self.opt['model']['dt']
        rootNode.findData('gravity').value = self.opt['model']['gravity']

        rootNode.addObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        rootNode.addObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisualModels')

        rootNode.addObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")

        ### filter data
        self.filterKind = self.opt['filter']['kind']
        if self.filterKind == 'ROUKF':
            self.filter = rootNode.addObject('ROUKFilter', name="ROUKF", verbose="1", useBlasToMultiply='0')
            estimatePosition = 1
        elif self.filterKind == 'UKFSimCorr':
            self.filter = rootNode.addObject('UKFilterSimCorr', name="UKFSC", verbose="1")
            estimatePosition = 0

        ### object loader
        rootNode.addObject('MeshVTKLoader', name='loader', filename=self.opt['model']['vol_mesh'])
        rootNode.addObject('MeshSTLLoader', name='sloader', filename=self.opt['model']['surf_mesh'])

        ### common components for simulation
        modelNode = rootNode.addChild('ModelNode')
        modelNode.addObject('StochasticStateWrapper', name="StateWrapper", verbose='1', langrangeMultipliers=self.planeCollision, estimatePosition=estimatePosition)

        ### solver to handle contact
        if self.planeCollision == 1:
            modelNode.addObject('GenericConstraintSolver', maxIterations='1000', tolerance='1e-6', printLog='0', allVerified='0')
            modelNode.addObject('CollisionPipeline', depth="6", verbose="0", draw="0")
            modelNode.addObject('BruteForceDetection', name="N2")
            modelNode.addObject('LocalMinDistance', name="Proximity",  alarmDistance='0.002', contactDistance='0.001',  angleCone='90.0', filterIntersection='0')
            modelNode.addObject('DefaultContactManager', name="Response", response="FrictionContact", responseParams='mu=0')

        ### general node
        simuNode = modelNode.addChild('cylinder')

        ### solvers
        intType = self.opt['model']['int']['type']
        intLinearType = self.opt['model']['int']['linear_type']
        intMaxit = self.opt['model']['int']['maxit']
        rmass = self.opt['model']['int']['rmass']
        rstiff = self.opt['model']['int']['rstiff']

        if intType == 'Euler':
            simuNode.addObject('EulerImplicitSolver', rayleighStiffness=rstiff, rayleighMass=rmass)
        elif intType == 'Newton':
            simuNode.addObject('NewtonStaticSolver', name="NewtonStatic", printLog="0", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt="2")
        else:
            print('Unknown solver type!')

        if intLinearType == 'Pardiso':
            simuNode.addObject('SparsePARDISOSolver', name='lsolver', verbose='0', pardisoSchurComplement=self.planeCollision, symmetric=self.opt['model']['linsol']['pardisoSym'], exportDataToFolder=self.opt['model']['linsol']['pardisoFolder'])
        elif intLinearType == 'LDL':
            simuNode.addObject('SparseLDLSolver', printLog="0")
        elif intLinearType == 'CG':
            simuNode.addObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')
            if self.opt['model']['linsol']['usePCG']:
                simuNode.addObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='1', use_precond='1', tolerance='1e-10', iterations='500', verbose='0', listening='1', preconditioners='lsolver')
        else:
            print('Unknown linear solver type!')

        ### mechanical object
        simuNode.addObject('MechanicalObject', src="@/loader", name="Volume")
        simuNode.addObject('BoxROI', box=self.opt['model']['bc']['boxes'], name='fixedBox')
        simuNode.addObject('FixedConstraint', indices='@fixedBox.indices')
        simuNode.addObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        simuNode.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.addObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        if 'total_mass' in self.opt['model'].keys():
            simuNode.addObject('UniformMass', totalMass=self.opt['model']['total_mass'])
        if 'density' in self.opt['model'].keys():
            simuNode.addObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.opt['model']['density'], name='mass')

        ### estimate stiffness
        simuNode.addObject('OptimParams', name="paramE", optimize="1", template="Vector", numParams=self.opt['filter']['nparams'], transformParams=self.opt['filter']['param_transform'], initValue=self.opt['filter']['param_init_exval'], stdev=self.opt['filter']['param_init_stdev'])

        ### material properties
        youngModuli = self.opt['model']['young_moduli']
        indices = list(range(1, len(youngModuli)+1))
        simuNode.addObject('Indices2ValuesMapper', indices=indices, values='@paramE.value', name='youngMapper', inputValues='@/loader.dataset')
        simuNode.addObject('TetrahedronFEMForceField', name='FEM', updateStiffness='1', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')

        ### export data
        if self.saveGeo:
            simuNode.addObject('VTKExporterDA', filename=self.geoFolder+'/object.vtk', XMLformat='0', listening='1', edges="0", triangles="0", quads="0", tetras="1", exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1")

        ### contact data
        if self.planeCollision == 1:
            # simuNode.addObject('LinearSolverConstraintCorrection')
            simuNode.addObject('PardisoConstraintCorrection', solverName='lsolver', schurSolverName='lsolver')

            surface = simuNode.addChild('collision')
            surface.addObject('TriangleSetTopologyContainer', position='@/sloader.position', name='TriangleContainer', triangles='@/sloader.triangles')
            surface.addObject('TriangleSetTopologyModifier', name='Modifier')
            surface.addObject('MechanicalObject', showIndices='false', name='mstate')
            surface.addObject('Triangle', color='1 0 0 1', group=0)
            surface.addObject('Line', color='1 0 0 1', group=0)
            surface.addObject('Point', color='1 0 0 1', group=0)
            surface.addObject('BarycentricMapping', name='bpmapping')

        ### node with groundtruth observations
        obsNode = simuNode.addChild('observations')
        obsNode.addObject('MeshVTKLoader', name='obsloader', filename=self.opt['filter']['obs_points'])
        obsNode.addObject('MechanicalObject', name='SourceMO', position='@obsloader.position')
        obsNode.addObject('VTKExporter', name='temporaryExporter', filename='tempObs.vtk', XMLformat='0', listening='1', edges="0", triangles="0", quads="0", tetras="0", exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="0", position='@SourceMO.position')
        obsNode.addObject('BarycentricMapping')
        obsNode.addObject('MappedStateObservationManager', name="MOBS", listening="1", stateWrapper="@../../StateWrapper", verbose="1", observationStdev=self.opt['filter']['observ_stdev'], doNotMapObservations="1", noiseStdev='0.0')
        obsNode.addObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.obsFile)
        obsNode.addObject('ShowSpheres', name="estimated", radius="0.002", color="1 0 0 1", position='@SourceMO.position')
        obsNode.addObject('ShowSpheres', name="groundTruth", radius="0.0015", color="1 1 0 1", position='@MOBS.mappedObservations')

        ### contact data
        if self.planeCollision == 1:
            floor = modelNode.addChild('floor')
            floor.addObject('RegularGrid', nx="2", ny="2", nz="2", xmin="-0.1", xmax="0.1",  ymin="-0.059", ymax="-0.061", zmin="0.0", zmax="0.3")
            floor.addObject('MechanicalObject', template="Vec3d")
            floor.addObject('Triangle',simulated="false", bothSide="true", contactFriction="0.00", color="1 1 0 1")
            floor.addObject('Line', simulated="false", bothSide="true", contactFriction="0.0", color="1 1 0 1")
            floor.addObject('Point', simulated="false", bothSide="true", contactFriction="0.0", color="1 1 0 1")

        return 0



    def initGraph(self, node):
        print('Init graph called (python side)')
        self.step = 0
        self.total_time = 0

        return 0


    def onAnimateEndEvent(self, deltaTime):
        ### save filtering data to files
        if self.saveEst:
            stateName = 'reducedState' if self.filterKind == 'ROUKF' else 'state'
            varName = 'reducedVariance' if self.filterKind == 'ROUKF' else 'variance'
            covarName = 'reducedCovariance' if self.filterKind == 'ROUKF' else 'covariance'

            rs = self.filter.findData(stateName).value
            state = [val for val in rs]
            # print('State:')
            # print(state)
            # print(reducedState)

            f1 = open(self.stateExpFile, "a")
            f1.write(" ".join(map(lambda x: str(x), state)))
            f1.write('\n')
            f1.close()

            rv = self.filter.findData(varName).value
            variance = [val for val in rv]
            # print('Stdev: ' + np.sqrt(variance))
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
        return 0;

    def onScriptEvent(self, senderNode, eventName, data):
        return 0;

