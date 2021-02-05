import Sofa
import sys
import math
import os
import csv
import yaml
sys.path.append(os.getcwd() + '/../../python_src/utils')
from FileSystemUtils import FolderHandler

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='Engine', pluginName='SofaEngine')
    rootNode.addObject('RequiredPlugin', name='ImplicitOdeSolver', pluginName='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='Loader', pluginName='SofaLoader')
    rootNode.addObject('RequiredPlugin', name='MiscForceField', pluginName='SofaMiscForceField')
    rootNode.addObject('RequiredPlugin', name='SimpleFem', pluginName='SofaSimpleFem')
    rootNode.addObject('RequiredPlugin', name='Deformable', pluginName='SofaDeformable')
    rootNode.addObject('RequiredPlugin', name='GraphComponent', pluginName='SofaGraphComponent')
    # rootNode.addObject('RequiredPlugin', name='Python3', pluginName='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='Optimus', pluginName='Optimus')

    try:
        sys.argv[0]
    except:
        commandLineArguments = []
    else:
        commandLineArguments = sys.argv

    # if len(commandLineArguments) > 1:
    #     configFileName = commandLineArguments[1]
    configFileName = "liver_nonlinear_BC_point_cloud_scene_config_bench.yml"
    # else:
    #     print('ERROR: Must supply a yaml config file as an argument!')
    #     return

    print("Command line arguments for python : " + str(commandLineArguments))

    with open(configFileName, 'r') as stream:
        try:
            options = yaml.safe_load(stream)

        except yaml.YAMLError as exc:
            print(exc)
            return

    if options['general_parameters']['linear_solver_kind'] == 'Pardiso':
        rootNode.addObject('RequiredPlugin', name='PardisoSolver', pluginName='SofaPardisoSolver')

    rootNode.addObject(LiverControlPointSDA_Controller(name="LiverControlPoint_SDA", node=rootNode, opt=options))
    return 0




class LiverControlPointSDA_Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.rootNode = kwargs["node"]
        self.options = kwargs["opt"]
        self.cameraReactivated = False
        self.generalFolderName = '.'

        observationInfix = self.options['system_parameters']['observation_points_file_name']
        observationInfix = observationInfix[observationInfix.rfind('/') + 1 : observationInfix.rfind('.')]
        self.folderName = 'roukf_testing'
        folderCreator = FolderHandler()
        folderCreator.createFolder(self.generalFolderName, self.folderName, archiveResults = 0)

        ### create file with parameters and additional information
        self.options['visual_parameters'] = {}
        self.stateFileName = 'state_' + self.options['filtering_parameters']['output_files_suffix'] + '.txt'
        self.options['visual_parameters']['state_file_name'] = self.stateFileName
        self.varianceFileName = 'variance_' + self.options['filtering_parameters']['output_files_suffix'] + '.txt'
        self.options['visual_parameters']['variance_file_name'] = self.varianceFileName
        self.covarianceFileName = 'covariance_' + self.options['filtering_parameters']['output_files_suffix'] + '.txt'
        self.options['visual_parameters']['covariance_file_name'] = self.covarianceFileName
        self.innovationFileName = 'innovation_' + self.options['filtering_parameters']['output_files_suffix'] + '.txt'
        self.options['visual_parameters']['innovation_file_name'] = self.innovationFileName
        
        self.informationFileName = self.folderName + '/daconfig.yml'

        with open(self.informationFileName, 'w') as stream:
            try:
                yaml.dump(self.options, stream, default_flow_style = None)

            except yaml.YAMLError as exc:
                print(exc)
                return

        self.createGraph(self.rootNode)
        if self.options['time_parameters']['time_profiling']:
            self.createTimeProfiler()
        return None



    def createGraph(self, rootNode):
        self.rootNode = rootNode

        self.lambdaScale = 1.0
        if self.options['filtering_parameters']['filter_kind'] == 'ROUKF':
            self.estimPosition='1'
            self.estimVelocity='0'

        if self.options['filtering_parameters']['filter_kind'] == 'UKFSimCorr':
            self.estimPosition='0'
            self.estimVelocity='0'

        if self.options['filtering_parameters']['filter_kind'] == 'UKFClassic':
            self.estimPosition='1'
            self.estimVelocity='0'

        self.createGlobalComponents(self.rootNode)
        masterNode = self.rootNode.addChild('MasterScene')
        self.createMasterScene(masterNode)

        return 0



    def createGlobalComponents(self, rootNode):
        ### scene global stuff
        self.iterations = 0
        rootNode.findData('gravity').value = self.options['general_parameters']['gravity']
        rootNode.findData('dt').value = self.options['general_parameters']['delta_time']

        rootNode.addObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        rootNode.addObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels showInteractionForceFields')

        rootNode.addObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")

        ### filter data
        if (self.options['filtering_parameters']['filter_kind'] == 'ROUKF'):
            self.filter = rootNode.addObject('ROUKFilter', name="ROUKF", verbose="1", useUnbiasedVariance=self.options['filtering_parameters']['use_unbiased_variance'], sigmaTopology=self.options['filtering_parameters']['sigma_points_topology'], lambdaScale=self.lambdaScale)
        elif (self.options['filtering_parameters']['filter_kind'] == 'UKFSimCorr'):
            self.filter = rootNode.addObject('UKFilterSimCorr', name="UKF", verbose="1", useUnbiasedVariance=self.options['filtering_parameters']['use_unbiased_variance'], sigmaTopology=self.options['filtering_parameters']['sigma_points_topology'], lambdaScale=self.lambdaScale)
        elif (self.options['filtering_parameters']['filter_kind'] == 'UKFClassic'):
            self.filter = rootNode.addObject('UKFilterClassic', name="UKFClas", verbose="1", exportPrefix=self.folderName, useUnbiasedVariance=self.options['filtering_parameters']['use_unbiased_variance'], sigmaTopology=self.options['filtering_parameters']['sigma_points_topology'], lambdaScale=self.lambdaScale)
        else:
            print('Unknown filter type!')

        ### object loader
        fileExtension = self.options['system_parameters']['volume_file_name']
        fileExtension = fileExtension[fileExtension.rfind('.') + 1:]
        if fileExtension == 'vtk' or fileExtension == 'vtu':
            rootNode.addObject('MeshVTKLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        elif fileExtension == 'msh':
            rootNode.addObject('MeshGmshLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        else:
            print('Unknown file type!')

        ### fixed nodes
        fixNode = rootNode.addChild('fixNode')
        fixNode.addObject('MechanicalObject', template='Vec3d', name='fixElements', showObject='true', position=self.options['boundary_parameters']['external_points'])
        fixNode.addObject('ShowSpheres', position='@fixElements.position', color='1.0 0.0 1.0 1', radius="0.01", showIndicesScale='0.0')

        ### external impact node
        impactSimu = rootNode.addChild('externalImpSimu')
        impactSimu.addObject('PreStochasticWrapper')
        impactSimu.addObject('EulerImplicitSolver')
        impactSimu.addObject('CGLinearSolver')
        impactSimu.addObject('MechanicalObject', name="state", template='Vec3d', useTopology='false', position=self.options['impact_parameters']['position'])
        impactSimu.addObject('SimulatedStateObservationSource', name="ImpactSim", template='Vec3d', printLog="1", monitorPrefix = self.generalFolderName + '/' + self.options['impact_parameters']['observation_file_name'], drawSize="0.0015", controllerMode="1")
        impactSimu.addObject('ShowSpheres', name="externImp", position='@state.position', color='1.0 0.0 1.0 1', radius="0.01", showIndicesScale='0.0')

        return 0



    ### common components for simulation
    def createCommonComponents(self, node):
        ### solvers
        if self.options['general_parameters']['solver_kind'] == 'Euler':
            node.addObject('EulerImplicitSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'])
        elif self.options['general_parameters']['solver_kind'] == 'Symplectic':
            node.addObject('VariationalSymplecticSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'], newtonError='1e-12', steps='1', verbose='0')
        elif self.options['general_parameters']['solver_kind'] == 'Newton':
            node.addObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="2")
        else:
            print('Unknown solver type!')

        if self.options['general_parameters']['linear_solver_kind'] == 'Pardiso':
            node.addObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToFolder="", iterativeSolverNumbering="0")
        elif self.options['general_parameters']['linear_solver_kind'] == 'CG':
            if self.options['precondition_parameters']['usePCG']:
                node.addObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='1', use_precond='1', tolerance='1e-10', iterations='500', verbose='1', listening='1', update_step=self.options['precondition_parameters']['PCGUpdateSteps'], preconditioners='precond')
            node.addObject('CGLinearSolver', iterations="20", tolerance="1e-12", threshold="1e-12")
            # node.addObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        else:
            print('Unknown linear solver type!')

        ### mechanical object
        node.addObject('MechanicalObject', src="@/loader", name="Volume")
        node.addObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        node.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        node.addObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        if 'total_mass' in self.options['general_parameters'].keys():
            node.addObject('UniformMass', totalMass=self.options['general_parameters']['total_mass'])
        if 'density' in self.options['general_parameters'].keys():
            node.addObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.options['general_parameters']['density'], name='mass')

        ### estimate stiffness
        node.addObject('OptimParams', name="paramE", optimize="1", numParams=self.options['filtering_parameters']['optim_params_size'], template="Vector", initValue=self.options['filtering_parameters']['initial_stiffness'], minValue=self.options['filtering_parameters']['minimal_stiffness'], maxValue=self.options['filtering_parameters']['maximal_stiffness'], stdev=self.options['filtering_parameters']['initial_standart_deviation'], transformParams=self.options['filtering_parameters']['transform_parameters'])
        # nu=0.45
        # E=5000
        # lamb=(E*nu)/((1+nu)*(1-2*nu))
        # mu=E/(2+2*nu)
        # materialParams='{} {}'.format(mu,lamb)
        # node.addObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        node.addObject('TetrahedronFEMForceField', name='FEM', updateStiffness='1', listening='true', drawHeterogeneousTetra='1', method='large', youngModulus=self.options['object_parameters']['young_modulus'], poissonRatio=self.options['object_parameters']['poisson_ratio'])

        ### boundary conditions
        if 'boundary_conditions_list' in self.options['boundary_parameters'].keys():
            for index in range(0, len(self.options['boundary_parameters']['boundary_conditions_list'])):
                bcElement = self.options['boundary_parameters']['boundary_conditions_list'][index]
                node.addObject('BoxROI', box=bcElement['boxes_coordinates'], name='boundBoxes'+str(index), doUpdate='0')
                if bcElement['condition_type'] == 'fixed':
                    node.addObject('FixedConstraint', indices='@boundBoxes'+str(index)+'.indices')
                elif bcElement['condition_type'] == 'elastic':
                    if self.options['boundary_parameters']['spring_type'] == 'PolynomialRestshape':
                        node.addObject('PolynomialRestShapeSpringsForceField', listening="1", printLog="0", points='@boundBoxes'+str(index)+'.indices', initialLength=self.options['boundary_parameters']['initial_length'], polynomialDegree=bcElement['polynomial_degrees'], polynomialStiffness='@paramE.value', drawSpring='1')
                    elif self.options['boundary_parameters']['spring_type'] == 'Polynomial':
                        node.addObject('PolynomialSpringsForceField', listening="1", printLog="0", object1='@.', firstObjectPoints='@boundBoxes'+str(index)+'.indices', object2='@../fixNode/fixElements', secondObjectPoints=self.options['boundary_parameters']['external_indices'], polynomialDegree=bcElement['polynomial_degrees'], polynomialStiffness='@paramE.value', drawMode='1')
                    else:
                        node.addObject('RestShapeSpringForceField', stiffness='@paramE.value', listening="1", angularStiffness="1", printLog="0", points='@boundBoxes'+str(index)+'.indices')
                else:
                    print('Unknown type of boundary conditions')

        ### attachment to external object
        node.addObject('BoxROI', name='impactBounds', box=self.options['object_parameters']['attachment_coordinates'], doUpdate='0')
        self.toolSprings = node.addObject('RestShapeSpringsForceField', name="impactSpring", stiffness="10000", angularStiffness='1', external_rest_shape='@../externalImpSimu/state', points='@impactBounds.indices')

        return 0



    def createMasterScene(self, node):
        node.addObject('StochasticStateWrapper',name="StateWrapper",verbose="1", estimatePosition=self.estimPosition, positionStdev=self.options['filtering_parameters']['positions_standart_deviation'], estimateVelocity=self.estimVelocity)
        self.createCommonComponents(node)
        ### node with groundtruth observations
        obsNode = node.addChild('obsNode')
        if 'use_point_cloud' in self.options['obs_generating_parameters'] and self.options['obs_generating_parameters']['save_observations'] == 1:
            obsNode.addObject('MeshObjLoader', name='obsLoader', filename=self.options['point_cloud_parameters']['mesh_file'])
        else:
            obsNode.addObject('MeshVTKLoader', name='obsLoader', filename=self.options['system_parameters']['observation_points_file_name'])
        obsNode.addObject('MechanicalObject', name='SourceMO', src="@obsLoader")
        obsNode.addObject('BarycentricMapping')
        obsNode.addObject('MappedStateObservationManager', name="MOBS", observationStdev=self.options['filtering_parameters']['observation_noise_standart_deviation'], noiseStdev="0.0", listening="1", stateWrapper="@../StateWrapper", doNotMapObservations="1", verbose="1")
        obsNode.addObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix = self.generalFolderName + '/' + self.options['system_parameters']['observation_file_name'])
        obsNode.addObject('ShowSpheres', name="estimated", radius="0.002", color="1 0 0 1", position='@SourceMO.position')
        obsNode.addObject('ShowSpheres', name="groundTruth", radius="0.0015", color="1 1 0 1", position='@MOBS.mappedObservations')

        return 0




    def initGraph(self, node):
        print('Init graph called (python side)')
        self.step = 0
        self.total_time = 0
        # self.process.initializationObjects(node)

        return 0



    def onAnimateBeginEvent(self, deltaTime):
        if self.options['time_parameters']['time_profiling']:
            Sofa.timerSetEnabled(self.options['time_parameters']['timer_name'], True)
            Sofa.timerBegin(self.options['time_parameters']['timer_name'])

        return 0



    def onAnimateEndEvent(self, deltaTime):
        ### save time statistics
        self.iterations = self.iterations + 1
        self.saveTimeStatistics()

        ### save filtering data to files
        if self.options['filtering_parameters']['save_state']:
            if (self.options['filtering_parameters']['filter_kind'] == 'ROUKF'):
                st = self.filter.findData('reducedState').value
            elif (self.options['filtering_parameters']['filter_kind'] == 'UKFSimCorr' or self.options['filtering_parameters']['filter_kind'] == 'UKFClassic'):
                st = self.filter.findData('state').value

            state = [val for val in st]
            # print('Reduced state:')
            # print(reducedState)

            self.stateExpValFile = self.folderName + '/' + self.stateFileName
            print('Storing to: ' + self.stateExpValFile)
            f1 = open(self.stateExpValFile, "a")
            f1.write(" ".join(map(lambda x: str(x), state)))
            f1.write('\n')
            f1.close()

            if (self.options['filtering_parameters']['filter_kind'] == 'ROUKF'):
                var = self.filter.findData('reducedVariance').value
            elif (self.options['filtering_parameters']['filter_kind'] == 'UKFSimCorr' or self.options['filtering_parameters']['filter_kind'] == 'UKFClassic'):
                var = self.filter.findData('variance').value

            variance = [val for val in var]
            # print('Reduced variance:')
            # print(reducedVariance)

            self.stateVarFile = self.folderName + '/' + self.varianceFileName
            f2 = open(self.stateVarFile, "a")
            f2.write(" ".join(map(lambda x: str(x), variance)))
            f2.write('\n')
            f2.close()

            if (self.options['filtering_parameters']['filter_kind'] == 'ROUKF'):
                covar = self.filter.findData('reducedCovariance').value
            elif (self.options['filtering_parameters']['filter_kind'] == 'UKFSimCorr' or self.options['filtering_parameters']['filter_kind'] == 'UKFClassic'):
                covar = self.filter.findData('covariance').value

            covariance = [val for val in covar]
            # print('Reduced Covariance:')
            # print(reducedCovariance)

            self.stateCovarFile = self.folderName + '/' + self.covarianceFileName
            f3 = open(self.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), covariance)))
            f3.write('\n')
            f3.close()

        if self.options['filtering_parameters']['save_internal_data']:
            if (self.options['filtering_parameters']['filter_kind'] == 'ROUKF'):
                innov = self.filter.findData('reducedInnovation').value
            elif (self.options['filtering_parameters']['filter_kind'] == 'UKFSimCorr' or self.options['filtering_parameters']['filter_kind'] == 'UKFClassic'):
                innov = self.filter.findData('innovation').value

            innovation = [val for val in innov]
            # print('Innovation:')
            # print(innovation)

            self.innovationFile = self.folderName + '/' + self.innovationFileName
            f4 = open(self.innovationFile, "a")
            f4.write(" ".join(map(lambda x: str(x), innovation)))
            f4.write('\n')
            f4.close()

        # print(self.basePoints.findData('indices_position').value)

        return 0


    ### timer functions
    def createTimeProfiler(self):
        print('Time statistics file: ' + self.folderName + '/' + self.options['time_parameters']['time_statistics_file'])
        Sofa.timerSetInterval(self.options['time_parameters']['timer_name'], self.options['time_parameters']['iterations_interval'])    ### Set the number of steps neded to compute the timer
        Sofa.timerSetOutputType(self.options['time_parameters']['timer_name'], 'json')    ### Set output file format
        with open(self.folderName + '/' + self.options['time_parameters']['time_statistics_file'], "a") as outputFile:
            outputFile.write('{')
            outputFile.close()

        return 0

    def saveTimeStatistics(self):
        if self.options['time_parameters']['time_profiling']:
            if self.iterations <= self.options['time_parameters']['iteration_amount']:
                result = Sofa.timerEnd(self.options['time_parameters']['timer_name'], self.rootNode)
                if result != None :
                    with open(self.folderName + '/' + self.options['time_parameters']['time_statistics_file'], "a") as outputFile:
                        outputFile.write(result + ",")
                        outputFile.close()
            ### replace last symbol
            if self.iterations == self.options['time_parameters']['iteration_amount']:
                with open(self.folderName + '/' + self.options['time_parameters']['time_statistics_file'], "a") as outputFile:
                    outputFile.seek(-1, os.SEEK_END)
                    outputFile.truncate()
                    outputFile.write("\n}")
                    outputFile.close()

        return 0


    def onScriptEvent(self, senderNode, eventName, data):
        return 0;

