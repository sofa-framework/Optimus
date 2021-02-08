import Sofa
import sys
import math
import os
import csv
import yaml
import pprint
import numpy as np
import time

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

    #if len(commandLineArguments) > 1:
    #    configFileName = commandLineArguments[1]
    configFileName = "beamApplyForce.yml"
    #else:
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

    rootNode.addObject(AppStiffSDA_Controller(name="AppStiff_SDA", node=rootNode, opt=options))
    return 0;




class AppStiffSDA_Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        ### These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rootNode = kwargs["node"]
        self.opt = kwargs["opt"]

        ### print options
        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(self.opt)

        ### extract configuration data
        self.saveEst = self.opt['io']['saveEst']

        self.folder = self.opt['io']['folder']
        self.sdaFolder = self.folder + '/' + self.opt['io']['sdaFolder']

        ### create folder to save the observations; back-up if the folder exists already
        stamp='_'+str(int(time.time()))
        os.system('mkdir -p '+self.folder+' /arch')
        os.system('mv --backup -S '+stamp+' '+self.sdaFolder+' '+self.folder+'/arch')
        os.system('mkdir -p '+self.sdaFolder)

        print('Reading observations from ', self.folder)
        self.obsFile = self.folder + '/' + self.opt['io']['obsFile']

        ### create file with parameters and additional information
        self.opt['visual_parameters'] = {}
        self.stateFileName = 'state.txt'
        self.opt['visual_parameters']['state_file_name'] = self.stateFileName
        self.varianceFileName = 'var.txt'
        self.opt['visual_parameters']['variance_file_name'] = self.varianceFileName
        self.covarianceFileName = 'covar.txt'
        self.opt['visual_parameters']['covariance_file_name'] = self.covarianceFileName
        self.innovationFileName = 'innov.txt'
        self.opt['visual_parameters']['innovation_file_name'] = self.innovationFileName

        configFile = self.sdaFolder + '/daconfig.yml'
        with open(configFile, 'w') as stream:
            try:
                yaml.dump(self.opt, stream, default_flow_style=False)
            except yaml.YAMLError as exc:
                print(exc)

        if self.opt['io']['saveGeo']:
            self.geoFolder = self.sdaFolder + '/VTK'
            os.system('mkdir -p '+self.geoFolder)

        self.stateExpFile=self.sdaFolder+'/state.txt'
        self.stateVarFile=self.sdaFolder+'/var.txt'
        self.stateCovarFile=self.sdaFolder+'/covar.txt'

        self.createGraph(self.rootNode)

        return




    def createGraph(self, rootNode):
        ### scene global stuff
        self.step = 0
        self.incStep = 0
        self.rootNode = rootNode
        self.iterations = 0

        rootNode.findData('dt').value = self.opt['model']['dt']
        rootNode.findData('gravity').value = self.opt['model']['gravity']

        rootNode.addObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisualModels')

        rootNode.addObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1", printLog='1')

        ### filter data
        self.filterKind = self.opt['filter']['kind']
        if self.filterKind == 'ROUKF':
            self.filter = rootNode.addObject('ROUKFilter', name="ROUKF", verbose="1", useBlasToMultiply='1', sigmaTopology=self.opt['filter']['sigma_points_topology'])
            estimatePosition = 1
        elif self.filterKind == 'UKFSimCorr':
            self.filter = rootNode.addObject('UKFilterSimCorr', name="UKFSC", verbose="1", sigmaTopology=self.opt['filter']['sigma_points_topology'])
            estimatePosition = 0
        elif self.filterKind == 'UKFClassic':
            self.filter = rootNode.addObject('UKFilterClassic', name="UKFClas", printLog='1', verbose="1", sigmaTopology=self.opt['filter']['sigma_points_topology'], exportPrefix=self.sdaFolder)
            estimatePosition = 1

        ### object loader
        rootNode.addObject('MeshVTKLoader', name='loader', filename=self.opt['model']['volumeMesh'])
        rootNode.addObject('MeshSTLLoader', name='sloader', filename=self.opt['model']['surfaceMesh'])

        ### general node
        modelNode = rootNode.addChild('ModelNode')

        ### object node
        simuNode = modelNode.addChild('cylinder')

        if self.filterKind == 'UKFClassic':
            posP0 = self.opt['filter']['posP0']
            posQ = self.opt['filter']['posQ']
            paramQ = self.opt['filter']['paramQ']
            simuNode.addObject('StochasticStateWrapper', name="StateWrapper", verbose='1', printLog='1', langrangeMultipliers=self.planeCollision, estimatePosition=estimatePosition, estimateVelocity='0', draw='1', radiusDraw='0.0002', posModelStdev=posQ, paramModelStdev=paramQ, positionStdev=posP0)
        else:
            simuNode.addObject('StochasticStateWrapper', name="StateWrapper", verbose='1', printLog='1', langrangeMultipliers=0, estimatePosition=estimatePosition, estimateVelocity='0', draw='1', radiusDraw='0.0002')

        ### solvers
        intType = self.opt['model']['int']['type']
        if intType == 'Euler':
            simuNode.addObject('EulerImplicitSolver', firstOrder = self.opt['model']['int']['first_order'], rayleighStiffness=self.opt['model']['int']['rstiff'], rayleighMass=self.opt['model']['int']['rmass'])
        elif intType == 'Newton':
            simuNode.addObject('StaticSolver', name="NewtonStatic", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations=self.opt['model']['int']['maxit'], printLog=self.opt['model']['int']['verbose'])

        linType = self.opt['model']['int']['lin_type']
        if linType == 'Pardiso':
            simuNode.addObject('SparsePARDISOSolver', name='lsolver', verbose='0', symmetric=self.opt['model']['linsol']['pardisoSym'], exportDataToFolder=self.opt['model']['linsol']['pardisoFolder'])
        elif linType == 'CG':
            simuNode.addObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')

        ### mechanical object
        simuNode.addObject('MechanicalObject', src="@/loader", name="Volume")
        simuNode.addObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        simuNode.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.addObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")

        simuNode.addObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.opt['model']['density'], name='mass')

        ### elasticiy properties estimation
        simuNode.addObject('OptimParams', name="paramE", optimize="1", template="Vector", numParams=self.opt['filter']['nparams'], transformParams=self.opt['filter']['param_transform'], initValue=self.opt['filter']['param_init_exval'], stdev=self.opt['filter']['param_init_stdev'], minValue=self.opt['filter']['param_min_val'], maxValue=self.opt['filter']['param_max_val'])

        method = self.opt['model']['fem']['method']
        nu = self.opt['model']['fem']['poisson_ratio']
        if  method[0:3] == 'Cor':
            simuNode.addObject('TetrahedronFEMForceField', name='FEM', method=method[3:].lower(), listening='true', poissonRatio=nu, youngModulus='@paramE.value', updateStiffness='1')
        elif method == 'StVenant':
            simuNode.addObject('Indices2ValuesTransformer', name='pm', indices=[1], values1='@paramE.value', values2=nu, inputValues='@loader.dataset', transformation='ENu2MuLambda')
            simuNode.addObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet='@pm.outputValues')

        ### boundary conditions
        simuNode.addObject('BoxROI', box=self.opt['model']['bc']['boxes'], name='fixedBox')
        simuNode.addObject('FixedConstraint', indices='@fixedBox.indices')

        ### applied force
        if 'applied_force' in self.opt['model'].keys():
            simuNode.addObject('BoxROI', name='forceBox', box=self.opt['model']['applied_force']['boxes'])
            self.appliedForce = simuNode.addObject('ConstantForceField', force=self.opt['model']['applied_force']['initial_force'], indices='@forceBox.indices')

        ### node with groundtruth observations
        obsNode = simuNode.addChild('observations')
        obsNode.addObject('MeshVTKLoader', name='obsloader', filename=self.opt['model']['observationPoints'])
        obsNode.addObject('MechanicalObject', name='SourceMO', position='@obsloader.position')
        obsNode.addObject('BarycentricMapping')
        obsNode.addObject('MappedStateObservationManager', name="MOBS", listening="1", stateWrapper="@../StateWrapper", verbose="1", observationStdev=self.opt['filter']['observ_stdev'], noiseStdev=self.opt['filter']['obs_added_noise_var'], doNotMapObservations='1')
        obsNode.addObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.obsFile)
        obsNode.addObject('ShowSpheres', name="estimated", radius="0.002", color="1 0 0 1", position='@SourceMO.position')
        obsNode.addObject('ShowSpheres', name="groundTruth", radius="0.0015", color="1 1 0 1", position='@MOBS.mappedObservations')

        ### export estimated mesh
        if self.opt['io']['saveGeo']:
            simuNode.addObject('VTKExporterDA', filename=self.geoFolder+'/object.vtk', XMLformat='0', listening='1', edges="0", triangles="0", quads="0", tetras="1", exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1", printLog='0')

        return 0




    def initGraph(self, node):
        return 0


    def bwdInitGraph(self, node):
        self.exportStochasticState()
        return 0


    def onAnimateBeginEvent(self, deltaTime):
        self.step += 1
        ### modify applied force
        if 'applied_force' in self.opt['model'].keys():
            maxTS = self.opt['model']['applied_force']['num_inc_steps']
            delta = np.array(self.opt['model']['applied_force']['delta'])
            if self.step < maxTS:
                fc = np.array(self.appliedForce.findData('force').value)
                fc += delta
                self.appliedForce.findData('force').value = fc.tolist()

        return 0



    def onAnimateEndEvent(self, deltaTime):
        self.iterations = self.iterations + 1
        self.exportStochasticState()

        return 0


    ### save filtering data to files
    def exportStochasticState(self):
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
            # print('Covariance:', covariance)
            estStd = np.sqrt(variance)
            # print('Correlation: ', covariance[0]/(np.prod(estStd)))

            f3 = open(self.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), covariance)))
            f3.write('\n')
            f3.close()

            return


    def cleanup(self):
        if self.saveEst:
            print('Estimations saved to '+self.sdaFolder)
        if self.opt['io']['saveGeo']:
            print('Geometries saved to '+self.geoFolder)
        return 0;

    def onScriptEvent(self, senderNode, eventName, data):
        return 0;

