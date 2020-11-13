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
    # rootNode.createObject('RequiredPlugin', name='SofaMiscFem')
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
            options = yaml.safe_load(stream)

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

        ### extract configuration data
        self.saveObs = opt['io']['saveObs']
        self.saveGeo = opt["io"]["saveGeo"]

        ### generate output folders
        if self.saveObs or self.saveGeo:
            prefix = opt['io']['prefix']
            suffix = opt['io']['suffix']

            self.mainFolder = prefix + opt['model']['fem']['method'] + '_' + opt['model']['int']['type'] + str(opt['model']['int']['maxit']) + suffix

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
        ### scene global stuff
        self.step = 0
        rootNode.findData('dt').value = self.opt['model']['dt']
        rootNode.findData('gravity').value = self.opt['model']['gravity']

        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')

        ### general node
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode
        simuNode.createObject('MeshVTKLoader', name='loader', filename=self.opt['model']['vol_mesh'])

        ### solvers
        intType = self.opt['model']['int']['type']
        if intType == 'Euler':
            rmass = self.opt['model']['int']['rmass']
            rstiff = self.opt['model']['int']['rstiff']
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness=rstiff, rayleighMass=rmass)
        elif intType == 'Newton':
            intMaxit = self.opt['model']['int']['maxit']
            simuNode.createObject('StaticSolver', name="NewtonStatic", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1",  newton_iterations=intMaxit, printLog="1")
        else:
            print 'Unknown solver type!'

        if self.opt['model']['linsol']['usePCG']:
            simuNode.createObject('StepPCGLinearSolver', name='lsolverit',use_precond='1', tolerance='1e-10', iterations='500', verbose='1', listening='1', preconditioners='lsolver',  printLog='0', precondOnTimeStep=self.opt['model']['linsol']['PCGOnTimeStep'], update_step=self.opt['model']['linsol']['PCGRegularUpdate'], numIterationsToRefactorize=self.opt['model']['linsol']['PCGNumIterToRefact'])

        linType = self.opt['model']['int']['lin_type']
        if linType == 'Pardiso':
            simuNode.createObject('SparsePARDISOSolver', name='lsolver', verbose='0', pardisoSchurComplement=0, analyseEachInvert=0, symmetric=self.opt['model']['linsol']['pardisoSym'], exportDataToFolder=self.opt['model']['linsol']['pardisoFolder'])
        elif linType == 'CG':
            # simuNode.createObject('ShewchukPCGLinearSolver', name='lsolverit', iterations='500', use_precond='1', tolerance='1e-10', preconditioners='lsolver')
            simuNode.createObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')
        else:
            print 'Unknown linear solver type!'

        ### mechanical object
        simuNode.createObject('MechanicalObject', src='@loader', name='Volume')
        simuNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        if 'total_mass' in self.opt['model'].keys():
            simuNode.createObject('UniformMass', totalMass=self.opt['model']['total_mass'])
        if 'density' in self.opt['model'].keys():
            simuNode.createObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.opt['model']['density'], name='mass')

        ### material and elasticiy properties
        youngModuli=self.opt['model']['fem']['young_modulus']
        poissonRatio = self.opt['model']['fem']['poisson_ratio']
        method = self.opt['model']['fem']['method']
        indices = range(1, len(youngModuli)+1)

        E=youngModuli[0]
        nu=poissonRatio
        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        materialParams='{} {}'.format(mu,lamb)
        if len(youngModuli) > 1:
            if  method[0:3] == 'Cor':
                simuNode.createObject('Indices2ValuesMapper', indices=indices, values=youngModuli, name='youngMapper', inputValues='@loader.dataset')
                simuNode.createObject('TetrahedronFEMForceField', name='FEM', method='large', listening='true', drawHeterogeneousTetra='1', poissonRatio=poissonRatio, youngModulus='@youngMapper.outputValues', updateStiffness='1')
            elif method == 'StVenant':
                simuNode.createObject('TetrahedronHyperelasticityFEMForceField', name="FEM", materialName='StVenantKirchhoff', ParameterSet=materialParams)
        else:
            if  method[0:3] == 'Cor':
                simuNode.createObject('TetrahedronFEMForceField', name='FEM', method='large', listening='true', drawHeterogeneousTetra='1', poissonRatio=poissonRatio, youngModulus=youngModuli[0], updateStiffness='1')
            elif method == 'StVenant':
                simuNode.createObject('TetrahedronHyperelasticityFEMForceField', name="FEM", materialName='StVenantKirchhoff', ParameterSet=materialParams, printLog='0')

        ### boundary conditions
        simuNode.createObject('BoxROI', box=self.opt['model']['bc']['boxes'], name='fixedBox', drawBoxes='1')
        simuNode.createObject('FixedConstraint', indices='@fixedBox.indices')

        ### external impact
        if 'applied_periodic_force' in self.opt['model'].keys():
            simuNode.createObject('BoxROI', name='forceBox', box=self.opt['model']['applied_periodic_force']['boxes'])
            self.appliedForce = simuNode.createObject('ConstantForceField', force=self.opt['model']['applied_periodic_force']['initial_force'], indices='@forceBox.indices')

        ### export data
        if self.saveGeo:
            simuNode.createObject('VTKExporter', filename=self.geoFolder+'/object.vtk', XMLformat='0', listening='1', edges="0", triangles="0", quads="0", tetras="1", exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1")

        if self.saveObs:
            simuNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1')
            simuNode.createObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName=self.obsFile, ExportPositions='1', ExportVelocities='0', ExportForces='0')

        ###  visual node
        oglNode = simuNode.createChild('oglNode')
        self.oglNode = oglNode
        oglNode.createObject('OglModel')
        oglNode.createObject('BarycentricMapping')

        return 0;



    def onBeginAnimationStep(self, deltaTime):
        ### apply external impact
        self.step += 1
        if 'applied_periodic_force' in self.opt['model'].keys():
            # maxTS = self.opt['model']['applied_force']['num_inc_steps']
            # delta = np.array(self.opt['model']['applied_force']['delta'])
            per=self.opt['model']['applied_periodic_force']['period']
            amp=self.opt['model']['applied_periodic_force']['amplitude']
            arg=2*math.pi * self.step / per
            actualForce = np.array(amp) * math.sin(arg)
            self.appliedForce.findData('force').value = actualForce.tolist()
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

    def onScriptEvent(self, senderNode, eventName, data):
        return 0;

