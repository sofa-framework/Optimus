import Sofa
import math
import os
import sys
import csv
import yaml
import pprint

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
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

        self.saveObs = opt['io']['saveObs']
        self.saveGeo = opt["io"]["saveGeo"]

        if self.saveObs or self.saveGeo:
            prefix = opt['io']['prefix']
            suffix = opt['io']['suffix']

            self.mainFolder = prefix + opt['model']['int']['type'] + str(opt['model']['int']['maxit']) + suffix

            os.system('mv '+self.mainFolder+' '+self.mainFolder+'_arch')
            os.system('mkdir '+self.mainFolder)

        if self.saveObs:
            self.obsFile = self.mainFolder + '/' + opt['io']['obsFileName']

        if self.saveGeo:
            self.geoFolder = self.mainFolder + '/' + opt['io']['obsFileName']+'VTK'
            os.system('mkdir -p '+self.geoFolder)

        ### prepare control
        self.actualPressure = [0, 0, 0]
        self.numStep = 0

        self.createGraph(rootNode)

        return None;



    def createGraph(self, rootNode):
        rootNode.findData('dt').value = self.opt['model']['dt']
        rootNode.findData('gravity').value = self.opt['model']['gravity']

        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')

        ### general node
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode
        simuNode.createObject('MeshVTKLoader', name='loader', filename=self.opt['model']['vol_mesh'])

        ### solvers
        mopt = self.opt['model']
        intType = self.opt['model']['int']['type']
        linType = self.opt['model']['int']['lin_type']
        intMaxit = self.opt['model']['int']['maxit']
        rmass = self.opt['model']['int']['rmass']
        rstiff = self.opt['model']['int']['rstiff']

        if intType == 'Euler':
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness=rstiff, rayleighMass=rmass)
        elif intType == 'Newton':
            simuNode.createObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="10")

        if linType == 'Pardiso':
            simuNode.createObject('SparsePARDISOSolver', name='lsolver', verbose='0', symmetric=self.opt['model']['linsol']['pardisoSym'], exportDataToFolder=self.opt['model']['linsol']['pardisoFolder'])
        elif linType == 'CG':
            simuNode.createObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')
            if self.opt['model']['linsol']['usePCG']:
                simuNode.createObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='1', use_precond='1', verbose='0', preconditioners='lsolver', listening='1', tolerance=mopt['linsol']['PCGTolerance'], update_step=mopt['linsol']['PCGUpdateSteps'], iterations=mopt['linsol']['PCGTolerance'])

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

        ### material properties
        youngModuli=self.opt['model']['young_moduli']
        indices = range(1, len(youngModuli)+1)
        simuNode.createObject('Indices2ValuesMapper', indices=indices, values=youngModuli, name='youngMapper', inputValues='@loader.dataset')
        simuNode.createObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method=self.opt['model']['fem_method'], poissonRatio='0.45', youngModulus='@youngMapper.outputValues')
        # E=4000
        # nu=0.45
        # lamb=(E*nu)/((1+nu)*(1-2*nu))
        # mu=E/(2+2*nu)
        # materialParams='{} {}'.format(mu,lamb)
        # simuNode.createObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)

        ### boundary conditions
        simuNode.createObject('BoxROI', box=self.opt['model']['bc']['boxes'], name='fixedBox', drawBoxes='1')
        simuNode.createObject('FixedConstraint', indices='@fixedBox.indices')

        ### control impact
        controlType = self.opt['model']['control']['type']
        if controlType == 'pressure':
            surfNode=simuNode.createChild('surfaceNode')
            surfNode.createObject('MeshSTLLoader', name='loader', filename=self.opt['model']['surf_mesh'])
            surfNode.createObject('MechanicalObject', src='@loader', name='surface')
            surfNode.createObject('TriangleSetTopologyContainer', name="Container", src="@loader", tags=" ")
            surfNode.createObject('TriangleSetTopologyModifier', name="Modifier")
            surfNode.createObject('TriangleSetTopologyAlgorithms', name="TopoAlgo")
            surfNode.createObject('TriangleSetGeometryAlgorithms', name="GeomAlgo")
            self.pressureField = surfNode.createObject('TrianglePressureForceField', name='forceField', normal='0 0 1', showForces='1', dmin=0.299, dmax=0.301)
            surfNode.createObject('BarycentricMapping', name='mapping')

        ### saving generated observations
        if self.saveGeo:
            simuNode.createObject('VTKExporter', filename=self.geoFolder+'/object.vtk', XMLformat='0',listening='1',edges="0",triangles="0",quads="0",tetras="1", exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1", cellsDataFields='FEM.youngModulus')

        if self.saveObs:
            simuNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1')
            simuNode.createObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName=self.obsFile, ExportPositions='1', ExportVelocities='0', ExportForces='0')

        ### visual model
        oglNode = simuNode.createChild('oglNode')
        self.oglNode = oglNode
        oglNode.createObject('OglModel')
        oglNode.createObject('BarycentricMapping')

        return 0;



    def onBeginAnimationStep(self, deltaTime):
    	self.numStep += 1

    	if self.numStep <= self.opt['model']['control']['nsteps']:
    		self.actualPressure = map(sum, zip(self.actualPressure, self.opt['model']['control']['deltaPressure']))
        self.pressureField.pressure = self.actualPressure

    	print 'Setting actual pressure: ', self.actualPressure

        return 0;



    def initGraph(self, node):
        return 0;

    def storeResetState(self):
        return 0;

    def cleanup(self):
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

