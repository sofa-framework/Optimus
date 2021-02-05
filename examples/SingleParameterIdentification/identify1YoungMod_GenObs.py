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
    #     configFileName = commandLineArguments[1]
    configFileName = "beamApplyForce.yml"
    # else:
    #     print 'ERROR: Must supply a yaml config file as an argument!'
    #     return

    with open(configFileName, 'r') as stream:
        try:
            options = yaml.safe_load(stream)

        except yaml.YAMLError as exc:
            print(exc)
            return

    if options['model']['int']['lin_type'] == 'Pardiso':
        rootNode.addObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')

    rootNode.addObject(AppStiffGenObs_Controller(name="AppStiff_GenObs", node=rootNode, opt=options))

    return 0;




class AppStiffGenObs_Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        ### These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rootNode = kwargs["node"]
        self.opt = kwargs["opt"]

        ### print options
        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(self.opt)

        ### set configuration
        self.folder = self.opt['io']['folder']

        ### create folder to save the observations; back-up if the folder exists already
        stamp='_'+str(int(time.time()))
        os.system('mkdir -p arch')
        os.system('mv --backup -S '+stamp+' '+self.folder+' arch')
        os.system('mkdir -p '+self.folder)

        ### create folder to save the VTK geometry
        if self.opt['io']['saveGeo']:
            self.geoFolder = self.folder+'/obsGeometry'
            os.system('mkdir -p '+self.geoFolder)

        self.createGraph(self.rootNode)

        return None;




    def createGraph(self, rootNode):
        self.step = 0
        self.incStep = 0
        rootNode.findData('dt').value = self.opt['model']['dt']
        rootNode.findData('gravity').value = self.opt['model']['gravity']

        rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')

        ### general node
        simuNode = rootNode.addChild('simuNode')

        ### integration in time
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
        simuNode.addObject('MeshVTKLoader', name='loader', filename=self.opt['model']['volumeMesh'])
        simuNode.addObject('MechanicalObject', src='@loader', name='Volume')
        simuNode.addObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.addObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")

        simuNode.addObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.opt['model']['density'], name='mass')

        ### material and elasticiy properties
        method = self.opt['model']['fem']['method']
        E = self.opt['model']['fem']['young_moduli']
        nu = self.opt['model']['fem']['poisson_ratio']
        if  method[0:3] == 'Cor':
            simuNode.addObject('TetrahedronFEMForceField', name='FEM', method=method[3:].lower(), listening='true', drawHeterogeneousTetra='1', poissonRatio=nu, youngModulus=E, updateStiffness='1')
        elif method == 'StVenant':
            simuNode.addObject('Indices2ValuesTransformer', name='pm', indices=[1], values1=E, values2=nu, inputValues='@loader.dataset', transformation='ENu2MuLambda')
            simuNode.addObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet='@pm.outputValues')

        ### boundary conditions
        simuNode.addObject('BoxROI', box=self.opt['model']['bc']['boxes'], name='fixedBox', drawBoxes='1')
        simuNode.addObject('FixedConstraint', indices='@fixedBox.indices')

        ### applied force
        simuNode.addObject('BoxROI', name='forceBox', box=self.opt['model']['applied_force']['boxes'], drawBoxes='1')
        self.appliedForce = simuNode.addObject('ConstantForceField', force=self.opt['model']['applied_force']['initial_force'], indices='@forceBox.indices')

        ### export geometry in VTK files in each step
        if self.opt['io']['saveGeo']:
            simuNode.addObject('VTKExporter', filename=self.geoFolder+'/object.vtk', XMLformat='0', listening='1', edges="0", triangles="0", quads="0", tetras="1", exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1", printLog='0')

        ### export observations
        obsNode = simuNode.addChild('obsNode')
        obsNode.addObject('MeshVTKLoader', name='obsloader', filename=self.opt['model']['observationPoints'])
        obsNode.addObject('MechanicalObject', src='@obsloader', name='MO')
        obsNode.addObject('BarycentricMapping')
        obsNode.addObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1')
        obsNode.addObject('ShowSpheres', radius="0.002", color="1 0 0 1", position='@MO.position')
        if self.opt['io']['saveObs']:
            obsFile = self.folder + '/' + self.opt['io']['obsFile']
            obsNode.addObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', ExportPositions='1', fileName=obsFile, ExportVelocities='0', ExportForces='0')

        ### visual node
        oglNode = simuNode.addChild('oglNode')
        oglNode.addObject('MeshSTLLoader', name='objectSLoader', filename=self.opt['model']['surfaceMesh'])
        oglNode.addObject('OglModel', color='0 0 1 1')
        oglNode.addObject('BarycentricMapping')

        return 0;



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
        return 0;

