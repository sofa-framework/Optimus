import Sofa
import math
import os
import sys
import csv
import yaml
import pprint

__file = __file__.replace('\\', '/') # windows

def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='Optimus', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')
    rootNode.createObject('RequiredPlugin', name='IMAUX', pluginName='ImageMeshAux')
    # rootNode.createObject('RequiredPlugin', name='MJED', pluginName='SofaMJEDFEM')
    
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
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
    

    AppliedForces_GenObs(rootNode, options)

    return 0;

class AppliedForces_GenObs (Sofa.PythonScriptController):

    def __init__(self, rootNode, opt):
        self.opt = opt

        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(opt)
                        
        self.saveObs = opt['io']['saveObs']
        self.saveGeo = opt["io"]["saveGeo"]
        self.planeCollision = opt['model']['plane_collision']
                        
        if self.saveObs or self.saveGeo:
            prefix = opt['io']['prefix']
            suffix = opt['io']['suffix']

            if self.planeCollision:
                prefix = prefix + 'plane_'

            self.mainFolder = prefix + opt['model']['int']['type'] + str(opt['model']['int']['maxit']) + suffix

            os.system('mv '+self.mainFolder+' '+self.mainFolder+'_arch')
            os.system('mkdir '+self.mainFolder)

        if self.saveObs:
            self.obsFile = self.mainFolder + '/' + opt['io']['obsFileName']

        if self.saveGeo:
            self.geoFolder = self.mainFolder + '/' + opt['io']['obsFileName']+'VTK'
            os.system('mkdir -p '+self.geoFolder)

        self.createGraph(rootNode)

        return None;

    def createGraph(self,rootNode):        
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

        # rootNode/simuNode
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode
        simuNode.createObject('MeshVTKLoader', name='loader', filename=self.opt['model']['vol_mesh'])

        intType = self.opt['model']['int']['type']
        intMaxit = self.opt['model']['int']['maxit']
        rmass = self.opt['model']['int']['rmass']
        rstiff = self.opt['model']['int']['rstiff']

        if intType == 'Euler':
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness=rstiff, rayleighMass=rmass)
        # simuNode.createObject('NewtonStaticSolver', maxIt='50', correctionTolerance='1e-8', residualTolerance='1e-8', convergeOnResidual='1')
        # simuNode.createObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='0', use_precond='1', tolerance='1e-10', iterations='500',
        #  verbose='0', update_step='10', listening='1', preconditioners='lsolver')
        # simuNode.createObject('ShewchukPCGLinearSolver', name='lsolverit', iterations='500', use_precond='1', tolerance='1e-10', preconditioners='lsolver')
        # simuNode.createObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')
        
        simuNode.createObject('SparsePARDISOSolver', name='lsolver', verbose='0', pardisoSchurComplement=self.planeCollision, 
            symmetric=self.opt['model']['linsol']['pardisoSym'], exportDataToFolder=self.opt['model']['linsol']['pardisoFolder'])
        simuNode.createObject('MechanicalObject', src='@loader', name='Volume')        
        simuNode.createObject('BoxROI', box=self.opt['model']['bc']['boxes'], name='fixedBox', drawBoxes='1')
        simuNode.createObject('FixedConstraint', indices='@fixedBox.indices')        
        simuNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")

        if 'total_mass' in self.opt['model'].keys():
            simuNode.createObject('UniformMass', totalMass=self.opt['model']['total_mass'])

        if 'density' in self.opt['model'].keys():
            simuNode.createObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.opt['model']['density'], name='mass')

        youngModuli=self.opt['model']['young_moduli']
        indices = range(1, len(youngModuli)+1)
        simuNode.createObject('Indices2ValuesMapper', indices=indices, values=youngModuli, 
            name='youngMapper', inputValues='@loader.dataset')

        simuNode.createObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')

        if self.saveGeo:
            simuNode.createObject('VTKExporter', filename=self.geoFolder+'/object.vtk', XMLformat='0',listening='1',edges="0",triangles="0",quads="0",tetras="1",
                exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1", cellsDataFields='FEM.youngModulus')


        if self.saveObs:
            simuNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1')
            simuNode.createObject('Monitor', name='ObservationMonitor', indices='@observationBox.indices', fileName=self.obsFile, ExportPositions='1', ExportVelocities='0', ExportForces='0')


        if self.planeCollision:
            simuNode.createObject('PardisoConstraintCorrection', solverName='lsolver', schurSolverName='lsolver')
            # simuNode.createObject('LinearSolverConstraintCorrection')


        if self.planeCollision:
            surface=simuNode.createChild('collision')
            surface.createObject('MeshSTLLoader', name='sloader', filename=self.opt['model']['surf_mesh'])
            surface.createObject('TriangleSetTopologyContainer', position='@sloader.position', name='TriangleContainer', triangles='@sloader.triangles')
            surface.createObject('TriangleSetTopologyModifier', name='Modifier')
            surface.createObject('MechanicalObject', showIndices='false', name='mstate')
            surface.createObject('Triangle', color='1 0 0 1', group=0)
            surface.createObject('Line', color='1 0 0 1', group=0)
            surface.createObject('Point', color='1 0 0 1', group=0)
            surface.createObject('BarycentricMapping', name='bpmapping')

        # rootNode/simuNode/oglNode
        oglNode = simuNode.createChild('oglNode')
        self.oglNode = oglNode
        oglNode.createObject('OglModel')
        oglNode.createObject('BarycentricMapping')        

        if self.planeCollision:
            floor = simuNode.createChild('floor')
            floor.createObject('RegularGrid', nx="2", ny="2", nz="2", xmin="-0.1", xmax="0.1",  ymin="-0.059", ymax="-0.061", zmin="0.0", zmax="0.3")
            floor.createObject('MechanicalObject', template="Vec3d")
            floor.createObject('Triangle',simulated="false", bothSide="true", contactFriction="0.00", color="1 0 0 1")
            floor.createObject('Line', simulated="false", bothSide="true", contactFriction="0.0")
            floor.createObject('Point', simulated="false", bothSide="true", contactFriction="0.0")

        return 0;
    

    def initGraph(self, node):
        
        return 0;

    def storeResetState(self):
        
        return 0;

    def cleanup(self):
        if self.saveObs or self.saveGeo:
            print 'Observations saved to '+self.mainFolder         
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


