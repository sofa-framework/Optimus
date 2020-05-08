import Sofa
import math
import os
import sys
import csv
import yaml

__file = __file__.replace('\\', '/') # windows


def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='Optimus', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', name='Python', pluginName='SofaPython')
    rootNode.createObject('RequiredPlugin', name='Exporter', pluginName='SofaExporter')

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
    print "Command line arguments for python : " + str(commandLineArguments)

    with open(configFileName, 'r') as stream:
        try:
            options = yaml.load(stream)

        except yaml.YAMLError as exc:
            print(exc)
            return

    if options['general_parameters']['linear_solver_kind'] == 'Pardiso':
        rootNode.createObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')

    liver_constForce_GenObs(rootNode, options)
    return 0



class liver_constForce_GenObs (Sofa.PythonScriptController):

    def __init__(self, rootNode, options):
        self.options = options
        self.generalFolderName = self.options['filtering_parameters']['common_directory_prefix'] + self.options['general_parameters']['solver_kind']
        if not os.path.isdir(self.generalFolderName):
            os.mkdir(self.generalFolderName)
        if not os.path.isdir(self.generalFolderName + '/observations'):
            os.mkdir(self.generalFolderName + '/observations')

        rootNode.findData('dt').value = options['general_parameters']['delta_time']
        rootNode.findData('gravity').value = options['general_parameters']['gravity']
        self.createGraph(rootNode)
        return None


    def createGraph(self,rootNode):

        self.iterations = 0
        self.timeIterations = 0
        self.amplitude = self.options['force_parameters']['force_amplitude']
        self.period = self.options['force_parameters']['force_period']

        # rootNode
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')
        #rootNode.createObject('DefaultAnimationLoop')
        #rootNode.createObject('CollisionPipeline', verbose="0", draw="0")
        #rootNode.createObject('BruteForceDetection', name="N2")
        #rootNode.createObject('NewProximityIntersection', name="Proximity", alarmDistance="0.05", contactDistance="0.025")
        #rootNode.createObject('CollisionResponse', name="Response", response="default")

        # general node
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode

        # solvers
        if self.options['general_parameters']['solver_kind'] == 'Euler':
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'])
        elif self.options['general_parameters']['solver_kind'] == 'Symplectic':
            simuNode.createObject('VariationalSymplecticSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'], newtonError='1e-12', steps='1', verbose='0')
        elif self.options['general_parameters']['solver_kind'] == 'Newton':
            simuNode.createObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="2")
        else:
            print 'Unknown solver type!'

        if self.options['general_parameters']['linear_solver_kind'] == 'Pardiso':
            simuNode.createObject('SparsePARDISOSolver', name='LDLsolver', verbose='0', symmetric='2', exportDataToFolder='')
        elif self.options['general_parameters']['linear_solver_kind'] == 'CG':
            simuNode.createObject('CGLinearSolver', iterations="20", tolerance="1e-12", threshold="1e-12")
        else:
            print 'Unknown linear solver type!'

        # mechanical object
        fileExtension = self.options['system_parameters']['volume_file_name']
        fileExtension = fileExtension[fileExtension.rfind('.') + 1:]
        if fileExtension == 'vtk' or fileExtension == 'vtu':
            simuNode.createObject('MeshVTKLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        elif fileExtension == 'msh':
            simuNode.createObject('MeshGmshLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        else:
            print 'Unknown file type!'

        simuNode.createObject('MechanicalObject', src='@loader', name='Volume')
        simuNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        if 'total_mass' in self.options['general_parameters'].keys():
            simuNode.createObject('UniformMass', totalMass=self.options['general_parameters']['total_mass'])
        if 'density' in self.options['general_parameters'].keys():
            simuNode.createObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.options['general_parameters']['density'], name='mass')

        simuNode.createObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method='large', youngModulus='5000', poissonRatio='0.45')

        # boundary conditions
        if 'boundary_conditions_list' in self.options['general_parameters'].keys():
            for index in range(0, len(self.options['general_parameters']['boundary_conditions_list'])):
                bcElement = self.options['general_parameters']['boundary_conditions_list'][index]
                simuNode.createObject('BoxROI', box=bcElement['boxes_coordinates'], name='boundBoxes'+str(index), drawBoxes='0', doUpdate='0')
                if bcElement['condition_type'] == 'fixed':
                    simuNode.createObject('FixedConstraint', indices=bcElement['fixed_indices'])
                elif bcElement['condition_type'] == 'elastic':
                    simuNode.createObject('RestShapeSpringsForceField', stiffness=bcElement['spring_stiffness_values'], angularStiffness="1", points='@boundBoxes'+str(index)+'.indices')
                else:
                    print 'Unknown type of boundary conditions'

        # add constant force field
        self.constantForce = simuNode.createObject('ConstantForceField', name='appliedForce', indices=self.options['impact_parameters']['applied_indices'], totalForce='0.0 0.0 0.0')

        # saving generated observations
        if self.options['obs_generating_parameters']['save_observations']:
            obsNode = simuNode.createChild('obsNode')        
            obsNode.createObject('MeshVTKLoader', name='obsLoader', filename=self.options['system_parameters']['observation_points_file_name'])
            obsNode.createObject('MechanicalObject', name='SourceMO', src="@obsLoader")
            obsNode.createObject('ShowSpheres', position='@SourceMO.position', color='1.0 1.0 0.0 1', radius="0.0034", showIndicesScale='0.0') 
            obsNode.createObject('BarycentricMapping')
            obsNode.createObject('BoxROI', name='observationNodeBox', box='-1 -1 -1 1 1 1', doUpdate='1')
            obsNode.createObject('OptimMonitor', name='ObservationMonitor', indices='@observationNodeBox.indices', fileName = self.generalFolderName + '/observations/node', ExportPositions='1', ExportVelocities='0', ExportForces='0')

        return 0


    def onEndAnimationStep(self, deltaTime):
        self.iterations = self.iterations + 1
        self.timeIterations = self.timeIterations + 1
        # changing force
        if self.options['impact_parameters']['impact_type'] == 'Periodic':
            currentValue = self.options['force_parameters']['force_value']
            currentValue = [elem * self.amplitude * 0.5 * (1 - math.cos(math.pi * self.iterations / self.period)) for elem in currentValue]
            self.constantForce.findData('totalForce').value = currentValue
        return 0


    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        return 0;

    def onKeyReleased(self, c):
        return 0;

    def initGraph(self, node):
        return 0;

    def onKeyPressed(self, c):
        return 0;

    def onMouseWheel(self, mouseX,mouseY,wheelDelta):
        return 0;

    def storeResetState(self):
        return 0;

    def cleanup(self):
        return 0;

    def onGUIEvent(self, strControlID,valueName,strValue):
        return 0;

    def onLoaded(self, node):
        return 0;

    def reset(self):
        return 0;

    def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
        return 0;

    def bwdInitGraph(self, node):
        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        return 0;

    def onMouseButtonRight(self, mouseX,mouseY,isPressed):
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        return 0;

