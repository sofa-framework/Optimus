import Sofa
import math
import os
import sys
import csv
import yaml

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='SofaGeneralImplicitOdeSolver')
    rootNode.createObject('RequiredPlugin', name='SofaGeneralEngine')
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
    print "Command line arguments for python : " + str(commandLineArguments)

    with open(configFileName, 'r') as stream:
        try:
            options = yaml.safe_load(stream)

        except yaml.YAMLError as exc:
            print(exc)
            return

    if options['general_parameters']['linear_solver_kind'] == 'Pardiso':
        rootNode.createObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')

    cyl3PartsGeomagic_GenObs(rootNode, options)
    return 0;




class cyl3PartsGeomagic_GenObs(Sofa.PythonScriptController):

    def __init__(self, rootNode, options):
        self.options = options
        self.vdamping = 5.0

        rootNode.findData('dt').value = options['general_parameters']['delta_time']
        rootNode.findData('gravity').value = options['general_parameters']['gravity']
        self.createGraph(rootNode)
        return None;



    def createGraph(self, rootNode):
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')

        ### external impact node
        dotNode = rootNode.createChild('dotNode')
        self.dotNode = dotNode
        dotNode.createObject('EulerImplicitSolver', firstOrder='false', vdamping=self.vdamping, rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'])
        dotNode.createObject('CGLinearSolver', iterations='25', tolerance='1e-5', threshold='1e-5')
        dotNode.createObject('GeoEmulator', name='GeomagicDevice', positionFilename='geomagicObservations/geomagic_x.txt', buttonFilename='geomagicObservations/listener.txt')
        dotNode.createObject('MechanicalObject', template='Rigid3d', name='GeomagicMO', position='@GeomagicDevice.positionDevice')
        dotNode.createObject('SphereCollisionModel', color='0.5 0.5 0.5 1', radius='0.014', template='Rigid3d')
        if self.options['obs_generating_parameters']['save_observations']:
            dotNode.createObject('BoxROI', name='geomagicBounds', box='-0.05 -0.05 -0.05 0.52 0.3 0.4', doUpdate='0')
            dotNode.createObject('OptimMonitor', name='toolMonitor', template='Rigid3d', showPositions='1', indices='@geomagicBounds.indices', ExportPositions='1', fileName='obs_testing/geomagic')

        mappingNode = dotNode.createChild('mappingNode')
        mappingNode.createObject('MechanicalObject', template='Vec3d', name='dot', showObject='true', position='0.0 0.0 0.0')
        mappingNode.createObject('RigidMapping', name='meshGeomagicMapping', input='@../GeomagicMO', output='@dot')
        if self.options['obs_generating_parameters']['save_observations']:
            mappingNode.createObject('BoxROI', name='dotBounds', box='-0.01 -0.03 -0.08 0.01 0.01 0.11', doUpdate='0')
            mappingNode.createObject('OptimMonitor', name='toolMonitor', template='Vec3d', showPositions='1', indices='@dotBounds.indices', ExportPositions='1', fileName=self.options['impact_parameters']['observation_file_name'])

        ### general node
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode

        ### solvers
        if self.options['general_parameters']['solver_kind'] == 'Euler':
            simuNode.createObject('EulerImplicitSolver', firstOrder='false', vdamping=self.vdamping, rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'])
        elif self.options['general_parameters']['solver_kind'] == 'Symplectic':
            simuNode.createObject('VariationalSymplecticSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'], newtonError='1e-12', steps='1', verbose='0')
        elif self.options['general_parameters']['solver_kind'] == 'Newton':
            simuNode.createObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="2")
        else:
            print 'Unknown solver type!'

        if self.options['general_parameters']['linear_solver_kind'] == 'Pardiso':
            simuNode.createObject('SparsePARDISOSolver', name='LDLsolver', verbose='0', symmetric='1', exportDataToFolder='')
        elif self.options['general_parameters']['linear_solver_kind'] == 'CG':
            simuNode.createObject('CGLinearSolver', iterations="20", tolerance="1e-12", threshold="1e-12")
        else:
            print 'Unknown linear solver type!'

        ### object loader
        fileExtension = self.options['system_parameters']['volume_file_name']
        fileExtension = fileExtension[fileExtension.rfind('.') + 1:]
        if fileExtension == 'vtk' or fileExtension == 'vtu':
            simuNode.createObject('MeshVTKLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        elif fileExtension == 'msh':
            simuNode.createObject('MeshGmshLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        else:
            print 'Unknown file type!'

        ### mechanical object
        simuNode.createObject('MechanicalObject', src='@loader', name='Volume')
        simuNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        if 'total_mass' in self.options['general_parameters'].keys():
            simuNode.createObject('UniformMass', totalMass=self.options['general_parameters']['total_mass'])
        if 'density' in self.options['general_parameters'].keys():
            simuNode.createObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.options['general_parameters']['density'], name='mass')

        ### material properties
        # nu=0.45
        # E=5000
        # lamb=(E*nu)/((1+nu)*(1-2*nu))
        # mu=E/(2+2*nu)
        # materialParams='{} {}'.format(mu,lamb)
        # simuNode.createObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        simuNode.createObject('Indices2ValuesMapper', indices='1 2 3 4 5 6 7 8 9 10', values=self.options['obs_generating_parameters']['object_young_moduli'], name='youngMapper', inputValues='@loader.dataset')
        simuNode.createObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')

        ### attachment to external object
        simuNode.createObject('BoxROI', name='impactBounds', box=self.options['impact_parameters']['bound'], doUpdate='0')
        simuNode.createObject('RestShapeSpringsForceField', name='Springs', stiffness='10000', angularStiffness='1', external_rest_shape='@../dotNode/mappingNode/dot', points='@impactBounds.indices')
        simuNode.createObject('GeoListener', template='Vec3d', geomagicButtonPressed='@../dotNode/GeomagicDevice.button1', geomagicSecondButtonPressed='@../dotNode/GeomagicDevice.button2', geomagicPosition='@../dotNode/GeomagicDevice.positionDevice', saveAttachmentData='true', filename='obs_testing/listener.txt')

        ### boundary conditions
        if 'boundary_conditions_list' in self.options['general_parameters'].keys():
            for index in range(0, len(self.options['general_parameters']['boundary_conditions_list'])):
                bcElement = self.options['general_parameters']['boundary_conditions_list'][index]
                print bcElement
                simuNode.createObject('BoxROI', box=bcElement['boxes_coordinates'], name='boundBoxes'+str(index), drawBoxes='0', doUpdate='0')
                if bcElement['condition_type'] == 'fixed':
                    simuNode.createObject('FixedConstraint', indices='@boundBoxes'+str(index)+'.indices')
                elif bcElement['condition_type'] == 'elastic':
                    simuNode.createObject('RestShapeSpringsForceField', stiffness=bcElement['spring_stiffness_values'], angularStiffness="1", points='@boundBoxes'+str(index)+'.indices')
                else:
                    print 'Unknown type of boundary conditions'

        ### saving generated observations
        if self.options['obs_generating_parameters']['save_observations']:
            simuNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1', doUpdate='0')
            simuNode.createObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName=self.options['system_parameters']['observation_file_name'], ExportPositions='1', ExportVelocities='0', ExportForces='0')

        return 0;



    def onMouseButtonLeft(self, mouseX, mouseY, isPressed):
        return 0;

    def onKeyReleased(self, c):
        return 0;

    def initGraph(self, node):
        return 0;

    def onKeyPressed(self, c):
        return 0;

    def onMouseWheel(self, mouseX, mouseY, wheelDelta):
        return 0;

    def storeResetState(self):
        return 0;

    def cleanup(self):
        return 0;

    def onGUIEvent(self, strControlID, valueName, strValue):
        return 0;

    def onEndAnimationStep(self, deltaTime):
        return 0;

    def onLoaded(self, node):
        return 0;

    def reset(self):
        return 0;

    def onMouseButtonMiddle(self, mouseX, mouseY, isPressed):
        return 0;

    def bwdInitGraph(self, node):
        return 0;

    def onScriptEvent(self, senderNode, eventName, data):
        return 0;

    def onMouseButtonRight(self, mouseX, mouseY, isPressed):
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        return 0;

