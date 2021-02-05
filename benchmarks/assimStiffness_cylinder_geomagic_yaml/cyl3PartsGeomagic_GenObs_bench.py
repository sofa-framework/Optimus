import Sofa
import sys
import math
import os
import csv
import yaml

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='Deformable', pluginName='SofaDeformable')
    rootNode.addObject('RequiredPlugin', name='Engine', pluginName='SofaEngine')
    rootNode.addObject('RequiredPlugin', name='GeneralEngine', pluginName='SofaGeneralEngine')
    rootNode.addObject('RequiredPlugin', name='ImplicitOdeSolver', pluginName='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='GeneralImplicitOdeSolver', pluginName='SofaGeneralImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='BoundaryCondition', pluginName='SofaBoundaryCondition')
    rootNode.addObject('RequiredPlugin', name='Loader', pluginName='SofaLoader')
    rootNode.addObject('RequiredPlugin', name='MiscForceField', pluginName='SofaMiscForceField')
    rootNode.addObject('RequiredPlugin', name='Rigid', pluginName='SofaRigid')
    rootNode.addObject('RequiredPlugin', name='SimpleFem', pluginName='SofaSimpleFem')
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
    configFileName = "cyl3PartsRestShape_scene_config_bench.yml"
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
        rootNode.addObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')

    rootNode.addObject(Cylinder3PartsGeomagicGenObs_Controller(name="Cylinder3PartsGeomagic_GenObs", node=rootNode, opt=options))
    return 0;




class Cylinder3PartsGeomagicGenObs_Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.rootNode = kwargs["node"]
        self.options = kwargs["opt"]
        self.vdamping = 5.0

        self.rootNode.findData('dt').value = self.options['general_parameters']['delta_time']
        self.rootNode.findData('gravity').value = self.options['general_parameters']['gravity']
        self.createGraph(self.rootNode)
        return None;



    def createGraph(self, rootNode):
        rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')

        ### external impact node
        dotNode = rootNode.addChild('dotNode')
        self.dotNode = dotNode
        dotNode.addObject('EulerImplicitSolver', firstOrder='false', vdamping=self.vdamping, rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'])
        dotNode.addObject('CGLinearSolver', iterations='25', tolerance='1e-5', threshold='1e-5')
        dotNode.addObject('GeoEmulator', name='GeomagicDevice', positionFilename='geomagicObservations/geomagic_x.txt', buttonFilename='geomagicObservations/listener.txt')
        dotNode.addObject('MechanicalObject', template='Rigid3d', name='GeomagicMO', position='@GeomagicDevice.positionDevice')
        dotNode.addObject('SphereCollisionModel', color='0.5 0.5 0.5 1', radius='0.014', template='Rigid3d')
        if self.options['obs_generating_parameters']['save_observations']:
            dotNode.addObject('BoxROI', name='geomagicBounds', box='-0.05 -0.05 -0.05 0.52 0.3 0.4', doUpdate='0')
            dotNode.addObject('OptimMonitor', name='toolMonitor', template='Rigid3d', showPositions='1', indices='@geomagicBounds.indices', ExportPositions='1', fileName='obs_testing/geomagic')

        mappingNode = dotNode.addChild('mappingNode')
        mappingNode.addObject('MechanicalObject', template='Vec3d', name='dot', showObject='true', position='0.0 0.0 0.0')
        mappingNode.addObject('RigidMapping', name='meshGeomagicMapping', input='@../GeomagicMO', output='@dot')
        if self.options['obs_generating_parameters']['save_observations']:
            mappingNode.addObject('BoxROI', name='dotBounds', box='-0.01 -0.03 -0.08 0.01 0.01 0.11', doUpdate='0')
            mappingNode.addObject('OptimMonitor', name='toolMonitor', template='Vec3d', showPositions='1', indices='@dotBounds.indices', ExportPositions='1', fileName=self.options['impact_parameters']['observation_file_name'])

        ### general node
        simuNode = rootNode.addChild('simuNode')
        self.simuNode = simuNode

        ### solvers
        if self.options['general_parameters']['solver_kind'] == 'Euler':
            simuNode.addObject('EulerImplicitSolver', firstOrder='false', vdamping=self.vdamping, rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'])
        elif self.options['general_parameters']['solver_kind'] == 'Symplectic':
            simuNode.addObject('VariationalSymplecticSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'], newtonError='1e-12', steps='1', verbose='0')
        elif self.options['general_parameters']['solver_kind'] == 'Newton':
            simuNode.addObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="2")
        else:
            print('Unknown solver type!')

        if self.options['general_parameters']['linear_solver_kind'] == 'Pardiso':
            simuNode.addObject('SparsePARDISOSolver', name='LDLsolver', verbose='0', symmetric='1', exportDataToFolder='')
        elif self.options['general_parameters']['linear_solver_kind'] == 'CG':
            simuNode.addObject('CGLinearSolver', iterations="20", tolerance="1e-12", threshold="1e-12")
        else:
            print('Unknown linear solver type!')

        ### object loader
        fileExtension = self.options['system_parameters']['volume_file_name']
        fileExtension = fileExtension[fileExtension.rfind('.') + 1:]
        if fileExtension == 'vtk' or fileExtension == 'vtu':
            simuNode.addObject('MeshVTKLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        elif fileExtension == 'msh':
            simuNode.addObject('MeshGmshLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        else:
            print('Unknown file type!')

        ### mechanical object
        simuNode.addObject('MechanicalObject', src='@loader', name='Volume')
        simuNode.addObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.addObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        if 'total_mass' in self.options['general_parameters'].keys():
            simuNode.addObject('UniformMass', totalMass=self.options['general_parameters']['total_mass'])
        if 'density' in self.options['general_parameters'].keys():
            simuNode.addObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.options['general_parameters']['density'], name='mass')

        ### material properties
        # nu=0.45
        # E=5000
        # lamb=(E*nu)/((1+nu)*(1-2*nu))
        # mu=E/(2+2*nu)
        # materialParams='{} {}'.format(mu,lamb)
        # simuNode.addObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        simuNode.addObject('Indices2ValuesMapper', indices='1 2 3 4 5 6 7 8 9 10', values=self.options['obs_generating_parameters']['object_young_moduli'], name='youngMapper', inputValues='@loader.dataset')
        simuNode.addObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')

        ### attachment to external object
        simuNode.addObject('BoxROI', name='impactBounds', box=self.options['impact_parameters']['bound'], doUpdate='0')
        simuNode.addObject('RestShapeSpringsForceField', name='Springs', stiffness='10000', angularStiffness='1', external_rest_shape='@../dotNode/mappingNode/dot', points='@impactBounds.indices')
        simuNode.addObject('GeoListener', template='Vec3d', geomagicButtonPressed='@../dotNode/GeomagicDevice.button1', geomagicSecondButtonPressed='@../dotNode/GeomagicDevice.button2', geomagicPosition='@../dotNode/GeomagicDevice.positionDevice', saveAttachmentData='true', filename='obs_testing/listener.txt')

        ### boundary conditions
        if 'boundary_conditions_list' in self.options['general_parameters'].keys():
            for index in range(0, len(self.options['general_parameters']['boundary_conditions_list'])):
                bcElement = self.options['general_parameters']['boundary_conditions_list'][index]
                simuNode.addObject('BoxROI', box=bcElement['boxes_coordinates'], name='boundBoxes'+str(index), drawBoxes='0', doUpdate='0')
                if bcElement['condition_type'] == 'fixed':
                    simuNode.addObject('FixedConstraint', indices='@boundBoxes'+str(index)+'.indices')
                elif bcElement['condition_type'] == 'elastic':
                    simuNode.addObject('RestShapeSpringsForceField', stiffness=bcElement['spring_stiffness_values'], angularStiffness="1", points='@boundBoxes'+str(index)+'.indices')
                else:
                    print('Unknown type of boundary conditions')

        ### saving generated observations
        if self.options['obs_generating_parameters']['save_observations']:
            obsNode = simuNode.addChild('obsNode')
            obsNode.addObject('MeshVTKLoader', name='obsloader', filename=self.options['system_parameters']['observation_points_file_name'])
            obsNode.addObject('MechanicalObject', name='observations', position='@obsloader.position')
            obsNode.addObject('BarycentricMapping')
            obsNode.addObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1', doUpdate='0')
            obsNode.addObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName=self.options['system_parameters']['observation_file_name'], ExportPositions='1', ExportVelocities='0', ExportForces='0')
            obsNode.addObject('ShowSpheres', name="obsVisu", radius="0.002", color="1 0 0 1", position='@observations.position')

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

    def onAnimateEndEvent(self, deltaTime):
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

    def onAnimateBeginEvent(self, deltaTime):
        return 0;

