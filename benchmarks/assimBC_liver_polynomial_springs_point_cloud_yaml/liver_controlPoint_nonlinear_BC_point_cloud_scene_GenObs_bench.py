import Sofa
import math
import os
import sys
import csv
import yaml

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='Engine', pluginName='SofaEngine')
    rootNode.createObject('RequiredPlugin', name='ImplicitOdeSolver', pluginName='SofaImplicitOdeSolver')
    rootNode.createObject('RequiredPlugin', name='Loader', pluginName='SofaLoader')
    rootNode.createObject('RequiredPlugin', name='MiscForceField', pluginName='SofaMiscForceField')
    rootNode.createObject('RequiredPlugin', name='SimpleFem', pluginName='SofaSimpleFem')
    rootNode.createObject('RequiredPlugin', name='Deformable', pluginName='SofaDeformable')
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

    liver_controlPoint_GenObs(rootNode, options)
    return 0




class liver_controlPoint_GenObs(Sofa.PythonScriptController):

    def __init__(self, rootNode, options):
        self.options = options
        if not os.path.isdir('obs_testing'):
            os.mkdir('obs_testing')
        self.scenario_type = 'Generate_data'
        if 'scenario_type' in self.options['general_parameters'].keys() and self.options['general_parameters']['scenario_type'] == 'Validate_estimations':
            self.scenario_type = 'Validate_estimations'

        rootNode.findData('dt').value = options['general_parameters']['delta_time']
        rootNode.findData('gravity').value = options['general_parameters']['gravity']
        self.createGraph(rootNode)

        return None



    def createGraph(self, rootNode):
        ### rootNode
        rootNode.createObject('VisualStyle', displayFlags='hideVisualModels showBehaviorModels hideCollisionModels hideMappings showForceFields showInteractionForceFields')

        ### fixed nodes
        fixNode = rootNode.createChild('fixNode')
        fixNode.createObject('MechanicalObject', template='Vec3d', name='fixElements', showObject='true', position=self.options['boundary_parameters']['external_points'])
        fixNode.createObject('ShowSpheres', position='@fixElements.position', color='1.0 0.0 1.0 1', radius="0.01", showIndicesScale='0.0')

        ### external impact node
        dotNode = rootNode.createChild('dotNode')
        self.impactPoint = dotNode.createObject('MechanicalObject', template='Vec3d', name='dot', showObject='true', position=self.options['impact_parameters']['init_position'])
        dotNode.createObject('ShowSpheres', position='@dot.position', color='1.0 0.0 1.0 1', radius="0.01", showIndicesScale='0.0')
        if self.options['obs_generating_parameters']['save_observations'] and self.scenario_type == 'Generate_data':
            dotNode.createObject('BoxROI', name='dotBounds', box='0.14 0.15 0.37 0.18 0.17 0.4', doUpdate='0')
            dotNode.createObject('OptimMonitor', name='toolMonitor', template='Vec3d', showPositions='1', indices='@dotBounds.indices', ExportPositions='1', fileName = self.options['impact_parameters']['observation_file_name'])
        self.index = 0

        ### general node
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode

        ### solvers
        if self.options['general_parameters']['solver_kind'] == 'Euler':
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'])
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
        simuNode.createObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method='large', youngModulus=self.options['object_parameters']['young_modulus'], poissonRatio=self.options['object_parameters']['poisson_ratio'])

        ### boundary conditions
        if 'boundary_conditions_list' in self.options['boundary_parameters'].keys():
            for index in range(0, len(self.options['boundary_parameters']['boundary_conditions_list'])):
                bcElement = self.options['boundary_parameters']['boundary_conditions_list'][index]
                simuNode.createObject('BoxROI', box=bcElement['boxes_coordinates'], name='boundBoxes'+str(index), drawBoxes='0', doUpdate='0')
                if bcElement['condition_type'] == 'fixed':
                    simuNode.createObject('FixedConstraint', indices='@boundBoxes'+str(index)+'.indices')
                elif bcElement['condition_type'] == 'elastic':
                    if self.options['boundary_parameters']['spring_type'] == 'PolynomialRestshape':
                        simuNode.createObject('PolynomialRestShapeSpringsForceField', listening="1", printLog="0", points='@boundBoxes'+str(index)+'.indices', initialLength=self.options['boundary_parameters']['initial_length'], polynomialDegree=bcElement['polynomial_degrees'], polynomialStiffness=bcElement['polynomial_stiffness_values'], drawMode='1')
                    elif self.options['boundary_parameters']['spring_type'] == 'Polynomial':
                        simuNode.createObject('PolynomialSpringsForceField', listening="1", printLog="0", object1='@.', firstObjectPoints='@boundBoxes'+str(index)+'.indices', object2='@../fixNode/fixElements', secondObjectPoints=self.options['boundary_parameters']['external_indices'], polynomialDegree=bcElement['polynomial_degrees'], polynomialStiffness=bcElement['polynomial_stiffness_values'], drawMode='1')
                    else:
                        simuNode.createObject('RestShapeSpringForceField', stiffness=bcElement['spring_stiffness_values'], angularStiffness="1", points='@boundBoxes'+str(index)+'.indices')
                else:
                    print 'Unknown type of boundary conditions'

        ### attachment to external object
        simuNode.createObject('BoxROI', name='impactBounds', box=self.options['object_parameters']['attachment_coordinates'], doUpdate='0')
        simuNode.createObject('RestShapeSpringsForceField', name='Springs', stiffness='10000', angularStiffness='1', external_rest_shape='@../dotNode/dot', points='@impactBounds.indices')

        ### saving generated observations
        if 'use_point_cloud' in self.options['obs_generating_parameters'] and self.options['obs_generating_parameters']['save_observations'] and self.scenario_type == 'Generate_data':
            pointCloudNode = simuNode.createChild('pointCloud')
            pointCloudNode.createObject('MeshObjLoader', name="pcloader", filename=self.options['point_cloud_parameters']['mesh_file'])
            pointCloudNode.createObject('MechanicalObject', src='@pcloader', name='PointCloud')
            pointCloudNode.createObject('BarycentricMapping', name='VisualMapping', input='@../Volume', output='@PointCloud')
            pointCloudNode.createObject('ShowSpheres', position='@PointCloud.position', color='0.0 0.5 0.0 1', radius="0.0014", showIndicesScale='0.0')

        if self.options['obs_generating_parameters']['save_observations'] and self.scenario_type == 'Generate_data':
            if 'use_point_cloud' in self.options['obs_generating_parameters'] and self.options['obs_generating_parameters']['save_observations']:
                pointCloudNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1', doUpdate='0')
                pointCloudNode.createObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName = self.options['system_parameters']['observation_file_name'], ExportPositions='1', ExportVelocities='0', ExportForces='0')
            else:
                simuNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1', doUpdate='0')
                simuNode.createObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName = self.options['system_parameters']['observation_file_name'], ExportPositions='1', ExportVelocities='0', ExportForces='0', saveZeroStep='0')

        ### validation grid
        if self.scenario_type == 'Validate_estimations':
            if 'validation_grid_parameters' in self.options.keys():
                for index in range(0, len(self.options['validation_grid_parameters'])):
                    gridElement = self.options['validation_grid_parameters'][index]
                    obsGrid = simuNode.createChild('obsGrid'+str(index))
                    obsGrid.createObject('RegularGrid', name="grid" + str(index), min=gridElement['minimum_values'], max=gridElement['maximum_values'], n=gridElement['amount_of_layers'])
                    obsGrid.createObject('MechanicalObject', name='MO' + str(index), template='Vec3d', src='@grid'+str(index), showIndicesScale='0.00025', showIndices='1')
                    obsGrid.createObject('BarycentricMapping')
                    obsGrid.createObject('ShowSpheres', position='@MO'+str(index)+'.position', color='0.0 0.5 0.0 1', radius="0.0054", showIndicesScale='0.0')
                    if self.options['obs_generating_parameters']['save_observations']:
                        obsGrid.createObject('BoxROI', name='gridBox'+str(index), box='-1 -1 -1 1 1 1', doUpdate='0')
                        obsGrid.createObject('OptimMonitor', name='GridMonitor'+str(index), indices='@gridBox'+str(index)+'.indices', fileName = gridElement['grid_file_name'], ExportPositions='1', ExportVelocities='0', ExportForces='0')

        return 0



    def onEndAnimationStep(self, deltaTime):
        ### modify external impact
        if self.index < 10000:
            position = self.impactPoint.findData('position').value
            position[0][1] = position[0][1] - 0.00002
            self.impactPoint.findData('position').value = position
            self.index = self.index + 1

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

