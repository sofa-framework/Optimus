# this scene is compatible with python3
import Sofa
import math
import os
import sys
import csv
import yaml

__file = __file__.replace('\\', '/') # windows

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='Optimus', pluginName='Optimus')
    # rootNode.addObject('RequiredPlugin', name='Python3', pluginName='SofaPython3')

    try:
        sys.argv[0]
    except:
        commandLineArguments = []
    else:
        commandLineArguments = sys.argv

    # if len(commandLineArguments) > 1:
    #     configFileName = commandLineArguments[1]
    configFileName = "yaml/cylRestShape_scene_config.yml"
    # else:
    #     print('ERROR: Must supply a yaml config file as an argum ent!')
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

    rootNode.addObject(cylRestShapeGenObs_Controller(name="cylRestShape_GenObs", node=rootNode, opt=options))
    return 0



class cylRestShapeGenObs_Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.rootNode = kwargs["node"]
        self.options = kwargs["opt"]
        self.generalFolderName = self.options['filtering_parameters']['common_directory_prefix'] + self.options['general_parameters']['solver_kind']
        if not os.path.isdir(self.generalFolderName):
            os.mkdir(self.generalFolderName)
        if not os.path.isdir(self.generalFolderName + '/observations'):
            os.mkdir(self.generalFolderName + '/observations')

        self.rootNode.findData('dt').value = self.options['general_parameters']['delta_time']
        self.rootNode.findData('gravity').value = self.options['general_parameters']['gravity']
        self.createGraph(self.rootNode)
        return None


    def createGraph(self, rootNode):
        
        # rootNode
        rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')

        # external impact simulation
        dotNode = rootNode.addChild('dotNode')
        self.impactPoint = dotNode.addObject('MechanicalObject', template='Vec3d', name='dot', showObject='true', showObjectScale='0.01', drawMode='1', position='0.0 -0.02 0.12')
        if self.options['obs_generating_parameters']['save_observations']:
            dotNode.addObject('BoxROI', name='impactBounds1', box='-0.01 -0.03 0.11 0.01 -0.01 0.13', doUpdate='0')
            dotNode.addObject('OptimMonitor', name='toolMonitor', template='Vec3d', showPositions='0', indices='0', ExportPositions='1', fileName = self.generalFolderName + '/' + self.options['impact_parameters']['observation_file_name'])
        self.index = 0

        # general node
        simuNode = rootNode.addChild('simuNode')
        self.simuNode = simuNode

        # solvers
        if self.options['general_parameters']['solver_kind'] == 'Euler':
            simuNode.addObject('EulerImplicitSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'])
        elif self.options['general_parameters']['solver_kind'] == 'Symplectic':
            simuNode.addObject('VariationalSymplecticSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'], newtonError='1e-12', steps='1', verbose='0')
        elif self.options['general_parameters']['solver_kind'] == 'Newton':
            simuNode.addObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="2")
        else:
            print('Unknown solver type!')

        if self.options['general_parameters']['linear_solver_kind'] == 'Pardiso':
            simuNode.addObject('SparsePARDISOSolver', name='LDLsolver', verbose='0', symmetric='2', exportDataToFolder='')
        elif self.options['general_parameters']['linear_solver_kind'] == 'CG':
            simuNode.addObject('CGLinearSolver', iterations="20", tolerance="1e-12", threshold="1e-12")
        else:
            print('Unknown linear solver type!')

        # mechanical object
        simuNode.addObject('MeshVTKLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        simuNode.addObject('MechanicalObject', src='@loader', name='Volume')
        simuNode.addObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.addObject('TetrahedronSetTopologyModifier', name="Modifier")
        simuNode.addObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        simuNode.addObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        if 'total_mass' in self.options['general_parameters'].keys():
            simuNode.addObject('UniformMass', totalMass=self.options['general_parameters']['total_mass'])
        if 'density' in self.options['general_parameters'].keys():
            simuNode.addObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.options['general_parameters']['density'], name='mass')

        simuNode.addObject('Indices2ValuesMapper', indices='1 2 3 4 5 6 7 8 9 10', values=self.options['obs_generating_parameters']['object_young_moduli'], name='youngMapper', inputValues='@loader.dataset')
        simuNode.addObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')
        #simuNode.addObject('VTKExporter', position='@Volume.position', edges='0', tetras='1', listening='0', XMLformat='0', exportAtEnd='1', exportEveryNumberOfSteps='0', filename='observations/directScene.vtk')

        # boundary conditions
        if 'boundary_conditions_list' in self.options['general_parameters'].keys():
            for index in range(0, len(self.options['general_parameters']['boundary_conditions_list'])):
                bcElement = self.options['general_parameters']['boundary_conditions_list'][index]
                print(bcElement)
                simuNode.addObject('BoxROI', box=bcElement['boxes_coordinates'], name='boundBoxes'+str(index), drawBoxes='0', doUpdate='1')
                if bcElement['condition_type'] == 'fixed':
                    simuNode.addObject('FixedConstraint', indices='@boundBoxes'+str(index)+'.indices')
                elif bcElement['condition_type'] == 'elastic':
                    simuNode.addObject('RestShapeSpringsForceField', stiffness=bcElement['spring_stiffness_values'], angularStiffness="1", points='@boundBoxes'+str(index)+'.indices')
                else:
                    print('Unknown type of boundary conditions')

        # saving generated observations
        if self.options['obs_generating_parameters']['save_observations']:
            simuNode.addObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1', doUpdate='0')
            simuNode.addObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName = self.generalFolderName + '/' + self.options['system_parameters']['observation_file_name'], ExportPositions='1', ExportVelocities='0', ExportForces='0')

        # attachement to external impact
        simuNode.addObject('BoxROI', name='impactBounds', box='-0.01 -0.03 0.11 0.01 0.01 0.12', doUpdate='0')
        simuNode.addObject('RestShapeSpringsForceField', name='Springs', stiffness='10000', angularStiffness='1', external_rest_shape='@dotNode/dot', points='@impactBounds.indices')

        return 0;



    def onAnimateEndEvent(self, eventType):
        # shift external impact
        if self.index < 10000:
            position = self.impactPoint.findData('position').value
            position[0][1] = position[0][1] - 0.00002
            self.impactPoint.findData('position').value = position
            self.index = self.index + 1

        return 0;

