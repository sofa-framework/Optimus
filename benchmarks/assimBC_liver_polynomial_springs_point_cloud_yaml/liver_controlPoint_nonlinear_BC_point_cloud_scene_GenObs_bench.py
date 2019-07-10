import Sofa
import math
import os
import sys
import csv
import yaml

__file = __file__.replace('\\', '/') # windows


def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='Optimus', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')
    rootNode.createObject('RequiredPlugin', name='IMAUX', pluginName='ImageMeshAux')
    # rootNode.createObject('RequiredPlugin', name='MJED', pluginName='SofaMJEDFEM')
    rootNode.createObject('RequiredPlugin', name='BoundaryConditions', pluginName="BoundaryConditionsPlugin")
    
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
    print "Command line arguments for python : " + str(commandLineArguments)

    with open(configFileName, 'r') as stream:
        try:
            options = yaml.load(stream)

        except yaml.YAMLError as exc:
            print(exc)
            return

    liver_controlPoint_GenObs(rootNode, options)
    return 0


class liver_controlPoint_GenObs (Sofa.PythonScriptController):

    def __init__(self, rootNode, options):
        self.options = options
        self.scenario_type = 'Generate_data'
        if 'scenario_type' in self.options['general_parameters'].keys() and self.options['general_parameters']['scenario_type'] == 'Validate_estimations':
            self.scenario_type = 'Validate_estimations'
        self.vdamping = 5.0

        rootNode.findData('dt').value = options['general_parameters']['delta_time']
        rootNode.findData('gravity').value = options['general_parameters']['gravity']
        self.createGraph(rootNode)
        return None



    def createGraph(self, rootNode):
        nu=0.45
        E=5000

        # rootNode
        rootNode.createObject('VisualStyle', displayFlags='hideVisualModels showBehaviorModels hideCollisionModels hideMappings showForceFields')

        # rootNode/fixedNodes
        fixNode = rootNode.createChild('fixNode')
        fixNode.createObject('MechanicalObject', template='Vec3d', name='fixElements', showObject='true', position=self.options['boundary_parameters']['external_points'])
        fixNode.createObject('ShowSpheres', position='@fixElements.position', color='1.0 0.0 1.0 1', radius="0.01", showIndicesScale='0.0')

        # rootNode/externalImpact
        dotNode = rootNode.createChild('dotNode')
        self.impactPoint = dotNode.createObject('MechanicalObject', template='Vec3d', name='dot', showObject='true', position='0.143369 0.156781 0.395153')
        dotNode.createObject('ShowSpheres', position='@dot.position', color='1.0 0.0 1.0 1', radius="0.01", showIndicesScale='0.0')
        if self.options['obs_generating_parameters']['save_observations'] and self.scenario_type == 'Generate_data':
            dotNode.createObject('BoxROI', name='dotBounds', box='0.14 0.15 0.37 0.18 0.17 0.4', doUpdate='0')
            dotNode.createObject('OptimMonitor', name='toolMonitor', template='Vec3d', showPositions='1', indices='@dotBounds.indices', ExportPositions='1', fileName = self.options['impact_parameters']['observation_file_name'])
        self.index = 0
        	
        # rootNode/simuNode
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode
        if self.options['general_parameters']['solver_kind'] == 'Euler':
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'])
        elif self.options['general_parameters']['solver_kind'] == 'Symplectic':
            simuNode.createObject('VariationalSymplecticSolver', rayleighStiffness=self.options['general_parameters']['rayleigh_stiffness'], rayleighMass=self.options['general_parameters']['rayleigh_mass'], newtonError='1e-12', steps='1', verbose='0')
        elif self.options['general_parameters']['solver_kind'] == 'Newton':
            simuNode.createObject('NewtonStaticSolver', name="NewtonStatic", printLog="0", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt="2")
        else:
            print 'Unknown solver type!'
        simuNode.createObject('SparsePARDISOSolver', name='LDLsolver', verbose='0', symmetric='1', exportDataToFolder='')

        fileExtension = self.options['system_parameters']['volume_file_name']
        fileExtension = fileExtension[fileExtension.rfind('.') + 1:]
        if fileExtension == 'vtk' or fileExtension == 'vtu':
            simuNode.createObject('MeshVTKLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        elif fileExtension == 'msh':
            simuNode.createObject('MeshGmshLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        else:
            print 'Unknown file type!'
        simuNode.createObject('MechanicalObject', src='@loader', name='Volume')

        if 'boundary_conditions_list' in self.options['boundary_parameters'].keys():
            for index in range(0, len(self.options['boundary_parameters']['boundary_conditions_list'])):
                bcElement = self.options['boundary_parameters']['boundary_conditions_list'][index]
                simuNode.createObject('BoxROI', box=bcElement['boxes_coordinates'], name='boundBoxes'+str(index), drawBoxes='0', doUpdate='0')
                if bcElement['condition_type'] == 'fixed':
                    simuNode.createObject('FixedConstraint', indices='@boundBoxes'+str(index)+'.indices')
                elif bcElement['condition_type'] == 'elastic':
                    if self.options['boundary_parameters']['spring_type'] == 'PolynomialRestshape':
                        simuNode.createObject('PolynomialRestShapeSpringsForceField', polynomialStiffness=bcElement['polynomial_stiffness_values'], howIndicesScale='0', springThickness="3", listening="1", updateStiffness="1", printLog="0", points='@boundBoxes'+str(index)+'.indices', initialLength=self.options['boundary_parameters']['initial_length'], polynomialDegree=bcElement['polynomial_degrees'], directionScale=self.options['boundary_parameters']['direction_scale'])
                    elif self.options['boundary_parameters']['spring_type'] == 'Polynomial':
                        simuNode.createObject('PolynomialRestShapeSpringsInitLengthForceField', polynomialStiffness=bcElement['polynomial_stiffness_values'], howIndicesScale='0', springThickness="3", listening="1", updateStiffness="1", printLog="0", points='@boundBoxes'+str(index)+'.indices', polynomialDegree=bcElement['polynomial_degrees'], external_rest_shape='../fixNode/fixElements', external_points=self.options['boundary_parameters']['external_indices'])
                    elif self.options['boundary_parameters']['spring_type'] == 'Exponential':
                        simuNode.createObject('ExponentialRestShapeSpringsForceField', exponentialStiffness=self.options['boundary_parameters']['exponential_stiffness'], howIndicesScale='0', springThickness="3", listening="1", updateStiffness="1", printLog="0", points='@boundBoxes'+str(index)+'.indices', initialLength=self.options['boundary_parameters']['initial_length'], exponentialIndicator=self.options['boundary_parameters']['exponential_indicator'], directionScale=self.options['boundary_parameters']['direction_scale'])
                    else:
                        simuNode.createObject('ExtendedRestShapeSpringForceField', stiffness=bcElement['spring_stiffness_values'], angularStiffness="1", points='@boundBoxes'+str(index)+'.indices')
                else:
                    print 'Unknown type of boundary conditions'

        simuNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        if 'total_mass' in self.options['general_parameters'].keys():
            simuNode.createObject('UniformMass', totalMass=self.options['general_parameters']['total_mass'])
        if 'density' in self.options['general_parameters'].keys():
            simuNode.createObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.options['general_parameters']['density'], name='mass')

        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        materialParams='{} {}'.format(mu,lamb)
        #simuNode.createObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        simuNode.createObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method='large', youngModulus='5000', poissonRatio='0.45')

        # rootNode/simuNode/attached
        simuNode.createObject('BoxROI', name='impactBounds', box='0.14 0.15 0.37 0.18 0.17 0.4', doUpdate='0')
        simuNode.createObject('RestShapeSpringsForceField', name='Springs', stiffness='10000', angularStiffness='1', external_rest_shape='@../dotNode/dot', points='@impactBounds.indices')

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

        if self.scenario_type == 'Validate_estimations':
            if 'validation_grid_parameters' in self.options.keys():
                for index in range(0, len(self.options['validation_grid_parameters'])):
                    gridElement = self.options['validation_grid_parameters'][index]
                    obsGrid = simuNode.createChild('obsGrid'+str(index))
                    obsGrid.createObject('RegularGrid', name="grid"+str(index), min=gridElement['minimum_values'], max=gridElement['maximum_values'], n=gridElement['amount_of_layers'])
                    obsGrid.createObject('MechanicalObject', name='MO'+str(index), template='Vec3d', src='@grid'+str(index), showIndicesScale='0.00025', showIndices='1')
                    obsGrid.createObject('BarycentricMapping')
                    obsGrid.createObject('ShowSpheres', position='@MO'+str(index)+'.position', color='0.0 0.5 0.0 1', radius="0.0054", showIndicesScale='0.0')
                    if self.options['obs_generating_parameters']['save_observations']:
                        obsGrid.createObject('BoxROI', name='gridBox'+str(index), box='-1 -1 -1 1 1 1', doUpdate='0')
                        obsGrid.createObject('OptimMonitor', name='GridMonitor'+str(index), indices='@gridBox'+str(index)+'.indices', fileName = gridElement['grid_file_name'], ExportPositions='1', ExportVelocities='0', ExportForces='0')

        return 0



    def onEndAnimationStep(self, deltaTime):

        if self.index < 10000:
            position = self.impactPoint.findData('position').value
            position[0][1] = position[0][1] - 0.00002
            self.impactPoint.findData('position').value = position
            self.index = self.index + 1

        return 0;

    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onKeyReleased(self, c):
        ## usage e.g.
        #if c=="A" :
        #    print "You released a"
        return 0;

    def initGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/ip/Work/sofa/MyPlugins/Optimus/scenes/assimStiffness//home/ip/Work/sofa/master/src/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onKeyPressed(self, c):
        ## usage e.g.
        #if c=="A" :
        #    print "You pressed control+a"
        return 0;

    def onMouseWheel(self, mouseX,mouseY,wheelDelta):
        ## usage e.g.
        #if isPressed : 
        #    print "Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta)
        return 0;

    def storeResetState(self):
        ## Please feel free to add an example for a simple usage in /home/ip/Work/sofa/MyPlugins/Optimus/scenes/assimStiffness//home/ip/Work/sofa/master/src/applications/plugins/SofaPython/scn2python.py
        return 0;

    def cleanup(self):
        ## Please feel free to add an example for a simple usage in /home/ip/Work/sofa/MyPlugins/Optimus/scenes/assimStiffness//home/ip/Work/sofa/master/src/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onGUIEvent(self, strControlID,valueName,strValue):
        ## Please feel free to add an example for a simple usage in /home/ip/Work/sofa/MyPlugins/Optimus/scenes/assimStiffness//home/ip/Work/sofa/master/src/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onLoaded(self, node):
        ## Please feel free to add an example for a simple usage in /home/ip/Work/sofa/MyPlugins/Optimus/scenes/assimStiffness//home/ip/Work/sofa/master/src/applications/plugins/SofaPython/scn2python.py
        return 0;

    def reset(self):
        ## Please feel free to add an example for a simple usage in /home/ip/Work/sofa/MyPlugins/Optimus/scenes/assimStiffness//home/ip/Work/sofa/master/src/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def bwdInitGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/ip/Work/sofa/MyPlugins/Optimus/scenes/assimStiffness//home/ip/Work/sofa/master/src/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        ## Please feel free to add an example for a simple usage in /home/ip/Work/sofa/MyPlugins/Optimus/scenes/assimStiffness//home/ip/Work/sofa/master/src/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onMouseButtonRight(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/ip/Work/sofa/MyPlugins/Optimus/scenes/assimStiffness//home/ip/Work/sofa/master/src/applications/plugins/SofaPython/scn2python.py
        return 0;


