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
        simuNode.createObject('SparsePARDISOSolver', name='LDLsolver', verbose='0', symmetric='2', exportDataToFolder='')

        fileExtension = self.options['system_parameters']['volume_file_name']
        fileExtension = fileExtension[fileExtension.rfind('.') + 1:]
        if fileExtension == 'vtk' or fileExtension == 'vtu':
            simuNode.createObject('MeshVTKLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        elif fileExtension == 'msh':
            simuNode.createObject('MeshGmshLoader', name='loader', filename=self.options['system_parameters']['volume_file_name'])
        else:
            print 'Unknown file type!'
        simuNode.createObject('MechanicalObject', src='@loader', name='Volume')

        if 'boundary_conditions_list' in self.options['general_parameters'].keys():
            for index in range(0, len(self.options['general_parameters']['boundary_conditions_list'])):
                bcElement = self.options['general_parameters']['boundary_conditions_list'][index]
                simuNode.createObject('BoxROI', box=bcElement['boxes_coordinates'], name='boundBoxes'+str(index), drawBoxes='0', doUpdate='0')
                if bcElement['condition_type'] == 'fixed':
                    #simuNode.createObject('FixedConstraint', indices='@boundBoxes'+str(index)+'.indices')
                    simuNode.createObject('FixedConstraint', indices=bcElement['fixed_indices'])
                elif bcElement['condition_type'] == 'elastic':
                    simuNode.createObject('RestShapeSpringsForceField', stiffness=bcElement['spring_stiffness_values'], angularStiffness="1", points='@boundBoxes'+str(index)+'.indices')
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

        #simuNode.createObject('Indices2ValuesMapper', indices='1 2 3 4 5 6 7 8 9 10', values=self.options['obs_generating_parameters']['liver_young_moduli'], name='youngMapper', inputValues='@loader.dataset')
        simuNode.createObject('TetrahedronFEMForceField', updateStiffness='1', name='FEM', listening='true', drawHeterogeneousTetra='1', method='large', youngModulus='5000', poissonRatio='0.45')

        #if self.options['obs_generating_parameters']['save_observations']:
        #    simuNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1', doUpdate='0')
        #    simuNode.createObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName = self.generalFolderName + '/' + self.options['system_parameters']['observation_file_name'], ExportPositions='1', ExportVelocities='0', ExportForces='0')

        # add constant force field
        simuNode.createObject('BoxROI', name='impactBounds', box='0.14 0.15 0.4 0.16 0.17 0.43', doUpdate='0')
        #self.constantForce = simuNode.createObject('ConstantForceField', name='appliedForce', indices='@impactBounds.indices', totalForce='0.0 0.0 0.0')
        self.constantForce = simuNode.createObject('ConstantForceField', name='appliedForce', indices=self.options['impact_parameters']['applied_indices'], totalForce='0.0 0.0 0.0')

        if self.options['obs_generating_parameters']['save_observations']:
            obsNode = simuNode.createChild('obsNode')        
            obsNode.createObject('MeshVTKLoader', name='obsLoader', filename=self.options['system_parameters']['observation_points_file_name'])
            obsNode.createObject('MechanicalObject', name='SourceMO', src="@obsLoader")
            obsNode.createObject('ShowSpheres', position='@SourceMO.position', color='1.0 1.0 0.0 1', radius="0.0034", showIndicesScale='0.0') 
            obsNode.createObject('BarycentricMapping')
            obsNode.createObject('BoxROI', name='observationNodeBox', box='-1 -1 -1 1 1 1', doUpdate='0')
            obsNode.createObject('OptimMonitor', name='ObservationMonitor', indices='@observationNodeBox.indices', fileName = self.generalFolderName + '/observations/node', ExportPositions='1', ExportVelocities='0', ExportForces='0')

        return 0


    def onEndAnimationStep(self, deltaTime):
        self.iterations = self.iterations + 1
        self.timeIterations = self.timeIterations + 1
        if self.options['impact_parameters']['impact_type'] == 'Periodic':
            currentValue = self.options['force_parameters']['force_value']
            currentValue = [elem * self.amplitude * 0.5 * (1 - math.cos(math.pi * self.iterations / self.period)) for elem in currentValue]
            self.constantForce.findData('totalForce').value = currentValue
        return 0


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


