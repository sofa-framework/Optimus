"""
filteredNoisyBeamPython
is based on the scene 
/home/rtrivi/work/Optimus/scenes/estimState/filteredNoisyBeam.scn
but it uses the SofaPython plugin. 
Further informations on the usage of the plugin can be found in 
sofa/applications/plugins/SofaPython/doc/SofaPython.pdf
To launch the scene, type 
runSofa /home/rtrivi/work/Optimus/scenes/estimState/filteredNoisyBeamPython.py --argv 123
The sofa python plugin might have to be added in the sofa plugin manager, 
i.e. add the sofa python plugin in runSofa->Edit->PluginManager.
The arguments given after --argv can be used by accessing self.commandLineArguments, e.g. combined with ast.literal_eval to convert a string to a number.

The current file has been written by the python script
/home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
Author of scn2python.py: Christoph PAULUS, christoph.paulus@inria.fr
"""

import sys
import Sofa

class filteredNoisyBeam (Sofa.PythonScriptController):

    def __init__(self, node, commandLineArguments) : 
        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        self.createGraph(node)
        return None;

    def createGraph(self,rootNode):

        # rootNode
        rootNode.createObject('RequiredPlugin', pluginName='ConstraintGeometryPlugin')
        rootNode.createObject('RequiredPlugin', pluginName='NeedleConstraintPlugin')
        rootNode.createObject('RequiredPlugin', pluginName='Optimus')
        rootNode.createObject('RequiredPlugin', pluginName='Navcat')
        rootNode.createObject('RequiredPlugin', pluginName='SofaPardisoSolver')
        rootNode.createObject('RequiredPlugin', pluginName='ImageMeshAux')
        rootNode.createObject('RequiredPlugin', pluginName='RegistrationConstraintPlugin')
        rootNode.createObject('RequiredPlugin', pluginName='OpenCVPlugin')
        rootNode.createObject('VisualStyle', displayFlags='hideBehaviorModels hideForceFields showCollisionModels')
        rootNode.createObject('FilteringAnimationLoop', name='StochAnimLoop', verbose='1')
        rootNode.createObject('OpenCVOpenGlStreamer', name='video')
        rootNode.createObject('OpenCVProjectiveCalibrator', name='opengl', listening='false', drawzNear='0.001', streamer='video', drawzFar='3', mode='0')
        rootNode.createObject('UKFilter', initModelVar='0.0001', ZinitModelVar='0.005', obsStdev='0.001', name='UKF', stateStdev='0.001', projectionMatrix='[394.055 -5.38021 721.776 6165.97,285.938 -729.93 0.343075 5410.04,0.999793 -0.019349 -0.00634126 19.0238]', verbose='1')

        # rootNode/Filtered GREEN
        Filtered_GREEN = rootNode.createChild('Filtered GREEN')
        self.Filtered_GREEN = Filtered_GREEN
        Filtered_GREEN.createObject('StochasticStateWrapper', estimateExternalForces='1', template="StdRigidTypes<3, double>, double>", estimatePosition='0', verbose='1', name='StateWrapper', estimateVelocity='0')
        Filtered_GREEN.createObject('OptimParams', name='optimParams')
        Filtered_GREEN.createObject('EulerImplicitSolver', rayleighStiffness='00', vdamping='10', rayleighMass='00')
        Filtered_GREEN.createObject('CGLinearSolver')
        Filtered_GREEN.createObject('EdgeSetTopologyContainer', position='0 0 0  0 0 1  0 0 2  0 0 3  0 0 4  0 0 5', edges='0 1  1 2   2 3  3 4  4 5 ', name='Container')
        Filtered_GREEN.createObject('Transform3dToRigid', name='transform')
        Filtered_GREEN.createObject('MechanicalObject', showObject='false', position='@transform.out_position', name='catDoFs', template='Rigid')
        Filtered_GREEN.createObject('UniformMass', totalmass='0.4')
        Filtered_GREEN.createObject('PartialFixedConstraint', indices='0', name='FixedConstraint')
        Filtered_GREEN.createObject('BeamFEMForceField', radius='0.13', name='FEM', poissonRatio='0.49', youngModulus='200000')
        Filtered_GREEN.createObject('Sphere', color='0 0 1 1', radius='0.05')

        # rootNode/Filtered GREEN/CollisionCatheter
        CollisionCatheter = Filtered_GREEN.createChild('CollisionCatheter')
        self.CollisionCatheter = CollisionCatheter
        CollisionCatheter.createObject('EdgeSetTopologyContainer', src='@../Container')
        CollisionCatheter.createObject('MechanicalObject', name='ms')
        CollisionCatheter.createObject('Line', color='1 1 0 1')
        CollisionCatheter.createObject('NeedleGeometry', name='catheter')
        CollisionCatheter.createObject('Sphere', color='1 1 1 1', radius='0.0001')
        CollisionCatheter.createObject('BeamLinearMapping', localCoord='false')

        # rootNode/Filtered GREEN/obsNode
        obsNode = Filtered_GREEN.createChild('obsNode')
        self.obsNode = obsNode
        obsNode.activated = '1'
        obsNode.createObject('EdgeSetTopologyContainer', position='0 0 0  0 0 1  0 0 2  0 0 3  0 0 4  0 0 5', edges='0 1  1 2   2 3  3 4  4 5 ', name='Container')
        obsNode.createObject('MechanicalObject', showObject='false', name='OBScatDoFs', template='Vec2d')
        obsNode.createObject('SimulatedStateObservationSource', printLog='1', monitorPrefix='TEST_planeforcefield', name='observations', template='Vec2d', verbose='1')
        obsNode.createObject('SimpleObservationManager', stateWrapper='@../StateWrapper', verbose='1', name='MOBS', listening='1', template='Vec2d')

        # rootNode/Filtered GREEN/GroundTruth
        GroundTruth = Filtered_GREEN.createChild('GroundTruth')
        self.GroundTruth = GroundTruth
        GroundTruth.createObject('EdgeSetTopologyContainer', position='0 0 0  0 0 1  0 0 2  0 0 3  0 0 4  0 0 5', edges='0 1  1 2   2 3  3 4  4 5 ', name='GTContainer')
        GroundTruth.createObject('Transform3dToRigid', name='transform')
        GroundTruth.createObject('MechanicalObject', showObject='false', position='@transform.out_position', name='catDoFs', template='Rigid')
        GroundTruth.createObject('ReadState', filename='GroundTruthCatheterTEST')

        # rootNode/Filtered GREEN/GroundTruth/GTCollisionCatheter
        GTCollisionCatheter = GroundTruth.createChild('GTCollisionCatheter')
        self.GTCollisionCatheter = GTCollisionCatheter
        GTCollisionCatheter.createObject('EdgeSetTopologyContainer', src='@../GTContainer')
        GTCollisionCatheter.createObject('MechanicalObject', name='ms')
        GTCollisionCatheter.createObject('Line', color='0 0 1 1')
        GTCollisionCatheter.createObject('NeedleGeometry', name='catheter')
        GTCollisionCatheter.createObject('Sphere', color='0 1 0 1', radius='0.05')
        GTCollisionCatheter.createObject('BeamLinearMapping', localCoord='false')

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
        ## Please feel free to add an example for a simple usage in /home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
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
        ## Please feel free to add an example for a simple usage in /home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
        return 0;

    def cleanup(self):
        ## Please feel free to add an example for a simple usage in /home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onGUIEvent(self, strControlID,valueName,strValue):
        ## Please feel free to add an example for a simple usage in /home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onEndAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onLoaded(self, node):
        ## Please feel free to add an example for a simple usage in /home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
        return 0;

    def reset(self):
        ## Please feel free to add an example for a simple usage in /home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def bwdInitGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        ## Please feel free to add an example for a simple usage in /home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onMouseButtonRight(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/rtrivi/work/Optimus/scenes/estimState//home/rtrivi/work/sofamimesis/applications/plugins/SofaPython/scn2python.py
        return 0;


def createScene(rootNode):
    rootNode.findData('dt').value = '0.1'
    rootNode.findData('gravity').value = '0 0 0 '
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    myfilteredNoisyBeam = filteredNoisyBeam(rootNode,commandLineArguments)
    return 0;
