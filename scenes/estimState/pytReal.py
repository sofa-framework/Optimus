"""
realXYZ_s_infPython
is based on the scene 
/home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/../../../../Optimus/scenes/estimState/realXYZ_s_inf.scn
but it uses the SofaPython plugin. 
Further informations on the usage of the plugin can be found in 
sofa/applications/plugins/SofaPython/doc/SofaPython.pdf
To launch the scene, type 
runSofa /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/../../../../Optimus/scenes/estimState/realXYZ_s_infPython.py --argv 123
The sofa python plugin might have to be added in the sofa plugin manager, 
i.e. add the sofa python plugin in runSofa->Edit->PluginManager.
The arguments given after --argv can be used by accessing self.commandLineArguments, e.g. combined with ast.literal_eval to convert a string to a number.

The current file has been written by the python script
/home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
Author of scn2python.py: Christoph PAULUS, christoph.paulus@inria.fr
"""

import sys
import Sofa

class realXYZ_s_inf (Sofa.PythonScriptController):

    #video='realDataSet/cat0/out.mp4'
    def __init__(self, node, commandLineArguments) : 
        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        self.createGraph(node)
        return None;

    def createGraph(self,rootNode):

        # rootNode
        rootNode.createObject('RequiredPlugin', pluginName='Optimus')
        rootNode.createObject('RequiredPlugin', pluginName='ConstraintGeometryPlugin')
        rootNode.createObject('RequiredPlugin', pluginName='NeedleConstraintPlugin')
        rootNode.createObject('RequiredPlugin', pluginName='OpenCVPlugin')
        rootNode.createObject('RequiredPlugin', pluginName='SofaCV')
        rootNode.createObject('RequiredPlugin', pluginName='ImageProcessing') 
        rootNode.createObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideMappings hideForceFields hideWireframe')
        rootNode.createObject('BackgroundSetting', color='1 1 1 1')
        rootNode.createObject('ViewerSetting', resolution='837 600')
        rootNode.createObject('FilteringAnimationLoop', name='StochAnimLoop', verbose='1')
        rootNode.createObject('UKFilterClassic', draw='0', useUnbiasedVariance='0', name='UKF', MOnodesDraw='12', exportPrefix='cov_', radiusDraw='0.0', filenameFinalState='realDataSet/test2', verbose='1')


        # rootNode/validationSofaCV
        #validationSofaCV = rootNode.createChild('validationSofaCV')
        #self.validationSofaCV = validationSofaCV
        #rootNode.createObject('CameraSettings', name="camSet", M="[-3452.3 219.165 -50.3532 251.620, -24.5691 647.057 3388.13 271.587, -0.00217632 0.993 -0.118096 0.474452]", imageSize="640 480")
        #rootNode.createObject('CalibratedCamera', name="calib", cam="@camSet" )  
        #rootNode.createObject('VideoGrabber', videoFile='realDataSet/cat0/out.mp4', name='grabber')
        #rootNode.createObject('FrameViewer', name='viewer', img='@grabber.img_out', corners="@camSet.3DCorners")




        ## rootNode/validationView
        validationView = rootNode.createChild('validationView')
        self.validationView = validationView
        rootNode.createObject('OpenCVPngStreamer', filename='/home/raffa/Desktop/cat0_/cat0__????.png', name='streamer', imageId='0001')
        rootNode.createObject('OpenCVProjectiveCalibrator', drawCamera='0', drawscreenSize='640 480', name='calibrator', drawscreenPosition='0 0', drawzFar='10000000', projectionMatrix='[-3452.3 219.165 -50.3532 251.620 -24.5691 647.057 3388.13 271.587 -0.00217632 0.993 -0.118096 0.474452]', drawViewPort='0', depth='0.9', drawswapMainView='1', mode='0', listening='1', drawzNear='0.0000001', streamer='streamer')
        rootNode.createObject('OpenCVViewer', name='viewer', drawcolor='1 1 1 1', streamer='streamer')
        rootNode.createObject('OpenCVScheduler', frequency='0', name='scheduler')


        # rootNode/Reference
        Reference = rootNode.createChild('Reference')
        self.Reference = Reference
        Reference.createObject('PreStochasticWrapper')

        # rootNode/Reference/init
        init = Reference.createChild('init')
        self.init = init
        init.createObject('EulerImplicitSolver')
        init.createObject('CGLinearSolver', threshold='1e-5', tolerance='1e-5', iterations='25')
        init.createObject('MechanicalObject', position='0.015043  0.053987  -0.058037  0.010226  0.054093  -0.058009  0.0053189  0.054576  -0.057995  0.00031872  0.054576  -0.057995  -0.0046815  0.054576  -0.057995  -0.0096816  0.054575  -0.057996  -0.014682  0.054575  -0.057996  -0.019682  0.054574  -0.057996  -0.024682  0.054572  -0.057997  -0.029682  0.054571  -0.057998  -0.034682  0.054569  -0.057999  -0.039682  0.054568  -0.058', name='MO', template='Vec3')
        init.createObject('ReadState', filename='realDataSet/referenceAll3')
        init.createObject('ShowCatheter', color='0 0 1 1', position='@MO.position', radius='0.0002')

        # rootNode/Reference/VisualRealSurface
        VisualRealSurface = Reference.createChild('VisualRealSurface')
        self.VisualRealSurface = VisualRealSurface
        VisualRealSurface.createObject('MeshSTLLoader', name='surface', filename='realDataSet/phantomScaled.stl')
        VisualRealSurface.createObject('MeshTopology', src='@surface')
        VisualRealSurface.createObject('MechanicalObject', name='DOFs')
        VisualRealSurface.createObject('OglModel', src='@surface', material='Default Diffuse 1 1 0.93 0.83 0.05 Ambient 0 0 0 0 0 Specular 0 0 0 0 0 Emissive 0 0 0 0 0 Shininess 0 45', name='VisualModel')

        # rootNode/filteredModel
        filteredModel = rootNode.createChild('filteredModel')
        self.filteredModel = filteredModel
        filteredModel.createObject('StochasticStateWrapper', draw='0', verbose='1', estimateVelocity='1', positionStdev='0.5e-04 0.5e-04 0.5e-04  ', estimatePosition='1', mappedState='cat/CollisionCat/ms', template='Rigid3d', langrangeMultipliers='1', estimateOnlyXYZ='1', velModelStdev='0.5e-04 0.5e-04 0.5e-04 0.5e-04 0.5e-04 0.5e-04   ', posModelStdev='0 0 0 ', radiusDraw='0.0', velocityStdev='0.5e-04  0.5e-04  0.5e-04 0.5e-04 0.5e-04 0.5e-04  ', name='StateWrapper')
        filteredModel.createObject('GenericConstraintSolver', maxIterations='1000', tolerance='1e-10', name='GenericConstraintSolver', allVerified='0', printLog='0')

        # rootNode/filteredModel/cat
        cat = filteredModel.createChild('cat')
        self.cat = cat
        cat.createObject('EulerImplicitSolver', rayleighStiffness='0.0', rayleighMass='0.00')
        cat.createObject('BTDLinearSolver', name='BTD')
        cat.createObject('MechanicalObject', showObject='0', position=' 0.015043 0.053987 -0.058037 -5.84121e-12 -0.707107 0.707107 -3.31838e-10                                          0.010226 0.054093 -0.058009 -7.7867e-12 -0.707107 0.707107 -3.29785e-10                                          0.00531889 0.054576 -0.057995 -1.94831e-12 -0.707107 0.707107 2.06597e-12                                         0.000318715 0.054576 -0.057995 6.338e-06 -0.707107 0.707107 -3.9493e-05                                          -0.00468152 0.0545756 -0.0579953 7.922e-06 -0.707107 0.707107 -4.9364e-05                                         -0.00968158 0.0545752 -0.0579956 2.255e-05 -0.707107 0.707107 -7.74e-05                                          -0.0146817 0.0545746 -0.057996 3.7177e-05 -0.707107 0.707107 -0.000105436                                         -0.0196818 0.0545736 -0.0579964 3.0473e-05 -0.707107 0.707107 -0.000126767                                         -0.0246819 0.0545724 -0.057997 4.3515e-05 -0.707107 0.707107 -0.000144928                                         -0.0296819 0.0545711 -0.0579977 5.8143e-05 -0.707107 0.707107 -0.000172964                                          -0.0346819 0.0545694 -0.0579987 5.3023e-05 -0.707107 0.707107 -0.00020417                                          -0.039682 0.0545677 -0.0579997 5.3023e-05 -0.707107 0.707107 -0.00020417', showObjectScale='0.001', name='MO', template='Rigid3d')
        cat.createObject('Mesh', lines='0 1  1 2 2 3 3 4 4 5 5 6 6 7 7 8 8 9 9 10 10 11  ', name='lines')
        cat.createObject('UniformMass', printLog='false', totalMass='0.0001')
        cat.createObject('BeamFEMForceField', poissonRatio='0.49', radius='0.0005', name='FEM', useSymmetricAssembly='1', youngModulus='1e6')
        cat.createObject('ConstantForceField', indices=' 11', force='0.0008 0 -0.0008  0.0 0.0  0.0 ')

        # rootNode/filteredModel/cat/CollisionCat
        CollisionCat = cat.createChild('CollisionCat')
        self.CollisionCat = CollisionCat
        CollisionCat.createObject('CubeTopology', nx='12', ny='1', nz='1', min='0 0 0', max='11 0 0')
        CollisionCat.createObject('MechanicalObject', name='ms')
        CollisionCat.createObject('BeamLinearMapping')
        CollisionCat.createObject('EdgeGeometry', name='edge')
        CollisionCat.createObject('PointGeometry', name='point')
        CollisionCat.createObject('ShowCatheter', color='0 1 0 1', position='@ms.position', radius='0.0002', drawOnlySpheres='false')
        CollisionCat.createObject('BoxROI', box='0 0 -0.1 0.1 0.1 0', drawSize='1', drawBoxes='0', name='boxA', position='@ms.position')
        cat.createObject('LinearSolverConstraintCorrection', solverName='BTD')

        # rootNode/filteredModel/cat/obsNode
        obsNode = cat.createChild('obsNode')
        self.obsNode = obsNode
        obsNode.activated = '1'
        obsNode.createObject('SimulatedStateObservationSource', asynObs='true', monitorPrefix='realDataSet/obs_N12', name='observations', template='Vec2d')
        obsNode.createObject('SimpleObservationManager', verbose='1', projectionMatrix='[-3320.65 174.572 -514.441 193.451 134.409 3340.13 44.4847 -64.7090 0.0480279 0.103531 -0.993466 0.400192]', observationStdev='0.0005', template='Vec2d,Rigid3d', use2dObservation='true', stateWrapper='@../../StateWrapper', listening='1', name='MSIDES')

        # rootNode/filteredModel/Vessel
        Vessel = filteredModel.createChild('Vessel')
        self.Vessel = Vessel
        Vessel.createObject('VisualStyle', displayFlags='showWireframe hideCollisionModels')
        Vessel.createObject('MeshSTLLoader', name='surface', filename='realDataSet/phantomScaled2.stl')
        Vessel.createObject('TransformStochasticEngine', input_position='@surface.position', optimParams='0 0 0 0 0 0 ', name='transform', triangles='@surface.triangles')
        Vessel.createObject('MeshTopology', position='@transform.output_position', name='mesh', triangles='@transform.triangles')
        Vessel.createObject('TriangleSetTopologyContainer', src='@mesh', name='Container')
        Vessel.createObject('MechanicalObject', position='@transform.output_position', name='MS', template='Vec3d', free_position='@transform.output_position')
        Vessel.createObject('TriangleGeometry', name='triangle')
        Vessel.createObject('AABBDecorator')
        filteredModel.createObject('NeedleContactConstraint', draw_normal_scale='0.001', surface='Vessel/triangle', tipIndex='@cat/CollisionCat/boxA.indices', needle='cat/CollisionCat/point', draw='0', friction='0.1')

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
        ## Please feel free to add an example for a simple usage in /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
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
        ## Please feel free to add an example for a simple usage in /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
        return 0;

    def cleanup(self):
        ## Please feel free to add an example for a simple usage in /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onGUIEvent(self, strControlID,valueName,strValue):
        ## Please feel free to add an example for a simple usage in /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onEndAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onLoaded(self, node):
        ## Please feel free to add an example for a simple usage in /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
        return 0;

    def reset(self):
        ## Please feel free to add an example for a simple usage in /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def bwdInitGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        ## Please feel free to add an example for a simple usage in /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onMouseButtonRight(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/raffa/work/sofaDevelopIP/applications/plugins/SofaPython/scn2python.py
        return 0;


def createScene(rootNode):
    rootNode.findData('dt').value = '0.001'
    rootNode.findData('gravity').value = '0 0 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    myrealXYZ_s_inf = realXYZ_s_inf(rootNode,commandLineArguments)
    return 0;