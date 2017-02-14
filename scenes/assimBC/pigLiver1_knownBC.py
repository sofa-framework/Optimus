import Sofa

class pigLiver1_knownBC (Sofa.PythonScriptController):

    def createGraph(self,rootNode):
        nu=0.45
        E=0.1
        rootNode.findData('dt').value="1"

        # rootNode
        rootNode.createObject('RequiredPlugin', pluginName='OpticalFlow', name='OpticalFlow plugin')
        rootNode.createObject('RequiredPlugin', pluginName='Optimus')
        rootNode.createObject('RequiredPlugin', pluginName='SofaPardisoSolver')
        rootNode.createObject('RequiredPlugin', pluginName='ImageMeshAux')
        rootNode.createObject('RequiredPlugin', pluginName='SofaMJEDFEM')
        rootNode.createObject('VisualStyle', displayFlags='hideVisualModels showBehaviorModels showCollisionModels hideMappings hideForceFields')
        rootNode.createObject('ViewerSetting', resolution='800 450')
        rootNode.createObject('DefaultAnimationLoop')

        # rootNode/Vision
        Vision = rootNode.createChild('Vision')
        Vision.createObject('LKOpticalFlowTrackerSimple', maskName='../../Data/pigLiver/porcineLiverMask.png', vidName='../../Data/pigLiver/porcineLiverCut1.avi', name='LK', winSize='31', detectorThresh='190', scaleImg='1', displayFeatures='1', view='1')
        Vision.createObject('MechanicalObject', scale3d='1 1 1', position='@LK.outputFeatures', name='DOFs')
        # Vision.createObject('Sphere', color='0.0 0.0 1.0 1', radius='5', template='Vec3d')
        #original video start: Vision.createObject('BoxROI', name="ToolIx", box='480 160 -10 580 180 10', drawBoxes='1')
        Vision.createObject('BoxROI', name="ToolIx", box='520 140 -10 620 160 10', drawBoxes='1')
        Vision.createObject('PointsFromIndices', name='ToolPoints', indices='@ToolIx.indices')
        #original video start: Vision.createObject('BoxROI', name="ObsIx", box='320 120 -10 370 160 10   220 200 -10 280 280 10    510 260 -10   550 300  10', drawBoxes='1')
        Vision.createObject('BoxROI', name="ObsIx", box='320 120 -10 390 160 10   280 200 -10 320 280 10    510 260 -10   550 280  10    330 320 -10  350 340 10', drawBoxes='1')
        Vision.createObject('PointsFromIndices', name='ObsPoints', indices='@ObsIx.indices')

        ToolNode = rootNode.createChild('tool')
        ToolNode.createObject('MechanicalObject', scale3d='1 1 1', position='@../Vision/ToolPoints.indices_position', name='MO')
        ToolNode.createObject('Sphere', color='1.0 0.0 1.0 1', radius='5', template='Vec3d')

        ToolNode = rootNode.createChild('obs')
        ToolNode.createObject('MechanicalObject', scale3d='1 1 1', position='@../Vision/ObsPoints.indices_position', name='MO')
        ToolNode.createObject('Sphere', color='1.0 1.0 0.0 1', radius='5', template='Vec3d')


        rootNode.createObject('NewtonStaticSolver', maxIt='10', name='NewtonStatic', correctionTolerance='1e-8', convergeOnResidual='1', residualTolerance='1e-8', printLog='1')
        rootNode.createObject('SparsePARDISOSolver', symmetric='1', exportDataToDir='', name='precond', iterativeSolverNumbering='1')            
        rootNode.createObject('MeshVTKLoader', name='meshLoader', filename='../../Data/pigLiver/liverLobe0_598.vtk', scale3d='2900 2900 2900', translation='450 215 0', rotation='-15 180 188')
        rootNode.createObject('TetrahedronSetTopologyContainer', src='@meshLoader', name='Container')
        rootNode.createObject('TetrahedronSetTopologyModifier', name='Modifier')
        rootNode.createObject('TetrahedronSetTopologyAlgorithms', name='TopoAlgo')
        rootNode.createObject('TetrahedronSetGeometryAlgorithms', name='GeomAlgo')
        rootNode.createObject('MechanicalObject', template='Vec3d', name='LiverMO')        
        rootNode.createObject('BoxROI', name="FixedBox", box='100 400 -100 700 500 100', drawBoxes='1')
        rootNode.createObject('FixedConstraint', name="FixedBC", indices="@FixedBox.indices")

        # rootNode.createObject('Gravity', gravity="0 9.81 0")
        # rootNode.createObject('UniformMass', totalmass='2000000')

        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        materialParams='{} {}'.format(mu,lamb)
        rootNode.createObject('MJEDTetrahedralForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)

        rootNode.createObject('Mapped3DoFForceField', mappedFEM='mappedTool/toolSpring', mappedMechObject='mappedTool/MO', printLog='0', mapping='mappedTool/baryMapping')
        mappedTool = rootNode.createChild('mappedTool')
        mappedTool.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@../Vision/ToolPoints.indices_position', name='MO')
        mappedTool.createObject('Sphere', color='0.0 1.0 1.0 1', radius='5', template='Vec3d')
        self.toolSprings=mappedTool.createObject('ExtendedRestShapeSpringForceField', external_rest_shape='/tool/MO', numStepsSpringOn='10000', stiffness='1e5', name='toolSpring', springColor='0 1 0 1', drawSpring='1', updateStiffness='1', printLog='0', listening='1', angularStiffness='0', startTimeSpringOn='0')
        mappedTool.createObject('BarycentricMapping', name="baryMapping")

        mappedObs = rootNode.createChild('mappedObs')
        mappedObs.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@../Vision/ObsPoints.indices_position', name='MO')
        mappedObs.createObject('Sphere', color='0.8 0.5 1.0 1', radius='3', template='Vec3d')
        mappedObs.createObject('BarycentricMapping', name="baryMapping")
        
        visNode = rootNode.createChild('visNode')
        visNode.createObject('VisualStyle', displayFlags='hideVisualModels showBehaviorModels showCollisionModels hideMappings hideForceFields showWireframe')
        visNode.createObject('MeshSTLLoader', name='objectSLoader', filename='../../Data/pigLiver/liverLobe0_3914.stl', scale3d='2900 2900 2900', translation='450 215 0', rotation='-15 180 188')
        visNode.createObject('MechanicalObject',src="@objectSLoader",name="Surface")        
        visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@objectSLoader")
        # visNode.createObject('Line',color="0 0 0 1")
        visNode.createObject('Triangle',color="1 1 1 1")        
        visNode.createObject('BarycentricMapping')


        # # rootNode/VisualLiver
        # VisualLiver = rootNode.createChild('VisualLiver')
        # VisualLiver.activated = '1'

        # VisualLiver.createObject('OglModel', scale3d='2900 2900 2900', rotation='-15 180 180', name='VisualModel', color='0.8 0.8 0.8 0.8', translation='400 225 0', fileMesh='Liver_Lobe.obj', depthTest='0')
        # VisualLiver.createObject('BarycentricMapping', input='@../LiverMO', name='visual mapping', output='@VisualModel')

        return 0;

    def onKeyPressed(self, c):
        return 0;

    def onKeyReleased(self, c):
        return 0;

    def onLoaded(self, node):
        return 0;

    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        return 0;

    def onMouseButtonRight(self, mouseX,mouseY,isPressed):
        return 0;

    def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
        return 0;

    def onMouseWheel(self, mouseX,mouseY,wheelDelta):
        return 0;

    def onGUIEvent(self, strControlID,valueName,strValue):
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        return 0;

    def onEndAnimationStep(self, deltaTime):
        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        return 0;

    def initGraph(self, node):
        return 0;

    def bwdInitGraph(self, node):
        return 0;

    def storeResetState(self):
        return 0;

    def reset(self):
        return 0;

    def cleanup(self):
        return 0;
