import Sofa

class pigLiver1_knownBC (Sofa.PythonScriptController):

    def createGraph(self,rootNode):
        nu=0.45
        E=0.1
        rootNode.findData('dt').value="1"
        fixedIX=[56, 57, 78, 81, 82, 83, 85, 86, 95, 96, 98, 124, 125, 126, 127, 133, 135, 141, 142, 143, 144, 145, 153, 154, 158, 159, 183, 185, 187, 188, 209, 212, 213, 221, 232]
        videoScaleFactor='0.000342'
        featuresScaleFactor=videoScaleFactor+' '+videoScaleFactor+' '+videoScaleFactor
        meshRotation='-15 180 191'
        meshTranslation='0.156 0.073 0.002'
        meshScale='1.05 1 1'
        displayFeatures='1';

        # rootNode
        rootNode.createObject('RequiredPlugin', pluginName='OpticalFlow', name='OpticalFlow plugin')
        rootNode.createObject('RequiredPlugin', pluginName='Optimus')
        rootNode.createObject('RequiredPlugin', pluginName='SofaPardisoSolver')
        rootNode.createObject('RequiredPlugin', pluginName='ImageMeshAux')
        rootNode.createObject('RequiredPlugin', pluginName='SofaMJEDFEM')
        rootNode.createObject('VisualStyle', displayFlags='hideVisualModels showBehaviorModels showCollisionModels hideMappings hideForceFields')
        # rootNode.createObject('ViewerSetting', resolution='800 450')
        rootNode.createObject('DefaultAnimationLoop')

        # rootNode/features
        features = rootNode.createChild('features')
        features.createObject('LKOpticalFlowTrackerSimple', maskName='../../Data/pigLiver/porcineLiverMask.png', vidName='../../Data/pigLiver/porcineLiverCut1.avi', name='LK', winSize='31', detectorThresh='190', scaleImg=videoScaleFactor, displayFeatures=displayFeatures, view='1')
        features.createObject('TransformEngine',name='scaleFeatures',input_position='@LK.outputFeatures',scale=featuresScaleFactor)
        features.createObject('MechanicalObject', position='@scaleFeatures.output_position', name='DOFs')
        features.createObject('Sphere', color='0.0 0.0 1.0 1', radius='0.001', template='Vec3d')
        # #original video start: features.createObject('BoxROI', name="ToolIx", box='480 160 -10 580 180 10', drawBoxes='1')
        # features.createObject('BoxROI', name="ToolIx", box='520 140 -10 620 160 10', drawBoxes='0')
        # features.createObject('PointsFromIndices', name='ToolPoints', indices='@ToolIx.indices')
        # #original video start: features.createObject('BoxROI', name="ObsIx", box='320 120 -10 370 160 10   220 200 -10 280 280 10    510 260 -10   550 300  10', drawBoxes='1')
        # features.createObject('BoxROI', name="ObsIx", box='320 120 -10 390 160 10   280 200 -10 320 280 10    510 260 -10   550 280  10', drawBoxes='0')
        # features.createObject('PointsFromIndices', name='ObsPoints', indices='@ObsIx.indices')

        # features.createObject('BoxROI', name="LandmarkIX", box='330 320 -10  350 350 10', drawBoxes='0')
        # features.createObject('PointsFromIndices', template='Vec3d', name='LandPoints', indices='@LandmarkIX.indices')

        # toolNode = rootNode.createChild('tool')
        # toolNode.createObject('MechanicalObject', scale3d='1 1 1', position='@../features/ToolPoints.indices_position', name='MO')
        # toolNode.createObject('Sphere', color='1.0 0.0 1.0 1', radius='5', template='Vec3d')

        # obsNode = rootNode.createChild('obs')
        # obsNode.createObject('MechanicalObject', scale3d='1 1 1', position='@../features/ObsPoints.indices_position', name='MO')
        # obsNode.createObject('Sphere', color='1.0 1.0 0.0 1', radius='5', template='Vec3d')

        # landNode = rootNode.createChild('land')
        # landNode.createObject('MechanicalObject', scale3d='1 1 1', position='@/features/LandPoints.indices_position', name='MO')
        # landNode.createObject('Sphere', color='1.0 0.0 0.8 1', radius='5', template='Vec3d')



        rootNode.createObject('NewtonStaticSolver', maxIt='10', name='NewtonStatic', correctionTolerance='1e-8', convergeOnResidual='1', residualTolerance='1e-8', printLog='1')
        rootNode.createObject('SparsePARDISOSolver', symmetric='1', exportDataToDir='', name='precond', iterativeSolverNumbering='1')            
        rootNode.createObject('MeshVTKLoader', name='meshLoader', filename='../../Data/pigLiver/liverLobe0_992.vtk', scale3d=meshScale, translation=meshTranslation, rotation=meshRotation)
        rootNode.createObject('TetrahedronSetTopologyContainer', src='@meshLoader', name='Container')
        rootNode.createObject('TetrahedronSetTopologyModifier', name='Modifier')
        rootNode.createObject('TetrahedronSetTopologyAlgorithms', name='TopoAlgo')
        rootNode.createObject('TetrahedronSetGeometryAlgorithms', name='GeomAlgo')
        rootNode.createObject('MechanicalObject', template='Vec3d', name='LiverMO')        
        # rootNode.createObject('BoxROI', name="FixedBox", box='100 400 -100 700 500 100', drawBoxes='0')
        rootNode.createObject('FixedConstraint', name="FixedBC", indices=fixedIX)

        # rootNode.createObject('Gravity', gravity="0 9.81 0")
        # rootNode.createObject('UniformMass', totalmass='2000000')

        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        materialParams='{} {}'.format(mu,lamb)
        rootNode.createObject('MJEDTetrahedralForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)

        rootNode.createObject('Mapped3DoFForceField', mappedFEM='mappedTool/toolSpring', mappedMechObject='mappedTool/MO', printLog='0', mapping='mappedTool/baryMapping')
        mappedTool = rootNode.createChild('mappedTool')
        mappedTool.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@../features/ToolPoints.indices_position', name='MO')
        mappedTool.createObject('Sphere', color='0.0 1.0 1.0 1', radius='0.001', template='Vec3d')
        self.toolSprings=mappedTool.createObject('ExtendedRestShapeSpringForceField', external_rest_shape='/tool/MO', numStepsSpringOn='10000', stiffness='1e5', name='toolSpring', springColor='0 1 0 1', drawSpring='1', updateStiffness='1', printLog='0', listening='1', angularStiffness='0', startTimeSpringOn='0')
        mappedTool.createObject('BarycentricMapping', name="baryMapping")

        mappedObs = rootNode.createChild('mappedObs')
        mappedObs.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@../features/ObsPoints.indices_position', name='MO')
        # mappedObs.createObject('Sphere', color='0.8 0.5 1.0 1', radius='3', template='Vec3d')
        mappedObs.createObject('BarycentricMapping', name="baryMapping")

        landNode = rootNode.createChild('landNode')
        landNode.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@/features/LandPoints.indices_position', name='MO')
        landNode.createObject('Sphere', color='0.5 0.6 1.0 1', radius='0.001', template='Vec3d')
        landNode.createObject('BarycentricMapping', name="baryMapping")

        
        visNode = rootNode.createChild('visNode')
        visNode.createObject('VisualStyle', displayFlags='hideVisualModels showBehaviorModels showCollisionModels hideMappings hideForceFields showWireframe')
        visNode.createObject('MeshSTLLoader', name='objectSLoader', filename='../../Data/pigLiver/liverLobe0_3914.stl', scale3d=meshScale, translation=meshTranslation, rotation=meshRotation)
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
