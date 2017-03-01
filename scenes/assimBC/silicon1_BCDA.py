import Sofa
import os

class silicon1_BCDA (Sofa.PythonScriptController):


    def createGraph(self,rootNode):
        self.showVideo=1
        self.saveState=1
        self.statParams=[0, 5, 1e-5]
        self.nu=0.45
        self.E=1e5
        self.material="SV"


        self.suffix='psd'+str(self.statParams[1])+'#osd'+str(self.statParams[2])+'#mat'+self.material

        os.system('mkdir -p outSilicon')
        if self.saveState:
            self.stateExpFile='outSilicon/state_'+self.suffix+'.txt'
            self.stateVarFile='outSilicon/variance_'+self.suffix+'.txt'
            self.stateCovarFile='outSilicon/covariance_'+self.suffix+'.txt'
            os.system('rm '+self.stateExpFile)
            os.system('rm '+self.stateVarFile)
            os.system('rm '+self.stateCovarFile)

        self.createScene(rootNode)
        return 0


    def createScene(self,rootNode):
        # rootNode
        rootNode.createObject('RequiredPlugin', pluginName='Optimus')
        rootNode.createObject('RequiredPlugin', pluginName='SofaPardisoSolver')
        rootNode.createObject('RequiredPlugin', pluginName='ImageMeshAux')
        rootNode.createObject('RequiredPlugin', pluginName='image')
        rootNode.createObject('RequiredPlugin', pluginName='SofaMJEDFEM')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels hideForceFields showCollisionModels hideWireframe')
        rootNode.createObject('MeshVTKLoader', name='objectLoader', filename='../../data/brickC/brickC_1216.vtk')
        rootNode.createObject('MeshSTLLoader', name='objectSLoader', filename='../../data/brickC/brickC_1216.stl')
        rootNode.createObject('MeshVTKLoader', name='obsLoader', filename='../obsBoundaryDA/brickC_150316/obsRestA.vtk')
        rootNode.createObject('MeshVTKLoader', name='toolLoader', filename='../obsBoundaryDA/brickC_150316/toolRestA.vtk')
        rootNode.createObject('FilteringAnimationLoop', name='StochAnimLoop', verbose='1')
        self.filter=rootNode.createObject('ROUKFilter', name='ROUKF', verbose='1')

        if self.showVideo:            
            video = rootNode.createChild('video')
            video.createObject('PreStochasticWrapper', name='PreStochasticWrap', verbose='1')
            video.createObject('ImageContainer', filename='../../data/videoSilicon1/testMirr_0-12.avi', transform='0.205 0.105 0 0 0 178 0.000219 0.000219 15 0 0.01 1', name='image')
            video.createObject('ImageViewer', src='@image', enableHistogram='0', name='viewer')

        # rootNode/model
        model = rootNode.createChild('model')
        model.createObject('StochasticStateWrapper', name='StateWrapper', verbose='1')
        model.createObject('OptimParams', initValue=self.statParams[0],  stdev=self.statParams[1], transformParams='1', name='springStiffness', numParams='53', printLog='1', template='Vector', optimize='1')
        model.createObject('NewtonStaticSolver', maxIt='1', name='NewtonStatic', correctionTolerance='1e-8', convergeOnResidual='1', residualTolerance='1e-8', printLog='1')
        model.createObject('StepPCGLinearSolver', preconditioners='precond', precondOnTimeStep='1', name='StepPCG', iterations='10000', tolerance='1e-12', verbose='1')
        model.createObject('SparsePARDISOSolver', symmetric='1', exportDataToDir='', name='precond', iterativeSolverNumbering='0')
        model.createObject('MechanicalObject', src='@/objectLoader', name='Volume')        
        model.createObject('TetrahedronSetTopologyContainer', name='Container', src="@/objectLoader")
        model.createObject('TetrahedronSetTopologyModifier', name='Modifier')
        model.createObject('TetrahedronSetTopologyAlgorithms', name='TopoAlgs')
        model.createObject('TetrahedronSetGeometryAlgorithms', name='GeomAlgs')
        model.createObject('UniformMass', totalMass='0.2513')
        model.createObject('BoxROI', box='-0.001 0.052 -0.001   0.1631 0.054 0.0131', drawBoxes='0', name='fixedBox1')
        model.createObject('ExtendedRestShapeSpringForceField', stiffness='@springStiffness.value', pointSize='2', name='fixedSpring', showIndicesScale='0', forceDir='', springThickness='3', updateStiffness='1', points='@fixedBox1.indices', printLog='0', listening='1')
        model.createObject('ColorMap', colorScheme='Blue to Red')        

        E=self.E;
        nu=self.nu;        

        if self.material=='SV':        
            lamb=(E*nu)/((1+nu)*(1-2*nu))
            mu=E/(2+2*nu)
            materialParams='{} {}'.format(mu,lamb)
            model.createObject('MJEDTetrahedralForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        elif self.material=='CR':
            model.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus=E, poissonRatio=nu, method="large")

        # rootNode/model/obsNode
        obsNode = model.createChild('obsNode')
        obsNode.activated = '1'
        obsNode.createObject('MechanicalObject', src='@/obsLoader')
        obsNode.createObject('Sphere', color='0.0 0.5 0.0 1', radius='0.0014')
        obsNode.createObject('BarycentricMapping')
        obsNode.createObject('MappedStateObservationManager', stateWrapper='@../StateWrapper', verbose='1', name='MOBS', listening='1', observationStdev=self.statParams[2], noiseStdev='0.0', doNotMapObservations='1')
        obsNode.createObject('SimulatedStateObservationSource', drawSize='0.000', monitorPrefix='../obsBoundaryDA/brickC_150316/obsA', name='ObsSource', printLog='1')

        # rootNode/model/toolTarget
        toolTarget = model.createChild('toolTarget')
        toolTarget.createObject('MechanicalObject', src='@/toolLoader', name='MO')
        toolTarget.createObject('Sphere', color='0.0 0.0 1.0 1.0', radius='0.0014')
        toolTarget.createObject('SimulatedStateObservationSource', drawSize='0.000', monitorPrefix='../obsBoundaryDA/brickC_150316/toolA', printLog='1', name='ToolA', controllerMode='1')
        model.createObject('Mapped3DoFForceField', mappedFEM='tool/toolSpring', mappedMechObject='tool/MO', printLog='0', mapping='tool/baryMapping')

        # rootNode/model/tool
        tool = model.createChild('tool')
        tool.createObject('MechanicalObject', src='@/toolLoader', name='MO')
        tool.createObject('ExtendedRestShapeSpringForceField', numStepsSpringOn='10000', stiffness='1e5', name='toolSpring', springColor='0 1 0 1', showIndicesScale='0', springThickness='1', updateStiffness='1', listening='1', startTimeSpringOn='0', external_rest_shape='../toolTarget/MO')
        tool.createObject('BarycentricMapping', name='baryMapping')

        # rootNode/model/visualization
        visualization = model.createChild('visualization')
        visualization.activated = '1'
        visualization.createObject('VisualStyle', displayFlags='showBehaviorModels hideForceFields showCollisionModels showWireframe')
        visualization.createObject('MechanicalObject', src='@/objectSLoader', name='Surface')
        visualization.createObject('TriangleSetTopologyContainer', name='Container', src="@/objectSLoader")
        visualization.createObject('TriangleSetTopologyModifier', name='Modifier')
        visualization.createObject('TriangleSetTopologyAlgorithms', name='TopoAlgs')
        visualization.createObject('TriangleSetGeometryAlgorithms', name='GeomAlgs')                  
        visualization.createObject('Line', color='0 0 0 1')
        visualization.createObject('Triangle', color='1 0 0 1')
        visualization.createObject('BarycentricMapping')

        # rootNode/model/visualization2
        # visualization2 = model.createChild('visualization2')
        # visualization2.activated = '1'
        # visualization2.createObject('MechanicalObject', src='@/objectSLoader', name='Surface')
        # visualization2.createObject('TriangleSetTopologyContainer', name='Container', src="@/objectSLoader")
        # visualization2.createObject('TriangleSetTopologyModifier', name='Modifier')
        # visualization2.createObject('TriangleSetTopologyAlgorithms', name='TopoAlgs')
        # visualization2.createObject('TriangleSetGeometryAlgorithms', name='GeomAlgs')                  
        # visualization2.createObject('include', src='@/objectSLoader', href='Objects/TriangleSetTopology.xml')
        # visualization2.createObject('Line', color='0 0 0 1')
        # visualization2.createObject('Triangle', color='1 0 0 0.18')
        # visualization2.createObject('BarycentricMapping')

        return 0;

    def onEndAnimationStep(self, deltaTime):
        if self.saveState:              
            rs=self.filter.findData('reducedState').value
            reducedState = [val for sublist in rs for val in sublist]
            #print 'Reduced state:'
            #print reducedState

            f1 = open(self.stateExpFile, "a")        
            f1.write(" ".join(map(lambda x: str(x), reducedState)))
            f1.write('\n')
            f1.close()    
                    
            rv=self.filter.findData('reducedVariance').value
            reducedVariance = [val for sublist in rv for val in sublist]
            #print 'Reduced variance:'
            #print reducedVariance

            f2 = open(self.stateVarFile, "a")        
            f2.write(" ".join(map(lambda x: str(x), reducedVariance)))
            f2.write('\n')
            f2.close()

            rcv=self.filter.findData('reducedCovariance').value
            reducedCovariance = [val for sublist in rcv for val in sublist]
            #print 'Reduced Covariance:'
            #print reducedCovariance

            f3 = open(self.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), reducedCovariance)))
            f3.write('\n')
            f3.close()
            
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
