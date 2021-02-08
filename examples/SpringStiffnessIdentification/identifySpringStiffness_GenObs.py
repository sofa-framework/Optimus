import Sofa
import sys
import os

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='Deformable', pluginName='SofaDeformable')
    rootNode.addObject('RequiredPlugin', name='Engine', pluginName='SofaEngine')
    rootNode.addObject('RequiredPlugin', name='GeneralEngine', pluginName='SofaGeneralEngine')
    rootNode.addObject('RequiredPlugin', name='ImplicitOdeSolver', pluginName='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='Loader', pluginName='SofaLoader')
    rootNode.addObject('RequiredPlugin', name='MiscForceField', pluginName='SofaMiscForceField')
    rootNode.addObject('RequiredPlugin', name='Rigid', pluginName='SofaRigid')
    rootNode.addObject('RequiredPlugin', name='SimpleFem', pluginName='SofaSimpleFem')
    rootNode.addObject('RequiredPlugin', name='MiscFem', pluginName='SofaMiscFem')
    rootNode.addObject('RequiredPlugin', name='MeshCollision', pluginName='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='BoundaryCondition', pluginName='SofaBoundaryCondition')
    rootNode.addObject('RequiredPlugin', name='Visual', pluginName='SofaOpenglVisual')
    # rootNode.addObject('RequiredPlugin', name='Python3', pluginName='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='Optimus', pluginName='Optimus')
    rootNode.addObject(SyntheticGenObs_Controller(name="Synthetic_GenObs", node=rootNode))

    return 0




class SyntheticGenObs_Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        ### These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rootNode = kwargs["node"]

        ### configuration
        volumeFileName = '../../data/brickD/brickD_536.vtk'
        surfaceSTL = '../../data/brickD/brickD_536.stl'
        outputDir = 'observations'
        saveObservations = 1
        self.solver = 'Newton' # options are 'Newton' and 'Euler'
        self.linearSolver = 'CG' # options are 'Pardiso' and 'CG'

        if saveObservations:
            os.system('mv '+outputDir+ ' observations/arch')
            os.system('mkdir -p '+outputDir)

        if self.linearSolver == 'Pardiso':
            self.rootNode.addObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')

        ### rootNode
        self.rootNode.addObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping showWireframe hideNormals')

        self.rootNode.findData('gravity').value = [0.0, 0.0, 0.0]
        self.rootNode.findData('dt').value = 1.0

        ### tool node
        tool = self.rootNode.addChild('tool')
        tool.addObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="3")
        tool.addObject('CGLinearSolver', iterations="100", tolerance="1e-20", threshold="1e-20")
        tool.addObject('PointSetTopologyContainer', name='pointTopo', position='0.045 0.1 0.0   0.05 0.1 0.0   0.055 0.1 0.0   0.045 0.1 -0.005   0.05 0.1 -0.005   0.055 0.1 -0.005    0.045 0.1 -0.01   0.05 0.1 -0.01   0.055 0.1 -0.01')
        tool.addObject('MechanicalObject', name='MO', position='@pointTopo.position')
        tool.addObject('SphereCollisionModel', color='0 0 1 1', radius='0.0014')
        tool.addObject('LinearMovementConstraint', indices='0 1 2 3 4 5 6 7 8', keyTimes='0 200', movements='0 0 0    0.0 0.04 0')

        ### save tool observations
        if saveObservations:
            tool.addObject('VTKExporter', position="@MO.position", edges="0", listening="0" , XMLformat='0', exportAtBegin='1', exportEveryNumberOfSteps="0", filename=outputDir+'/tool.vtk')
            tool.addObject('BoxROI', name='toolDOFs', box='-1 -1 -1 1 1 1')
            tool.addObject('OptimMonitor', name='toolMonitor', fileName=outputDir+'/tool', showPositions='1', indices="@toolDOFs.indices", ExportPositions="1", ExportVelocities="1", ExportForces="1")

        ### object node
        simuNode = self.rootNode.addChild('simu')

        ### solvers
        if self.solver == 'Euler':
            simuNode.addObject('EulerImplicitSolver', rayleighStiffness='0.1', rayleighMass='0.1')
        elif self.solver == 'Newton':
            simuNode.addObject('StaticSolver', name="NewtonStatic", newton_iterations='3', correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", printLog='0')
        else:
            print('Unknown solver type')

        if self.linearSolver == 'CG':
            simuNode.addObject('CGLinearSolver', iterations="100", tolerance="1e-20", threshold="1e-20")
            # simuNode.addObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        elif self.linearSolver == 'Pardiso':
            simuNode.addObject('SparsePARDISOSolver', symmetric='1', exportDataToFolder='', name='precond', iterativeSolverNumbering='1')
        else:
            print('Unknown linear solver type')

        ### mechanical object
        simuNode.addObject('MeshVTKLoader', name='loader', filename=volumeFileName)
        simuNode.addObject('MechanicalObject', src='@loader', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='0')
        simuNode.addObject('TetrahedronSetTopologyContainer', name='Container', src="@loader")
        simuNode.addObject('TetrahedronSetTopologyModifier', name='Modifier')
        simuNode.addObject('TetrahedronSetGeometryAlgorithms', name='GeomAlgs')
        simuNode.addObject('UniformMass', totalMass='0.01')

        ### material and elasticiy properties
        nu=0.45
        E=5000
        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        materialParams='{} {}'.format(mu,lamb)
        simuNode.addObject('TetrahedronHyperelasticityFEMForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        # simuNode.addObject('TetrahedronFEMForceField', name='FEM', youngModulus=E, poissonRatio=nu, computeVonMisesStress='0', method='large')
        # simuNode.addObject('FastTetrahedralCorotationalForceField', name='FEM', youngModulus=E, poissonRatio=nu)

        simuNode.addObject('BoxROI', box='-0.001 -0.001 -0.011 0.025 0.001 0.001', drawBoxes='0', name='FROI1')
        simuNode.addObject('RestShapeSpringsForceField', stiffness='1e5', name='fixingSpring', points='@FROI1.indices', springColor='0 1 0 1')
        if self.linearSolver == 'Pardiso':
            simuNode.addObject('Mapped3DoFForceField', mappedFEM='mappedTool/toolSpring', mappedMechObject='mappedTool/MO', printLog='0', mapping='mappedTool/baryMapping')
        simuNode.addObject('PointsFromIndices', template='Vec3d', name='FixedPoints', indices='@FROI1.indices')

        if saveObservations:
            simuNode.addObject('VTKExporter', position="@MO.position", listening="1" , XMLformat='0', exportAtBegin="1", exportAtEnd='0', exportEveryNumberOfSteps="0", filename=outputDir+'/object.vtk', tetras='1', edges='0')

        ### mapped tool node
        mappedTool = simuNode.addChild('mappedTool')
        mappedTool.addObject('MechanicalObject', position='0.045 0.1 0.0   0.05 0.1 0.0   0.055 0.1 0.0   0.045 0.1 -0.005   0.05 0.1 -0.005   0.055 0.1 -0.005    0.045 0.1 -0.01   0.05 0.1 -0.01   0.055 0.1 -0.01', name='MO')
        mappedTool.addObject('RestShapeSpringsForceField', stiffness='1e5', name='toolSpring', springColor='0 1 0 1', drawSpring='1', printLog='0', listening='1', external_rest_shape='@/tool/MO')
        mappedTool.addObject('SphereCollisionModel', color='0 0 1 1', radius='0.001')
        mappedTool.addObject('BarycentricMapping', name='baryMapping')

        ### observation nodes
        obsGrid = simuNode.addChild('obsGrid')
        monitorFile=outputDir+'/observations'
        obsGrid.addObject('RegularGridTopology', name="grid", min='0.0 0.08 0.0', max='0.1 0.1 -0.0', n='10 3 1')  # obs. grid
        obsGrid.addObject('MechanicalObject', src='@grid', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='1')
        obsGrid.addObject('BarycentricMapping')
        obsGrid.addObject('SphereCollisionModel', color='0.0 0.5 0.0 1', radius="0.0014", template='Vec3d')

        if saveObservations:
            obsGrid.addObject('VTKExporter', filename=monitorFile+'.vtk', position="@MO.position", listening="0" , XMLformat='0', exportAtBegin='1', exportEveryNumberOfSteps="0")
            obsGrid.addObject('BoxROI', name='gridDOFs', box='-1 -1 -1 1 1 1')
            obsGrid.addObject('OptimMonitor', name='observationMonitor', fileName=monitorFile, indices="@gridDOFs.indices", ExportPositions="1", ExportVelocities="0", ExportForces="0")

        ### object visualization
        visNode = simuNode.addChild('ObjectVisualization1')
        visNode.addObject('MeshSTLLoader', name='objectSLoader', filename=surfaceSTL)
        visNode.addObject('MechanicalObject', src="@objectSLoader", name="Surface")
        visNode.addObject('TriangleSetTopologyContainer', name="Container", src="@objectSLoader", tags=" ")
        visNode.addObject('LineCollisionModel', color="0 0 0 1")
        visNode.addObject('TriangleCollisionModel', color="1 0 0 1")
        visNode.addObject('BarycentricMapping')

        visNode2 = simuNode.addChild('ObjectVisualization2')
        visNode2.addObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping hideWireframe hideNormals')
        visNode2.addObject('MeshSTLLoader', name='objectSLoader', filename=surfaceSTL)
        visNode2.addObject('MechanicalObject',src="@objectSLoader",name="Surface")
        visNode2.addObject('TriangleSetTopologyContainer', name="Container", src="@objectSLoader", tags=" ")
        visNode2.addObject('TriangleCollisionModel', color="1 0 0 0.2")
        visNode2.addObject('BarycentricMapping')

        ### verification node
        asNode = simuNode.addChild('assessNode')
        asNode.addObject('RegularGridTopology', name="grid", min='0.01 0.005 -0.005', max='0.09 0.07 -0.005', n='8 5 1')  # obs. grid
        asNode.addObject('MechanicalObject', src='@grid', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='1')
        asNode.addObject('SphereCollisionModel', color='1 0 1 1', radius="0.001", template='Vec3d')
        asNode.addObject('BarycentricMapping')

        return None;



    def onAnimateEndEvent(self, deltaTime):
        return 0;

    def onKeyPressed(self, c):
        return 0;

    def onKeyReleased(self, c):
        return 0;

    def onLoaded(self, node):
        return 0;

    def onMouseButtonLeft(self, mouseX, mouseY, isPressed):
        return 0;

    def onMouseButtonRight(self, mouseX, mouseY, isPressed):
        return 0;

    def onMouseButtonMiddle(self, mouseX, mouseY, isPressed):
        return 0;

    def onMouseWheel(self, mouseX, mouseY, wheelDelta):
        return 0;

    def onGUIEvent(self, strControlID, valueName, strValue):
        return 0;

    def onAnimateBeginEvent(self, deltaTime):
        return 0;

    def onScriptEvent(self, senderNode, eventName, data):
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

