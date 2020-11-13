import Sofa
import os
import time

__file = __file__.replace('\\', '/') # windows



def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='Exporter', pluginName='SofaExporter')
    rootNode.createObject('RequiredPlugin', name='Visual', pluginName='SofaOpenglVisual')
    rootNode.createObject('RequiredPlugin', name='Python', pluginName='SofaPython')
    rootNode.createObject('RequiredPlugin', pluginName='Optimus')

    rootNode.createObject('PythonScriptController', name='GenerateObservations', filename=__file, classname='synth1_GenObs')




class synth1_GenObs(Sofa.PythonScriptController):

    def createGraph(self, rootNode):
        ### configuration
        nu=0.45
        E=5000

        volumeFileName='../../data/brickD/brickD_536.vtk'
        surfaceSTL='../../data/brickD/brickD_536.stl'
        self.geometry = 'brickD'
        self.fixedPoints = 'L1'          #L1, L4
        self.fixingMethod = 'proj'       #proj, penal

        # self.obsPoints = 'GT3x10'    # observations in a grid 3x10 top part of the object
        # self.obsPoints = 'GC2x2'    # observations in a grid 2x2, corners
        self.obsPoints = 'PLB'  	# single point of observation, left bottom

        #self.integration = 'Euler'
        #self.numIter = 1

        self.integration = 'Newton'
        self.numIter = 3

        # self.integration = 'VarSym'
        # self.numIter = 3

        self.linearSolver = 'CG' # options are 'Pardiso' and 'CG'
        self.toolTrajectory = 1
        self.saveToolForces = 1
        self.saveObservations = 1

        rootDir=self.geometry+'_fix-'+self.fixedPoints+self.fixingMethod+'_obs-'+self.obsPoints+'_int-'+self.integration+str(self.numIter)+'_TR'+str(self.toolTrajectory)
        outputDir = rootDir+'/observations'

        if self.saveObservations:
            os.system('mkdir -p arch')
            stamp='_'+str(int(time.time()))
            os.system('mv --backup=t -S '+stamp+' '+rootDir+' arch')
            os.system('mkdir -p '+outputDir)

        if self.saveToolForces:
            self.toolForceFile=outputDir+'/toolForce.txt'

        ### rootNode
        rootNode.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping showWireframe hideNormals')

        rootNode.findData('gravity').value = "0 0 0"
        rootNode.findData('dt').value = "1"

        ### tool node
        tool = rootNode.createChild('tool')
        tool.createObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="3")
        tool.createObject('CGLinearSolver', iterations="100", tolerance="1e-20", threshold="1e-20")
        tool.createObject('PointSetTopologyContainer', name='pointTopo', position='0.045 0.1 0.0   0.05 0.1 0.0   0.055 0.1 0.0   0.045 0.1 -0.005   0.05 0.1 -0.005   0.055 0.1 -0.005    0.045 0.1 -0.01   0.05 0.1 -0.01   0.055 0.1 -0.01')
        tool.createObject('MechanicalObject', name='MO', position='@pointTopo.position')
        tool.createObject('SphereCollisionModel', color='0 0 1 1', radius='0.0014')
        if self.toolTrajectory == 1:
            tool.createObject('LinearMovementConstraint', indices='0 1 2 3 4 5 6 7 8', keyTimes='0 200', movements='0 0 0    0.0 0.04 0')

        ### save tool observations
        if self.saveObservations:
            tool.createObject('VTKExporter', name='toolExporter', position="@MO.position", edges="1", listening="0" , XMLformat='0', exportAtBegin='1', exportEveryNumberOfSteps="0", filename=outputDir+'/tool.vtk')
            tool.createObject('BoxROI', name='toolDOFs', box='-1 -1 -1 1 1 1')
            tool.createObject('OptimMonitor', name='toolMonitor', fileName=outputDir+'/tool', showPositions='1', indices="@toolDOFs.indices", ExportPositions="1", ExportVelocities="0", ExportForces="0")


        ### object node
        simuNode = rootNode.createChild('simu')
        simuNode.activated = 'true'

        ### solvers
        if self.integration == 'Euler':
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness='0.1', rayleighMass='0.1')
        elif self.integration == 'Newton':
            simuNode.createObject('StaticSolver', name="NewtonStatic", printLog="0", correction_tolerance_threshold="1e-8", residual_tolerance_threshold="1e-8", should_diverge_when_residual_is_growing="1", newton_iterations="3")
        elif self.integration == 'VarSym':
            simuNode.createObject('VariationalSymplecticSolver', rayleighStiffness='1', rayleighMass='1', newtonError='1e-12', steps=self.numIter, verbose='0', useIncrementalPotentialEnergy='1')
        else:
            print 'Unknown solver type'

        if self.linearSolver == 'CG':
            simuNode.createObject('CGLinearSolver', iterations="100", tolerance="1e-20", threshold="1e-20")
            # simuNode.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        elif self.linearSolver == 'Pardiso':
            simuNode.createObject('SparsePARDISOSolver', symmetric='1', exportDataToFolder='', name='precond', iterativeSolverNumbering='1')
        else:
            print 'Unknown linear solver type'

        ### mechanical object
        simuNode.createObject('MeshVTKLoader', name='loader', filename=volumeFileName)
        simuNode.createObject('TetrahedronSetTopologyContainer', name='Container', src="@loader")
        simuNode.createObject('TetrahedronSetTopologyModifier', name='Modifier')
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name='TopoAlgs')
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name='GeomAlgs')

        simuNode.createObject('MechanicalObject', src='@loader', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='0')
        simuNode.createObject('UniformMass', totalMass='0.01')

        ### material parameters
        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        materialParams='{} {}'.format(mu,lamb)
        simuNode.createObject('TetrahedronHyperelasticityFEMForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        # simuNode.createObject('TetrahedronFEMForceField', name='FEM', youngModulus=E, poissonRatio=nu, computeVonMisesStress='0', method='large')
        # simuNode.createObject('FastTetrahedralCorotationalForceField', name='FEM', youngModulus=E, poissonRatio=nu)

        if self.fixedPoints == 'L4':
            simuNode.createObject('BoxROI', box='-0.001 -0.001 -0.011 0.025 0.001 0.001', drawBoxes='0', name='FROI1')
        elif self.fixedPoints == 'L2':
            simuNode.createObject('BoxROI', box='-0.001 -0.001 -0.011 0.001 0.001 0.001', drawBoxes='0', name='FROI1')
        elif self.fixedPoints == 'L1':
            simuNode.createObject('BoxROI', box='-0.001 -0.001 -0.001 0.001 0.001 0.001', drawBoxes='0', name='FROI1')

        # simuNode.createObject('BoxROI', box='0.075 -0.001 -0.011 0.101 0.001 0.001', drawBoxes='1', name='FROI2')
        # simuNode.createObject('BoxROI', box='-0.001 -0.001 -0.011 0.101 0.001 0.001', drawBoxes='1', name='FROI')

        if self.fixingMethod == 'proj':
            simuNode.createObject('FixedConstraint', indices='@FROI1.indices', template='Vec3d')
        elif self.fixingMethod == 'penal':
            simuNode.createObject('RestShapeSpringForceField', stiffness='1e5', name='fixingSpring', points='@FROI1.indices', showIndicesScale='0', springColor='0 1 0 1')
        simuNode.createObject('PointsFromIndices', template='Vec3d', name='FixedPoints', indices='@FROI1.indices')

        ### export observations
        if self.saveObservations:
            simuNode.createObject('VTKExporter', position="@MO.position", listening="1" , XMLformat='0', exportAtBegin="1", exportAtEnd='0', exportEveryNumberOfSteps="1", filename=outputDir+'/object.vtk', tetras='1', edges='0')

        ### mapped tool node
        mappedTool = simuNode.createChild('mappedTool')
        mappedTool.createObject('MechanicalObject', position='0.045 0.1 0.0   0.05 0.1 0.0   0.055 0.1 0.0   0.045 0.1 -0.005   0.05 0.1 -0.005   0.055 0.1 -0.005    0.045 0.1 -0.01   0.05 0.1 -0.01   0.055 0.1 -0.01', name='MO')
        self.toolSprings=mappedTool.createObject('RestShapeSpringsForceField', name='toolSpring', stiffness='1e5', springColor='0 1 0 1', drawSpring='1', printLog='0', listening='1', angularStiffness='0', external_rest_shape='@/tool/MO')
        mappedTool.createObject('SphereCollisionModel', color='0 0 1 1', radius='0.001')
        mappedTool.createObject('BarycentricMapping', name='baryMapping')

        mappedTool2 = simuNode.createChild('mappedTool2')
        mappedTool2.createObject('MechanicalObject', position='0.045 0.1 0.0   0.05 0.1 0.0   0.055 0.1 0.0   0.045 0.1 -0.005   0.05 0.1 -0.005   0.055 0.1 -0.005    0.045 0.1 -0.01   0.05 0.1 -0.01   0.055 0.1 -0.01', name='MO')
        # self.toolSprings2=mappedTool.createObject('RestShapeSpringsForceField', name='toolSpring', stiffness='1e5', springColor='0 1 0 1', drawSpring='1', printLog='0', listening='1', angularStiffness='0', external_rest_shape='@/tool/MO')
        mappedTool2.createObject('SphereCollisionModel', color='0 0 1 0.1', radius='0.001')
        # mappedTool2.createObject('BarycentricMapping', name='baryMapping')

        fixedVisu=simuNode.createChild('fixedVisu')
        fixedVisu.createObject('MechanicalObject', position="@../FixedPoints.indices_position")
        fixedVisu.createObject('SphereCollisionModel', color='0.5 0.6 1.0 1', radius='0.0014', template='Vec3d')


        ### observation nodes
        obsGrid = simuNode.createChild('obsGrid')
        monitorFile=outputDir+'/observations'
        # obsGrid.createObject('RegularGridTopology', name="grid", min='0.0 0.01 0.0', max='0.1 0.1 -0.01', n='10 10 3')  # obs. grid1
        # obsGrid.createObject('RegularGridTopology', name="grid", min='0.0 0.01 0.0', max='0.1 0.1 -0.0', n='10 10 1')  # obs. grid2
        # obsGrid.createObject('RegularGridTopology', name="grid", min='0.0 0.01 0.0', max='0.1 0.03 -0.0', n='10 3 1')  # obs. grid3

        if self.obsPoints == 'GT3x10':
            obsGrid.createObject('RegularGridTopology', name="grid", min='0.0 0.08 0.0', max='0.1 0.1 -0.0', n='10 3 1')
        elif self.obsPoints == 'GC2x2':
        	obsGrid.createObject('RegularGridTopology', name="grid", min='0.01 0.01 0.0', max='0.09 0.09 0.0', n='2 2 1')
        elif self.obsPoints == 'PLB':
        	obsGrid.createObject('RegularGridTopology', name="grid", min='0.01 0.01 0.0', max='0.01 0.01 0.0', n='1 1 1')


        obsGrid.createObject('MechanicalObject', src='@grid', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='1')
        obsGrid.createObject('BarycentricMapping')
        # obsGrid.createObject('SphereCollisionModel', radius="0.0006", color="1 0 1 1")
        # obsGrid.createObject('SphereCollisionModel', color='0.0 0.5 0.0 1', radius="0.0014", template='Vec3d')
        obsGrid.createObject('ShowSpheres', position='@MO.position', color='1 1 0 1', radius='0.0014')

        if self.saveObservations:
            obsGrid.createObject('VTKExporter', filename=monitorFile+'.vtk', position="@MO.position", listening="0" , XMLformat='0', exportAtBegin='1', exportEveryNumberOfSteps="0")
            obsGrid.createObject('BoxROI', name='gridDOFs', box='-1 -1 -1 1 1 1')
            obsGrid.createObject('OptimMonitor', name='observationMonitor', fileName=monitorFile, indices="@gridDOFs.indices", ExportPositions="1", ExportVelocities="0", ExportForces="0")

        ### object visualization
        visNode = simuNode.createChild('ObjectVisualization1')
        visNode.createObject('MeshSTLLoader', name='objectSLoader', filename=surfaceSTL)
        visNode.createObject('MechanicalObject',src="@objectSLoader",name="Surface")
        visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@objectSLoader", tags=" ")
        visNode.createObject('LineCollisionModel', color="0 0 0 1")
        visNode.createObject('TriangleCollisionModel', color="1 0 0 1")
        visNode.createObject('BarycentricMapping')

        visNode2 = simuNode.createChild('ObjectVisualization2')
        visNode2.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping hideWireframe hideNormals')
        visNode2.createObject('MeshSTLLoader', name='objectSLoader', filename=surfaceSTL)
        visNode2.createObject('MechanicalObject',src="@objectSLoader",name="Surface")
        visNode2.createObject('TriangleSetTopologyContainer', name="Container", src="@objectSLoader", tags=" ")
        visNode2.createObject('TriangleCollisionModel', color="1 0 0 0.2")
        visNode2.createObject('BarycentricMapping')

        # visNode3 = simuNode.createChild('ObjectVisualization3')
        # visNode3.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping hideWireframe hideNormals')
        # visNode3.createObject('MeshSTLLoader', name='objectSLoader', filename=surfaceSTL)
        # visNode3.createObject('MechanicalObject',src="@objectSLoader",name="Surface")
        # visNode3.createObject('TriangleSetTopologyContainer', name="Container", src="@objectSLoader", tags=" ")
        # visNode3.createObject('TriangleCollisionModel',color="0 0 0.1 0.2")
        # visNode3.createObject('BarycentricMapping')

        # simuNode.createObject('BoxROI', box='-0.001 -0.001 -0.011 0.105 0.001 0.001', drawBoxes='0', name='baseROI')
        # self.basePoints=simuNode.createObject('PointsFromIndices', template='Vec3d', name='fixedA', indices='@baseROI.indices', position="@Volume.position")

        ### verification node
        asNode = simuNode.createChild('assessNode')
        asNode.createObject('RegularGridTopology', name="grid", min='0.01 0.005 -0.005', max='0.09 0.07 -0.005', n='8 5 1')  # obs. grid4
        self.asMO=asNode.createObject('MechanicalObject', src='@grid', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='1')
        asNode.createObject('SphereCollisionModel', color='1 0 1 1', radius="0.001", template='Vec3d')
        asNode.createObject('BarycentricMapping')

        return 0;



    def onEndAnimationStep(self, deltaTime):
        return 0;

    def onKeyPressed(self, c):
        return 0;

    def onKeyReleased(self, c):
        return 0;

    def onLoaded(self, node):
        return 0;

    def onBeginAnimationStep(self, deltaTime):
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

