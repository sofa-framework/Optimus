import Sofa
import math
import os
import sys
import csv
import yaml
import pprint
import time
import datetime

__file = __file__.replace('\\', '/') # windows

def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', name='Optimus', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', name='Pardiso', pluginName='SofaPardisoSolver')
    rootNode.createObject('RequiredPlugin', name='IMAUX', pluginName='ImageMeshAux')
    # rootNode.createObject('RequiredPlugin', name='MJED', pluginName='SofaMJEDFEM')
    
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

    with open(configFileName, 'r') as stream:
        try:
            options = yaml.load(stream)            

        except yaml.YAMLError as exc:
            print(exc)
            return    

    Synth_AppliedDisp_GenObs(rootNode, options)

    return 0;

class Synth_AppliedDisp_GenObs (Sofa.PythonScriptController):    

    def __init__(self, rootNode, opt):
        self.opt = opt

        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(opt)
                        
        self.saveObs = opt['io']['saveObs']
        self.saveGeo = opt["io"]["saveGeo"]        
                        
        if self.saveObs or self.saveGeo:
            prefix = opt['io']['prefix']
            suffix = opt['io']['suffix']    

            self.mainFolder = prefix + opt['model']['int']['type'] + str(opt['model']['int']['maxit']) + '_' + opt['model']['bc']['tag'] + '_' + opt['model']['control']['tag'] + '_' + opt['io']['obs_id'] + suffix

            if opt['io']['save_existing_folder']:                
                stmp = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H_%M_%S')
                os.system('mv '+self.mainFolder+' '+self.mainFolder+'_'+stmp)
            else:
                os.system('rm -rf '+self.mainFolder)

            self.obsFolder  = self.mainFolder + '/obs'            
            os.system('mkdir -p '+self.obsFolder)

        if self.saveGeo:
            self.geoFolder = self.mainFolder + '/obs/VTK'
            os.system('mkdir -p '+self.geoFolder)

        self.createGraph(rootNode)

        return None;

    def createGraph(self,rootNode):
        nu=self.opt['model']['poisson_ratio']
        E=self.opt['model']['young_modulus']
        
        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        materialParams='{} {}'.format(mu,lamb)
                        
        self.obsID = self.opt['io']['obs_id']

        # rootNode
        rootNode.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping showWireframe hideNormals')        

        rootNode.findData('gravity').value=self.opt['model']['gravity']
        rootNode.findData('dt').value=self.opt['model']['dt']

        # rootNode/tool
        tool = rootNode.createChild('tool')
        tool.createObject('MechanicalObject', name='MO', position='0.045 0.1 0.0   0.05 0.1 0.0   0.055 0.1 0.0   0.045 0.1 -0.005   0.05 0.1 -0.005   0.055 0.1 -0.005    0.045 0.1 -0.01   0.05 0.1 -0.01   0.055 0.1 -0.01')
        tool.createObject('Mesh', position='@MO.position', edges='0 1  1 2  2 3  3 4  4 5  5 6  6 7  7 8')   # added only to avoid VTKExporter crashing
        tool.createObject('ShowSpheres', position='@MO.position', color='0 0 1 1', radius='0.0014')        
        tool.createObject('LinearMotionStateController', indices='0 1 2 3 4 5 6 7 8', keyTimes=self.opt['model']['control']['times'], keyDisplacements=self.opt['model']['control']['displ'])

        if self.saveObs:
            tool.createObject('VTKExporter', name='toolExporter', position="@MO.position", filename=self.obsFolder+'/tool.vtk', edges="1", listening="0" , XMLformat='0', exportAtBegin='1', exportEveryNumberOfSteps="0")
            tool.createObject('BoxROI', name='toolDOFs', box='-1 -1 -1 1 1 1')        
            tool.createObject('Monitor', name='toolMonitor', fileName=self.obsFolder+'/tool', showPositions='1', indices="@toolDOFs.indices", ExportPositions="1", ExportVelocities="1", ExportForces="1")

        
        # rootNode/simuNode
        simuNode = rootNode.createChild('simu')

        intType = self.opt['model']['int']['type']
        intMaxit = self.opt['model']['int']['maxit']
        rmass = self.opt['model']['int']['rmass']
        rstiff = self.opt['model']['int']['rstiff']
        if intType == 'Euler':
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness=rstiff, rayleighMass=rmass)
        elif intType == 'Newton':
            simuNode.createObject('NewtonStaticSolver', maxIt=intMaxit, name='NewtonStatic', correctionTolerance='1e-8', convergeOnResidual='1', residualTolerance='1e-8', printLog='1')
        elif intType == 'VarSym':
            simuNode.createObject('VariationalSymplecticSolver', rayleighStiffness=rstiff, rayleighMass=rmass,
             newtonError='1e-12', steps=intMaxit, verbose='0', useIncrementalPotentialEnergy='1')

                
        simuNode.createObject('SparsePARDISOSolver', name='lsolver', verbose='0',
            symmetric=self.opt['model']['linsol']['pardisoSym'], exportDataToFolder=self.opt['model']['linsol']['pardisoFolder'])


        simuNode.createObject('MeshVTKLoader', name='loader', filename=self.opt['model']['vol_mesh'])
        simuNode.createObject('TetrahedronSetTopologyContainer', name='Container', src="@loader")
        simuNode.createObject('TetrahedronSetTopologyModifier', name='Modifier')
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name='TopoAlgs')
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name='GeomAlgs')
        # simuNode.createObject('Hexa2TetraTopologicalMapping', input="@/grid/grid", output="@Container", swapping='1')
        
        # simuNode.createObject('MechanicalObject', src='@/grid/grid', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='0')
        simuNode.createObject('MechanicalObject', src='@loader', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='0')
        simuNode.createObject('UniformMass', totalmass=self.opt['model']['total_mass'])
                
        simuNode.createObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        #simuNode.createObject('MJEDTetrahedralForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)        
        # simuNode.createObject('TetrahedronFEMForceField', name='FEM', youngModulus=E, poissonRatio=nu, computeVonMisesStress='0', method='large')
        # simuNode.createObject('FastTetrahedralCorotationalForceField', name='FEM', youngModulus=E, poissonRatio=nu)    
        
        simuNode.createObject('BoxROI', box=self.opt['model']['bc']['boxes'], drawBoxes='0', name='FROI1')        
        simuNode.createObject('ExtendedRestShapeSpringForceField', stiffness='1e5', name='fixingSpring', points='@FROI1.indices', showIndicesScale='0', springColor='0 1 0 1', startTimeSpringOn='0', numStepsSpringOn='10000')
        # simuNode.createObject('ExtendedRestShapeSpringForceField', stiffness='1e5', name='fixingSpring', points='@FROI2.indices', springColor='0 1 0 1', startTimeSpringOn='0', numStepsSpringOn='10000')
        # simuNode.createObject('FixedConstraint', indices='@FROI.indices', template='Vec3d')

        simuNode.createObject('Mapped3DoFForceField', mappedFEM='mappedTool/toolSpring', mappedMechObject='mappedTool/MO', printLog='0', mapping='mappedTool/baryMapping')
        simuNode.createObject('PointsFromIndices', template='Vec3d', name='FixedPoints', indices='@FROI1.indices')

        if self.saveGeo:
            simuNode.createObject('VTKExporter', position="@MO.position", listening="1" , XMLformat='0', exportAtBegin="1", exportAtEnd='0', exportEveryNumberOfSteps="1", 
                filename=self.geoFolder+'/object.vtk', tetras='1', edges='0')

        # rootNode/object/mappedTool
        mappedTool = simuNode.createChild('mappedTool')
        mappedTool.createObject('MechanicalObject', position='0.045 0.1 0.0   0.05 0.1 0.0   0.055 0.1 0.0   0.045 0.1 -0.005   0.05 0.1 -0.005   0.055 0.1 -0.005    0.045 0.1 -0.01   0.05 0.1 -0.01   0.055 0.1 -0.01', name='MO')
        self.toolSprings=mappedTool.createObject('ExtendedRestShapeSpringForceField', numStepsSpringOn='10000', stiffness='1e5', name='toolSpring', springColor='0 1 0 1', drawSpring='1', updateStiffness='1', printLog='0', listening='1', angularStiffness='0', startTimeSpringOn='0', external_rest_shape='/tool/MO')
        mappedTool.createObject('Sphere', color='0 0 1 1', radius='0.001')
        mappedTool.createObject('BarycentricMapping', name='baryMapping')

        mappedTool2 = simuNode.createChild('mappedTool2')
        mappedTool2.createObject('MechanicalObject', position='0.045 0.1 0.0   0.05 0.1 0.0   0.055 0.1 0.0   0.045 0.1 -0.005   0.05 0.1 -0.005   0.055 0.1 -0.005    0.045 0.1 -0.01   0.05 0.1 -0.01   0.055 0.1 -0.01', name='MO')
        # self.toolSprings2=mappedTool.createObject('ExtendedRestShapeSpringForceField', numStepsSpringOn='10000', stiffness='1e5', name='toolSpring', springColor='0 1 0 1', drawSpring='1', updateStiffness='1', printLog='0', listening='1', angularStiffness='0', startTimeSpringOn='0', external_rest_shape='/tool/MO')
        mappedTool2.createObject('Sphere', color='0 0 1 0.1', radius='0.001')
        # mappedTool2.createObject('BarycentricMapping', name='baryMapping')

        fixedVisu=simuNode.createChild('fixedVisu')
        fixedVisu.createObject('MechanicalObject', position="@../FixedPoints.indices_position")
        fixedVisu.createObject('Sphere', color='0.5 0.6 1.0 1', radius='0.0014', template='Vec3d')
        

        # rootNode/simu/obsGrid
        obsGrid = simuNode.createChild('obsGrid')

        if self.obsID == 'ogrid1':
            obsGrid.createObject('RegularGrid', name="grid", min='0.0 0.01 0.0', max='0.1 0.1 -0.01', n='10 10 3')  # obs. grid1        
        elif self.obsID == 'ogrid2':
            obsGrid.createObject('RegularGrid', name="grid", min='0.0 0.01 0.0', max='0.1 0.1 -0.0', n='10 10 1')  # obs. grid2
        elif self.obsID == 'ogrid3':
            obsGrid.createObject('RegularGrid', name="grid", min='0.0 0.01 0.0', max='0.1 0.03 -0.0', n='10 3 1')  # obs. grid3
        elif self.obsID == 'ogrid4':        
            obsGrid.createObject('RegularGrid', name="grid", min='0.0 0.08 0.0', max='0.1 0.1 -0.0', n='10 3 1')  # obs. ogrid4

        obsGrid.createObject('MechanicalObject', src='@grid', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='1')        
        obsGrid.createObject('BarycentricMapping')
        # obsGrid.createObject('Sphere', radius="0.0006", color="1 0 1 1")
        obsGrid.createObject('Sphere', color='0.0 0.5 0.0 1', radius="0.0014", template='Vec3d')

        if self.saveObs:
            obsGrid.createObject('VTKExporter', filename=self.obsFolder+'/obsPoints.vtk', position="@MO.position", listening="0" , XMLformat='0', exportAtBegin='1', exportEveryNumberOfSteps="0")
            obsGrid.createObject('BoxROI', name='gridDOFs', box='-1 -1 -1 1 1 1')                
            obsGrid.createObject('Monitor', name='observationMonitor', fileName=self.obsFolder+'/obsPoints', indices="@gridDOFs.indices", ExportPositions="1", ExportVelocities="0", ExportForces="0")
            
        visNode = simuNode.createChild('ObjectVisualization1')
        visNode.createObject('MeshSTLLoader', name='objectSLoader', filename=self.opt['model']['surf_mesh'])
        visNode.createObject('MechanicalObject',src="@objectSLoader",name="Surface")
        visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@objectSLoader", tags=" ")        
        visNode.createObject('Line',color="0 0 0 1")
        visNode.createObject('Triangle',color="1 0 0 1")
        visNode.createObject('BarycentricMapping')

        visNode2 = simuNode.createChild('ObjectVisualization2')
        visNode2.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping hideWireframe hideNormals')
        visNode2.createObject('MeshSTLLoader', name='objectSLoader', filename=self.opt['model']['surf_mesh'])
        visNode2.createObject('MechanicalObject',src="@objectSLoader",name="Surface")
        visNode2.createObject('TriangleSetTopologyContainer', name="Container", src="@objectSLoader", tags=" ")                
        visNode2.createObject('Triangle',color="1 0 0 0.2")
        visNode2.createObject('BarycentricMapping')
        
        asNode = simuNode.createChild('assessNode')
        asNode.createObject('RegularGrid', name="grid", min='0.01 0.005 -0.005', max='0.09 0.07 -0.005', n='8 5 1')  # obs. grid4
        self.asMO=asNode.createObject('MechanicalObject', src='@grid', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='1')
        asNode.createObject('Sphere', color='1 0 1 1', radius="0.001", template='Vec3d')
        asNode.createObject('BarycentricMapping')

        return 0;

    def onEndAnimationStep(self, deltaTime):
        totalToolForce=self.toolSprings.findData('totalForce').value
        #print 'Total tool force: ',totalToolForce        
        #self.toolForceFile.write("%g %g %g\n" % (totalToolForce[0][0], totalToolForce[0][1], totalToolForce[0][2]))

        #asPos = self.asMO.findData('position').value
        #print 'AsPos ', asPos        
        #print len(asPos)        

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
        #self.toolForceFile.close()
        return 0;
