import Sofa
import math
import os
import sys
import csv
import yaml
import pprint
import numpy as np
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

    Synth_AppliedDisp_SDA(rootNode, options)

    return 0;

class Synth_AppliedDisp_SDA(Sofa.PythonScriptController):

    def __init__(self, rootNode, opt):
        self.opt = opt

        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(opt)
                        
        self.saveEst = opt['io']['saveEst']                                
        self.saveGeo = opt["io"]["saveGeo"]        
        
        self.obsID = self.opt['io']['obs_id']
        prefix = opt['io']['prefix']
        suffix = opt['io']['suffix']    

        self.mainFolder = prefix + opt['model']['int']['type'] + str(opt['model']['int']['maxit']) + '_' + opt['model']['bc']['tag'] + '_' + opt['model']['control']['tag'] + '_' + opt['io']['obs_id'] + suffix        
        self.obsFolder  = self.mainFolder + '/obs'

        self.estFolder = self.mainFolder + '/' + opt['filter']['kind'] + opt['io']['sdaFolderSuffix']
        if self.saveEst:                      
            if opt['io']['save_existing_folder']:
                stmp = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H_%M_%S')
                os.system('mv '+self.estFolder+' '+self.estFolder+'_'+stmp)
            else:
                os.system('rm -rf '+self.estFolder)

            os.system('mkdir '+self.estFolder)

            self.stateExpFile=self.estFolder+'/state.txt'
            self.stateVarFile=self.estFolder+'/variance.txt'
            self.stateCovarFile=self.estFolder+'/covariance.txt'            

        if self.saveGeo:
            self.geoFolder = self.estFolder + '/VTK'
            os.system('mkdir -p '+self.geoFolder)

        self.rootNode=rootNode
        self.createGraph(rootNode)

        return None;    

    def createGraph(self, rootNode):
        rootNode.findData('gravity').value="0 0 0"
        rootNode.findData('dt').value="1"
        
        rootNode.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        rootNode.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        rootNode.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")

        self.filterKind = self.opt['filter']['kind']
        if self.filterKind == 'ROUKF':
            self.filter = rootNode.createObject('ROUKFilter', name="ROUKF", verbose="1", useBlasToMultiply='1')
            self.estimatePosition = 1            
        elif self.filterKind == 'UKFSimCorr':
            self.filter = rootNode.createObject('UKFilterSimCorr', name="UKFSC", verbose="1")
            self.estimatePosition = 0
        
        rootNode.createObject('MeshVTKLoader', name='objectLoader', filename=self.opt['model']['vol_mesh'])
        rootNode.createObject('MeshSTLLoader', name='objectSLoader', filename=self.opt['model']['surf_mesh'])
        rootNode.createObject('MeshVTKLoader', name='obsLoader', filename=self.obsFolder + '/obsPoints_0.vtk')
        rootNode.createObject('MeshVTKLoader', name='toolLoader', filename=self.obsFolder + '/tool_0.vtk')

        simuNode=rootNode.createChild('SimuNode')            
        self.createSimulation(simuNode)


    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createSimulation(self, simuNode):        
        simuNode.createObject('StochasticStateWrapper',name="StateWrapper",verbose='1', estimatePosition=self.estimatePosition, estimateVelocity='0')                                  

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

        # simuNode.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        if self.opt['model']['linsol']['usePCG']:
            simuNode.createObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='1', use_precond='1', tolerance='1e-10', iterations='500',
                verbose='0', listening='1', preconditioners='lsolver')

        simuNode.createObject('SparsePARDISOSolver', name='lsolver', verbose='0',
            symmetric=self.opt['model']['linsol']['pardisoSym'], exportDataToFolder=self.opt['model']['linsol']['pardisoFolder'])
            

        simuNode.createObject('MechanicalObject', src="@/objectLoader", name="Volume")
        simuNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/objectLoader", tags=" ")
        simuNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        simuNode.createObject('UniformMass', totalMass="0.2513")
        
        simuNode.createObject('BoxROI', box=self.opt['filter']['estim_bc_box'], drawBoxes='0', name='fixedBox1')

        self.optimParams= simuNode.createObject('OptimParams', name="springStiffness", optimize="1", template="Vector", 
            numParams='@fixedBox1.nbIndices', transformParams=self.opt['filter']['param_transform'],
            initValue=self.opt['filter']['param_init_exval'], stdev=self.opt['filter']['param_init_stdev'],
            minValue=self.opt['filter']['param_min_val'], maxValue=self.opt['filter']['param_max_val'])

        simuNode.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points="@fixedBox1.indices", stiffness="@springStiffness.value",
            showIndicesScale='0', springThickness="3", listening="1", updateStiffness="1", printLog="0")
        simuNode.createObject('ColorMap',colorScheme="Blue to Red")                                                             

        nu=self.opt['model']['poisson_ratio']
        E=self.opt['model']['young_modulus']        
        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        materialParams='{} {}'.format(mu,lamb)

        #simuNode.createObject('MJEDTetrahedralForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=self.materialParams)
        simuNode.createObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        # simuNode.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large")       

        #simuNode.createObject('BoxROI', name='fixedBoxA', box='-0.001 -0.001 -0.011 0.003 0.001 0.001', drawBoxes='0')
        #simuNode.createObject('PointsFromIndices', template='Vec3d', name='fixedA', indices='@fixedBoxA.indices', position="@Volume.rest_position")
        #simuNode.createObject('BoxROI', name='fixedBoxB', box='0.013 -0.001 -0.011 0.016 0.001 0.001', drawBoxes='0')
        #simuNode.createObject('PointsFromIndices', template='Vec3d', name='fixedB', indices='@fixedBoxB.indices', position="@Volume.rest_position")
        #simuNode.createObject('BoxROI', name='fixedBoxC', box='0.025 -0.001 -0.011 0.030 0.001 0.001', drawBoxes='0')
        #simuNode.createObject('PointsFromIndices', template='Vec3d', name='fixedC', indices='@fixedBoxC.indices', position="@Volume.rest_position")
        #simuNode.createObject('BoxROI', name='fixedBoxD', box='0.035 -0.001 -0.011 0.101 0.001 0.001', drawBoxes='0')
        #simuNode.createObject('PointsFromIndices', template='Vec3d', name='fixedD', indices='@fixedBoxD.indices', position="@Volume.rest_position")

        #fixedA = simuNode.createChild('fixedNA')
        #fixedA.createObject('MechanicalObject',name='MO', position='@../fixedA.indices_position')
        #fixedA.createObject('Sphere', color='1.0 0.0 0.0 1', radius="0.0019", template='Vec3d')

        #fixedB = simuNode.createChild('fixedNB')
        #fixedB.createObject('MechanicalObject',name='MO', position='@../fixedB.indices_position')
        #fixedB.createObject('Sphere', color='0.0 0.8 0.0 1', radius="0.0019", template='Vec3d')

        #fixedC = simuNode.createChild('fixedNC')
        #fixedC.createObject('MechanicalObject',name='MO', position='@../fixedC.indices_position')
        #fixedC.createObject('Sphere', color='1.0 0.64 0.0 1', radius="0.0019", template='Vec3d')

        #fixedD = simuNode.createChild('fixedND')
        #fixedD.createObject('MechanicalObject',name='MO', position='@../fixedD.indices_position')
        #fixedD.createObject('Sphere', color='0.0 1.0 1.0 1', radius="0.0019", template='Vec3d')

        toolEmu = simuNode.createChild('toolEmu')        
        toolEmu.createObject('MechanicalObject',name="MO",src="@/toolLoader")        
        toolEmu.createObject('SimulatedStateObservationSource', name="ToolA",printLog="1", monitorPrefix=self.obsFolder+'/tool', drawSize="0.0015",controllerMode="1")
        
        simuNode.createObject('Mapped3DoFForceField', mappedFEM="mappedTool/toolSpring", mappedMechObject="mappedTool/MO", mapping="mappedTool/baryMapping", printLog="0")        
        toolMapped = simuNode.createChild('mappedTool');
        toolMapped.createObject('MechanicalObject',name="MO",src="@/toolLoader")
        self.toolSprings=toolMapped.createObject('ExtendedRestShapeSpringForceField', name="toolSpring", stiffness="1e5", external_rest_shape="../toolEmu/MO", springThickness="1", listening="1", updateStiffness="1", springColor="0 1 0 1",startTimeSpringOn="0",numStepsSpringOn="10000")
        toolMapped.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        toolMapped.createObject('Sphere',radius="0.002",color="0 0 1 1") 
        toolMapped.createObject('BarycentricMapping',name="baryMapping")

        if self.saveGeo:
            simuNode.createObject('VTKExporterDA', filename=self.geoFolder+'/DAobject.vtk', XMLformat='0',listening='1',edges="0",triangles="0",quads="0",tetras="1",
        	   exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1")


        obsNode = simuNode.createChild('obsNode')
        obsNode.createObject('MechanicalObject', name='MO', src="@/obsLoader")                
        obsNode.createObject('Sphere', color='0.0 0.5 0.0 1', radius="0.0014", template='Vec3d')

        obsNode.createObject('BarycentricMapping')                   
        obsNode.createObject('MappedStateObservationManager', name="MOBS", listening="1", stateWrapper="@../StateWrapper",doNotMapObservations="1",verbose="1",
            observationStdev=self.opt['filter']['observ_stdev'])

        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.obsFolder+'/obsPoints', drawSize="0.000")
        obsNode.createObject('ShowSpheres', position='@MOBS.observations',color='1.0 0.0 1.0 1', radius="0.0012")
    
        visNode = simuNode.createChild('ObjectVisualization')
        visNode.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping showWireframe hideNormals')
        visNode.createObject('MechanicalObject',src="@/objectSLoader",name="Surface")
        visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@/objectSLoader", tags=" ")        
        visNode.createObject('Line',color="0 0 0 1")
        visNode.createObject('Triangle',color="1 0 0 1")
        visNode.createObject('BarycentricMapping')        

        # simuNode.createObject('BoxROI', box='-0.001 -0.001 -0.011 0.105 0.001 0.001', drawBoxes='0', name='baseROI', doUpdate='0')
        # self.basePoints=simuNode.createObject('PointsFromIndices', template='Vec3d', name='fixedA', indices='@baseROI.indices', position="@Volume.position")

        #asNode = simuNode.createChild('assessNode')
        #asNode.createObject('RegularGrid', name="grid", min='0.01 0.005 -0.005', max='0.09 0.07 -0.005', n='8 5 1')  # obs. grid4
        #self.asMO=asNode.createObject('MechanicalObject', src='@grid', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='1')
        #asNode.createObject('Sphere', color='1 0 1 1', radius="0.001", template='Vec3d')
        #asNode.createObject('BarycentricMapping')

        # self.createMasterScene(masterNode)
        #masterNode.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels')

        return 0


    def exportStochasticState(self):
        if self.saveEst:              
            stateName = 'reducedState' if self.filterKind == 'ROUKF' else 'state'
            varName = 'reducedVariance' if self.filterKind == 'ROUKF' else 'variance'
            covarName = 'reducedCovariance' if self.filterKind == 'ROUKF' else 'covariance'
        
            rs=self.filter.findData(stateName).value
            state = [val for sublist in rs for val in sublist]
            print 'State:',state            
            #print reducedState

            f1 = open(self.stateExpFile, "a")        
            f1.write(" ".join(map(lambda x: str(x), state)))
            f1.write('\n')
            f1.close()    
                    
            rv=self.filter.findData(varName).value
            variance = [val for sublist in rv for val in sublist]
            print 'Stdev: ', np.sqrt(variance)
            #print 'Reduced variance:'
            #print reducedVariance

            f2 = open(self.stateVarFile, "a")        
            f2.write(" ".join(map(lambda x: str(x), variance)))
            f2.write('\n')
            f2.close()

            rcv=self.filter.findData(covarName).value
            covariance = [val for sublist in rcv for val in sublist]
            #print 'Reduced Covariance:'
            #print reducedCovariance

            f3 = open(self.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), covariance)))
            f3.write('\n')
            f3.close()    

            return
 

    def bwdInitGraph(self, node):
        self.exportStochasticState()
        return 0

    def onEndAnimationStep(self, deltaTime):         
        self.exportStochasticState()
        return 0


    def initGraph(self,node):
        print 'Init graph called (python side)'
        self.step    =     0
        self.total_time =     0
        
        # self.process.initializationObjects(node)
        return 0

    def cleanup(self):
        if self.saveEst:
            print 'Estimations saved to '+self.estFolder

        if self.saveGeo:
            print 'Geometries saved to '+self.geoFolder

        return 0;


    # def onEndAnimationStep(self, deltaTime):  

        # if self.saveEst:                          
        #     if self.filterKind == 'ROUKF':
        #         rs=self.filter.findData('reducedState').value
        #     else:
        #         rs=self.filter.findData('state').value
        #     reducedState = [val for sublist in rs for val in sublist]
        #     #print 'Reduced state:'
        #     #print reducedState

        #     f1 = open(self.stateExpFile, "a")        
        #     f1.write(" ".join(map(lambda x: str(x), reducedState)))
        #     f1.write('\n')
        #     f1.close()    
            
        #     if self.filterKind == 'ROUKF':
        #         rv=self.filter.findData('reducedVariance').value
        #     else:
        #         rv=self.filter.findData('variance').value        
            
        #     reducedVariance = [val for sublist in rv for val in sublist]
        #     #print 'Reduced variance:'
        #     #print reducedVariance

        #     f2 = open(self.stateVarFile, "a")        
        #     f2.write(" ".join(map(lambda x: str(x), reducedVariance)))
        #     f2.write('\n')
        #     f2.close()

        #     # rcv=self.filter.findData('reducedCovariance').value
        #     # reducedCovariance = [val for sublist in rcv for val in sublist]
        #     # #print 'Reduced Covariance:'
        #     # #print reducedCovariance

        #     # f3 = open(self.stateCovarFile, "a")
        #     # f3.write(" ".join(map(lambda x: str(x), reducedCovariance)))
        #     # f3.write('\n')
        #     # f3.close()

        #     # if (self.filterKind == 'ROUKF'):
        #     #     innov=self.filter.findData('reducedInnovation').value
        #     # elif (self.filterKind == 'UKFSimCorr' or self.options.filter.kind == 'UKFClassic'):
        #     #     innov=self.filter.findData('innovation').value

        #     # innovation = [val for sublist in innov for val in sublist]            

        #     # f3b = open(self.innovationFile, "a")        
        #     # f3b.write(" ".join(map(lambda x: str(x), innovation)))
        #     # f3b.write('\n')
        #     # f3b.close()   

        # if self.saveToolForces:
        #     tsp=self.toolSprings.findData('totalForce').value
        #     f4 = open(self.toolForceFile, "a")
        #     f4.write(" ".join(map(lambda x: str(x), tsp[0])))
        #     f4.write('\n')
        #     f4.close()            
        #     #print 'Tool forces:'
        #     #print tsp

        # # if self.saveAssess:
        # #     tsp=self.asMO.findData('position').value
        # #     f5 = open(self.assessFile, "a")
        # #     f5.write(" ".join(map(lambda x: str(x), tsp)))
        # #     f5.write('\n')
        # #     f5.close()                        

        # #print self.basePoints.findData('indices_position').value

        # return 0

    def onScriptEvent(self, senderNode, eventName,data):        
        return 0;

