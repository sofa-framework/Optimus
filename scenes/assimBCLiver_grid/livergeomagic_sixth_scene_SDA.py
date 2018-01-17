import Sofa
import math
import os
import sys
import csv
sys.path.append(os.getcwd() + '/../assimStiffness')
import DAOptions

__file = __file__.replace('\\', '/') # windows

def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', pluginName='SofaPardisoSolver')
    rootNode.createObject('RequiredPlugin', pluginName='ImageMeshAux')
    #rootNode.createObject('RequiredPlugin', pluginName='SofaMJEDFEM')
    
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    rootNode.createObject('PythonScriptController', name='SynthBCDA', filename=__file, classname='synth1_BCDA', variables=' '.join(commandLineArguments[1:]))



# Class definition 
class synth1_BCDA(Sofa.PythonScriptController):

    options = DAOptions.DAOptions()

    def createGraph(self,node):
    	self.cameraReactivated=False
        self.rootNode=node    

        print  "Create graph called (Python side)\n"
	
        # load configuration from yaml file
        if len(self.findData("variables").value) > 0:
            self.configFileName = self.findData("variables").value[0][0]
        else:
            self.configFileName = "cyl_scene_config.yml"

        self.options.parseYaml(self.configFileName)
        self.options.export.createFolder(archiveResults=1)
        self.options.init(copyConfigFile=1)
        
        if self.options.filter.kind=='ROUKF':
            self.options.filter.estimPosition='1'
            self.options.filter.estimVelocity='0'          

        if self.options.filter.kind=='UKFSimCorr':
            self.options.filter.estimPosition='0'
            self.options.filter.estimVelocity='0'

        if self.options.filter.kind=='UKFClassic':
            self.options.filter.estimPosition='1'
            self.options.filter.estimVelocity='0'
            
        
        self.createScene(node)        
        
        return 0

    
    def createGlobalComponents(self, node):
        # scene global stuff                
        node.findData('gravity').value=self.options.model.gravity
        node.findData('dt').value=self.options.model.dt
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")        

        if (self.options.filter.kind == 'ROUKF'):
            self.filter = node.createObject('ROUKFilter', name="ROUKF", verbose="1", useUnbiasedVariance=self.options.filter.useUnbiasedVariance, sigmaTopology=self.options.filter.sigmaPointsTopology, lambdaScale=self.options.filter.lambdaScale)        
        elif (self.options.filter.kind == 'UKFSimCorr'):
            self.filter = node.createObject('UKFilterSimCorr', name="UKF", verbose="1", useUnbiasedVariance=self.options.filter.useUnbiasedVariance, sigmaTopology=self.options.filter.sigmaPointsTopology, lambdaScale=self.options.filter.lambdaScale) 
        elif (self.options.filter.kind == 'UKFClassic'):
            self.filter = node.createObject('UKFilterClassic', name="UKFClas", verbose="1", exportPrefix=self.options.export.folder, useUnbiasedVariance=self.options.filter.useUnbiasedVariance, sigmaTopology=self.options.filter.sigmaPointsTopology, lambdaScale=self.options.filter.lambdaScale)
        else:
            print 'Unknown filter type!'
            
        # node.createObject('MeshVTKLoader', name='loader', filename=self.options.model.volumeFileName)
        node.createObject('MeshGmshLoader', name='loader', filename=self.options.model.volumeFileName)
        #node.createObject('MeshSTLLoader', name='objectSLoader', filename=self.surfaceSTL)
        
        return 0        


    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createCommonComponents(self, node):                                  
        #node.createObject('StaticSolver', applyIncrementFactor="0")        
        node.createObject('EulerImplicitSolver', rayleighStiffness=self.options.model.rayleighStiffness, rayleighMass=self.options.model.rayleighMass)
        # node.createObject('NewtonStaticSolver', name="NewtonStatic", printLog="0", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt="2")   
        #node.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        node.createObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToFolder="", iterativeSolverNumbering="0")

        node.createObject('MechanicalObject', src="@/loader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass=self.options.model.totalMass)

        node.createObject('OptimParams', name="paramE", optimize="1", numParams=self.options.filter.nparams, template="Vector", initValue=self.options.filter.paramInitExpVal, min=self.options.filter.paramMinExpVal, max=self.options.filter.paramMaxExpVal, stdev=self.options.filter.paramInitStdev, transformParams=self.options.filter.transformParams)
        nu=0.45
        E=5000
        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        materialParams='{} {}'.format(mu,lamb)
        node.createObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        # node.createObject('TetrahedronFEMForceField', name='FEM', updateStiffness='1', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='5000')

        for index in range(0, len(self.options.model.bcList)):
            bcElement = self.options.model.bcList[index]
            node.createObject('BoxROI', box=bcElement.boundBoxes, name='boundBoxes'+str(index))
            if (bcElement.bcType == 'fixed'):
                node.createObject('FixedConstraint', indices='@boundBoxes'+str(index)+'.indices')
            elif (bcElement.bcType == 'elastic'):
                node.createObject('ExtendedRestShapeSpringForceField', stiffness='@paramE.value', showIndicesScale='0', springThickness="3", listening="1", updateStiffness="1", printLog="0", points='@boundBoxes'+str(index)+'.indices')
            else:
                print 'Unknown type of boundary conditions'

        # rootNode/simuNode/oglNode
        oglNode = node.createChild('oglNode')
        self.oglNode = oglNode
        oglNode.createObject('OglModel', color='0 0 0 0')
        # node.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large")

        impactSimu = node.createChild('externalImpSimu')
        impactSimu.createObject('MechanicalObject', name="state", template='Rigid', useTopology='false', position=self.options.impact.position)
        impactSimu.createObject('SimulatedStateObservationSource', name="ImpactSim", template='Rigid', printLog="1", monitorPrefix=self.options.impact.positionFileName, drawSize="0.0015", controllerMode="1")

        externalNode = node.createChild('ExternalImpact')
        externalNode.createObject('MechanicalObject', name='dofs', template='Rigid', position=self.options.impact.position)
        self.toolSprings = externalNode.createObject('RestShapeSpringsForceField', name="impactSpring", stiffness="100", angularStiffness='1', external_rest_shape='@../externalImpSimu/state')
        externalNode.createObject('BarycentricMapping')
                
        return 0


    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1", estimatePosition=self.options.filter.estimPosition, positionStdev=self.options.filter.positionStdev, estimateVelocity=self.options.filter.estimVelocity)
        self.createCommonComponents(node)
        obsNode = node.createChild('obsNode')        
        obsNode.createObject('MeshVTKLoader', name='obsLoader', filename=self.options.observations.positionFileName)        
        obsNode.createObject('MechanicalObject', name='SourceMO', src="@obsLoader")
        obsNode.createObject('BarycentricMapping')
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev=self.options.observations.stdev, noiseStdev="0.0", listening="1", stateWrapper="@../StateWrapper", verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.options.observations.valueFileName)
        obsNode.createObject('ShowSpheres', radius="0.002", color="1 0 0 1", position='@SourceMO.position')
        obsNode.createObject('ShowSpheres', radius="0.0015", color="1 1 0 1", position='@MOBS.mappedObservations')

        return 0
  

    

    def createScene(self,node):
        # r_slaves = [] # list of created auxiliary nodes
        self.createGlobalComponents(node)
                
        masterNode=node.createChild('MasterScene')
        self.createMasterScene(masterNode)        
 
        return 0


    def initGraph(self,node):
        print 'Init graph called (python side)'
        self.step    =     0
        self.total_time =     0
        
        # self.process.initializationObjects(node)
        return 0

    def onEndAnimationStep(self, deltaTime):  

        if self.options.export.state:
            if (self.options.filter.kind == 'ROUKF'):
                st=self.filter.findData('reducedState').value
            elif (self.options.filter.kind == 'UKFSimCorr' or self.options.filter.kind == 'UKFClassic'):
                st=self.filter.findData('state').value

            state = [val for sublist in st for val in sublist]
            #print 'Reduced state:'
            #print reducedState

            print 'Storing to',self.options.export.stateExpValFile
            f1 = open(self.options.export.stateExpValFile, "a")        
            f1.write(" ".join(map(lambda x: str(x), state)))
            f1.write('\n')
            f1.close()    

            if (self.options.filter.kind == 'ROUKF'):
                var=self.filter.findData('reducedVariance').value
            elif (self.options.filter.kind == 'UKFSimCorr' or self.options.filter.kind == 'UKFClassic'):
                var=self.filter.findData('variance').value
                                
            variance = [val for sublist in var for val in sublist]
            #print 'Reduced variance:'
            #print reducedVariance

            f2 = open(self.options.export.stateVarFile, "a")        
            f2.write(" ".join(map(lambda x: str(x), variance)))
            f2.write('\n')
            f2.close()

            if (self.options.filter.kind == 'ROUKF'):
                covar=self.filter.findData('reducedCovariance').value
            elif (self.options.filter.kind == 'UKFSimCorr' or self.options.filter.kind == 'UKFClassic'):
                covar=self.filter.findData('covariance').value
            
            covariance = [val for sublist in covar for val in sublist]
            #print 'Reduced Covariance:'
            #print reducedCovariance

            f3 = open(self.options.export.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), covariance)))
            f3.write('\n')
            f3.close()

        if self.options.export.internalData:
            if (self.options.filter.kind == 'ROUKF'):
                innov=self.filter.findData('reducedInnovation').value
            elif (self.options.filter.kind == 'UKFSimCorr' or self.options.filter.kind == 'UKFClassic'):
                innov=self.filter.findData('innovation').value

            innovation = [val for sublist in innov for val in sublist]
            #print 'Reduced state:'
            #print reducedState

            f4 = open(self.options.export.innovationFile, "a")        
            f4.write(" ".join(map(lambda x: str(x), innovation)))
            f4.write('\n')
            f4.close()      

        # print self.basePoints.findData('indices_position').value

        return 0

    def onScriptEvent(self, senderNode, eventName,data):        
        return 0;

