import Sofa
import math
import os
import sys
import csv

__file = __file__.replace('\\', '/') # windows

def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', pluginName='SofaPardisoSolver')
    rootNode.createObject('RequiredPlugin', pluginName='ImageMeshAux')
    #rootNode.createObject('RequiredPlugin', pluginName='SofaMJEDFEM')
    
    rootNode.createObject('PythonScriptController', name='SynthBCDA', filename=__file, classname='synth1_BCDA')



# Class definition 
class synth1_BCDA(Sofa.PythonScriptController):


    def createGraph(self,node):          
        self.cameraReactivated=False
        self.rootNode=node              
         
        print  "Create graph called (Python side)\n"

        E=5000
        nu=0.45
        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        self.materialParams='{} {}'.format(mu,lamb)

        self.volumeFileName='../../data/cylinder/cylinder10_4245.vtk'
        self.surfaceFileName='../../data/cylinder/cylinder10_4245.stl'
        self.observationFileName='observations/cylinder10plane_4245'
        self.dt='0.01'
        self.gravity='0 -9.81 0'
        self.totalMass='0.2'
        #self.totalMass='0.3769'
        self.rayleighMass=0.1
        self.rayleighStiffness=3
                        
        self.outDir='outCyl10plane'
        
        self.saveState = 1
        self.suffix='test'   #psd'+str(self.paramInitSD)+'#osd'+str(self.obsInitSD)+'#ogrid'+str(self.ogridID)
        if self.saveState:
            self.stateExpFile=self.outDir+'/state_'+self.suffix+'.txt'
            self.stateVarFile=self.outDir+'/variance_'+self.suffix+'.txt'
            self.stateCovarFile=self.outDir+'/covariance_'+self.suffix+'.txt'
            os.system('rm '+self.stateExpFile)
            os.system('rm '+self.stateVarFile)
            os.system('rm '+self.stateCovarFile)
          
        # self.m_slaveSceneCount = 0
        # self.m_saveToFile = 0
        # self.m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method

                
        # self.paramInitExp = 0.0
        # self.paramInitSD = 5
        # self.obsInitSD= 1e-4

        self.createScene(node)
        
        return 0

    
    def createGlobalComponents(self, node):
        # scene global stuff                
        node.findData('gravity').value=self.gravity
        node.findData('dt').value=self.dt
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")        
        self.filter = node.createObject('ROUKFilter', name="ROUKF", verbose="1")        
            
        node.createObject('MeshVTKLoader', name='loader', filename=self.volumeFileName)
        #node.createObject('MeshSTLLoader', name='objectSLoader', filename=self.surfaceSTL)
        
        return 0        


    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createCommonComponents(self, node):                                  
        #node.createObject('StaticSolver', applyIncrementFactor="0")        
        node.createObject('EulerImplicitSolver', rayleighStiffness=self.rayleighStiffness, rayleighMass=self.rayleighMass)
        # node.createObject('NewtonStaticSolver', name="NewtonStatic", printLog="0", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt="2")   
        # node.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        node.createObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToFolder="", iterativeSolverNumbering="0")

        node.createObject('MechanicalObject', src="@/loader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass=self.totalMass)

        node.createObject('BoxROI', box='-0.05 -0.05 -0.002   0.05 0.05 0.002', name='fixedBox1')
        node.createObject('BoxROI', box='-0.05 -0.05  0.298   0.05 0.05 0.302', name='fixedBox2')
        node.createObject('MergeSets', name='mergeIndices', in2='@fixedBox2.indices', in1='@fixedBox1.indices')
        node.createObject('FixedConstraint', indices='@mergeIndices.out')
                    
        node.createObject('OptimParams', name="paramE", optimize="1", numParams='10', template="Vector", initValue="6000", stdev="1000", transformParams="1")
        node.createObject('Indices2ValuesMapper', name='youngMapper', indices='1 2 3 4 5 6 7 8 9 10', values='@paramE.value', inputValues='@/loader.dataset')
        node.createObject('TetrahedronFEMForceField', name='FEM', updateStiffness='1', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')

                # rootNode/simuNode/oglNode
        oglNode = node.createChild('oglNode')
        self.oglNode = oglNode
        oglNode.createObject('OglModel', color='0 0 0 0')
        # node.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="1e5", poissonRatio="0.45", method="large")
                
        return 0


    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        
        bodyNode=node.createChild('bodyNode')

        self.createCommonComponents(bodyNode)

        obsNode = bodyNode.createChild('obsNode')
        obsNode.createObject('MechanicalObject', name='SourceMO', position='0.02 0 0.08 0.02 0 0.16    0.0141 0.0141 0.08    0.0141 -0.0141 0.08    0.0141 0.0141 0.16    0.0141 -0.0141 0.16    0.02 0 0.0533    0.02 0 0.107   \
            0.02 0 0.133    0.02 0 0.187    0.02 0 0.213    0.0175 0.00961 0.0649    0.00925 0.0177 0.0647    0.0139 0.0144 0.0398    0.00961 -0.0175 0.0649    0.0177 -0.00925 0.0647  \
            0.0144 -0.0139 0.0402    0.0177 0.00936 0.145    0.0095 0.0176 0.145    0.0175 0.00961 0.0951    0.00925 0.0177 0.0953    0.0139 0.0144 0.12    0.00937 -0.0177 0.145   \
            0.0176 -0.00949 0.145    0.00935 -0.0177 0.0953    0.0176 -0.00949 0.095    0.0142 -0.0141 0.12    0.0177 0.00937 0.175    0.00949 0.0176 0.175    0.014 0.0143 0.2   \
            0.00959 -0.0175 0.175    0.0177 -0.00924 0.175    0.0143 -0.014 0.2')        
        obsNode.createObject('BarycentricMapping')
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev="2e-3", noiseStdev="0.0", listening="1", stateWrapper="@../../StateWrapper", verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.observationFileName)
        obsNode.createObject('ShowSpheres', radius="0.002", color="1 0 0 1", position='@SourceMO.position')
        obsNode.createObject('ShowSpheres', radius="0.0015", color="1 1 0 1", position='@MOBS.mappedObservations')



        return 0
  

    # def createSlaveScene(self, node):
    #     node.createObject('VisualStyle', name='VisualStyle', displayFlags='hideBehaviorModels hideForceFields hideCollisionModels')
    #     wrapper=node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
    #     wrapper.findData("name").value = "StochasticWrapperSlave"
    #     wrapper.findData("slave").value = 1;        
        
    #     self.createCommonComponents(node)        
    #     self.m_slaveScenesCreated+=1

    #     return 0

    

    def createScene(self,node):
        # r_slaves = [] # list of created auxiliary nodes
        self.createGlobalComponents(node)
                
        masterNode=node.createChild('MasterScene')
        self.createMasterScene(masterNode)        
        # masterNode.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels')
        
        # slaveSubordinate=node.createChild('SlaveSubordinate')
        # for i in range(1,self.m_slaveSceneCount):
        #     slave=slaveSubordinate.createChild('SlaveScene_'+str(i))
        #     #slave.createObject('VisualStyle', name='VisualStyle', displayFlags='hideAll')
        #     self.createSlaveScene(slave)
        #     r_slaves.append(slave)        
            
        return 0


    def initGraph(self,node):
        print 'Init graph called (python side)'
        self.step    =     0
        self.total_time =     0
        
        # self.process.initializationObjects(node)
        return 0

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

        # print self.basePoints.findData('indices_position').value

        return 0

    def onScriptEvent(self, senderNode, eventName,data):        
        return 0;

