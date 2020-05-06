import Sofa
import math
import os
import sys
import csv

__file = __file__.replace('\\', '/') # windows

def createScene(rootNode):
    rootNode.createObject('RequiredPlugin', pluginName='Optimus')
    rootNode.createObject('RequiredPlugin', name='Python', pluginName='SofaPython')
    rootNode.createObject('PythonScriptController', name='SynthBCDA', filename=__file, classname='synth1_BCDA')


# Class definition 
class synth1_BCDA(Sofa.PythonScriptController):

    def createGraph(self, node):          
        self.cameraReactivated=False
        self.rootNode=node              
        print "Create graph called (Python side)\n"

        # configuration
        self.volumeFileName='../../data/cylinder/cylinder10_4245.vtk'
        self.observationFileName='observations/cylinder10_4245'
        self.observationPointsVTK='../../data/cylinder/cyl10_4245_obs41.vtk'
        self.dt='0.01'
        self.gravity='0 -9.81 0'
        self.totalMass='0.2' # '0.3769'
        self.rayleighMass=0.1
        self.rayleighStiffness=3
        self.linearSolver='CG'  # options are 'CG' and 'Pardiso'
        self.outDir='outCyl10'
        self.saveState = 1
        self.suffix='test'

        if self.saveState:
            self.stateExpFile=self.outDir+'/state_'+self.suffix+'.txt'
            self.stateVarFile=self.outDir+'/variance_'+self.suffix+'.txt'
            self.stateCovarFile=self.outDir+'/covariance_'+self.suffix+'.txt'
            if os.path.isfile(self.stateExpFile):
                os.system('rm '+self.stateExpFile)
            if os.path.isfile(self.stateVarFile):
                os.system('rm '+self.stateVarFile)
            if os.path.isfile(self.stateCovarFile):
                os.system('rm '+self.stateCovarFile)

        self.filterType='ROUKF' # options are 'UKFClassic', 'ROUKF', and 'UKFSimmCorr'
        self.paramInitExp = 6000
        self.paramInitSD = 500
        self.obsNoiseSD= 2e-3


        # create scene components
        if self.linearSolver=='Pardiso':
            rootNode.createObject('RequiredPlugin', pluginName='SofaPardisoSolver')
        self.createComponents(node)
  
        return 0



    def createComponents(self, node):
        # scene global stuff   
        node.findData('gravity').value=self.gravity
        node.findData('dt').value=self.dt
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")

        if self.filterType == 'UKFClassic':
            self.filter = node.createObject('UKFilterClassic', name="UKFClas", verbose="1")
            self.estimPosition='1'
            self.estimVelocity='0'
        elif self.filterType == 'UKFSimCorr':
            self.filter = node.createObject('UKFSimCorr', name="UKFSimCorr", verbose="1")
            self.estimPosition='0'
            self.estimVelocity='0'
        elif self.filterType == 'ROUKF':
            self.filter = node.createObject('ROUKFilter', name="ROUKF", verbose="1")
            self.estimPosition='1'
            self.estimVelocity='0'

        # create stochastic scene
        # solvers
        masterNode = node.createChild('MasterScene')
        masterNode.createObject('StochasticStateWrapper', name="StateWrapper", verbose="1", estimatePosition=self.estimPosition, positionStdev=1e-3, posModelStdev=1e-5, estimateVelocity=self.estimVelocity)
        masterNode.createObject('EulerImplicitSolver', rayleighStiffness=self.rayleighStiffness, rayleighMass=self.rayleighMass)
        if self.linearSolver=='CG':
            masterNode.createObject('CGLinearSolver', iterations="20", tolerance="1e-12", threshold="1e-12")
        elif self.linearSolver=='Pardiso':
            masterNode.createObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToFolder="", iterativeSolverNumbering="0")

        # mechanical object
        masterNode.createObject('MeshVTKLoader', name='loader', filename=self.volumeFileName)
        masterNode.createObject('MechanicalObject', src="@loader", name="Volume")
        masterNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        masterNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        masterNode.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        masterNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        masterNode.createObject('UniformMass', totalMass=self.totalMass)

        # fixed BCs
        masterNode.createObject('BoxROI', box='-0.05 -0.05 -0.002 0.05 0.05 0.002  -0.05 -0.05  0.298 0.05 0.05 0.302', name='fixedBox')
        masterNode.createObject('FixedConstraint', indices='@fixedBox.indices')

        # material parameters 
        masterNode.createObject('OptimParams', name="paramE", optimize="1", numParams='10', template="Vector", initValue=self.paramInitExp, stdev=self.paramInitSD, transformParams="1")
        masterNode.createObject('Indices2ValuesMapper', name='youngMapper', indices='1 2 3 4 5 6 7 8 9 10', values='@paramE.value', inputValues='@loader.dataset')
        masterNode.createObject('TetrahedronFEMForceField', name='FEM', updateStiffness='1', listening='true', drawHeterogeneousTetra='1', method='large', poissonRatio='0.45', youngModulus='@youngMapper.outputValues')

        # groundtruth observations node
        obsNode = masterNode.createChild('obsNode')
        obsNode.createObject('MeshVTKLoader', name='obsLoader', filename=self.observationPointsVTK)        
        obsNode.createObject('MechanicalObject', name='SourceMO', src="@obsLoader")
        obsNode.createObject('BarycentricMapping')
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev=self.obsNoiseSD, noiseStdev="0.0", listening="1", stateWrapper="@../StateWrapper", verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.observationFileName)
        obsNode.createObject('ShowSpheres', name="estimated", radius="0.002", color="1 0 0 1", position='@SourceMO.position')
        obsNode.createObject('ShowSpheres', name="groundTruth", radius="0.0015", color="1 1 0 1", position='@MOBS.mappedObservations')

        return 0



    def initGraph(self,node):
        print 'Init graph called (python side)'
        self.step = 0
        self.total_time = 0
        # self.process.initializationObjects(node)

        return 0



    def onEndAnimationStep(self, deltaTime):  

        if self.saveState:
            if self.filterType == 'ROUKF':
                rs=self.filter.findData('reducedState').value
            elif self.filterType == 'UKFSimCorr' or self.filterType == 'UKFClassic':
                rs=self.filter.findData('state').value
            reducedState = [val for sublist in rs for val in sublist]
            #print 'Reduced state:'
            #print reducedState

            f1 = open(self.stateExpFile, "a")
            f1.write(" ".join(map(lambda x: str(x), reducedState)))
            f1.write('\n')
            f1.close()

            if self.filterType == 'ROUKF':
                rv=self.filter.findData('reducedVariance').value
            elif self.filterType == 'UKFSimCorr' or self.filterType == 'UKFClassic':
                rv=self.filter.findData('variance').value
            reducedVariance = [val for sublist in rv for val in sublist]
            #print 'Reduced variance:'
            #print reducedVariance

            f2 = open(self.stateVarFile, "a")
            f2.write(" ".join(map(lambda x: str(x), reducedVariance)))
            f2.write('\n')
            f2.close()

            if self.filterType == 'ROUKF':
                rcv=self.filter.findData('reducedCovariance').value
            elif self.filterType == 'UKFSimCorr' or self.filterType == 'UKFClassic':
                rcv=self.filter.findData('covariance').value
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

