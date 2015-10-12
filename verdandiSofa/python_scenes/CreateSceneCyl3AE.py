import Sofa
import math
import os

import numpy as np
from decimal import Decimal

from CreateParallelizableScene import createParallelizableScene

# Class definition 
class CreateScene:   

############################"  
# Parameters definitions :
############################"

    # Set to true if you want to run batch of simulation
    runFromScript=True    

    # counts
    
    m_sigmaPointType = 'simplex'
    m_slaveSceneCount = 0
    m_saveToFile = 0

    m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method



    #############################################################################################################################   
    
    if(runFromScript):
        #Get environment variable
        sofapythonData= os.environ["SofapythonData"] # command line parameters 
    
        token=sofapythonData.split("_")
        if len(token)>32:
            print 'Python: Too many parameters'
            print "Usage : sh Test_python.sh sigmaPointType_threadCount_saveToFile[0/1]"
            print "Example : sh Test_python.sh simplex_4_1 "
            print " "
            print " Note: Use threadCount:=0 for serialized version."
            print " "
            print " "
        if len(token)<3:
            print 'Python: Save to file not specified. Not saving.'
        else:
            m_saveToFile=int(token[2])

        if len(token)<2:
            print 'Python: Thread count not given, creating serialized scene.'
            m_slaveSceneCount = 0
        else:
            m_slaveSceneCount = int(token[1])

        if len(token)<1:
            print 'Python: Sigma count not given, using simplex.'
        else:
            if (token[0] == 'simplex') or (token[0] == 'canonical') or (token[0] == 'star'):
                m_sigmaPointType = token[0]
                print m_sigmaPointType

    
    
    

    ########################################################################################################################################
    #                            Node creation                                                   
    ########################################################################################################################################     


    ## create a scene using ordinary SofaModelWrapper ##
    def createConsciseScene(self, node):
        # scene global stuff
        node.createObject('RequiredPlugin', pluginName='Optimus', name='Optimus')
        node.createObject('RequiredPlugin', pluginName='SofaPardisoWrapper', name='SofaPardisoWrapper')
        node.createObject('RequiredPlugin', pluginName='BilikimoAux', name='BilikimoAux')
        node.findData('gravity').value="0 -9.81 0"
        node.findData('dt').value="0.01"

        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1400 1000', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('VerdandiAnimationLoop', name="verdAnimLoop", verbose="0")
        
        # simplex, canoncal, star
        ROUKF = node.createObject('SofaReducedOrderUKF', name="sofaROUKF", paramFileName="daCyl3AE/surfNoise2_params.out", paramVarFileName="")
        ROUKF.findData('sigmaPointType').value=self.m_sigmaPointType
        node.createObject('OptimParams', name="paramE", template="Vector", initValue="6000 6000 6000", stdev="2000 2000 2000", transformParams="1")
        
        node.createObject('MeshVTKLoader', filename="../scenes/data/cylinder3_770.vtk", name="loader")
        node.createObject('Indices2ValuesMapper', name="youngMapper", inputValues="@loader.dataset", indices="1 2 3", values="@paramE.value")
        
        # place the cylinder inside the root node
        Cylinder = node

        Cylinder.createObject('StaticSolver', applyIncrementFactor="1")
        PARDISO = Cylinder.createObject('SparsePARDISOSolver')
        if (self.m_saveToFile != 0):
            PARDISO.findData("saveDataToFile").value="1"
        Cylinder.createObject('MechanicalObject', src="@loader", name="Volume")
        Cylinder.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        Cylinder.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        Cylinder.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo", template="Vec3d")
        Cylinder.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
        Cylinder.createObject('UniformMass', totalMass="0.2513")
        Cylinder.createObject('BoxROI', name="fixedBox1", box="-0.05 -0.05 -0.002   0.05 0.05 0.002")
        Cylinder.createObject('BoxROI', name="fixedBox2", box="-0.05 -0.05  0.238   0.05 0.05 0.242")
        Cylinder.createObject('MergeSets', name="mergeIndices", in1="@fixedBox1.indices", in2="@fixedBox2.indices")
        Cylinder.createObject('FixedConstraint', indices="@mergeIndices.out")
        Cylinder.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="@youngMapper.outputValues", poissonRatio="0.45", method="large", computeVonMisesStress="0", drawHeterogeneousTetra="1")

        Obs = node.createChild('obsNode')
        Obs.createObject('MechanicalObject', name='SourceMO', position="0.0 0.0 0.02  0.0 0.0 0.04   0.0 0.0 0.08   0.0 0.0 0.09   0.0 0.0 0.12   0.0 0.0 0.13   0.0 0.0 0.14  0.0 0.0 0.17  0.0 0.0 0.19  0.0 0.0 0.22")
        Obs.createObject('Sphere', radius="0.002", color="1 0 0 1")
        Obs.createObject('BarycentricMapping')
        Obs.createObject('MappedPointsObservationManager', name="MOBS", observationStdev="2e-3", noiseStdev="2e-3", listening="1")
        Obs.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix="../scenes/observations/cylinder3_770_YMStat")

        Src = node.createChild('SourceMO')
        Src.createObject('MechanicalObject', name="SourceMO", position="@/obsNode/MOBS.mappedObservations")
        Src.createObject('Sphere', radius="0.002", color="0 0 0.3 1")
        
    def createGlobalComponents(self, node):
        # scene global stuff
        node.createObject('RequiredPlugin', pluginName='Optimus', name='Optimus')
        node.createObject('RequiredPlugin', pluginName='SofaPardisoWrapper', name='SofaPardisoWrapper')
        node.createObject('RequiredPlugin', pluginName='BilikimoAux', name='BilikimoAux')
        node.findData('gravity').value="0 -9.81 0"
        node.findData('dt').value="0.01"
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1400 1000', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('VerdandiAnimationLoop', name="verdAnimLoop", verbose="0")

        #simplex, canonical, star
        ROUKF = node.createObject('SofaReducedOrderUKFParallel', name="sofaROUKF", sigmaPointType="simplex", paramFileName="daCyl3AE/surfNoise2_params.out", paramVarFileName="")
        ROUKF.findData('sigmaPointType').value=self.m_sigmaPointType
        node.createObject('MeshVTKLoader', filename="../scenes/data/cylinder3_770.vtk", name="loader")


    def createMasterScene(self, node):

        node.createObject('OptimParams', name="paramMaster", optimize="0", template="Vector", initValue="6000 6000 6000", stdev="2000 2000 2000", transformParams="1", loader="@/loader")
	#node.createObject('Indices2ValuesMapper', name="youngSlaveMapper", inputValues="@/loader.dataset", indices="1 2 3", values="@paramMaster.value")

        Cylinder = node.createChild('Cylinder')
        Cylinder.findData('activated').value="1"

        Cylinder.createObject('StaticSolver', applyIncrementFactor="1")

        PARDISO = Cylinder.createObject('SparsePARDISOSolver')
        if (self.m_saveToFile != 0):
            pardisoLabel = "Master"
            PARDISO.findData("saveDataToFile").value="1"
            PARDISO.findData("fileLabel").value=pardisoLabel
        Cylinder.createObject('MechanicalObject', src="@/loader", name="Volume")
        Cylinder.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        Cylinder.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        Cylinder.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo", template="Vec3d")
        Cylinder.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
        Cylinder.createObject('UniformMass', totalMass="0.2513")
        Cylinder.createObject('BoxROI', name="fixedBox1", box="-0.05 -0.05 -0.002   0.05 0.05 0.002")
        Cylinder.createObject('BoxROI', name="fixedBox2", box="-0.05 -0.05  0.238   0.05 0.05 0.242")
        Cylinder.createObject('MergeSets', name="mergeIndices", in1="@fixedBox1.indices", in2="@fixedBox2.indices")
        Cylinder.createObject('FixedConstraint', indices="@mergeIndices.out")
        Cylinder.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="@../paramMaster.value", poissonRatio="0.45", method="large", computeVonMisesStress="0", drawHeterogeneousTetra="1")


        Obs = Cylinder.createChild('obsNode')
        # test=Obs.createObject('MechanicalObject', name='SourceMO', position="0.0 0.0 0.02  0.0 0.0 0.04   0.0 0.0 0.08   0.0 0.0 0.09   0.0 0.0 0.12   0.0 0.0 0.13   0.0 0.0 0.14  0.0 0.0 0.17  0.0 0.0 0.19  0.0 0.0 0.22")
        #test=Obs.createObject('MechanicalObject', name='SourceMO', position="@../Volume.position")
        test=Obs.createObject('MechanicalObject', name='SourceMO', position="0.02 0 0.08    0.02 0 0.16    0.0141 0.0141 0.08    0.0141 -0.0141 0.08    0.0141 0.0141 0.16    0.0141 -0.0141 0.16    0.02 0 0.0533    0.02 0 0.107   \
		    0.02 0 0.133    0.02 0 0.187    0.02 0 0.213    0.0175 0.00961 0.0649    0.00925 0.0177 0.0647    0.0139 0.0144 0.0398    0.00961 -0.0175 0.0649    0.0177 -0.00925 0.0647  \
		    0.0144 -0.0139 0.0402    0.0177 0.00936 0.145    0.0095 0.0176 0.145    0.0175 0.00961 0.0951    0.00925 0.0177 0.0953    0.0139 0.0144 0.12    0.00937 -0.0177 0.145   \
		    0.0176 -0.00949 0.145    0.00935 -0.0177 0.0953    0.0176 -0.00949 0.095    0.0142 -0.0141 0.12    0.0177 0.00937 0.175    0.00949 0.0176 0.175    0.014 0.0143 0.2   \
		    0.00959 -0.0175 0.175    0.0177 -0.00924 0.175    0.0143 -0.014 0.2")
        Obs.createObject('Sphere', radius="0.002", color="1 0 0 1")
        Obs.createObject('BarycentricMapping')
        Obs.createObject('MappedPointsObservationManagerParallel', name="MOBS", observationStdev="2e-3", noiseStdev="2e-3", listening="1")
        Obs.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix="../scenes/observations/cylinder3_770_YMStat")

        Src = Cylinder.createChild('SourceNode')
        Src.createObject('MechanicalObject', name="aux_Source", position="@../obsNode/MOBS.mappedObservations")
        Src.createObject('Sphere', radius="0.002", color="0 0 0.3 1")
 
    def createSlaveScene(self, node):    

        node.createObject('OptimParams', name="paramSlave", optimize="0", template="Vector", initValue="6000 6000 6000", stdev="2000 2000 2000", transformParams="1", loader="@/loader")
	#node.createObject('Indices2ValuesMapper', name="youngSlaveMapper", inputValues="@/loader.dataset", indices="1 2 3", values="@paramSlave.value")

        Cylinder = node.createChild('Cylinder')
        Cylinder.findData('activated').value="1"

        Cylinder.createObject('StaticSolver', applyIncrementFactor="1")
        PARDISO = Cylinder.createObject('SparsePARDISOSolver')
        if (self.m_saveToFile != 0):
            pardisoLabel = "Slave"+ str(self.m_slaveScenesCreated)
            PARDISO.findData("saveDataToFile").value="1"
            PARDISO.findData("fileLabel").value=pardisoLabel
        Cylinder.createObject('MechanicalObject', src="@/loader", name="Volume")
        Cylinder.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        Cylinder.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        Cylinder.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo", template="Vec3d")
        Cylinder.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
        Cylinder.createObject('UniformMass', totalMass="0.2513")
        Cylinder.createObject('BoxROI', name="fixedBox1", box="-0.05 -0.05 -0.002   0.05 0.05 0.002")
        Cylinder.createObject('BoxROI', name="fixedBox2", box="-0.05 -0.05  0.238   0.05 0.05 0.242")
        Cylinder.createObject('MergeSets', name="mergeIndices", in1="@fixedBox1.indices", in2="@fixedBox2.indices")
        Cylinder.createObject('FixedConstraint', indices="@mergeIndices.out")
        Cylinder.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="@../paramSlave.value", poissonRatio="0.45", method="large", computeVonMisesStress="0", drawHeterogeneousTetra="1")

        self.m_slaveScenesCreated+=1

    ########################################################################################################################################
    #                            Scene creation                                                   
    ########################################################################################################################################
    def createScene(self,node):


        if self.m_slaveSceneCount==0:
            print 'Python: Creating serialized scene: ', self.m_sigmaPointType,'.'
            self.createConsciseScene(node)
        else:
            print 'Python: Creating parallelized scene:', self.m_sigmaPointType,',', self.m_slaveSceneCount, 'threads.'
            createParallelizableScene(self, node, self.m_slaveSceneCount)

    
        return 0




    #############################################################################################################
    # Initialization functions
    #
   
               
    def initializationObjects(self,node):          
        self.cameraReactivated=False

        self.rootNode=node
      
        if(self.runFromScript):
            self.rootNode.getRootContext().animate = False
           
         
        print  "Init Graph called (Python side)"
        print " \n \n \n"
        return 0

