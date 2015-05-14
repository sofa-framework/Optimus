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
    slaveSceneCount = 0



    #############################################################################################################################   
    
    if(runFromScript):
        #Get environment variable
        sofapythonData= os.environ["SofapythonData"] # / parametry prikazove radky /
    
        token=sofapythonData.split("_")
        if len(token)!=1:
            pass
            print " "
            print " "
            print " "
            print "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
            print "X                                    ERROR                                    X"
            print "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
            print " "
            print "Usage : no params needed at this point"
            print " "
            print " "
            print " "
            print " "
        else:
             slaveSceneCount = int(token[0])

    
    
    

    ########################################################################################################################################
    #                            Node creation                                                   
    ########################################################################################################################################     


    ## create an exact copy of IP scene ##
    def createConsciseScene(self, node):
        # scene global stuff
        node.createObject('RequiredPlugin', pluginName='Optimus', name='Optimus')
        node.createObject('RequiredPlugin', pluginName='SofaPardisoWrapper', name='SofaPardisoWrapper')
        node.createObject('RequiredPlugin', pluginName='BilikimoAux', name='BilikimoAux')
        node.findData('gravity').value="0 -9.81 0"
        node.findData('dt').value="0.01"
        #node.findData('showBoundingTree').value="0"

        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1400 1000', objectPickingMethod='Ray casting') # describes resolution # #camera mode, object picking method? #  
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels') # umoznuje nastavit co bude videt na startupu, viz popisek ale hideVisual napriklad #

        node.createObject('VerdandiAnimationLoop', name="verdAnimLoop", verbose="0")
        node.createObject('SofaReducedOrderUKFParallel', name="sofaROUKF", sigmaPointType="simplex", paramFileName="daHeteroCylinderConstant/pHardSmoothIm3_estim.out", paramVarFileName="daHeteroCylinderConstant/pHardSmoothIm3_var.out")
        node.createObject('OptimParams', name="paramE", template="Vector", initValue="6000 6000 6000", stdev="2000 2000 2000", transformParams="1")
        
        node.createObject('MeshVTKLoader', filename="data/cylinder3_770.vtk", name="loader")
        node.createObject('Indices2ValuesMapper', name="youngMapper", inputValues="@loader.dataset", indices="1 2 3", values="@paramE.value")
        
        # place the cylinder inside the root node
        Cylinder = node

        Cylinder.createObject('StaticSolver', applyIncrementFactor="1")
        Cylinder.createObject('SparsePARDISOSolver')
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
        Obs.createObject('MappedPointsObservationManagerParallel', name="MOBS", observationStdev="2e-3", noiseStdev="2e-3", listening="1")
        Obs.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix="cylinderModulusMonitorStatEvolve1Smooth")

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
        #node.findData('showBoundingTree').value="0"
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1400 1000', objectPickingMethod='Ray casting') # describes resolution # #camera mode, object picking method? #  
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels') # umoznuje nastavit co bude videt na startupu, viz popisek ale hideVisual napriklad #

        node.createObject('VerdandiAnimationLoop', name="verdAnimLoop", verbose="0")
        node.createObject('SofaReducedOrderUKFParallel', name="sofaROUKF", sigmaPointType="simplex", paramFileName="daHeteroCylinderConstant/pHardSmoothIm3_estim.out", paramVarFileName="daHeteroCylinderConstant/pHardSmoothIm3_var.out")
        
        node.createObject('MeshVTKLoader', filename="data/cylinder3_770.vtk", name="loader")


    def createSlaveScene(self, node):

        node.createObject('OptimParams', name="paramSlave", optimize="0", template="Vector", initValue="6000 6000 6000", stdev="2000 2000 2000", transformParams="1")
	node.createObject('Indices2ValuesMapper', name="youngSlaveMapper", inputValues="@/loader.dataset", indices="1 2 3", values="@paramSlave.value") # check if visitor works fine, @works fine?

        Cylinder = node.createChild('Cylinder')
        Cylinder.findData('activated').value="1"

        Cylinder.createObject('StaticSolver', applyIncrementFactor="1")
        Cylinder.createObject('SparsePARDISOSolver')
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
        Cylinder.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="@../youngSlaveMapper.outputValues", poissonRatio="0.45", method="large", computeVonMisesStress="0", drawHeterogeneousTetra="1")
        

        return
        Cylinder = node.createChild('Cylinder5')
        Cylinder.findData('activated').value="1"

        Cylinder.createObject('StaticSolver', applyIncrementFactor="1")
        Cylinder.createObject('SparsePARDISOSolver')
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
        Cylinder.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="@../youngSlaveMapper.outputValues", poissonRatio="0.45", method="large", computeVonMisesStress="0", drawHeterogeneousTetra="1")

        #return
        Cylinder = node.createChild('al')
        Cylinder.findData('activated').value="1"

        Cylinder.createObject('StaticSolver', applyIncrementFactor="1")
        Cylinder.createObject('SparsePARDISOSolver')
        Cylinder.createObject('MechanicalObject', src="@/loader", name="wolume")
        Cylinder.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        Cylinder.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        Cylinder.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo", template="Vec3d")
        Cylinder.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
        Cylinder.createObject('UniformMass', totalMass="0.2513")
        Cylinder.createObject('BoxROI', name="fixedBox1", box="-0.05 -0.05 -0.002   0.05 0.05 0.002")
        Cylinder.createObject('BoxROI', name="fixedBox2", box="-0.05 -0.05  0.238   0.05 0.05 0.242")
        Cylinder.createObject('MergeSets', name="mergeIndices", in1="@fixedBox1.indices", in2="@fixedBox2.indices")
        Cylinder.createObject('FixedConstraint', indices="@mergeIndices.out")
        Cylinder.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="@../youngSlaveMapper.outputValues", poissonRatio="0.45", method="large", computeVonMisesStress="0", drawHeterogeneousTetra="1")

        #return
        Cylinder = node.createChild('fantomas')
        Cylinder.findData('activated').value="1"

        Cylinder.createObject('StaticSolver', applyIncrementFactor="1")
        Cylinder.createObject('SparsePARDISOSolver')
        Cylinder.createObject('MechanicalObject', src="@/loader", name="fen")
        Cylinder.createObject('TetrahedronSetTopologyModifier', name="Modifier")
        Cylinder.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/loader", tags=" ")
        Cylinder.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo", template="Vec3d")
        Cylinder.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
        Cylinder.createObject('UniformMass', totalMass="0.2513")
        Cylinder.createObject('BoxROI', name="fixedBox1", box="-0.05 -0.05 -0.002   0.05 0.05 0.002")
        Cylinder.createObject('BoxROI', name="fixedBox2", box="-0.05 -0.05  0.238   0.05 0.05 0.242")
        Cylinder.createObject('MergeSets', name="mergeIndices", in1="@fixedBox1.indices", in2="@fixedBox2.indices")
        Cylinder.createObject('FixedConstraint', indices="@mergeIndices.out")
        Cylinder.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus="@../youngSlaveMapper.outputValues", poissonRatio="0.45", method="large", computeVonMisesStress="0", drawHeterogeneousTetra="1")
        
        #node.createObject('VisualStyle', name='VisualStyle', displayFlags='hideAll') #add when final	


    def createObserverNode(self, node):

        Obs = node.createChild('obsNode')
        test=Obs.createObject('MechanicalObject', name='SourceMO', position="0.0 0.0 0.02  0.0 0.0 0.04   0.0 0.0 0.08   0.0 0.0 0.09   0.0 0.0 0.12   0.0 0.0 0.13   0.0 0.0 0.14  0.0 0.0 0.17  0.0 0.0 0.19  0.0 0.0 0.22")
        Obs.createObject('Sphere', radius="0.002", color="1 0 0 1")
        Obs.createObject('BarycentricMapping', input='@../Cylinder')
        Obs.createObject('MappedPointsObservationManagerParallel', name="MOBS", observationStdev="2e-3", noiseStdev="2e-3", listening="1")
        Obs.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix="cylinderModulusMonitorStatEvolve1Smooth")

        Src = node.createChild('SourceNode')
        Src.createObject('MechanicalObject', name="aux_Source", position="@../obsNode/MOBS.mappedObservations")
        Src.createObject('Sphere', radius="0.002", color="0 0 0.3 1")

        

    ########################################################################################################################################
    #                            Nodes creation                                                   
    ########################################################################################################################################
    def createScene(self,node):

        print "slaveSceneCount is", self.slaveSceneCount
        if self.slaveSceneCount==0:
            print 'Concise requested\n'
            self.createConsciseScene(node)
        else:
            createParallelizableScene(self, node, self.slaveSceneCount)
       
        ## Create cylinder ##############################################################################################
        
       
        ## Create source #################################################################################

    
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
