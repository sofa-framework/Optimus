import Sofa
import math
import os

# Class definition 
class pigLiver1_BCDA(Sofa.PythonScriptController):

    def createGraph(self,node):          
        print  "Create graph called (Python side)\n"

        # filtering parameters parameters:
        self.givenStiffness=-1             # <0: do filtering,  >=0: no filtering, value determines the actual stiffness        

        self.estimatedStiffness=[]

        #self.estimatedSource='psd0.01#osd1e-06#ctr3#MJED-estimated100';
        #self.estimatedStiffness=[0.253677955647,0.0326708350841,0.016893126874,0.0528991719788,0.033124206143,-0.0222311340379,-0.025892272203,0.0106244850782,0.0823942896652,4.01215626782,-0.00163784338652,-0.120171241699,0.064212686568,-0.019688452053,21.1073659023,-0.0517618671309,-0.022609122988,0.285343154729,0.116221774541,-0.114189303431,-0.0121011793543,-0.0460738863449,-0.0104024476007,0.0680537684953,0.868422553701,-0.0182715795728,0.0270589524903,6.58013912082,0.043059799363,-0.0373155274026,24.0694679584,29.6835479623,58.2875337191,0.0135453006207,0.0247029736874]
        
        #self.estimatedSource='psd0.0002#osd1e-06#ctr0#MJED-estimated130'
        #self.estimatedStiffness=[0.0503930003429,0.118892643534,0.352365299848,2.55007805898,0.280714598378,0.170030317639,0.102737512529,0.262473186394,0.313235176649,0.236778939147,0.0417311014687,2.49618192949,1.40611374571,3.34133828332,4.3218739461,0.344480530676,0.303068988454,3.9829290212,0.141242705325,0.110135081928,0.315370320569,0.196903180664,5.70428436096,5.37791658386,0.228066881081,0.144180183905,0.118512825719,0.410681847038,0.361364755675,0.16069332385,3.04463362132,0.527573642036,0.478733701219,0.119395325158,0.106566095058]
        
        self.statParams=[0, 0.01, 1e-6]           # initial param value, init param stdev, measurement stdev
        self.fixedIX=[56, 57, 78, 81, 82, 83, 85, 86, 95, 96, 98, 124, 125, 126, 127, 133, 135, 141, 142, 143, 144, 145, 153, 154, 158, 159, 183, 185, 187, 188, 209, 212, 213, 221, 232]        
        #self.fixedIX=[56]
        self.obsControl = 3                  # 0: pull only in tool points, 1: pull also in auxiliary points               

        print len(self.estimatedStiffness)

        self.nu=0.45
        self.E=5000
        self.material="MJED"                    
        
        # visualization
        self.sr1=0.001
        self.sr2=0.0012
        self.sr3=0.0014 
        self.sr4=0.0016  
        self.sr5=0.0018        
        self.drawBoxes=0           
        self.dispFeatures=0

        # parallelization
        self.m_slaveSceneCount = 0     
        self.m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method

        # saving to files and printing
        self.saveState=0
        self.saveToolForces=0
        self.saveFeatError=0
        self.saveTrackedPositions=0

        #initial registration parameters
        self.videoScaleFactor='0.000342'
        self.featuresScaleFactor=self.videoScaleFactor+' '+self.videoScaleFactor+' '+self.videoScaleFactor
        self.meshRotation='-15 180 191'
        self.meshTranslation='0.156 0.073 0.002'
        self.meshScale='1.05 1 1'     

        if len(self.estimatedStiffness) > 0:
            self.suffix=self.estimatedSource
        else:
            if self.givenStiffness < 0:
                self.suffix='psd'+str(self.statParams[1])+'#osd'+str(self.statParams[2])+'#ctr'+str(self.obsControl)+'#'+self.material
            else:
                self.suffix = 'givenStiffness'+str(self.givenStiffness)+'#ctr'+str(self.obsControl)+'#'+self.material

        print '############ Storing to '+self.suffix
        
        os.system('mkdir -p outLiver')
        if self.saveState:
            self.stateExpFile='outLiver/state_'+self.suffix+'.txt'
            self.stateVarFile='outLiver/variance_'+self.suffix+'.txt'
            self.stateCovarFile='outLiver/covariance_'+self.suffix+'.txt'
            os.system('rm '+self.stateExpFile)
            os.system('rm '+self.stateVarFile)
            os.system('rm '+self.stateCovarFile)
      
        if self.saveToolForces:
            self.toolForceFile='outLiver/toolForce_'+self.suffix+'.txt'
            os.system('rm '+self.toolForceFile);

        if self.saveFeatError:
            self.featErrorFile='outLiver/featError_'+self.suffix+'.txt'
            os.system('rm '+self.featErrorFile);

        if self.saveTrackedPositions:
            self.toolFile='outLiver/toolPoints.txt';
            self.obsFile='outLiver/obsPoints.txt';
            self.assFile='outLiver/assessPoints.txt';
            os.system('rm '+self.toolFile+' '+self.obsFile+' '+self.assFile);


        # create the simulation graph
        self.nFixedIX=len(self.fixedIX)
        self.rootNode=node
        node.findData('dt').value="1"
        self.createScene(node)
        
        return 0

    
    def createGlobalComponents(self, node):
        # scene global stuff
        node.createObject('RequiredPlugin', pluginName='Optimus', name='Optimus')
        node.createObject('RequiredPlugin', pluginName='SofaPardisoSolver', name='SofaPardisoSolver')
        node.createObject('RequiredPlugin', pluginName='ImageMeshAux', name='ImageMeshAux')
        node.createObject('RequiredPlugin', pluginName='image', name='image')
        node.createObject('RequiredPlugin', pluginName='OpticalFlow', name='OpticalFlow')
        node.createObject('RequiredPlugin', pluginName='SofaMJEDFEM')
        
        node.findData('gravity').value="0 0 0"
        node.findData('dt').value="1"
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels hideForceFields showCollisionModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")        
        self.filter=node.createObject('ROUKFilter', name="ROUKF", verbose="1")        

        node.createObject('MeshVTKLoader', name='objectLoader', filename='../../data/pigLiver/liverLobe0_992.vtk', scale3d=self.meshScale, translation=self.meshTranslation, rotation=self.meshRotation)
        node.createObject('MeshSTLLoader', name='objectSLoader', filename='../../data/pigLiver/liverLobe0_3914.stl', scale3d=self.meshScale, translation=self.meshTranslation, rotation=self.meshRotation)
        node.createObject('MeshVTKLoader', name='fixedPointsLoader', filename='../../data/pigLiver/fixedPoints992_35.vtk', scale3d=self.meshScale, translation=self.meshTranslation, rotation=self.meshRotation)

        
        features = node.createChild('features')
        features.createObject('PreStochasticWrapper')
        features.createObject('LKOpticalFlowTrackerSimple', maskName='', vidName='../../data/pigLiver/porcineLiverCut1.avi', name='LK', winSize='31', detectorThresh='190', scaleImg=self.videoScaleFactor, displayFeatures=self.dispFeatures, view='1')
        features.createObject('TransformEngine',name='scaleFeatures',input_position='@LK.outputFeatures',scale=self.featuresScaleFactor)
        features.createObject('MechanicalObject', position='@scaleFeatures.output_position', name='DOFs')
        
        features.createObject('BoxROI', name="Z0", box='0.18 0.045 -0.001   0.21 0.055 0.001', drawBoxes=self.drawBoxes, drawSize="3", position='@DOFs.rest_position')
        features.createObject('BoxROI', name="Z1", box='0.115 0.060 -0.001   0.125 0.065 0.001', drawBoxes=self.drawBoxes, drawSize="3", position='@DOFs.rest_position')
        #features.createObject('BoxROI', name="Z1", box='0.125 0.040 -0.001   0.135 0.045 0.001', drawBoxes=self.drawBoxes, drawSize="3", position='@DOFs.rest_position')
        # features.createObject('BoxROI', name="Z1", box='0.137 0.042 -0.001   0.147 0.046 0.001', drawBoxes=self.drawBoxes, drawSize="3", position='@DOFs.rest_position')
        features.createObject('BoxROI', name="Z2", box='0.11 0.090 -0.001   0.12 0.094 0.001', drawBoxes=self.drawBoxes, drawSize="3", position='@DOFs.rest_position')

        features.createObject('BoxROI', name="Z3", box='0.11 0.110 -0.001   0.12 0.115 0.001', drawBoxes=self.drawBoxes, drawSize="3", position='@DOFs.rest_position')       
        features.createObject('BoxROI', name="Z4", box='0.17 0.110 -0.001   0.18 0.115 0.001', drawBoxes=self.drawBoxes, drawSize="3", position='@DOFs.rest_position')       

        features.createObject('BoxROI', name="Z10", box='0.1 0.03 -0.001   0.16 0.12 0.001', drawBoxes=self.drawBoxes, drawSize="3", position='@DOFs.rest_position')
        features.createObject('BoxROI', name="Z11", box='0.16 0.083 -0.001   0.22 0.12 0.001', drawBoxes=self.drawBoxes, drawSize="3", position='@DOFs.rest_position') 


        
        if self.obsControl == 0:    # only the tool            
            features.createObject('MergeVectors', name='ControlIx', nbInputs='1', input1='@Z0.indices')            
            
        elif self.obsControl == 1:            
            features.createObject('MergeVectors', name='ControlIx', nbInputs='3', input1='@Z0.indices', input2='@Z1.indices', input3='@Z2.indices')
        
        elif self.obsControl == 2:
            features.createObject('MergeVectors', name='ControlIx', nbInputs='2', input1='@Z0.indices', input2='@Z1.indices')

        elif self.obsControl == 3:
            features.createObject('MergeVectors', name='ControlIx', nbInputs='2', input1='@Z0.indices', input2='@Z2.indices')

        features.createObject('MergeVectors', name='ObservIx', nbInputs='2', input1='@Z10.indices', input2='@Z11.indices')
        features.createObject('MergeVectors', name='LandmIx', nbInputs='3', input1='@Z1.indices', input2='@Z3.indices', input3='@Z4.indices')

        
        features.createObject('PointsFromIndices', template='Vec3d', name='ControlPoints', indices='@ControlIx.output')
        features.createObject('PointsFromIndices', template='Vec3d', name='ObservPoints', indices='@ObservIx.output')        
        features.createObject('PointsFromIndices', template='Vec3d', name='LandmarkPoints', indices='@LandmIx.output')

        toolNode = node.createChild('tool')
        self.trackToolMO=toolNode.createObject('MechanicalObject', scale3d='1 1 1', position='@/features/ControlPoints.indices_position', name='MO')
        toolNode.createObject('Sphere', color='0.0 0.0 1.0 1', radius=self.sr5, template='Vec3d')
        # toolNode.createObject('BoxROI', name="allBox", box='-1 -1 -1 1 1 1', drawBoxes="0", drawSize="3")
        # toolNode.createObject('Monitor', name='outLiver/tool', showPositions='1', indices="@allBox.indices", ExportPositions="1", ExportVelocities="0", ExportForces="0")

        obsNode = node.createChild('obs')
        self.trackObsMO=obsNode.createObject('MechanicalObject', scale3d='1 1 1', position='@/features/ObservPoints.indices_position', name='MO')
        obsNode.createObject('Sphere', color='0.0 0.5 0.0 1', radius=self.sr4, template='Vec3d')

        landNode = node.createChild('land')
        self.imageLM=landNode.createObject('MechanicalObject', scale3d='1 1 1', position='@/features/LandmarkPoints.indices_position', name='MO')
        landNode.createObject('Sphere', color='1.0 0 1.0 1', radius=self.sr5, template='Vec3d')

        return 0
        


    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createCommonComponents(self, node):                    
        node.createObject('NewtonStaticSolver', name="NewtonStatic", printLog="0", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt="2")   
        # node.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        node.createObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToDir="", iterativeSolverNumbering="0")

        node.createObject('MechanicalObject', src="@/objectLoader", name="Volume")
        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/objectLoader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        node.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        node.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")
        node.createObject('UniformMass', totalMass="0.2513")

        # node.createObject('BoxROI', name="fixedBox1", box="-0.001 0.052 -0.001   0.1631 0.054 0.0131")
        # node.createObject('BoxROI', name="fixedBox1", box='100 400 -100 700 500 100', drawBoxes='1')
        node.createObject('OptimParams', name="springStiffness", template="Vector", numParams=self.nFixedIX, initValue=self.statParams[0], stdev=self.statParams[1], transformParams="1", optimize="1", printLog="0")

        if len(self.estimatedStiffness) > 0:
            estimSpring=node.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points=self.fixedIX, stiffness=0, springThickness="3", listening="1",  showIndicesScale="0", updateStiffness="1", printLog="0")
            estimSpring.findData('stiffness').value = self.estimatedStiffness;
        else:
            if self.givenStiffness < 0:
                node.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points=self.fixedIX, stiffness="@springStiffness.value", showIndicesScale="0", springThickness="3", listening="1", updateStiffness="1", printLog="0")
            else:
                node.createObject('ExtendedRestShapeSpringForceField', name="fixedSpring", points=self.fixedIX, stiffness=self.givenStiffness,  showIndicesScale="0", springThickness="3", listening="1", updateStiffness="1", printLog="0")
                        
        node.createObject('ColorMap',colorScheme="Blue to Red")                                                                  
        
        E=self.E;
        nu=self.nu;        

        if self.material=='MJED':        
            lamb=(E*nu)/((1+nu)*(1-2*nu))
            mu=E/(2+2*nu)
            materialParams='{} {}'.format(mu,lamb)
            node.createObject('MJEDTetrahedralForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet=materialParams)
        elif self.material=='COROT':
            node.createObject('TetrahedronFEMForceField', name="FEM", listening="true", updateStiffness="1", youngModulus=E, poissonRatio=nu, method="large")

        # toolEmu = node.createChild('toolEmu')
        # toolEmu.createObject('MechanicalObject',name="MO",src="@/toolLoader")
        # toolEmu.createObject('SimulatedStateObservationSource', name="ToolA",printLog="1", monitorPrefix="../obsBoundaryDA/brickC_150316/toolA",drawSize="0.0015",controllerMode="1")
        
        node.createObject('Mapped3DoFForceField', mappedFEM="mappedTool/toolSpring", mappedMechObject="mappedTool/MO", mapping="mappedTool/baryMapping", printLog="0")        
        mappedTool = node.createChild('mappedTool')
        mappedTool.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@/features/ControlPoints.indices_position', name='MO')
        mappedTool.createObject('Sphere', color='0.0 1.0 1.0 1', radius=self.sr1, template='Vec3d')
        self.toolSprings=mappedTool.createObject('ExtendedRestShapeSpringForceField', external_rest_shape='/features/MO', numStepsSpringOn='10000', stiffness='1e8', name='toolSpring', springColor='0 1 0 1', updateStiffness='1', printLog='0', listening='1', angularStiffness='0', startTimeSpringOn='0')
        mappedTool.createObject('BarycentricMapping', name="baryMapping")


        return 0



    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        
        self.createCommonComponents(node)
        
        obsNode = node.createChild('obsNode')
        obsNode.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@/features/ObservPoints.indices_position', name='MO')
        #obsNode.createObject('Sphere', color='0.8 0.5 1.0 1', radius=self.sr1, template='Vec3d')
        obsNode.createObject('BarycentricMapping', name="baryMapping")
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev=self.statParams[2], noiseStdev="0.0", listening="1", stateWrapper="@../StateWrapper",doNotMapObservations="1",verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", trackedObservations='@/features/ObservPoints.indices_position', monitorPrefix="../obsBoundaryDA/brickC_150316/obsA")

        landNode = node.createChild('landNode')
        self.modelLM=landNode.createObject('MechanicalObject', scale3d='1 1 1', rest_position='@/features/LandmarkPoints.indices_position', name='MO')
        #landNode.createObject('Sphere', color='0.5 0.6 1.0 1', radius=self.sr1, template='Vec3d')
        landNode.createObject('BarycentricMapping', name="baryMapping")
        # landNode.createObject('TargetFeatureErrorMetric', targetPositions='@MO.position', sourcePosition='@/landNode/MO.position', )

        

        # obsNode = node.createChild('obsNode')
        # obsNode.createObject('MechanicalObject', name='MO', src="@/obsLoader")
        #obsNode.createObject('MechanicalObject', name='MO', position="0.01 0.06 0.01  0.01 0.06 0.04  0.01 0.06 0.07  0.03 0.06 0.01  0.03 0.06 0.04  0.03 0.06 0.07  0.05 0.06 0.01  0.05 0.06 0.04  0.05 0.06 0.07  0.07 0.06 0.01  0.07 0.06 0.04  0.07 0.06 0.07  0.09 0.06 0.01  0.09 0.06 0.04  0.09 0.06 0.07  0.11 0.06 0.01  0.11 0.06 0.04  0.11 0.06 0.07  0.13 0.06 0.01  0.13 0.06 0.04  0.13 0.06 0.07  0.15 0.06 0.01  0.15 0.06 0.04  0.15 0.06 0.07   0.18 0.01 0.01  0.18 0.01 0.04  0.18 0.01 0.07  0.18 0.03 0.01  0.18 0.03 0.04  0.18 0.03 0.07  0.18 0.05 0.01  0.18 0.05 0.04  0.18 0.05 0.07")
        # obsNode.createObject('Sphere', radius="0.001", color="0.2 0.8 0.2 1")
        # obsNode.createObject('BarycentricMapping')                   
        # obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev="1e-5", noiseStdev="0.0", listening="1", stateWrapper="@../StateWrapper",doNotMapObservations="1",verbose="1")
        # obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix="../obsBoundaryDA/brickC_150316/obsA")
    
        visNode = node.createChild('ObjectVisualization')
        visNode.createObject('VisualStyle', name='VisualStyle', displayFlags='showWireframe showBehaviorModels showForceFields showCollisionModels')
        visNode.createObject('MechanicalObject',src="@/objectSLoader",name="Surface")
        visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@/objectSLoader", tags=" ")
        visNode.createObject('TriangleSetTopologyModifier', name="Modifier")        
        visNode.createObject('TriangleSetTopologyAlgorithms', name="TopoAlgo")
        visNode.createObject('TriangleSetGeometryAlgorithms', name="GeomAlgo")        
        # visNode.createObject('Line',color="0 0 0 1")
        visNode.createObject('Triangle',color="0.8 0.8 0.8 1")
        visNode.createObject('BarycentricMapping')
        #visNode.createObject('VTKExporter',filename="vtkExp/beam",XMLformat="true",listening="true",edges="0",triangles="1",quads="0",tetras="0",exportAtBegin="1",exportAtEnd="0",exportEveryNumberOfSteps="1")

        obsVisuNode = node.createChild('ObservationVisualization')
        obsVisuNode.createObject('MechanicalObject', name="aux_Source", position="@../obsNode/MOBS.mappedObservations")
        # obsVisuNode.createObject('Sphere', radius=self.sr2, color="0.2 0.8 0.2 1")

        #node.createObject('MeshSubsetEngine', name='attachPoints', inputPosition='@../MO.position', indices=self.fixedIX)
        attachmentVisu=node.createChild('AttachmentVisu')        
        attachmentVisu.createObject('MechanicalObject', name='MO', src="@/fixedPointsLoader")
        attachmentVisu.createObject('Sphere', color='0.5 0.6 1.0 1', radius=self.sr4, template='Vec3d')


        return 0
 
 

    def createSlaveScene(self, node):
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='hideBehaviorModels hideForceFields hideCollisionModels')
        wrapper=node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        wrapper.findData("name").value = "StochasticWrapperSlave"
        wrapper.findData("slave").value = 1;        
        
        self.createCommonComponents(node)        
        self.m_slaveScenesCreated+=1

        return 0

    

    def createScene(self,node):
        r_slaves = [] # list of created auxiliary nodes
        self.createGlobalComponents(node)
                
        masterNode=node.createChild('MasterScene')
        self.createMasterScene(masterNode)        
        masterNode.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels')
        
        slaveSubordinate=node.createChild('SlaveSubordinate')
        for i in range(1,self.m_slaveSceneCount):
            slave=slaveSubordinate.createChild('SlaveScene_'+str(i))
            #slave.createObject('VisualStyle', name='VisualStyle', displayFlags='hideAll')
            self.createSlaveScene(slave)
            r_slaves.append(slave)        
            
        return 0
    

    def initGraph(self,node):
        print 'Init graph called (python side)'
        self.step    =     0
        self.total_time =     0

        # print self.fixedROI.findData("indices").value
        
        
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

        if self.saveToolForces:
            tsp=self.toolSprings.findData('totalForce').value
            f3 = open(self.toolForceFile, "a")
            f3.write(" ".join(map(lambda x: str(x), tsp[0])))
            f3.write('\n')
            f3.close()            
            print 'Tool forces:'
            print tsp

        if self.saveFeatError:
            pos1=self.imageLM.findData('position').value
            pos2=self.modelLM.findData('position').value

            print pos1
            dst=[]
            for i in range (0,3):
                ax=pos1[i][0] 
                ay=pos1[i][1]
                bx=pos2[i][0]
                by=pos2[i][1]

                dst.append(math.sqrt((ax-bx)**2 + (ay-by)**2))
                  
            f4 = open(self.featErrorFile, "a")        
            f4.write(" ".join(map(lambda x: str(x), dst)))  
            f4.write('\n')
            f4.close

        if self.saveTrackedPositions:
            posTool=self.trackToolMO.findData('position').value            
            posObs=self.trackObsMO.findData('position').value
            posAssess=self.imageLM.findData('position').value            
            g1 = open(self.toolFile, "a")
            g1.write(" ".join(map(lambda x: str(x), posTool)))  
            g1.write('\n')
            g1.close

            g2 = open(self.obsFile, "a")
            g2.write(" ".join(map(lambda x: str(x), posObs)))  
            g2.write('\n')
            g2.close

            g3 = open(self.assFile, "a")
            g3.write(" ".join(map(lambda x: str(x), posAssess)))  
            g3.write('\n')
            g3.close



        return 0

