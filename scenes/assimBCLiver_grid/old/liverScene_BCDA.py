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
    # rootNode.createObject('RequiredPlugin', pluginName='image', name='image')
    rootNode.createObject('RequiredPlugin', pluginName='BoundaryConditionsPlugin', name='BoundaryConditions')
    
    rootNode.createObject('PythonScriptController', name='LiverBCDA', filename=__file, classname='liverScene_BCDA')


# Class definition 
class liverScene_BCDA(Sofa.PythonScriptController):


    def createGraph(self,node):          
        self.cameraReactivated=False
        self.rootNode=node              
         
        print  "Create graph called (Python side)\n"

        E=5000.0
        nu=0.4
        lamb=(E*nu)/((1+nu)*(1-2*nu))
        mu=E/(2+2*nu)
        self.materialParams='{} {}'.format(mu,lamb)
        
        self.ogridID=4
        inputDir='observations'
        outDir='measurements'
        self.volumeVTK=inputDir+'/object_0.vtk'
        self.surfaceSTL='../../data/brickD/brickD_536.stl'
        self.obsVTK=inputDir+'/observations_0.vtk'
        self.impactVTK=inputDir+'/impact_0.vtk'

        self.obsMonitorPrefix=inputDir+'/observations'
        self.impactMonitorPrefix=inputDir+'/impact'
        
        self.m_slaveSceneCount = 0
        self.m_saveToFile = 0
        self.m_slaveScenesCreated = 0 # auxiliary, circumvents attribute of instance method

        #self.filterKind='ROUKF'
        self.filterKind='UKFSimCorr'
        #self.filterKind='UKFClassic'

        self.saveState = 1
        self.saveToolForces = 1
        self.saveAssess = 0

        self.suffix='test'   #psd'+str(self.paramInitSD)+'#osd'+str(self.obsInitSD)+'#ogrid'+str(self.ogridID)
        if self.saveState:
            self.stateExpFile=outDir+'/state_'+self.suffix+'.txt'
            self.stateVarFile=outDir+'/variance_'+self.suffix+'.txt'
            self.stateCovarFile=outDir+'/covariance_'+self.suffix+'.txt'
            os.system('rm '+self.stateExpFile)
            os.system('rm '+self.stateVarFile)
            os.system('rm '+self.stateCovarFile)
          
        if self.filterKind=='UKFSimCorr':
            self.estimPosition='0'
            self.estimVelocity='0'

        if self.filterKind=='UKFClassic':
            self.estimVelocity='0'

        self.paramInitExp = [100, 100]
        #self.paramInitExp = 1
        self.paramInitSD = 50
        self.obsInitSD= 1e-4

        if self.saveState:
            self.stateExpFile=outDir+'/state_'+self.suffix+'.txt'
            self.stateVarFile=outDir+'/variance_'+self.suffix+'.txt'
            self.stateCovarFile=outDir+'/covariance_'+self.suffix+'.txt'
            os.system('rm ' + self.stateExpFile)
            #fil = open(self.stateExpFile, "w")
            #fil.close()
            os.system('rm ' + self.stateVarFile)
            #fil = open(self.stateVarFile, "w")
            #fil.close()
            os.system('rm ' + self.stateCovarFile)
            #fil = open(self.stateCovarFile, "w")
            #fil.close()

      
        if self.saveToolForces:
            self.toolForceFile=outDir+'/toolForce_'+self.suffix+'.txt'
            os.system('rm '+self.toolForceFile);


        if self.saveAssess:
            self.assessFile=outDir+'/assess_'+self.suffix+'.txt'
            os.system('rm '+self.assessFile);

        try:
            os.stat(outDir)
        except:
            os.mkdir(outDir)            

        self.createScene(node)
        
        return 0

    
    def createGlobalComponents(self, node):
        # scene global stuff
        node.findData('gravity').value="0 0 0"
        node.findData('dt').value="0.01"
        
        node.createObject('ViewerSetting', cameraMode='Perspective', resolution='1000 700', objectPickingMethod='Ray casting')
        node.createObject('VisualStyle', name='VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels')

        node.createObject('FilteringAnimationLoop', name="StochAnimLoop", verbose="1")
        if (self.filterKind == 'ROUKF'):
            self.filter = node.createObject('ROUKFilter', name="ROUKF", useUnbiasedVariance='false', verbose="1")        
        elif (self.filterKind == 'UKFSimCorr'):
            self.filter = node.createObject('UKFilterSimCorr', name="UKF", useUnbiasedVariance='false', verbose="1") 
        elif (self.filterKind == 'UKFClassic'):
            self.filter = node.createObject('UKFilterClassic', name="UKFClas", useUnbiasedVariance='false', verbose="1") 
        else:
            print 'Unknown filter type!'     
            
        node.createObject('MeshObjLoader', name='devloader', filename='data/sphere.obj')
        node.createObject('MeshGmshLoader', name='vloader', filename='data/liver.msh')
        #node.createObject('MeshVTKLoader', name='loader', filename='data/liver.vtk')
        #node.createObject('MeshObjLoader', name='meshloader', filename='data/checkLigamentsPoints.obj')
        #node.createObject('MeshObjLoader', name='mesh1loader', filename='data/falciformLigPoints.obj')
        node.createObject('MeshVTKLoader', name='obsloader', filename=self.obsVTK)
        node.createObject('MeshVTKLoader', name='impactloader', filename=self.impactVTK)          

        return 0
        


    #components common for both master and slave: the simulation itself (without observations and visualizations)
    def createCommonComponents(self, node):                                  
        
        #node.createObject('NewtonStaticSolver', name="NewtonStatic", printLog="0", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt="2")   
        #node.createObject('StepPCGLinearSolver', name="StepPCG", iterations="10000", tolerance="1e-12", preconditioners="precond", verbose="1", precondOnTimeStep="1")
        node.createObject('EulerImplicitSolver', firstOrder='false', vdamping='5.0', rayleighMass='0.1', rayleighStiffness='0.1')
        #node.createObject('CGLinearSolver', name='linear solver', iterations='25', tolerance='1e-10', threshold='10e-10')
        node.createObject('SparsePARDISOSolver', name="precond", symmetric="1", exportDataToDir="", iterativeSolverNumbering="0")

        self.optimParams = node.createObject('OptimParams', name="springStiffness", template="Vector", numParams="2",initValue=self.paramInitExp, stdev=self.paramInitSD,  transformParams="1", optimize="1", printLog="1")

        node.createObject('TetrahedronSetTopologyContainer', name="Container", src="@/vloader", tags=" ")
        node.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        node.createObject('MechanicalObject', name="dofs")
        node.createObject('UniformMass', totalMass="0.2")
        node.createObject('TetrahedronFEMForceField', name='FEM', listening="true", ParameterSet=self.materialParams)
        #node.createObject('ExtendedRestShapeSpringForceField', name='Springs', stiffness='@springStiffness.value', showIndicesScale='0', springThickness="3", listening="1", updateStiffness="1", printLog="0", external_rest_shape="Bound/mstate", points='418 496 107 190 357 470 467 441 447 429 378 351 331 403 494 497 495 488 489 283 286 352 414 397 339 481 462 428 433 471 482 483 469 445 452 474', external_points='0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36')
        #node.createObject('ExtendedRestShapeSpringForceField', name='Springs', stiffness='@springStiffness.value', showIndicesScale='0', springThickness="3", listening="1", updateStiffness="1", printLog="0", points='246 320 370 413 493 471 447 430 494 492 381 473 448 284 348 36 70 107 190 76 30 60 266 18 91 132 248 238')
        node.createObject('ExtendedRestShapeSpringForceField', name='Springs', stiffness='@springStiffness.value', showIndicesScale='0', springThickness="3", listening="1", updateStiffness="1", printLog="0", points='83 116')
        #node.createObject('SpringStiffnessFileWriter', name='StiffnessWriter', initialDataFile='observations/initial.txt', errorRateFile='measurements/stiffnessError.txt')
        node.createObject('ColorMap',colorScheme="Blue to Red")
        

        visualNode = node.createChild('ModifiedVisual')
        visualNode.createObject('TriangleSetTopologyContainer', src="@/devloader", tags=" ")
        visualNode.createObject('MechanicalObject', name='dofs', template='Vec3d', showObject='true', translation='0.15 0.21 0.37')
        self.toolSprings=visualNode.createObject('ExtendedRestShapeSpringForceField', name="toolSpring", stiffness="2", external_rest_shape="../externalImpactSimu/MO", springThickness="1", listening="1", updateStiffness="1", springColor="0 1 0 1", startTimeSpringOn="0", numStepsSpringOn="10000")
        visualNode.createObject('ColorMap',colorScheme="Blue to Red")
        visualNode.createObject('BarycentricMapping')

        #boxNode = node.createChild('Bound')
        #boxNode.createObject('TetrahedronSetTopologyContainer', name='Container', src="@/meshloader", tags=" ")
        #boxNode.createObject('MechanicalObject', name='mstate')
        #boxNode.createObject('UniformMass', totalMass='0.05')
        #boxNode.createObject('FixedConstraint', name="FixConstraint", fixAll='true')

        #boxNode1 = node.createChild('Bound1')
        #boxNode1.createObject('TetrahedronSetTopologyContainer', name='Container', src="@/mesh1loader", tags=" ")
        #boxNode1.createObject('MechanicalObject', name='mstate')
        #boxNode1.createObject('UniformMass', totalMass='0.05')
        #boxNode1.createObject('FixedConstraint', name="FixConstraint", fixAll='true')

        impactSimu = node.createChild('externalImpactSimu')
        impactSimu.createObject('MechanicalObject', name="MO",src="@/impactloader")
        impactSimu.createObject('SimulatedStateObservationSource', name="ImpactSim",printLog="1", monitorPrefix=self.impactMonitorPrefix, drawSize="0.0015", controllerMode="1")

        # do not use external impact node now
        #impactNode = node.createChild('externalImpact')
        #impactNode.createObject('TriangleSetTopologyContainer', name='Container', src="@/devloader")  
        #impactNode.createObject('MechanicalObject', name='state', template='Vec3d', dx="50.0", src="@/devloader")
        #self.toolSprings=impactNode.createObject('ExtendedRestShapeSpringForceField', name="toolSpring", stiffness="1e5", external_rest_shape="../externalImpactSimu/MO", springThickness="1", listening="1", updateStiffness="1", springColor="0 1 0 1", startTimeSpringOn="0", numStepsSpringOn="10000")
        #impactNode.createObject('UniformMass', name='mass', mass='0.005')

        
        

        #node.createObject('SceneArraySplitter', name='splitter', inputData='@springStiffness.value')
        #node.createObject('BoxStiffSpringForceField', template='Vec3d', name='Spring1', stiffness='@splitter.firstValue', forceOldBehavior='false', object1="@collision", object2="@Box", box_object1='90.0 150.0 260.0 120.0 220.0 330.0', box_object2='60.0 150.0 260.0 80.0 220.0 330.0')
        #node.createObject('BoxStiffSpringForceField', template='Vec3d', name='Spring2', stiffness='@splitter.secondValue', forceOldBehavior='false', object1="@collision", object2="@Box", box_object1='90.0 220.0 260.0 120.0 290.0 330.0', box_object2='60.0 220.0 260.0 80.0 290.0 330.0')
        #node.createObject('BoxStiffSpringForceField', template='Vec3d', name='Spring3', stiffness='@splitter.thirdValue', forceOldBehavior='false', object1="@collision", object2="@Box", box_object1='90.0 150.0 330.0 120.0 220.0 400.0', box_object2='60.0 150.0 330.0 80.0 220.0 400.0')
        #node.createObject('BoxStiffSpringForceField', template='Vec3d', name='Spring4', stiffness='@splitter.forthValue', forceOldBehavior='false', object1="@collision", object2="@Box", box_object1='90.0 220.0 330.0 120.0 290.0 400.0', box_object2='60.0 220.0 330.0 80.0 290.0 400.0')

        return 0



    def createMasterScene(self, node):
        node.createObject('StochasticStateWrapper',name="StateWrapper",verbose="1")
        
        self.createCommonComponents(node)

        obsNode = node.createChild('obsNode')
        obsNode.createObject('MechanicalObject', name='MO', src="@/obsloader")                
        obsNode.createObject('Sphere', color='0.0 0.5 0.0 1', radius="0.005", template='Vec3d')

        obsNode.createObject('BarycentricMapping')                   
        obsNode.createObject('MappedStateObservationManager', name="MOBS", observationStdev=self.obsInitSD, listening="1", stateWrapper="@../StateWrapper",doNotMapObservations="1",verbose="1")
        obsNode.createObject('SimulatedStateObservationSource', name="ObsSource", monitorPrefix=self.obsMonitorPrefix, listening="true")
    
        #visNode = node.createChild('ObjectVisualization')
        #visNode.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping showWireframe hideNormals')
        #visNode.createObject('MechanicalObject',src="@/objectSLoader",name="Surface")
        #visNode.createObject('TriangleSetTopologyContainer', name="Container", src="@/objectSLoader", tags=" ")        
        #visNode.createObject('Line',color="0 0 0 1")
        #visNode.createObject('Triangle',color="1 0 0 1")
        #visNode.createObject('BarycentricMapping')
        #visNode.createObject('VTKExporter',filename="vtkExp/beam",XMLformat="true",listening="true",edges="0",triangles="1",quads="0",tetras="0",exportAtBegin="1",exportAtEnd="0",exportEveryNumberOfSteps="1")

        #visNode2 = node.createChild('ObjectVisualization2')
        #visNode2.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping hideWireframe hideNormals')
        #visNode2.createObject('MechanicalObject',src="@/objectSLoader",name="Surface")
        #visNode2.createObject('TriangleSetTopologyContainer', name="Container", src="@/objectSLoader", tags=" ")                
        #visNode2.createObject('Triangle',color="1 0 0 0.2")
        #visNode2.createObject('BarycentricMapping')

        #node.createObject('BoxROI', box='-0.001 -0.001 -0.011 0.105 0.001 0.001', drawBoxes='0', name='baseROI')
        #self.basePoints=node.createObject('PointsFromIndices', template='Vec3d', name='fixedA', indices='@baseROI.indices', position="@Volume.position")


        # visNode2 = node.createChild('ObjectVisualization2')
        # visNode2.createObject('VisualStyle', displayFlags='showVisual showBehavior showCollision hideMapping hideWireframe hideNormals')        
        # visNode.createObject('MechanicalObject',src="@/objectSLoader",name="Surface")
        # visNode2.createObject('TriangleSetTopologyContainer', name="Container", src="@objectSLoader", tags=" ")                
        # visNode2.createObject('Triangle',color="1 0 0 0.2")
        # visNode2.createObject('BarycentricMapping')

        #obsVisuNode = node.createChild('ObservationVisualization')
        #obsVisuNode.createObject('MechanicalObject', name="aux_Source", position="@../obsNode/MOBS.observations")
        #obsVisuNode.createObject('Sphere', radius="0.002", color="0.2 0.8 0.2 1")

        #asNode = node.createChild('assessNode')
        #asNode.createObject('RegularGrid', name="grid", min='0.01 0.005 -0.005', max='0.09 0.07 -0.005', n='8 5 1')  # obs. grid4
        #self.asMO=asNode.createObject('MechanicalObject', src='@grid', showIndicesScale='0.00025', name='MO', template='Vec3d', showIndices='1')
        #asNode.createObject('Sphere', color='1 0 1 1', radius="0.001", template='Vec3d')
        #asNode.createObject('BarycentricMapping')

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
        
        # self.process.initializationObjects(node)
        return 0

    def onEndAnimationStep(self, deltaTime):  

        if self.saveState:
            if (self.filterKind == 'ROUKF'):
                st=self.filter.findData('reducedState').value
            elif (self.filterKind == 'UKFSimCorr' or self.filterKind == 'UKFClassic'):
                st=self.filter.findData('state').value

            state = [val for sublist in st for val in sublist]
            #print 'Reduced state:'
            #print reducedState

            f1 = open(self.stateExpFile, "a")        
            f1.write(" ".join(map(lambda x: str(x), state)))
            f1.write('\n')
            f1.close()    

            if (self.filterKind == 'ROUKF'):
                var=self.filter.findData('reducedVariance').value
            elif (self.filterKind == 'UKFSimCorr' or self.filterKind == 'UKFClassic'):
                var=self.filter.findData('variance').value
                                
            variance = [val for sublist in var for val in sublist]
            #print 'Reduced variance:'
            #print reducedVariance

            f2 = open(self.stateVarFile, "a")        
            f2.write(" ".join(map(lambda x: str(x), variance)))
            f2.write('\n')
            f2.close()

            if (self.filterKind == 'ROUKF'):
                covar=self.filter.findData('reducedCovariance').value
            elif (self.filterKind == 'UKFSimCorr' or self.filterKind == 'UKFClassic'):
                covar=self.filter.findData('covariance').value
            
            covariance = [val for sublist in covar for val in sublist]
            #print 'Reduced Covariance:'
            #print reducedCovariance

            f3 = open(self.stateCovarFile, "a")
            f3.write(" ".join(map(lambda x: str(x), covariance)))
            f3.write('\n')
            f3.close()

        return 0

    def onScriptEvent(self, senderNode, eventName,data):        
        return 0;

