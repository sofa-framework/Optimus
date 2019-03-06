import Sofa
import math
import os
import sys
import csv
import yaml
import pprint
import numpy as np

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
    

    AppliedForces_GenObs(rootNode, options)

    return 0;

class AppliedForces_GenObs (Sofa.PythonScriptController):

    def __init__(self, rootNode, opt):
        self.opt = opt

        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(opt)
                        
        self.saveObs = opt['io']['saveObs']
        self.saveGeo = opt["io"]["saveGeo"]        
                        
        if self.saveObs or self.saveGeo:
            prefix = opt['io']['prefix']
            suffix = opt['io']['suffix']        

            self.mainFolder = prefix + opt['model']['int']['type'] + str(opt['model']['int']['maxit']) + suffix

            os.system('mv '+self.mainFolder+' '+self.mainFolder+'_arch')
            os.system('mkdir '+self.mainFolder)

        if self.saveObs:
            self.obsFile = self.mainFolder + '/' + opt['io']['obsFileName']

        if self.saveGeo:
            self.geoFolder = self.mainFolder + '/VTK'
            os.system('mkdir -p '+self.geoFolder)

        self.createGraph(rootNode)

        return None;

    def createGraph(self,rootNode):   
        self.step = 0     
        rootNode.findData('dt').value = self.opt['model']['dt']
        rootNode.findData('gravity').value = self.opt['model']['gravity']

        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showForceFields showCollisionModels hideVisual')
        
        # rootNode/simuNode
        simuNode = rootNode.createChild('simuNode')
        self.simuNode = simuNode
        simuNode.createObject('MeshVTKLoader', name='loader', filename=self.opt['model']['vol_mesh'])

        intType = self.opt['model']['int']['type']
                
        if intType == 'Euler':
            rmass = self.opt['model']['int']['rmass']
            rstiff = self.opt['model']['int']['rstiff']
            simuNode.createObject('EulerImplicitSolver', rayleighStiffness=rstiff, rayleighMass=rmass)
        elif intType == 'Newton':
            intMaxit = self.opt['model']['int']['maxit']
            simuNode.createObject('NewtonStaticSolver', name="NewtonStatic", printLog="0", correctionTolerance="1e-8", residualTolerance="1e-8", convergeOnResidual="1", maxIt=intMaxit)

        # simuNode.createObject('StepPCGLinearSolver', name='lsolverit', precondOnTimeStep='0', use_precond='1', tolerance='1e-10', iterations='500',
        #  verbose='0', update_step='10', listening='1', preconditioners='lsolver')
        # simuNode.createObject('ShewchukPCGLinearSolver', name='lsolverit', iterations='500', use_precond='1', tolerance='1e-10', preconditioners='lsolver')
        # simuNode.createObject('CGLinearSolver', name='lsolverit', tolerance='1e-10', threshold='1e-10', iterations='500', verbose='0')
        
        simuNode.createObject('SparsePARDISOSolver', name='lsolver', verbose='0', pardisoSchurComplement=0, 
            symmetric=self.opt['model']['linsol']['pardisoSym'], exportDataToFolder=self.opt['model']['linsol']['pardisoFolder'])
        simuNode.createObject('MechanicalObject', src='@loader', name='Volume')        
        simuNode.createObject('BoxROI', box=self.opt['model']['bc']['boxes'], name='fixedBox', drawBoxes='1')
        simuNode.createObject('FixedConstraint', indices='@fixedBox.indices')        
        simuNode.createObject('TetrahedronSetTopologyContainer', name="Container", src="@loader", tags=" ")
        simuNode.createObject('TetrahedronSetTopologyModifier', name="Modifier")        
        simuNode.createObject('TetrahedronSetTopologyAlgorithms', name="TopoAlgo")
        simuNode.createObject('TetrahedronSetGeometryAlgorithms', name="GeomAlgo")

        if 'total_mass' in self.opt['model'].keys():
            simuNode.createObject('UniformMass', totalMass=self.opt['model']['total_mass'])

        if 'density' in self.opt['model'].keys():
            simuNode.createObject('MeshMatrixMass', printMass='0', lumping='1', massDensity=self.opt['model']['density'], name='mass')

        youngModuli=self.opt['model']['fem']['young_modulus']
        poissonRatio = self.opt['model']['fem']['poisson_ratio']
        indices = range(1, len(youngModuli)+1)

        method = self.opt['model']['fem']['method']    
        if  method[0:3] == 'Cor':
            simuNode.createObject('Indices2ValuesMapper', indices=indices, values=youngModuli, name='youngMapper', inputValues='@loader.dataset')
            simuNode.createObject('TetrahedronFEMForceField', name='FEM', method=method[3:].lower(), listening='true', drawHeterogeneousTetra='1', 
                poissonRatio=poissonRatio, youngModulus='@youngMapper.outputValues', updateStiffness='1')        
        elif method == 'StVenant':            
            poissonRatii = poissonRatio * np.ones([1,len(youngModuli)])
            simuNode.createObject('Indices2ValuesTransformer', name='paramMapper', indices=indices,
                values1=youngModuli, values2=poissonRatii, inputValues='@loader.dataset', transformation='ENu2MuLambda')
            simuNode.createObject('TetrahedralTotalLagrangianForceField', name='FEM', materialName='StVenantKirchhoff', ParameterSet='@paramMapper.outputValues', drawHeterogeneousTetra='1')


        if 'applied_periodic_force' in self.opt['model'].keys():
            simuNode.createObject('BoxROI', name='forceBox', box=self.opt['model']['applied_periodic_force']['boxes'])
            self.appliedForce = simuNode.createObject('ConstantForceField', force=self.opt['model']['applied_periodic_force']['initial_force'], indices='@forceBox.indices')


        if self.saveGeo:
            simuNode.createObject('VTKExporter', filename=self.geoFolder+'/object.vtk', XMLformat='0',listening='1',edges="0",triangles="0",quads="0",tetras="1",
                exportAtBegin="1", exportAtEnd="0", exportEveryNumberOfSteps="1")


        if self.saveObs:
            simuNode.createObject('BoxROI', name='observationBox', box='-1 -1 -1 1 1 1')
            simuNode.createObject('OptimMonitor', name='ObservationMonitor', indices='@observationBox.indices', fileName=self.obsFile, ExportPositions='1', ExportVelocities='0', ExportForces='0')
        

        # rootNode/simuNode/oglNode
        oglNode = simuNode.createChild('oglNode')
        self.oglNode = oglNode
        oglNode.createObject('OglModel')
        oglNode.createObject('BarycentricMapping')            

        return 0;


    def onBeginAnimationStep(self, deltaTime):
        self.step += 1
        if 'applied_periodic_force' in self.opt['model'].keys():
            #maxTS = self.opt['model']['applied_force']['num_inc_steps']
            #delta = np.array(self.opt['model']['applied_force']['delta'])
            per=self.opt['model']['applied_periodic_force']['period']
            amp=self.opt['model']['applied_periodic_force']['amplitude']            
            arg=2*math.pi * self.step / per                    
            actualForce = np.array(amp) * math.sin(arg)
            self.appliedForce.findData('force').value = actualForce.tolist()    
        
        return 0;
    

    def initGraph(self, node):
        
        return 0;

    def storeResetState(self):
        
        return 0;

    def cleanup(self):
        if self.saveObs or self.saveGeo:
            print 'Observations saved to '+self.mainFolder         
        return 0;

    def onEndAnimationStep(self, deltaTime):
        
        return 0;

    def onLoaded(self, node):
        
        return 0;

    def reset(self):
        
        return 0;

    def bwdInitGraph(self, node):
        
        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        
        return 0;




