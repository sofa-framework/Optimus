import yaml
import datetime
import os
import shutil



class BoundaryConditions:
    def __init__(self):
        return

    bcType = ''
    boundBoxes = '-0.05 -0.05 -0.002 0.05 0.05 0.002   -0.05 -0.05 0.238 0.05 0.05 0.242'
    boundaryStiffness = 2000

    def parseYaml(self, configData, index):
        self.bcType = configData['scene_parameters']['general_parameters']['boundary_conditions_list'][index]['condition_type']
        self.boundBoxes = configData['scene_parameters']['general_parameters']['boundary_conditions_list'][index]['boxes_coordinates']
        if 'spring_stiffness_values' in configData['scene_parameters']['general_parameters']['boundary_conditions_list'][index]:
            self.boundaryStiffness = configData['scene_parameters']['general_parameters']['boundary_conditions_list'][index]['spring_stiffness_values']
        return



class Model:
    def __init__(self):
        return
    
    volumeFileName = ''
    gravity = [0, 0, 0]
    totalMass = 0
    dt = 0.01
    rayleighMass = 0.1
    rayleighStiffness = 0.1
    bcList = []

    def parseYaml(self, configData):
        self.volumeFileName = configData['scene_parameters']['system_parameters']['volume_file_name']
        self.gravity = configData['scene_parameters']['general_parameters']['gravity']
        self.totalMass = configData['scene_parameters']['general_parameters']['total_mass']
        self.dt = configData['scene_parameters']['general_parameters']['delta_time']
        self.rayleighMass = configData['scene_parameters']['general_parameters']['rayleigh_mass']
        self.rayleighStiffness = configData['scene_parameters']['general_parameters']['rayleigh_stiffness']
        for index in range(0, len(configData['scene_parameters']['general_parameters']['boundary_conditions_list'])):
            self.bcList.append(BoundaryConditions())
            self.bcList[len(self.bcList) - 1].parseYaml(configData, index)

        return



class Filter:
    def __init__(self):
        return

    kind = ''  
    nparams = 3
    transformParams = ''    
    positionStdev = 1e-3
    transformParams = 1
    paramInitExpVal = 0    
    paramInitStdev = 0
    paramMinExpVal = 0
    paramMaxExpVal = 0
    estimPosition = 0
    estimVelocity = 0
    useUnbiasedVariance = 0
    sigmaPointsTopology = ''
    lambdaScale = 1

    def parseYaml(self, configData):    
        self.kind = configData['scene_parameters']['filtering_parameters']['filter_kind']
        self.transformParams = configData['scene_parameters']['filtering_parameters']['transform_parameters']
        self.nparams = configData['scene_parameters']['filtering_parameters']['optim_params_size']
        self.paramInitExpVal = configData['scene_parameters']['filtering_parameters']['initial_stiffness']        
        self.paramInitStdev = configData['scene_parameters']['filtering_parameters']['initial_standart_deviation']
        self.paramMinExpVal = configData['scene_parameters']['filtering_parameters']['minimal_stiffness']
        self.paramMaxExpVal = configData['scene_parameters']['filtering_parameters']['maximal_stiffness']
        self.positionStdev = configData['scene_parameters']['filtering_parameters']['positions_standart_deviation']
        self.useUnbiasedVariance = configData['scene_parameters']['filtering_parameters']['use_unbiased_variance']
        self.sigmaPointsTopology = configData['scene_parameters']['filtering_parameters']['sigma_points_topology']
        self.lambdaScale = configData['scene_parameters']['filtering_parameters']['sigma_points_scale']

        return



class Impact:
    def __init__(self):
        return

    positionFileName = ''  
    position = ''

    def parseYaml(self, configData):
        if 'impact_parameters' in configData['scene_parameters']:
            self.positionFileName = configData['scene_parameters']['impact_parameters']['observation_file_name']
            self.position = configData['scene_parameters']['impact_parameters']['position']

        return



class Observations:    
    def __init__(self):
        return

    valueFileName = ''
    positionFileName = ''
    groundTruth = ''
    save = 1
    stdev = 1e-3
    youngModuli = 6000

    def parseYaml(self, configData):    
        self.valueFileName = configData['scene_parameters']['system_parameters']['observation_file_name']        
        self.positionFileName = configData['scene_parameters']['system_parameters']['observation_points_file_name']   
        self.stdev = configData['scene_parameters']['filtering_parameters']['observation_noise_standart_deviation']
        self.youngModuli = configData['scene_parameters']['obs_generating_parameters']['object_young_moduli']
        self.groundTruth = self.youngModuli
        


class Export:
    def __init__(self):
        return
    
    folder = ''
    fileSuffix = ''
    internalData = 1
    state = 1    
    
    stateExpValFile = ''
    stateVarFile = ''
    stateCovarFile = ''
    innovationFile = ''
    
    def parseYaml(self, configData, filter):   
        self.state = configData['scene_parameters']['filtering_parameters']['save_state']
        self.internalData = configData['scene_parameters']['filtering_parameters']['save_internal_data'] 
        folderPrefix = configData['scene_parameters']['filtering_parameters']['output_directory_prefix'] 
        folderSuffix = configData['scene_parameters']['filtering_parameters']['output_directory_suffix']
        self.folder = folderPrefix + filter.kind + folderSuffix;                               
        self.fileSuffix = configData['scene_parameters']['filtering_parameters']['output_files_suffix']

        self.stateExpValFile = self.folder + '/state_' + self.fileSuffix + '.txt'
        self.stateVarFile = self.folder + '/variance_' + self.fileSuffix + '.txt'
        self.stateCovarFile = self.folder + '/covariance_' + self.fileSuffix + '.txt'
        self.innovationFile = self.folder + '/innovation_' + self.fileSuffix + '.txt'
        
        return    

    def createFolder(self, archiveResults):
        print "Storing the data to: ", self.folder            

        if (os.path.isdir(self.folder)):
            if (archiveResults):
                if not os.path.isdir('archive'):
                    os.mkdir('archive')
                archFolder = 'archive/'+self.folder+datetime.datetime.now().strftime("%Y_%b_%d-%I:%M")
                print "Archiving existing results to archive: ", archFolder
                shutil.move(self.folder, archFolder)

            else:
                shutil.rmtree(self.folder)                        
        
        os.mkdir(self.folder)



class DAOptions:
    def __init__(self):
        return
    
    configFile = ''

    model = Model()
    filter = Filter()
    observations = Observations()
    export = Export()
    impact = Impact()


    def parseYaml(self, configFileName):
        self.configFile = configFileName
        with open(configFileName, 'r') as stream:
            try:                
                configData = yaml.load(stream)
                self.model.parseYaml(configData)
                self.filter.parseYaml(configData)
                self.observations.parseYaml(configData)
                self.export.parseYaml(configData, self.filter)
                self.impact.parseYaml(configData)

            except yaml.YAMLError as exc:
                print(exc)
        return

        
    def init(self, copyConfigFile):
        if copyConfigFile:
            shutil.copy(self.configFile,self.export.folder+'/daconfig.yml')        
        
        return
