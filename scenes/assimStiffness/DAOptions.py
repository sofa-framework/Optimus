import yaml
import datetime
import os
import shutil

class Model:
    def __init__(self):
        return
    
    volumeFileName = ''
    dt = 0.01
    gravity = [0, 0, 0]
    totalMass = 0
    rayleighMass = 0.1
    rayleighStiffness = 3
    fixedBox1 = '-0.05 -0.05 -0.002   0.05 0.05 0.002'
    fixedBox2 = ''

    def parseYaml(self, configData):
        self.volumeFileName = configData['scene_parameters']['system_parameters']['volume_file_name']
        self.gravity = configData['scene_parameters']['general_parameters']['gravity']
        self.totalMass = configData['scene_parameters']['general_parameters']['total_mass']
        #self.fixedBox1 = configData['scene_parameters']['general_parameters']['first_fixed_box_coordinates']
        self.fixedBox2 = configData['scene_parameters']['general_parameters']['second_fixed_box_coordinates']
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

    def parseYaml(self, configData):    
        self.kind = configData['scene_parameters']['filtering_parameters']['filter_kind']
        self.transformParams = configData['scene_parameters']['filtering_parameters']['transform_parameters']
        self.nparams = configData['scene_parameters']['filtering_parameters']['optim_params_size']
        self.paramInitExpVal = configData['scene_parameters']['filtering_parameters']['initial_stiffness']        
        self.paramInitStdev = configData['scene_parameters']['filtering_parameters']['initial_standart_deviation']
        self.paramMinExpVal = configData['scene_parameters']['filtering_parameters']['minimal_stiffness']
        self.paramMaxExpVal = configData['scene_parameters']['filtering_parameters']['maximal_stiffness']
        self.positionStdev = configData['scene_parameters']['filtering_parameters']['positions_standart_deviation']

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


    def parseYaml(self, configFileName):
        self.configFile = configFileName
        with open(configFileName, 'r') as stream:
            try:                
                configData = yaml.load(stream)
                self.model.parseYaml(configData)
                self.filter.parseYaml(configData)
                self.observations.parseYaml(configData)
                self.export.parseYaml(configData, self.filter)

            except yaml.YAMLError as exc:
                print(exc)
        return

        
    def init(self, copyConfigFile):
        if copyConfigFile:
            shutil.copy(self.configFile,self.export.folder+'/daconfig.yml')        
        
        return
