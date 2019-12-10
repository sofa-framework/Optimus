import sys
import os
import numpy
import types
import math
import yaml
import matplotlib.pyplot as plt
sys.path.append(os.getcwd() + '/../../python_src/utils')
from FileSystemUtils import DataLoader

# select the visual style for images
plt.style.use('classic')
#print(plt.style.available)


######### configuration for visualisation
inputList = ['collisionModel_point_onlyPositions_Euler/UKFClassic_sphere_mesh_decimated_test_velocity_Qvelocity1e-7_Qparam0_W1e-3_posDev1e-3_paramDev5_init-2']
inputList.append('collisionModel_point_onlyPositions_Euler/UKFClassic_sphere_mesh_decimated_test_velocity_Qvelocity1e-6_Qparam0_W1e-3_posDev1e-3_paramDev5_init-2')
inputList.append('collisionModel_point_onlyPositions_Euler/UKFClassic_sphere_mesh_decimated_test_velocity_Qvelocity1e-5_Qparam0_W1e-3_posDev1e-3_paramDev5_init-2')
inputList.append('collisionModel_point_onlyPositions_Euler/UKFClassic_sphere_mesh_decimated_test_velocity_Qvelocity1e-4_Qparam0_W1e-3_posDev1e-3_paramDev5_init-2')
inputList.append('collisionModel_point_onlyPositions_Euler/UKFClassic_sphere_mesh_decimated_test_velocity_Qvelocity1e-3_Qparam0_W1e-3_posDev1e-3_paramDev5_init-2')
inputList.append('collisionModel_point_onlyPositions_Euler/UKFClassic_sphere_mesh_decimated_test_velocity_Qvelocity1e-2_Qparam0_W1e-3_posDev1e-3_paramDev5_init-2')

firstVarIndexConf = 2
secondVarIndexConf = 3
covarIndexConf = 5

labels = ['Correlation between elements ' + str(firstVarIndexConf + 1) + ' and ' + str(secondVarIndexConf + 1) + ', Qvelocity=1e-7']
labels.append('Correlation between elements ' + str(firstVarIndexConf + 1) + ' and ' + str(secondVarIndexConf + 1) + ', Qvelocity=1e-6')
labels.append('Correlation between elements ' + str(firstVarIndexConf + 1) + ' and ' + str(secondVarIndexConf + 1) + ', Qvelocity=1e-5')
labels.append('Correlation between elements ' + str(firstVarIndexConf + 1) + ' and ' + str(secondVarIndexConf + 1) + ', Qvelocity=1e-4')
labels.append('Correlation between elements ' + str(firstVarIndexConf + 1) + ' and ' + str(secondVarIndexConf + 1) + ', Qvelocity=1e-3')
labels.append('Correlation between elements ' + str(firstVarIndexConf + 1) + ' and ' + str(secondVarIndexConf + 1) + ', Qvelocity=1e-2')
######### end of configuration for visualisation

try : 
    sys.argv[0]
except :
    commandLineArguments = []
else :
    commandLineArguments = sys.argv
if (len(commandLineArguments) > 1):
    folder=commandLineArguments[1]
else:
    folder = inputList


# plot correlation
fig1 = plt.figure(1)
spl1 = fig1.add_subplot(111)


loader = DataLoader()
cmap1 = plt.cm.get_cmap('hsv', len(folder) + 1)
for generalIndex in range (0, len(folder)):

    options = dict()
    with open(folder[generalIndex]+'/daconfig.yml', 'r') as stream:
        try:
            options = yaml.load(stream)            

        except yaml.YAMLError as exc:
            print(exc)
            sys.exit()

    stateVar = loader.loadDataFromFilterFile(folder[generalIndex]+'/'+options['visual_parameters']['variance_file_name'])
    stateCovar = loader.loadDataFromFilterFile(folder[generalIndex]+'/'+options['visual_parameters']['covariance_file_name'])

    nstate=numpy.size(stateVar[1,:])
    covarSize=numpy.size(stateCovar[1,:])
    nparams=options['filtering_parameters']['distance_optim_params_size']
    nsteps=numpy.size(stateVar[:,1])

    print "Number of steps: ", nsteps
    print "Number of parameters: ", nparams
    print "Size of the state: ", nstate
    print "Size of covariance: ", nsteps

    rng=xrange(0,nsteps)
    #rng=[i*options['general_parameters']['delta_time'] for i in rng]

    cmap = plt.cm.get_cmap('hsv', covarSize+1)

    firstVarInd = firstVarIndexConf
    secondVarInd = secondVarIndexConf
    covarIndex = covarIndexConf

    if options['filtering_parameters']['transform_parameters'] == 'absolute':
        firstVar = abs(stateVar[:,firstVarInd])
        secondVar = abs(stateVar[:,secondVarInd])
        covar = stateCovar[:,covarIndex]
    elif options['filtering_parameters']['transform_parameters'] == 'exponential':
        firstVar = numpy.exp(stateVar[:,firstVarInd])
        secondVar = numpy.exp(stateVar[:,secondVarInd])
        covar = numpy.exp(stateCovar[:,covarIndex])
    else:
        firstVar = stateVar[:,firstVarInd]
        secondVar = stateVar[:,secondVarInd]
        covar = stateCovar[:,covarIndex]

    stdev1 = [math.sqrt(x) for x in firstVar]
    stdev2 = [math.sqrt(x) for x in secondVar]


    predCorr = numpy.squeeze([x / y for x,y in zip(covar, stdev1)])
    corr = numpy.squeeze([x / y for x,y in zip(predCorr, stdev2)])

    spl1.plot(rng, corr, color=cmap1(generalIndex), linestyle='solid', linewidth=2*(covarSize + 1 - generalIndex), label=labels[generalIndex])

### description for correlation plot
spl1.set_xlabel('iterations', fontsize=50)
spl1.set_ylabel('Correlation', fontsize=50)
spl1.tick_params(axis = 'both', which = 'major', labelsize=40)
spl1.grid(color='k', linestyle=':', linewidth=1)
spl1.set_title('Correlation comparison')
legendForSpl1 = spl1.legend(loc='upper center', shadow=True, fontsize='x-large')
legendForSpl1.get_frame().set_facecolor('#FFFFFF')

plt.show()

