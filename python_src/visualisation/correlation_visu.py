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


try : 
    sys.argv[0]
except :
    commandLineArguments = []
else :
    commandLineArguments = sys.argv
if (len(commandLineArguments) > 1):
    folder=commandLineArguments[1]
else:
    print 'ERROR: Must supply a folder with results as an argument!'
    sys.exit()

print "Command line arguments for python : " + str(commandLineArguments)

options = dict()
with open(folder+'/daconfig.yml', 'r') as stream:
    try:
        options = yaml.load(stream)            

    except yaml.YAMLError as exc:
        print(exc)
        sys.exit()

loader = DataLoader()
stateVar = loader.loadDataFromFilterFile(folder+'/'+options['visual_parameters']['variance_file_name'])
stateCovar = loader.loadDataFromFilterFile(folder+'/'+options['visual_parameters']['covariance_file_name'])


# plot correlation
fig1 = plt.figure(1)
spl1 = fig1.add_subplot(111)

nstate=numpy.size(stateVar[1,:])
covarSize=numpy.size(stateCovar[1,:])
nparams=options['filtering_parameters']['optim_params_size']
nsteps=numpy.size(stateVar[:,1])

print "Number of steps: ", nsteps
print "Number of parameters: ", nparams
print "Size of the state: ", nstate
print "Size of covariance: ", nsteps


rng=xrange(0,nsteps)
rng=[i*options['general_parameters']['delta_time'] for i in rng]

cmap = plt.cm.get_cmap('hsv', nsteps+1)

firstVarInd = 0
secondVarInd = 0

for i in range(0, covarSize):
    # update indices for related variance
    secondVarInd = secondVarInd + 1
    if secondVarInd == nstate:
        firstVarInd = firstVarInd + 1
        secondVarInd = firstVarInd + 1

    if options['filtering_parameters']['transform_parameters'] == 'absolute':
        firstVar = abs(stateVar[:,firstVarInd])
        secondVar = abs(stateVar[:,secondVarInd])
        covar = stateCovar[:,i]
    elif options['filtering_parameters']['transform_parameters'] == 'exponential':
        firstVar = numpy.exp(stateVar[:,firstVarInd])
        secondVar = numpy.exp(stateVar[:,secondVarInd])
        covar = numpy.exp(stateCovar[:,i])
    else:
        firstVar = stateVar[:,firstVarInd]
        secondVar = stateVar[:,secondVarInd]
        covar = stateCovar[:,i]

    stdev1 = [max(1e-3, math.sqrt(x)) for x in firstVar]
    stdev2 = [max(1e-3, math.sqrt(x)) for x in secondVar]

    predCorr = numpy.squeeze([x / y for x,y in zip(covar, stdev1)])
    corr = numpy.squeeze([x / y for x,y in zip(predCorr, stdev2)])

    spl1.plot(rng, corr, color=cmap(i), linestyle='solid', linewidth=4, label='Correlation between elements ' + str(firstVarInd + 1) + ' and ' + str(secondVarInd + 1))


### description for correlation plot
spl1.set_xlabel('iterations', fontsize=50)
spl1.set_ylabel('Correlation', fontsize=50)
spl1.tick_params(axis = 'both', which = 'major', labelsize=40)
spl1.grid(color='k', linestyle=':', linewidth=1)
spl1.set_title('Correlation ' + folder)
legendForSpl1 = spl1.legend(loc='upper center', shadow=True, fontsize='x-large')
legendForSpl1.get_frame().set_facecolor('#FFFFFF')

plt.show()

