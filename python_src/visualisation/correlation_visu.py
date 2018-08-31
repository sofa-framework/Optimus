import sys
import os
import numpy
import types
import math
import yaml
import matplotlib.pyplot as plt
from itertools import repeat

# select the visual style for images
plt.style.use('classic')
#print(plt.style.available)

def load_matrix_from_file(f):
    if type(f) == types.StringType:
        fo = open(f, 'r')
        matrix = load_matrix_from_file(fo)
        fo.close()
        return matrix
    elif type(f) == types.FileType:
        file_content = f.read().strip()
        file_content = file_content.replace('\r\n', ';')
        file_content = file_content.replace('\n', ';')
        file_content = file_content.replace('\r', ';')

        return numpy.matrix(file_content)

    raise TypeError('f must be a file object or a file name.')

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

stateVar = load_matrix_from_file(folder+'/'+options['visual_parameters']['variance_file_name'])
stateCovar = load_matrix_from_file(folder+'/'+options['visual_parameters']['covariance_file_name'])


# plot correlation
fig1 = plt.figure(1)
spl1 = fig1.add_subplot(111)

nstate=numpy.size(stateVar[1,:])
covarSize=numpy.size(stateCovar[1,:])
nparams=options['filter']['nparams']
nsteps=numpy.size(stateVar[:,1])

print "Number of steps: ", nsteps
print "Number of parameters: ", nparams
print "Size of the state: ", nstate
print "Size of covariance: ", nsteps


rng=xrange(0,nsteps)
rng=[i*options['model']['dt'] for i in rng]

cmap = plt.cm.get_cmap('hsv', nsteps+1)

firstVarInd = 0
secondVarInd = 1

for i in range(0, covarSize):
    if options['filter']['param_transform'] == 'absolute':
        firstVar = abs(stateVar[:,firstVarInd])
        secondVar = abs(stateVar[:,secondVarInd])
        covar = stateCovar[:,i]
    elif options['filter']['param_transform'] == 'exponential':
        firstVar = numpy.exp(stateVar[:,firstVarInd])
        secondVar = numpy.exp(stateVar[:,secondVarInd])
        covar = numpy.exp(stateCovar[:,i])
    else:
        firstVar = stateVar[:,firstVarInd]
        secondVar = stateVar[:,secondVarInd]
        covar = stateCovar[:,i]

    # update indices for related variance
    secondVarInd = secondVarInd + 1
    if secondVarInd == nstate:
        firstVarInd = firstVarInd + 1
        secondVarInd = firstVarInd + 1

    stdev1 = [math.sqrt(x) for x in firstVar]
    stdev2 = [math.sqrt(x) for x in secondVar]


    predCorr = numpy.squeeze([x / y for x,y in zip(covar, stdev1)])
    corr = numpy.squeeze([x / y for x,y in zip(predCorr, stdev2)])

    spl1.plot(rng, corr, color=cmap(i),  linestyle='solid', linewidth=4, label='Correlation between elements ' + str(firstVarInd) + ' and ' + str(secondVarInd))

    spl1.set_xlabel('iterations')
    spl1.set_ylabel('Correlation')
    spl1.set_title('Correlation ' + folder)

legendForSpl1 = spl1.legend(loc='upper center', shadow=True, fontsize='x-large')
legendForSpl1.get_frame().set_facecolor('#FFFFFF')
plt.show()


