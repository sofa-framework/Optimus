from DAOptions import DAOptions
import numpy
import types
import math
import sys
import matplotlib.pyplot as plt
from itertools import repeat

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
    folder='outCyl3_770_P1_pull_ROUKF_1'

options = DAOptions()
options.parseYaml(folder+'/daconfig.yml')

stateExpVal = load_matrix_from_file(options.export.stateExpValFile)
stateVar = load_matrix_from_file(options.export.stateVarFile)


# plot stiffnesses with variances
plt.figure(1)

nparams=numpy.size(stateVar[1,:])
nsteps=numpy.size(stateVar[:,1])

print "Number of steps: ", nsteps
print "Number of parameters: ", nparams


rng=xrange(0,nsteps)
rng=[i*options.model.dt for i in rng]

cmap = plt.cm.get_cmap('hsv', nparams+1)

groundTruthStr = options.observations.groundTruth
groundTruthValues = groundTruthStr.split(' ')
data = numpy.ones(nsteps);

for i in range(0,nparams):
	ev = stateExpVal[:,i]
	var = stateVar[:,i]
	stdev = [math.sqrt(x) for x in var]
	vll = numpy.squeeze([x - y for x,y in zip(ev, stdev)])
	vlu = numpy.squeeze([x + y for x,y in zip(ev, stdev)])

	plt.plot(rng, ev, color=cmap(i),  linestyle='solid')
	plt.plot(rng, vll, color=cmap(i),  linestyle='dashed')
	plt.plot(rng, vlu, color=cmap(i),  linestyle='dashed')

	# print options.observations.groundTruth
	#, rng, vll, rng, vlu)
        
        groundTruthData = [numpy.int(elem) * groundTruthValues[i] for elem in data]
        plt.plot(rng, groundTruthData, color=cmap(i), linestyle='None', marker=r'$\clubsuit$', markersize=5)
	# plt.setp(lines, color=cmap(i), linewidth=2.0)

# plot innovation values
if options.export.internalData == 1:
    innovationVal = load_matrix_from_file(options.export.innovationFile)

    plt.figure(2)
    
    nparams=numpy.size(innovationVal[1,:])
    nsteps=numpy.size(innovationVal[:,1])

    rng=xrange(0,nsteps)
    rng=[i*options.model.dt for i in rng]

    cmap = plt.cm.get_cmap('hsv', nparams+1)

    for i in range(0,nparams):
	innov = innovationVal[:,i]
	plt.plot(rng, innov, color=cmap(i),  linestyle='solid')

	
plt.show()


