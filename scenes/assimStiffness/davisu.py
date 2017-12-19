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
fig1 = plt.figure(1)
spl1 = fig1.add_subplot(111)

nstate=numpy.size(stateVar[1,:])
nparams=options.filter.nparams
nsteps=numpy.size(stateVar[:,1])

print "Number of steps: ", nsteps
print "Number of parameters: ", nparams
print "Size of the state: ", nstate


rng=xrange(0,nsteps)
rng=[i*options.model.dt for i in rng]

cmap = plt.cm.get_cmap('hsv', nparams+1)

groundTruthStr = options.observations.groundTruth
groundTruthValues = groundTruthStr.split(' ')
data = numpy.ones(nsteps);

for i in range(0, nparams):
    si = i +nstate - nparams;
    # print i,' ',si
    if options.filter.transformParams == 'absolute':        
        ev = abs(stateExpVal[:,si])
        var = abs(stateVar[:,si])
    elif options.filter.transformParams == 'exponential':
        ev = numpy.exp(stateExpVal[:,si])
        var = numpy.exp(stateVar[:,si])        
    else:
        ev = stateExpVal[:,si]
        var = stateVar[:,si]        

    stdev = [math.sqrt(x) for x in var]


    
    vll = numpy.squeeze([x - y for x,y in zip(ev, stdev)])
    vlu = numpy.squeeze([x + y for x,y in zip(ev, stdev)])

    spl1.plot(rng, ev, color=cmap(i),  linestyle='solid')
    spl1.plot(rng, vll, color=cmap(i),  linestyle='dashed')
    spl1.plot(rng, vlu, color=cmap(i),  linestyle='dashed')


    # print options.observations.groundTruth
    #, rng, vll, rng, vlu)
    
    
    # print groundTruthValues    
    groundTruthData = [numpy.int(elem) * groundTruthValues[i] for elem in data]
    # plt.plot(rng, groundTruthData, color=cmap(i), linestyle='None', marker=r'$\clubsuit$', markersize=5)
    spl1.plot(rng, groundTruthData, color=cmap(i), linestyle='dotted')
    # plt.setp(lines, color=cmap(i), linewidth=2.0)

    spl1.set_title('Params '+folder)

# plot innovation values
if options.export.internalData == 1:    
    innovationVal = load_matrix_from_file(options.export.innovationFile)

    fig2 = plt.figure(2)
    spl2 = fig2.add_subplot(111)
    
    ninnov=numpy.size(innovationVal[1,:])
    nsteps=numpy.size(innovationVal[:,1])

    print "Innovation size: ",ninnov

    rng=xrange(0,nsteps)
    rng=[i*options.model.dt for i in rng]

    cmap = plt.cm.get_cmap('hsv', ninnov+1)

    for i in range(0,ninnov):
        innov = innovationVal[:,i]
        spl2.plot(rng, innov, color=cmap(i),  linestyle='solid')

    spl2.set_title('Innovation '+folder)

    
plt.show()


