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
stateExpVal = loader.loadDataFromFilterFile(folder+'/'+options['visual_parameters']['state_file_name'])
stateVar = loader.loadDataFromFilterFile(folder+'/'+options['visual_parameters']['variance_file_name'])


# plot stiffnesses with variances
fig1 = plt.figure(1)
spl1 = fig1.add_subplot(111)

nstate=numpy.size(stateVar[0,:])
nparams=options['filter']['nparams']
nsteps=numpy.size(stateVar[:,0])

print "Number of steps: ", nsteps
print "Number of parameters: ", nparams
print "Size of the state: ", nstate


rng=xrange(0,nsteps)
rng=[i*options['model']['dt'] for i in rng]

cmap = plt.cm.get_cmap('hsv', nparams+1)

groundTruthValues = options['model']['young_moduli']
data = numpy.ones(nsteps);

for i in range(0, nparams):
    si = i +nstate - nparams;
    # print i,' ',si
    if options['filter']['param_transform'] == 'absolute':
        ev = abs(stateExpVal[:,si])
        var = abs(stateVar[:,si])
    elif options['filter']['param_transform'] == 'exponential':
        ev = numpy.exp(stateExpVal[:,si])
        var = numpy.exp(stateVar[:,si])        
    else:
        ev = stateExpVal[:,si]
        var = stateVar[:,si]        

    stdev = [math.sqrt(x) for x in var]


    
    vll = numpy.squeeze([x - y for x,y in zip(ev, stdev)])
    vlu = numpy.squeeze([x + y for x,y in zip(ev, stdev)])

    spl1.plot(rng, ev, color=cmap(i),  linestyle='solid', linewidth=4)
    spl1.plot(rng, vll, color=cmap(i),  linestyle='dashed', linewidth=4)
    spl1.plot(rng, vlu, color=cmap(i),  linestyle='dashed', linewidth=4)


    # print options['scene_parameters']['obs_generating_parameters']['object_young_moduli']
    #, rng, vll, rng, vlu)

    # print groundTruthValues    
    groundTruthData = [numpy.int(elem) * float(groundTruthValues[i]) for elem in data]
    # plt.plot(rng, groundTruthData, color=cmap(i), linestyle='None', marker=r'$\clubsuit$', markersize=5)
    spl1.plot(rng, groundTruthData, color=cmap(i), linestyle='dotted', linewidth=4)
    # plt.setp(lines, color=cmap(i), linewidth=2.0)

### descriptions for estimation
spl1.set_xlabel('iterations') #, fontsize=20)
spl1.set_ylabel('stiffness with variance') #, fontsize=20)
spl1.tick_params(axis = 'both', which = 'major', labelsize=10)
spl1.grid(color='k', linestyle=':', linewidth=1)
spl1.set_title('Params '+folder)

# plot innovation values
# if options['filter']['save_internal_data'] == 1:
#     innovationVal = loader.loadDataFromFilterFile(folder+'/'+options['visual_parameters']['innovation_file_name'])

#     fig2 = plt.figure(2)
#     spl2 = fig2.add_subplot(111)
    
#     ninnov=numpy.size(innovationVal[0,:])
#     nsteps=numpy.size(innovationVal[:,0])

#     print "Innovation size: ",ninnov

#     rng=xrange(0,nsteps)
#     rng=[i*options['general_parameters']['delta_time'] for i in rng]

#     cmap = plt.cm.get_cmap('hsv', ninnov+1)

#     for i in range(0,ninnov):
#         innov = innovationVal[:,i]
#         spl2.plot(rng, innov, color=cmap(i),  linestyle='solid')

#     spl2.set_xlabel('iterations')
#     spl2.set_ylabel('innovation values')
#     spl2.set_title('Innovation '+folder)

    
plt.show()


