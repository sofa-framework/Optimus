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




######### configuration for visualisation
inputList = ['outCyl3_770_UKFSimCorr_cyl3ForceField770Verification_init2000']
inputList.append('outCyl3_770_ROUKF_cyl3ForceField770Verification_init2000')
#inputList.append('outCyl2_385_UKFClassic_forceField385Verification_init2000')
######### end of configuration for visualisation




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


def load_time_data(f):
    fo = open(f, 'r')
    timeLine = []
    # throw out first line
    fo.readline()
    line = fo.readline()
    line = fo.readline()
    while line:
        tokens = line.split()
        startTime = int(tokens[len(tokens) - 1])
        line = fo.readline()
        tokens = line.split()
        endTime = int(tokens[len(tokens) - 1])
        deltaTime = endTime - startTime
        timeLine.append(deltaTime)
        line = fo.readline()
        line = fo.readline()

    return timeLine

try : 
    sys.argv[0]
except :
    commandLineArguments = []
else :
    commandLineArguments = sys.argv
if (len(commandLineArguments) > 1):
    folder = commandLineArguments[1]
else:
    folder = inputList

# plot computation time
fig2 = plt.figure(100)
spl2 = fig2.add_subplot(111)
spl2.set_axisbelow(True)
spl2.xaxis.grid(color='gray', linestyle='dotted')
spl2.yaxis.grid(color='gray', linestyle='dotted')

# plot variance values
fig3 = plt.figure(200)
spl3 = fig3.add_subplot(111)

for generalIndex in range (0, len(folder)):

    options = dict()
    with open(folder[generalIndex]+'/daconfig.yml', 'r') as stream:
        try:
            options = yaml.load(stream)            

        except yaml.YAMLError as exc:
            print(exc)
            sys.exit()

    stateExpVal = load_matrix_from_file(folder[generalIndex]+'/'+options['visual_parameters']['state_file_name'])
    stateVar = load_matrix_from_file(folder[generalIndex]+'/'+options['visual_parameters']['variance_file_name'])


    # plot stiffnesses with variances
    fig1 = plt.figure(generalIndex)
    spl1 = fig1.add_subplot(111)

    nstate=numpy.size(stateVar[1,:])
    nparams=options['filtering_parameters']['optim_params_size']
    nsteps=numpy.size(stateVar[:,1])

    print "Number of steps: ", nsteps
    print "Number of parameters: ", nparams
    print "Size of the state: ", nstate


    rng=xrange(0,nsteps)
    rng=[i*options['general_parameters']['delta_time'] for i in rng]

    cmap = plt.cm.get_cmap('hsv', nparams+1)

    groundTruthStr = options['obs_generating_parameters']['object_young_moduli']
    groundTruthValues = groundTruthStr.split(' ')
    data = numpy.ones(nsteps)

    averageDiff = numpy.zeros(nsteps)
    averageVariance = numpy.zeros(nsteps)

    for i in range(0, nparams):
        si = i +nstate - nparams;
        # print i,' ',si
        if options['filtering_parameters']['transform_parameters'] == 'absolute':
            ev = abs(stateExpVal[:,si])
            var = abs(stateVar[:,si])
        elif options['filtering_parameters']['transform_parameters'] == 'exponential':
            ev = numpy.exp(stateExpVal[:,si])
            var = numpy.exp(stateVar[:,si])        
        else:
            ev = stateExpVal[:,si]
            var = stateVar[:,si]        

        stdev = [math.sqrt(x) for x in var]


    
        vll = numpy.squeeze([x - y for x,y in zip(ev, stdev)])
        vlu = numpy.squeeze([x + y for x,y in zip(ev, stdev)])

        spl1.plot(rng, ev, color=cmap(i), linestyle='solid', linewidth=4)
        spl1.plot(rng, vll, color=cmap(i), linestyle='dashed', linewidth=4)
        spl1.plot(rng, vlu, color=cmap(i), linestyle='dashed', linewidth=4)


        # print options['scene_parameters']['obs_generating_parameters']['object_young_moduli']
        #, rng, vll, rng, vlu)
    
    
        # print groundTruthValues    
        groundTruthData = [numpy.int(elem) * float(groundTruthValues[i]) for elem in data]
        # plt.plot(rng, groundTruthData, color=cmap(i), linestyle='None', marker=r'$\clubsuit$', markersize=5)
        spl1.plot(rng, groundTruthData, color=cmap(i), linestyle='dotted', linewidth=4)
        # plt.setp(lines, color=cmap(i), linewidth=2.0)

        spl1.set_xlabel('iterations', fontsize=50)
        spl1.set_ylabel('stiffness with variance', fontsize=50)
        spl1.tick_params(axis = 'both', which = 'major', labelsize=40)
        spl1.grid(color='k', linestyle=':', linewidth=1)
        spl1.set_title('Params ' + folder[generalIndex])

        diffVal = numpy.squeeze([abs(x - int(y)) for x,y in zip(ev, groundTruthData)])
        averageDiff = [x + y for x,y in zip(averageDiff, diffVal)]
        averageVariance = [x + y for x,y in zip(averageVariance, stdev)]



    averageDiff = [x / nparams for x in averageDiff]
    averageVariance = [x / nparams for x in averageVariance]

    # extract time result data
    timeVal = load_time_data(folder[generalIndex]+'/'+options['time_parameters']['computation_time_file_name'])
    timeShift = []
    timeShift.append(timeVal[0])
    for index in range(1, len(timeVal)):
        timeShift.append(timeShift[len(timeShift) - 1] + timeVal[index])

    # convert to milliseconds
    timeShift = [x / 1000 for x in timeShift]

    cmap = plt.cm.get_cmap('hsv', len(folder) + 1)

    # print averageDiff
    spl2.plot(timeShift, averageDiff, color=cmap(generalIndex),  linestyle='solid', label=options['filtering_parameters']['filter_kind'])
    spl2.set_xlabel('time in milliseconds', fontsize=50)
    spl2.set_ylabel('average difference between estimation and groundtruth', fontsize=50)
    spl2.tick_params(axis = 'both', which = 'major', labelsize=40)
    spl2.grid(color='k', linestyle=':', linewidth=1)
    spl2.set_title('General computation time:')

    # print averageDiff
    spl3.plot(timeShift, averageVariance, color=cmap(generalIndex),  linestyle='solid', label=options['filtering_parameters']['filter_kind'])
    spl3.set_xlabel('time in milliseconds', fontsize=50)
    spl3.set_ylabel('average standart deviation', fontsize=50)
    spl3.tick_params(axis = 'both', which = 'major', labelsize=40)
    spl3.grid(color='k', linestyle=':', linewidth=1)
    spl3.set_title('Standart deviation values:')
    
legendForSpl2 = spl2.legend(loc='upper center', shadow=True, fontsize='x-large')
legendForSpl2.get_frame().set_facecolor('#FFFFFF')
legendForSpl3 = spl3.legend(loc='upper center', shadow=True, fontsize='x-large')
legendForSpl3.get_frame().set_facecolor('#FFFFFF')
plt.show()



