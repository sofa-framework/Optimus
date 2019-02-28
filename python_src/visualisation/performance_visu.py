import sys
import os
import numpy
import types
import math
import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from itertools import repeat
import pygraphviz as PG

sys.path.append(os.getcwd() + '/../../python_src/utils')
from AdvancedTimerUtils import AdvancedTimerHandler


# select the visual style for images
plt.style.use('classic')
#print(plt.style.available)

only_leaves_performance = 0


try : 
    sys.argv[0]
except :
    commandLineArguments = []
else :
    commandLineArguments = sys.argv

if (len(commandLineArguments) > 1):
    dataFile = commandLineArguments[1]
else:
    print("ERROR: Must supply a json performance file as an argument!")
    sys.exit()

print("Command line arguments for python : " + str(commandLineArguments))

with open(dataFile, 'r') as stream:
    try:
        statistics = json.load(stream)

    except ValueError as exc:
        print(exc)
        sys.exit()

dataFolder = dataFile[ : dataFile.rfind('/') + 1]

### create a tree of dependences
fullNode = statistics['1']['records']
insertionsGraph = PG.AGraph(directed=True, strict=True)
nodesList = []

timerHandler = AdvancedTimerHandler()
timerHandler.create_nested_steps_tree(fullNode, 'TOTAL', only_leaves_performance, insertionsGraph, nodesList)

# pygraphviz renders graphs in neato by default, 
# so you need to specify dot as the layout engine
insertionsGraph.layout(prog='dot')

# save image with graph
insertionsGraph.draw(dataFolder + 'timer_insertions.png')

# load image and show it
img = mpimg.imread(dataFolder + 'timer_insertions.png')
fig1 = plt.figure(1)
spl1 = fig1.add_subplot(111)
imgplot = spl1.imshow(img)
spl1.set_title('Graph of nested timesteps')


### create histogramm of performance
executedTime = numpy.zeros(len(nodesList))
for index in range(1, len(statistics) + 1):
    fullNode = statistics[str(index)]['records']
    timerHandler.compute_performance(fullNode, 'TOTAL', nodesList, executedTime)

totalTime = numpy.zeros(len(nodesList))
for index in range(1, len(statistics) + 1):
    fullNode = statistics[str(index)]['TOTAL']
    timerHandler.compute_full_performance(fullNode, 'TOTAL', nodesList, totalTime)


### estimate average time
iterations = numpy.zeros(len(nodesList))
fullNode = statistics['1']['records']
timerHandler.compute_iterations(fullNode, 'TOTAL', nodesList, iterations)
for index in range(0, len(nodesList)):
    if nodesList[index] == 'TOTAL':
        iterations[index] = 1

averageTime = numpy.zeros(len(nodesList))
for index in range(0, len(nodesList)):
    if math.fabs(iterations[index]) > 1e-03:
        averageTime[index] = totalTime[index] / iterations[index]



fig2 = plt.figure(2)
spl2 = fig2.add_subplot(111)
amount = numpy.arange(len(executedTime))
width = 0.5
rects1 = spl2.bar(amount, averageTime, width, color='r')
rects2 = spl2.bar(amount + width, totalTime, width, color='b')

# Label the functions below  bars
for index in range(0, len(rects1)):
    rect = rects1[index]
    height = rect.get_height()
    position = numpy.array((rect.get_x(), -1.05))
    trans_angle = plt.gca().transData.transform_angles(numpy.array((90,)), position.reshape((1, 2)))[0]
    spl2.text(rect.get_x() + rect.get_width() * 1.3, 0, nodesList[index], rotation=trans_angle, fontsize=25, rotation_mode='anchor', ha='center', va='bottom')
    spl2.text(rect.get_x() + rect.get_width() / 2, 1.05 * height, '%d' % iterations[index], fontsize=20, color='lime', ha='center', va='bottom')

spl2.set_ylabel('general time, ms', fontsize=50)
spl2.tick_params(axis = 'both', which = 'major', labelsize=40)
spl2.grid(color='k', linestyle=':', linewidth=1)
spl2.legend((rects1[0], rects2[0]), ('Average', 'Total'))
spl2.set_title('Computational time after ' + str(len(statistics)) + ' iterations')


# Give ourselves some more room at the bottom of the plot
plt.subplots_adjust(bottom=0.15)
plt.show()

