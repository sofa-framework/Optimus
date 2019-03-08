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


######### configuration for visualisation
inputList = ['cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_Gui_init100']
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step1_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step5_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step10_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step15_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step20_Gui_init100')
######### end of configuration for visualisation


####### labels for visualisation
sigma_points_amount = 11.0
labels = ['no Step PCG']
labels.append('Step PCG 1 iteration')
labels.append('Step PCG 5 iterations')
labels.append('Step PCG 10 iterations')
labels.append('Step PCG 15 iterations')
labels.append('Step PCG 20 iterations')



draw_total_performance = 1
draw_dynamic_performance = 1
estimate_parallel_performance = 1
indices_without_PCG = [0]
only_leaves_performance = 0
####### end labels for visualisation


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

print("Command line arguments for python : " + str(commandLineArguments))

# create extra vector for additional cases
if draw_total_performance == 1:
    totalPerformanceVector = []
    if estimate_parallel_performance == 1:
        parallelizedDataVector = []
        totalPerformanceVectorNoPCG = []


if draw_dynamic_performance == 1:
    fig3 = plt.figure(100000)
    spl3 = fig3.add_subplot(111)
    cmap = plt.cm.get_cmap('hsv', len(folder) + 1)
    selectedNode = ['TOTAL']



timerHandler = AdvancedTimerHandler()
for generalIndex in range (0, len(folder)):

    dataFile = folder[generalIndex] + '/computationTime.txt' 
    with open(dataFile, 'r') as stream:
        try:
            statistics = json.load(stream)

        except statistics.JSONError as exc:
            print(exc)
            sys.exit()


    if draw_dynamic_performance == 1:
        dynamicPerformanceVector = []
        fullDynamicPerformanceVector = []

    ### create a tree of dependences
    fullNode = statistics['1']['records']
    insertionsGraph = PG.AGraph(directed=True, strict=True)
    nodesList = []

    timerHandler.create_nested_steps_tree(fullNode, 'TOTAL', only_leaves_performance, insertionsGraph, nodesList)

    # pygraphviz renders graphs in neato by default, 
    # so you need to specify dot as the layout engine
    insertionsGraph.layout(prog='dot')

    # save image with graph
    insertionsGraph.draw(folder[generalIndex] + '/timer_insertions.png')

    # load image and show it
    img = mpimg.imread(folder[generalIndex] + '/timer_insertions.png')
    fig1 = plt.figure(2 * generalIndex)
    spl1 = fig1.add_subplot(111)
    imgplot = spl1.imshow(img)
    spl1.set_title('Graph of nested timesteps')


    ### compute performance for all nested elements
    executedTime = numpy.zeros(len(nodesList))
    for index in range(1, len(statistics) + 1):
        currentTime = numpy.zeros(len(nodesList))
        fullNode = statistics[str(index)]['records']
        timerHandler.compute_performance(fullNode, 'TOTAL', nodesList, currentTime)
        for selIndex in range(0, len(nodesList)):
            executedTime[selIndex] = executedTime[selIndex] + currentTime[selIndex]
            if draw_dynamic_performance == 1:
                if nodesList[selIndex] == selectedNode[0]:
                    dynamicPerformanceVector.append(currentTime[selIndex])

    totalTime = numpy.zeros(len(nodesList))
    for index in range(1, len(statistics) + 1):
        currentTime = numpy.zeros(len(nodesList))
        fullNode = statistics[str(index)]['TOTAL']
        timerHandler.compute_full_performance(fullNode, 'TOTAL', nodesList, currentTime)
        for selIndex in range(0, len(nodesList)):
            totalTime[selIndex] = totalTime[selIndex] + currentTime[selIndex]
            if draw_dynamic_performance == 1:
                if nodesList[selIndex] == selectedNode[0]:
                    fullDynamicPerformanceVector.append(currentTime[selIndex])


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


    ### save data for total performance
    if draw_total_performance == 1:
        for index in range(0, len(nodesList)):
            if nodesList[index] == 'TOTAL':
                totalPerformanceVector.append(totalTime[index])

        ### save data for parallel estimation
        if estimate_parallel_performance == 1:
            parallel_data = 0
            for index in range (0, len(indices_without_PCG)):
                if generalIndex == indices_without_PCG[index]:
                    parallel_data = 1
                    break

            if parallel_data == 1:
                for index in range(0, len(nodesList)):
                    if nodesList[index] == 'KalmanFilterPrediction':
                        parallelizedDataVector.append(totalTime[index])
                    elif nodesList[index] == 'TOTAL':
                        totalPerformanceVectorNoPCG.append(totalTime[index])



    ### draw histogram for performance
    fig2 = plt.figure(2 * generalIndex + 1)
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
        spl2.text(rect.get_x() + rect.get_width() * 1.3, 0, nodesList[index], fontsize=25, rotation=trans_angle, rotation_mode='anchor', ha='center', va='bottom')
        spl2.text(rect.get_x() + rect.get_width() / 2, 1.05 * height, '%d' % iterations[index], fontsize=20, color='lime', ha='center', va='bottom')

    spl2.set_ylabel('general time, ms', fontsize=50)
    spl2.tick_params(axis = 'both', which = 'major', labelsize=40)
    spl2.grid(color='k', linestyle=':', linewidth=1)
    spl2.legend((rects1[0], rects2[0]), ('Average', 'Total'))
    spl2.set_title('Computational time after ' + str(len(statistics)) + ' iterations')


    ### draw chart for dynamic performance
    if draw_dynamic_performance == 1:
        iterations = xrange(0, len(statistics))
        dynamicPerformanceArray = numpy.array(fullDynamicPerformanceVector)
        spl3.plot(iterations, dynamicPerformanceArray, color=cmap(generalIndex), linestyle='solid', linewidth=4, label=labels[generalIndex])
        spl3.set_ylabel('time, ms', fontsize=50)
        spl3.set_xlabel('iterations', fontsize=50)
        spl3.tick_params(axis = 'both', which = 'major', labelsize=40)
        spl3.grid(color='k', linestyle=':', linewidth=1)
        spl3.set_title('Computational time for every iteration after ' + str(len(statistics)) + ' iterations')
        legendForSpl3 = spl3.legend(loc='upper center', shadow=True, fontsize='x-large')
        legendForSpl3.get_frame().set_facecolor('#FFFFFF')



### draw chart for total performance
if draw_total_performance == 1:
    fig4 = plt.figure(200000)
    spl4 = fig4.add_subplot(111)

    data_size = len(folder)
    if estimate_parallel_performance == 1:
        data_size = data_size + 2 * len(parallelizedDataVector)
        for index in range (0, len(parallelizedDataVector)):
            improvement_eight = sigma_points_amount / math.ceil(sigma_points_amount / 8.0)
            eightThread_computationTime = totalPerformanceVectorNoPCG[index] - parallelizedDataVector[index] + parallelizedDataVector[index] / improvement_eight
            totalPerformanceVector.insert(1, eightThread_computationTime)
            labels.insert(1, 'estimated 8 threads parallelization')
            improvement_four = sigma_points_amount / math.ceil(sigma_points_amount / 4.0)
            fourThread_computationTime = totalPerformanceVectorNoPCG[index] - parallelizedDataVector[index] + parallelizedDataVector[index] / improvement_four
            totalPerformanceVector.insert(1, fourThread_computationTime)
            labels.insert(1, 'estimated 4 threads parallelization')
    amount = xrange(0, data_size)
    totalPerformanceArray = numpy.array(totalPerformanceVector)
    cmap = plt.cm.get_cmap('hsv', data_size + 1)

    spl4.plot(amount, totalPerformanceArray, color=cmap(0), linestyle='solid', linewidth=4)
    # Label the functions below  bars
    for index in range(0, data_size):
        position = numpy.array((index, -1.05))
        trans_angle = plt.gca().transData.transform_angles(numpy.array((90,)), position.reshape((1, 2)))[0]
        spl4.text(float(index) + 0.1, totalPerformanceVector[0], labels[index], fontsize=25, rotation=trans_angle, rotation_mode='anchor', ha='center', va='bottom')

    spl4.set_ylabel('general time, ms', fontsize=50)
    spl4.tick_params(axis = 'both', which = 'major', labelsize=40)
    spl4.grid(color='k', linestyle=':', linewidth=1)
    spl4.set_title('Total computational time after ' + str(len(statistics)) + ' iterations')


# Give ourselves some more room at the bottom of the plot
plt.subplots_adjust(bottom=0.15)
plt.show()

