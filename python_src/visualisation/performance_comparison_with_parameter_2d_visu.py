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
inputList = ['cyl10_constForce_Newton/Results_deformation_maxForce0_0.06_0/ROUKF_cyl10_4245_obs33_forceField385_Gui_init100']
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.06_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step1_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.06_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step5_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.06_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step10_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.06_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step15_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.06_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step20_Gui_init100')

inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step1_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step5_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step10_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step15_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.12_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step20_Gui_init100')

inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.15_0/ROUKF_cyl10_4245_obs33_forceField385_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.15_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step1_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.15_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step5_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.15_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step10_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.15_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step15_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.15_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step20_Gui_init100')

inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.18_0/ROUKF_cyl10_4245_obs33_forceField385_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.18_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step1_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.18_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step5_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.18_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step10_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.18_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step15_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.18_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step20_Gui_init100')

inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.21_0/ROUKF_cyl10_4245_obs33_forceField385_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.21_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step1_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.21_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step5_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.21_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step10_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.21_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step15_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.21_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step20_Gui_init100')

inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.25_0/ROUKF_cyl10_4245_obs33_forceField385_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.25_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step1_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.25_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step5_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.25_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step10_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.25_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step15_Gui_init100')
inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.25_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step20_Gui_init100')

#inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.5_0/ROUKF_cyl10_4245_obs33_forceField385_Gui_init100')
#inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.5_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step1_Gui_init100')
#inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.5_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step5_Gui_init100')
#inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.5_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step10_Gui_init100')
#inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.5_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step15_Gui_init100')
#inputList.append('cyl10_constForce_Newton/Results_deformation_maxForce0_0.5_0/ROUKF_cyl10_4245_obs33_forceField385_PCG_step20_Gui_init100')
######### end of configuration for visualisation


####### labels for visualisation
performance_labels = ['no Step PCG']
performance_labels.append('Step PCG 1 iteration')
performance_labels.append('Step PCG 5 iterations')
performance_labels.append('Step PCG 10 iterations')
performance_labels.append('Step PCG 15 iterations')
performance_labels.append('Step PCG 20 iterations')


parameter_labels = ['angle 1']
parameter_labels.append('angle 2')
parameter_labels.append('angle 3')
parameter_labels.append('angle 4')
parameter_labels.append('angle 5')
parameter_labels.append('angle 6')
#parameter_labels.append('angle 7')

text_start_position = 100000   ### to set vertical position of the text on the plot
draw_intermediate_performance = 0
draw_total_performance = 1
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


### create a plot to show tottal performance
cmap = plt.cm.get_cmap('hsv', len(parameter_labels) + 1)
if draw_total_performance == 1:
    fig3 = plt.figure(100000)
    spl3 = fig3.add_subplot(111)


timerHandler = AdvancedTimerHandler()
append_labels = 1
initialPerfomanceLabelsAmount = len(performance_labels)
for generalIndex in range (0, len(parameter_labels)):

    # create extra vector for additional cases
    if draw_total_performance == 1:
        totalPerformanceVector = []

        if estimate_parallel_performance == 1:
            parallelizedDataVector = []
            totalPerformanceVectorNoPCG = []

    
    for internalIndex in range (0, initialPerfomanceLabelsAmount):

        dataFile = folder[internalIndex + generalIndex * initialPerfomanceLabelsAmount] + '/computationTime.txt' 
        with open(dataFile, 'r') as stream:
            try:
                statistics = json.load(stream)

            except statistics.JSONError as exc:
                print(exc)
                sys.exit()


        ### create a tree of dependences
        fullNode = statistics['1']['records']
        insertionsGraph = PG.AGraph(directed=True, strict=True)
        nodesList = []

        timerHandler.create_nested_steps_tree(fullNode, 'TOTAL', only_leaves_performance, insertionsGraph, nodesList)

        # pygraphviz renders graphs in neato by default, 
        # so you need to specify dot as the layout engine
        insertionsGraph.layout(prog='dot')

        # save image with graph
        insertionsGraph.draw(folder[internalIndex] + '/timer_insertions.png')

        if draw_intermediate_performance == 1:
            # load image and show it
            img = mpimg.imread(folder[internalIndex] + '/timer_insertions.png')
            fig1 = plt.figure(2 * (internalIndex + generalIndex * initialPerfomanceLabelsAmount))
            spl1 = fig1.add_subplot(111)
            imgplot = spl1.imshow(img)
            spl1.set_title('Graph of nested timesteps')


        ### compute performance for all nested elements
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


        ### save data for total performance
        if draw_total_performance == 1:
            for index in range(0, len(nodesList)):
                if nodesList[index] == 'TOTAL':
                    totalPerformanceVector.append(totalTime[index])

            ### save data for parallel estimation
            if estimate_parallel_performance == 1:
                parallel_data = 0
                for index in range (0, len(indices_without_PCG)):
                    if internalIndex == indices_without_PCG[index]:
                        parallel_data = 1
                        break

                if parallel_data == 1:
                    for index in range(0, len(nodesList)):
                        if nodesList[index] == 'KalmanFilterPrediction':
                            parallelizedDataVector.append(totalTime[index])
                        elif nodesList[index] == 'TOTAL':
                            totalPerformanceVectorNoPCG.append(totalTime[index])



        if draw_intermediate_performance == 1:
            ### draw histogram for performance
            fig2 = plt.figure(2 * (internalIndex + generalIndex * initialPerfomanceLabelsAmount) + 1)
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



    if draw_total_performance == 1:
        if estimate_parallel_performance == 1:
            currentDataSize = initialPerfomanceLabelsAmount + 2 * len(parallelizedDataVector)
            for index in range (0, len(parallelizedDataVector)):
                eightThread_computationTime = totalPerformanceVectorNoPCG[index] - parallelizedDataVector[index] + parallelizedDataVector[index] / 8
                totalPerformanceVector.insert(1, eightThread_computationTime)
                if append_labels == 1:
                    performance_labels.insert(1, 'estimated 8 threads parallelization')        
                fourThread_computationTime = totalPerformanceVectorNoPCG[index] - parallelizedDataVector[index] + parallelizedDataVector[index] / 4
                totalPerformanceVector.insert(1, fourThread_computationTime)
                if append_labels == 1:
                    performance_labels.insert(1, 'estimated 4 threads parallelization')
            append_labels = 0
        totalPerformanceArray = numpy.array(totalPerformanceVector)

        ### draw the performance
        amount = xrange(0, currentDataSize)
        spl3.plot(amount, totalPerformanceArray, color=cmap(generalIndex), linestyle='solid', linewidth=4, label=parameter_labels[generalIndex])

if draw_total_performance == 1:
    ### Label the functions below bars (one time after plotting all data)
    for index in range(0, len(performance_labels)):
        position = numpy.array((index, -1.05))
        trans_angle = plt.gca().transData.transform_angles(numpy.array((90,)), position.reshape((1, 2)))[0]
        spl3.text(float(index) + 0.1, text_start_position, performance_labels[index], fontsize=25, rotation=trans_angle, rotation_mode='anchor', ha='center', va='bottom')

    spl3.set_ylabel('general time, ms', fontsize=50)
    spl3.tick_params(axis = 'both', which = 'major', labelsize=40)
    spl3.grid(color='k', linestyle=':', linewidth=1)
    spl3.set_title('Total computational time after ' + str(len(statistics)) + ' iterations')
    legendForSpl3 = spl3.legend(loc='upper center', shadow=True, fontsize='x-large')
    legendForSpl3.get_frame().set_facecolor('#FFFFFF')


# Give ourselves some more room at the bottom of the plot
plt.subplots_adjust(bottom=0.15)
plt.show()

