import sys
import os
import numpy
import types
import math
import json
import yaml
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from itertools import repeat



######### configuration for visualisation
stdElAmount = 3
updateStepElAmount = 4
inputList = ['liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std10_update1']
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std10_update10')
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std10_update20')
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std10_update60')
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std100_update1')
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std100_update10')
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std100_update20')
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std100_update60')
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std1000_update1')
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std1000_update10')
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std1000_update20')
inputList.append('liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std1000_update60')
######### end of configuration for visualisation




only_leaves_performance = 0


def create_list(node, nodeName, nodesList):
    leafNode = 1
    if not isinstance(node, (float, int)) and len(node) > 0:
        for item in node.keys():
            if item == "end_time" or item == "start_time" or item == 'iterations':
                continue
            ##### fix due to empty nodes in result file #####
            if len(item) < 1:
                continue
            ##### fix due to empty nodes in result file #####
            leafNode = 0
            create_list(node[item], item, nodesList)
    if only_leaves_performance == 0 or leafNode == 1:
        foundNode = 0
        for item in nodesList:
            if item == nodeName:
                foundNode = 1
                break
        if not foundNode:
            nodesList.append(nodeName)


def compute_full_performance(node, nodeName, nodesList, executedTime):
    if not isinstance(node, (float, int)) and len(node) > 0:
        for item in node.keys():
            if item == "Values":
                for index in range(0, len(nodesList)):
                    if nodesList[index] == nodeName:
                        executedTime[index] = executedTime[index] + float(node['Values']['Total'])
            else:
                compute_full_performance(node[item], item, nodesList, executedTime)



# allocate memory for surface
surfaceDraw = numpy.zeros((updateStepElAmount, stdElAmount))
xVector = []
yVector = []

# fill the surface
for generalIndex in range (0, len(inputList)):

    # load options
    options = dict()
    with open(inputList[generalIndex]+'/daconfig.yml', 'r') as stream:
        try:
            options = yaml.load(stream)            

        except yaml.YAMLError as exc:
            print(exc)
            sys.exit()

    xValue = options['filtering_parameters']['initial_standart_deviation']
    xIndex = -1
    for index in range(0, len(xVector)):
        if math.fabs(xVector[index] - xValue) < 1e-08:
            xIndex = index
    if xIndex == -1:
        xVector.append(xValue)
        xIndex = len(xVector) - 1

    yValue = options['precondition_parameters']['PCGUpdateSteps']
    yIndex = -1
    for index in range(0, len(yVector)):
        if math.fabs(yVector[index] - yValue) < 1e-08:
            yIndex = index
    if yIndex == -1:
        yVector.append(yValue)
        yIndex = len(yVector) - 1


    # load performance data
    dataFile = inputList[generalIndex] + '/computationTime.txt' 
    with open(dataFile, 'r') as stream:
        try:
            statistics = json.load(stream)

        except statistics.JSONError as exc:
            print(exc)
            sys.exit()


    ### compute performance
    fullNode = statistics['1']['records']
    nodesList = []
    create_list(fullNode, 'TOTAL', nodesList)
    totalTime = numpy.zeros(len(nodesList))
    for index in range(1, len(statistics) + 1):
        fullNode = statistics[str(index)]['TOTAL']
        compute_full_performance(fullNode, 'TOTAL', nodesList, totalTime)

    for index in range(0, len(nodesList)):
        if nodesList[index] == 'TOTAL':
            surfaceDraw[yIndex][xIndex] = totalTime[index]

xArray = numpy.array(xVector)
yArray = numpy.array(yVector)



fig1 = plt.figure(1)
spl1 = fig1.gca(projection='3d')

# Plot the surface.
zMin = surfaceDraw.min()
zMax = surfaceDraw.max()
xArray, yArray = numpy.meshgrid(xArray, yArray)
surf = spl1.plot_surface(xArray, yArray, surfaceDraw, cmap=cm.coolwarm, linewidth=0, antialiased=False)

# Format axis.
spl1.set_zlim(zMin, zMax)
spl1.zaxis.set_major_locator(LinearLocator(10))
spl1.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

spl1.set_xlabel('initial standart deviation', fontsize=30)
spl1.set_ylabel('update PCG iteration', fontsize=30)
spl1.set_zlabel('TOTAL computation time', fontsize=30)
spl1.set_title('Dependency of computation time on ' + 'std value and PCGupdate steps amount')

# Add a color bar which maps values to colors.
fig1.colorbar(surf, shrink=0.5)

# show data
plt.show()


