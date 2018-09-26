import sys
import os
import numpy
import types
import math
import json
import yaml
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from itertools import repeat

# select the visual style for images
plt.style.use('classic')
#print(plt.style.available)




######### configuration for visualisation
### the folders goes in pairs without preconditioning and with preconditioning
inputList = ['assimStiffness/cyl3gravity_Euler1/ROUKF_obs33_0_proj2']
inputList.append('assimStiffness/cyl3gravity_Euler1/ROUKF_obs33_1_proj2_std1000_update1')
inputList.append('assimStiffness/cyl10gravity_Euler1/ROUKF_obs120_0_proj5_std1000_withoutPreconditioning')
inputList.append('assimStiffness/cyl10gravity_Euler1/ROUKF_obs120_1_proj5_std1000_update1')
inputList.append('assimBCShape_grid/brick_controlPoint_449_Euler/ROUKF_brick_obs10_withoutPreconditioning_std500')
inputList.append('assimBCShape_grid/brick_controlPoint_449_Euler/ROUKF_brick_obs10_withPreconditioning_std500_update1')
inputList.append('assimBCLiver_grid/liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withoutPreconditioning_std100')
inputList.append('assimBCLiver_grid/liver_138_geomagic_performance_Euler/ROUKF_liverGen_obs7_variant1_withPreconditioning_std100_update1')
### list of vertices and finite elements amount
verticesAmount = [208, 1160, 449, 181]
finiteElementsAmount = [770, 4245, 1233, 596]
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





# process input data
folder = inputList

generalTimeLine = numpy.zeros(len(folder))
generalStateSize = numpy.zeros(len(folder))
for generalIndex in range (0, len(folder)):

    dataFile = folder[generalIndex] + '/computationTime.txt' 
    with open(dataFile, 'r') as stream:
        try:
            statistics = json.load(stream)

        except statistics.JSONError as exc:
            print(exc)
            sys.exit()


    ### create a tree of dependences
    fullNode = statistics['1']['records']
    nodesList = []
    create_list(fullNode, 'TOTAL', nodesList)
    totalTime = numpy.zeros(len(nodesList))
    for index in range(1, len(statistics) + 1):
        fullNode = statistics[str(index)]['TOTAL']
        compute_full_performance(fullNode, 'TOTAL', nodesList, totalTime)

    for index in range(0, len(nodesList)):
        if nodesList[index] == 'TOTAL':
            generalTimeLine[generalIndex] = totalTime[index]


    # compute state size
    options = dict()
    with open(folder[generalIndex]+'/daconfig.yml', 'r') as stream:
        try:
            options = yaml.load(stream)            

        except yaml.YAMLError as exc:
            print(exc)
            sys.exit()

    stateVar = load_matrix_from_file(folder[generalIndex]+'/'+options['visual_parameters']['variance_file_name'])
    generalStateSize[generalIndex] = numpy.size(stateVar[1,:])


# compute system speedup
speedup = numpy.zeros(len(folder) / 2)
for index in range(0, len(folder) / 2):
    speedup[index] = generalTimeLine[2 * index] / generalTimeLine[2 * index + 1]
modifiedStateSize = numpy.zeros(len(folder) / 2)
for index in range(0, len(modifiedStateSize)):
    modifiedStateSize[index] = generalStateSize[2 * index]


# get filter kind
filterKind = 'ROUKF'
if 'filter' in options:
    if 'kind' in options['filter']:
        filterKind = options['filter']['kind']
else:
    filterKind = options['filtering_parameters']['filter_kind']


# draw speedup results
cmap = plt.cm.get_cmap('hsv', 10)

fig1 = plt.figure(1)
spl1 = fig1.add_subplot(111)
spl1.scatter(verticesAmount, speedup, color='b', marker='x', s=40)
spl1.set_xlabel('Amount of object vertices', fontsize=50)
spl1.set_ylabel('Speedup', fontsize=50)
spl1.tick_params(axis = 'both', which = 'major', labelsize=40)
spl1.grid(color='k', linestyle=':', linewidth=1)
spl1.set_title('Speedup dependent on object vertices for ' + filterKind + ' filter')



fig2 = plt.figure(2)
spl2 = fig2.add_subplot(111)
spl2.scatter(finiteElementsAmount, speedup, color='r', marker='x', s=40)
spl2.set_xlabel('Amount of finite elements', fontsize=50)
spl2.set_ylabel('Speedup', fontsize=50)
spl2.tick_params(axis = 'both', which = 'major', labelsize=40)
spl2.grid(color='k', linestyle=':', linewidth=1)
spl2.set_title('Speedup dependent on finite elements for ' + filterKind + ' filter')



fig3 = plt.figure(3)
spl3 = fig3.add_subplot(111)
spl3.scatter(modifiedStateSize, speedup, color='g', marker='x', s=40)
spl3.set_xlabel('State size', fontsize=50)
spl3.set_ylabel('Speedup', fontsize=50)
spl3.tick_params(axis = 'both', which = 'major', labelsize=40)
spl3.grid(color='k', linestyle=':', linewidth=1)
spl3.set_title('Speedup dependent on system state size for ' + filterKind + ' filter')

plt.show()

