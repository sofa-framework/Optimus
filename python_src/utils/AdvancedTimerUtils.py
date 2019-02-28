
class AdvancedTimerHandler:
    def __init__(self):
        return


    ### generate tree of nested timer steps and list of their names
    def create_nested_steps_tree(self, statisticsItem, rootNodeName, computeOnlyLeavesPerformance, graphOfNestedElems, nodesNamesList):
        leafNode = 1
        if not isinstance(statisticsItem, (float, int)) and len(statisticsItem) > 0:
            for subItem in statisticsItem.keys():
                if subItem == "end_time" or subItem == "start_time" or subItem == 'iterations':
                    continue
                ##### fix due to empty nodes in result file #####
                if len(subItem) < 1:
                    continue
                ##### fix due to empty nodes in result file #####
                graphOfNestedElems.add_edge(rootNodeName, subItem)
                leafNode = 0
                self.create_nested_steps_tree(statisticsItem[subItem], subItem, computeOnlyLeavesPerformance, graphOfNestedElems, nodesNamesList)
        if computeOnlyLeavesPerformance == 0 or leafNode == 1:
            foundNode = 0
            for name in nodesNamesList:
                if name == rootNodeName:
                    foundNode = 1
                    break
            if not foundNode:
                nodesNamesList.append(rootNodeName)


    ### generate list of names for nested timer steps 
    def create_nested_steps_list(self, statisticsItem, rootNodeName, computeOnlyLeavesPerformance, nodesNamesList):
        leafNode = 1
        if not isinstance(statisticsItem, (float, int)) and len(statisticsItem) > 0:
            for subItem in statisticsItem.keys():
                if subItem == "end_time" or subItem == "start_time" or subItem == 'iterations':
                    continue
                ##### fix due to empty nodes in result file #####
                if len(subItem) < 1:
                    continue
                ##### fix due to empty nodes in result file #####
                leafNode = 0
                self.create_nested_steps_list(statisticsItem[subItem], subItem, computeOnlyLeavesPerformance, nodesNamesList)
        if computeOnlyLeavesPerformance == 0 or leafNode == 1:
            foundNode = 0
            for subItem in nodesNamesList:
                if subItem == rootNodeName:
                    foundNode = 1
                    break
            if not foundNode:
                nodesNamesList.append(rootNodeName)


    ### compute performance as difference between start time and end time
    def compute_performance(self, statisticsItem, rootNodeName, nodesNamesList, executedTime):
        if not isinstance(statisticsItem, (float, int)) and len(statisticsItem) > 0:
            for subItem in statisticsItem.keys():
                if subItem == "start_time" or len(subItem) < 1:
                    continue
                if subItem == "end_time":
                    for index in range(0, len(nodesNamesList)):
                        if nodesNamesList[index] == rootNodeName:
                            executedTime[index] = executedTime[index] + float(statisticsItem['end_time']) - float(statisticsItem['start_time'])
                else:
                    self.compute_performance(statisticsItem[subItem], subItem, nodesNamesList, executedTime)


    ### compute performance as value TOTAL time item
    def compute_full_performance(self, statisticsItem, rootNodeName, nodesNamesList, executedTime):
        if not isinstance(statisticsItem, (float, int)) and len(statisticsItem) > 0:
            for subItem in statisticsItem.keys():
                if subItem == "Values":
                    for index in range(0, len(nodesNamesList)):
                        if nodesNamesList[index] == rootNodeName:
                            executedTime[index] = executedTime[index] + float(statisticsItem['Values']['Total'])
                else:
                    self.compute_full_performance(statisticsItem[subItem], subItem, nodesNamesList, executedTime)


    ### compute iteration for all time steps elements
    def compute_iterations(self, statisticsItem, rootNodeName, nodesNamesList, iterations):
        if not isinstance(statisticsItem, (float, int)) and len(statisticsItem) > 0:
            for subItem in statisticsItem.keys():
                if len(subItem) < 1:
                    continue
                if subItem == "iterations":
                    for index in range(0, len(nodesNamesList)):
                        if nodesNamesList[index] == rootNodeName:
                            iterations[index] = int(statisticsItem['iterations'])
                else:
                    self.compute_iterations(statisticsItem[subItem], subItem, nodesNamesList, iterations)

