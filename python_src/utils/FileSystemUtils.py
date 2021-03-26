import datetime
import os
import types
import shutil
import numpy


class FolderHandler:
    def __init__(self):
        return

    ###
    ### create folder for results file with archiving the existing folder
    ###
    def createFolder(self, generalFolderName, folderName, archiveResults):
        self.folder = generalFolderName + '/' + folderName
        #print "Storing the data to: ", self.folder

        if (os.path.isdir(self.folder)):
            if (archiveResults):
                if not os.path.isdir(generalFolderName + '/archive'):
                    os.mkdir(generalFolderName + '/archive')
                archFolder = generalFolderName + '/archive/' + folderName + datetime.datetime.now().strftime("%Y_%b_%d-%I:%M-%S")
                #print "Archiving existing results to archive: ", archFolder
                shutil.move(self.folder, archFolder)

            else:
                shutil.rmtree(self.folder)                        
        
        os.mkdir(self.folder)




class DataLoader:
    def __init__(self):
        return

    ###
    ### load matrix data from filter file (state, variance, covariance)
    ###
    def loadDataFromFilterFile(self, File):
        if type(File) == types.StringType:
            fo = open(File, 'r')
            matrix = self.loadDataFromFilterFile(fo)
            fo.close()
            return matrix
        elif type(File) == types.FileType:
            file_content = File.read().strip()
            file_content = file_content.replace('\r\n', ';')
            file_content = file_content.replace('\n', ';')
            file_content = file_content.replace('\r', ';')

            return numpy.matrix(file_content)

        raise TypeError('a parameter must be a file object or a file name.')



    ###
    ### load location data from monitor file
    ###
    def loadDataFromMonitorFile(self, File):
        fo = open(File, 'r')
        ### throw out first lines
        fo.readline()
        fo.readline()
        data_list = []
        line = fo.readline()
        while line:
            tokens = line.split()
            tokens = [float(elem) for elem in tokens]
            data_list.append(tokens[1:])
            line = fo.readline()

        return numpy.matrix(data_list)



    ###
    ### load data from computational performance file
    ###
    def loadDataFromComputationalFile(self, File):
        fo = open(File, 'r')
        timeLine = []
        ### throw out first line
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


