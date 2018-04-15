import datetime
import os
import shutil


class FolderHandler:
    def __init__(self):
        return

    def createFolder(self, generalFolderName, folderName, archiveResults):
        self.folder = generalFolderName + '/' + folderName
        print "Storing the data to: ", self.folder            

        if (os.path.isdir(self.folder)):
            if (archiveResults):
                if not os.path.isdir(generalFolderName + '/archive'):
                    os.mkdir(generalFolderName + '/archive')
                archFolder = generalFolderName + '/archive/' + folderName + datetime.datetime.now().strftime("%Y_%b_%d-%I:%M-%S")
                print "Archiving existing results to archive: ", archFolder
                shutil.move(self.folder, archFolder)

            else:
                shutil.rmtree(self.folder)                        
        
        os.mkdir(self.folder)
