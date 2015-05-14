import Sofa

def createParallelizableScene(in_CreateScene, in_root, in_count):
    """Creates a parallelizable scene by enriching the global scene with a given number of auxiliary scenes.
    I/O:
    input1	Class CreateComponents - defines necessary functions and parameters for scene creation
    input2	root - where the created scene should be rooted
    input3	integer count - number of scenes to be created
    return	list of created auxiliary nodes
    
    Prerequisites:
    Class CreateScene has the following keyword attributes and methods
    (1) attribute 'self.rootNode' - root of the scene
    (2) method 'createGlobalComponents' - creates the necessary global components
    (3) method 'createMasterScene' - creates the master scene
    (4) method 'createSlaveScene' - creates the slave scene

    !!Global indexing is advised!!
    """
    r_slaves = [] # list of created auxiliary nodes
    in_CreateScene.createGlobalComponents(in_root)
    
    
    masterNode=in_root.createChild('MasterScene')
    
    #if "createMasterScene" in dir(in_CreateScene):
    #    # createMasterScene is defined
    #    in_CreateScene.createMasterScene(masterNode)
    #else:
    #    in_CreateScene.createSlaveScene(masterNode)
    in_CreateScene.createSlaveScene(masterNode)
    masterNode.createObject('VisualStyle', name='VisualStyle', displayFlags='showAll') #unadd when final	
    in_CreateScene.createObserverNode(masterNode)
    

    slaveSubordinate=in_root.createChild('SlaveSubordinate')
    for i in range(0,in_count):
        slave=slaveSubordinate.createChild('SlaveScene_'+str(i))
        slave.createObject('VisualStyle', name='VisualStyle', displayFlags='hideAll') #add when final	
        in_CreateScene.createSlaveScene(slave)
        r_slaves.append(slave)
    return r_slaves
