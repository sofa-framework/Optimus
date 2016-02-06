/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/common/PrintVisitor.h>
#include <sofa/simulation/common/FindByTypeVisitor.h>
#include <sofa/simulation/common/ExportGnuplotVisitor.h>
#include <sofa/simulation/common/InitVisitor.h>
#include <sofa/simulation/common/AnimateVisitor.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/CollisionVisitor.h>
#include <sofa/simulation/common/CollisionBeginEvent.h>
#include <sofa/simulation/common/CollisionEndEvent.h>
#include <sofa/simulation/common/UpdateContextVisitor.h>
#include <sofa/simulation/common/UpdateMappingVisitor.h>
#include <sofa/simulation/common/ResetVisitor.h>
#include <sofa/simulation/common/VisualVisitor.h>
#include <sofa/simulation/common/ExportOBJVisitor.h>
#include <sofa/simulation/common/WriteStateVisitor.h>
#include <sofa/simulation/common/XMLPrintVisitor.h>
#include <sofa/simulation/common/PropagateEventVisitor.h>
#include <sofa/simulation/common/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/UpdateMappingEndEvent.h>
#include <sofa/simulation/common/CleanupVisitor.h>
#include <sofa/simulation/common/DeleteVisitor.h>
#include <sofa/simulation/common/UpdateBoundingBoxVisitor.h>
#include <sofa/simulation/common/xml/NodeElement.h>

#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/PipeProcess.h>
#include <sofa/helper/AdvancedTimer.h>

#include <sofa/core/visual/VisualParams.h>

#include <stdlib.h>
#include <math.h>
#include <algorithm>

#include <sofa/core/VecId.h>
#include <sofa/simulation/common/MechanicalOperations.h>
#include <sofa/simulation/common/SolveVisitor.h>
#include <sofa/simulation/common/VectorOperations.h>

#include <sofa/simulation/common/IntegrateBeginEvent.h>
#include <sofa/simulation/common/IntegrateEndEvent.h>

#include "SofaModelWrapperParallel.h"
#include "VerdandiClasses.h"

#include <algorithm>    /* std::sort */
#include "boost/bind.hpp" // comparison of std::pair


namespace sofa
{

namespace simulation
{

/***********************************************************************************************************
 ***********************************************************************************************************
 *********************************      START OF THE THREAD SECTION      ***********************************
 ***********************************************************************************************************
 ***********************************************************************************************************/



template <class Type>
struct threadLoopParam_t
{
    SofaModelWrapperParallel<Type>* model; // model the thread computes on
    int threadID; // the thread's ID in the model
};



/**
  * Function to be used for slave threads.
  * Recovers sigma points assigned to this thread from t_sigmaPoints and performs a simulation step on its local scene.
  * Then saves the resulting verdandi state to the sigma point array.
  *
  * @param model to perform forward step on
  * @param structure ThreadData - holds (1) the slaveScene this thread performs computations on and (2) vector of sigma point indices to compute
  */
template<class Type>
void thread_applyOperator(SofaModelWrapperParallel<Type>* const& myModel,
                   typename SofaModelWrapperParallel<Type>::ThreadData_t& myData)
{

    //const bool preserve = myModel->t_input_preserveState;
    const bool update = myModel->t_input_updateForce;

    for (int i=0; i<myData.sigmaPointIndices.size();i++)
    {
        // works correctly
        myModel->StateVerdandi2SofaParallel(myData.sofaObjectsSlave, myData.paramsSlave,
                                            myModel->t_sigmaPoints[myData.sigmaPointIndices[i]]);
        myModel->Forward(update, false, myData.localRoot);
        myModel->StateSofa2VerdandiParallel(myData.sofaObjectsSlave, myData.paramsSlave,
                                            myModel->t_sigmaPoints[myData.sigmaPointIndices[i]]);
    }

} // thread_applyOperator



/**
 * To be used with pthread_create.
 * The thread synchronizes with others after initialization.
 * Then it enters an infinite loop, synchronizing alternately at "await work" and "work done" barriers.
 * The thread can be passed a signal to cancel itself nonviolently.
 * @param threadLoopParam_t - loop parameters
 * @return exit code
 */
template<class Type>
void *thread_startLoop (void *in_args)
{

    // copy the startLoop arguments and wait for other threads to initialize
    threadLoopParam_t<Type> args = *((threadLoopParam_t<Type>*) in_args);
    SofaModelWrapperParallel<Type>* const myModel = args.model; // model the thread computes on
    const int myTid = args.threadID;
    typename SofaModelWrapperParallel<Type>::ThreadData_t& myData= myModel->t_threadData[myTid];
    pthread_barrier_wait (&myModel->t_initDone); // ensure all the threads have initiated correctly

    std::cout<<"[THREAD"<<myTid<<"] started.\n";

    //initiate working loop
    while (true)
    {
        // wait for work to be assigned by master
        pthread_barrier_wait(&myModel->t_workAssigned);

        switch (myData.threadSignal)
        {
        case WORK_ENQUEUED:
            // work has been assigned to this thread
            thread_applyOperator(myModel, myData);
            // inform the master that the work is done
            pthread_barrier_wait(&myModel->t_workDone);
            break;

        case NO_WORK:
            std::cout<<"[THREAD"<<myTid<<"] has no work assigned.\n";
            // inform the master that the work is done
            pthread_barrier_wait(&myModel->t_workDone);
            break;

        case INITIALIZED:
            // same as work done at this point, no break.
            std::cout<<"[THREAD"<<myTid<<"] workAssigned barrier passed with signal still in INITIALIZED.\n";
            return (void*)1;

        case CANCEL_THREAD:
            std::cout<<"[THREAD"<<myTid<<"] canceled.\n";
            return 0;
            break;

        default:
            std::cout<<"[THREAD"<<myTid<<"] encountered unknown signal.\n";
            return (void*)1;
        }
    }

    // should never get here
    return (void*)1;
} // thread_startLoop

/***********************************************************************************************************
 ***********************************************************************************************************
 **************************      START OF THE SOFA MODEL PARALLEL SECTION      *****************************
 ***********************************************************************************************************
 ***********************************************************************************************************/


/**
 * Parameterless constructor of class SofaModelWrapperParallel.
 */
template <class Type>
SofaModelWrapperParallel<Type>::SofaModelWrapperParallel()
    : Inherit1()
    , current_row_(-1)
    , dim_(3)
    , state_size_parallel(0)
    , reduced_state_size_parallel(0)
{
    constraintSolver = NULL;
} // SofaModelWrapperParallel

/**
 * Destructor of class SofaModelWrapperParallel
 */
template <class Type>
SofaModelWrapperParallel<Type>::~SofaModelWrapperParallel()
{
    pthread_barrier_destroy (&t_initDone);
    pthread_barrier_destroy (&t_workAssigned);
    pthread_barrier_destroy (&t_workDone);

    delete [] t_threadData;
    delete [] t_threadHandlers;

    delete[] t_sigmaPoints; // invokes the respective destructors

} // ~SofaModelWrapperParallel

template <class Type>
void SofaModelWrapperParallel<Type>::Message(string _message) {
    std::cout << "Message: " << _message << std::endl;
    if (_message.find("initial condition") != string::npos || _message.find("forecast") != string::npos) {
        //Save();
    }
} // Message



/**
 * Generates a SofaObjectParallel vector, consisting of wrapped rigid3d and vec3d mechanical states from the tree starting at the argument node.
 * Mapped mechanical objects are excluded from the selection and so are the objects with "aux_" prefix.
 * Assumes that at most one (rigid3d or vec3d) mechanical object per node.
 * @param localRoot - root of the subtree to be searched
 * @return SofaObjectParallel consisting of wrapped mechanical states found in the searched subtree
 */
template <class Type>
helper::vector<typename SofaModelWrapperParallel<Type>::SofaObjectParallel> SofaModelWrapperParallel<Type>::getSofaObjects(const simulation::Node* localRoot)
{
    helper::vector<SofaObjectParallel> objects; // resulting wrapped objects
    helper::vector<MechStateVec3d*> mechanicalObjectsV; // holds vec3d mechanical objects
    helper::vector<MechStateRigid3d*> mechanicalObjectsR; // holds rigid3d mechanical objects

    // get all the vec3d mechanical objects in the subtree starting with localRoot
    localRoot->get<MechStateVec3d>
            (&mechanicalObjectsV, BaseContext::SearchDown);
    // get all the vec3d mechanical objects in the subtree starting with localRoot
    localRoot->get<MechStateRigid3d>
            (&mechanicalObjectsR, BaseContext::SearchDown);

    size_t objVCount=mechanicalObjectsV.size();
    size_t objRCount=mechanicalObjectsR.size();


    sofa::core::BaseMapping* mapping;
    for (int i = 0; i<objVCount; i++)
    {
        MechStateVec3d* object = mechanicalObjectsV[i];
        simulation::Node* context = dynamic_cast<simulation::Node*>(object->getContext());

        if (object->getName().substr(0,4).compare("aux_")==0) continue; // intentionally uncounted object
        context->get(mapping);
        if (mapping!=NULL) continue; // mapped object

        // authentic node, add to objects
        SofaObjectParallel obj;
        obj.node = context;

        context->get(obj.vecMS);
        context->get(obj.vecFC);
        context->get(obj.rigidMS);
        context->get(obj.rigidFC);
        objects.push_back(obj);


    }

    for (int i = 0; i<objRCount; i++)
    {
        MechStateRigid3d* object = mechanicalObjectsR[i];
        simulation::Node* context = dynamic_cast<simulation::Node*>(object->getContext());

        if (object->getName().substr(0,4).compare("aux_")==0) continue; // intentionally uncounted object, ignore
        context->get(mapping);
        if (mapping!=NULL) continue; // mapped object, ignore

        // authentic node, add to objects
        SofaObjectParallel obj;
        obj.node = context;

        context->get(obj.vecMS);
        context->get(obj.vecFC);
        context->get(obj.rigidMS);
        context->get(obj.rigidFC);
        objects.push_back(obj);

    }
    return objects;
} // getSofaObjects



/**
 * Creates a vector of objID (name-object pairs) from a vector of objects.
 * @param SofaObjectParallel vector
 * @return objID vector
 */
template<class Type>
helper::vector<typename SofaModelWrapperParallel<Type>::ObjID> SofaModelWrapperParallel<Type>::getObjIDsFromObjects (const helper::vector<SofaObjectParallel>& objects)
{
    helper::vector<ObjID> objectIDs;
    ObjID temp;

    // bind node name and object name to the object
    for (int i=0;i<objects.size();i++)
    {
        temp.first=objects[i].node->getName()+objects[i].vecMS->getName();
        temp.second=objects[i];
        objectIDs.push_back(temp);
    }

    return objectIDs;
} // getObjIDsFromObjects



/**
 * Sorts an object vector lexicographically by concatenation of parent name and object name
 */
template<class Type>
helper::vector<typename SofaModelWrapperParallel<Type>::ObjID> SofaModelWrapperParallel<Type>::getSortedObjIDsFromObjects (const helper::vector<SofaObjectParallel>& objects)
{
    helper::vector<ObjID> objectIDs=getObjIDsFromObjects(objects);

    // sort by node name+object name
    std::sort(objectIDs.begin(), objectIDs.end(),
              boost::bind(&ObjID::first, _1) <
              boost::bind(&ObjID::first, _2));

    return objectIDs;
} // getSortedObjIDsFromObjects



/**
 * Synchronizes slave sofa objects with the of the master.
 * Each pair of objects must differ in node or object name, or undefined behavior occurs.
 * No object is allowed in root node of slave or master scene.
 */
template <class Type>
void SofaModelWrapperParallel<Type>::synchronizeSofaObjects ()
{
    helper::vector<ObjID> masterIDs;
    helper::vector<ObjID> slaveIDs;

    // apply ordering to objects
    masterIDs=getSortedObjIDsFromObjects(sofaObjectsMaster);
    for (int i=0;i<masterIDs.size();i++)
    {
        sofaObjectsMaster[i]=masterIDs[i].second;
    }

    // for each slave scene, match its object order to that of the master scene
    for (int i=0; i<m_slaveCount;i++)
    {
        slaveIDs=getSortedObjIDsFromObjects(t_threadData[i].sofaObjectsSlave);
        if (slaveIDs.size() != masterIDs.size())
        {
            SERRP("ERROR: Failed to match Slave to Master; the scenes differ in authentic node count. Undefined behavior.")
            continue;
        }

        // compare objects in slave scene to that in master scene so as to ensure same ordering
        for (int j=0;j<slaveIDs.size();j++)
        {
            std::cout<<"Master: "<<masterIDs[j].first<<"\t\tSlave:"<<slaveIDs[j].first<<std::endl; // remove later
            if (masterIDs[j].first.compare(slaveIDs[j].first))
            {
                // forbids mechanical object in root of subscene!
                SERRP("ERROR: Failed to match Slave to Master; the names of nodes or objects in scenes differ. Undefined behavior.")
                continue;
            }
            {
                // mechanical objects matched
                t_threadData[i].sofaObjectsSlave[j]=slaveIDs[j].second;

            }
        }
    }
}



/**
 * Initializes the thread structures for parallel simulation
 * Class attribute m_slaveCount must already hold the number of slave threads demanded
 */
template<class Type>
void SofaModelWrapperParallel<Type>::initThreadStructures()
{
    // init barriers to n+1 as the threads will synchronize with the master
    pthread_barrier_init (&t_initDone, NULL, m_slaveCount+1);
    pthread_barrier_init (&t_workAssigned, NULL, m_slaveCount+1);
    pthread_barrier_init (&t_workDone, NULL, m_slaveCount+1);

    // init other thread structures
    t_threadData = new ThreadData_t[m_slaveCount];
    t_threadHandlers = new pthread_t[m_slaveCount];

    for (size_t tid=0; tid <m_slaveCount; tid++)
    {
        t_threadData[tid].threadSignal=INITIALIZED;
    }
} // initThreadStructures


/**
 * Registers the SofaModelWrapperParallel object to the scene and initializes it. Notably, it starts the slave threads.
 * @param structure containing params for initializing the object
 */
template <class Type>
void SofaModelWrapperParallel<Type>::initSimuData(const ModelData &_md)
{
    Verb("initSimuData");

    // set variables given by animation loop
    modelData = _md;
    simulation::Node* gnode = modelData.gnode;
    m_sigmaType = modelData.sigmaType;
    switch (m_sigmaType)
    {
    case SIMPLEX:
        std::cout<<"Simplex type in model!\n";
        break;
    case CANONICAL:
        std::cout<<"Canonical type in model!\n";
        break;
    case STAR:
        std::cout<<"Star type in model!\n";
        break;
    default:
        std::cout<<"WARNING: Unknown type in model!\n";

    }


    // register the object in the scene
    //std::cout << "Registering object: " << this->GetName() << std::endl;
    this->setName("SofaModelWrapperParallel");
    //gnode->addObject(this);


    // attempt to get constraint solver
    gnode->get(constraintSolver, core::objectmodel::BaseContext::SearchDown);
    if (constraintSolver == NULL)
        std::cout << "No ConstraintSolver found, considering the version with no contacts" << std::endl;
    else
        std::cout << "Constraint solver " << constraintSolver->getName() << " found, modeling contacts" << std::endl;


    // get all optim params to listOP
    std::cout << "Searching optim params: " << std::endl;
    helper::vector<OptimParams*> listOP;
    gnode->get<OptimParams>(&listOP, BaseContext::SearchRoot );
    size_t optimParamsCount = listOP.size();
    std::cout<<"params found: "<<optimParamsCount<<std::endl;


    m_slaveCount = optimParamsCount-1;
    initThreadStructures(); // initialize the thread structures

    /** Establish master scene and slave scenes; each slave scene is equipped with a slave thread.*/

    // arguments for launched threads
    threadLoopParam_t<Type>* args = new threadLoopParam_t<Type>[m_slaveCount];
    //current indices
    int masterIndex = 0;
    int slaveIndex = 0;

    // it is assumed that there is one master and optimParamsCount-1 scenes
    // the checks do NOT uncover all incorrectly formed scenes
    for (size_t iop = 0; iop < listOP.size(); iop++) {

        OptimParams* oparam = listOP[iop];
        simulation::Node* opnode = dynamic_cast<simulation::Node*>(oparam->getContext()); // node in which oparam is rooted

        if (opnode->getName().compare("MasterScene") == 0)
        {
            if (masterIndex)
            {
                SERRP("== Two master nodes detected. Unexpected behavior.")
            }
            // master node is to be optimized
            oparam->setOptimize(true);

            rootMaster = opnode;
            sofaObjectsMaster = getSofaObjects(opnode);
            paramsMaster = oparam;

            masterIndex++;

        }
        else if (opnode->getName().substr(0,10).compare("SlaveScene") == 0)
        {
            if (m_slaveCount == slaveIndex)
            {
                SERRP("== All slave nodes. A master node must hold the optimParams to be optimized. Exiting.")
                        exit(1);
            }
            // sofaObjectsSlave
            t_threadData[slaveIndex].localRoot=opnode;
            t_threadData[slaveIndex].sofaObjectsSlave = getSofaObjects(opnode);
            t_threadData[slaveIndex].paramsSlave=oparam;
            t_threadData[slaveIndex].sigmaPointIndices=helper::vector<int>();

            // slave nodes shall not be optimized
            oparam->setOptimize(false);

            args[slaveIndex].model=this;
            args[slaveIndex].threadID=slaveIndex;
            pthread_create(t_threadHandlers+slaveIndex, NULL, thread_startLoop<Type>, (void *) (args+slaveIndex));

            slaveIndex++;
        }
        else
        {
            SERRP("== non-master, non-slave node; ignoring.")
        }
    }
    synchronizeSofaObjects(); // all the threads get their mechanical states correctly paired
    pthread_barrier_wait (&t_initDone); // ensure all the threads have initiated correctly
    delete[] args; // can now be freed (only after the threads have initiated!)

    std::cout << this->getName() << " NUMBER of SOFA objects: " << sofaObjectsMaster.size() << std::endl;
} // initSimuData



/**
 * Searches the master scene for a requested mechanical state <vec3d> and returns a pointer to it (or NULL if not found).
 * The returned pointer allows for modification of the object!
 * @param requested mechanical state
 * @return pointer to the requested state if found, NULL otherwise
 */
template<class Type>
typename SofaModelWrapperParallel<Type>::SofaObjectParallel* SofaModelWrapperParallel<Type>::getObject(const typename SofaModelWrapperParallel<Type>::MechStateVec3d *_state){
    for (size_t obj = 0; obj < sofaObjectsMaster.size(); obj++) {
        if (sofaObjectsMaster[obj].vecMS == _state) {
            std::cout << this->getName() << " : found SOFA requested object " << obj << std::endl;
            return &sofaObjectsMaster[obj];
        }
    }
    return(NULL);
} // getObject



/**
 * Saves to a given vec3Coord mechanical object vector vector the state that represents - fixed node positions of the wrapped mechanical object AND free node positions defined by the verdandi state
 * If the first argument is in fact the sofa vector of the SofaObjectParallel used, this function applied the verdandi state to it.
 * @param sofa mechanical object vector
 * @param verdandi state
 * @param wrapped sofa object
 */
template<class Type>
void SofaModelWrapperParallel<Type>::SetSofaVectorFromVerdandiState(defaulttype::Vec3dTypes::VecCoord & vec, const state &_state, SofaObjectParallel* obj) {
    vec.clear();
    typename MechStateVec3d::ReadVecCoord pos = obj->vecMS->readPositions();
    vec.resize(pos.size());

    // save all nodes as in the wrapped object
    for (size_t i = 0; i < vec.size(); i++) {
        vec[i] = pos[i];
    }

    // overwrite the free nodes by the values in verdandi state
    for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj->positionPairs.begin(); it != obj->positionPairs.end(); it++)
        for (size_t d = 0; d < dim_; d++) {
            vec[it->first][d] = _state(dim_*it->second + d);
        }

} // SetSofaVectorFromVerdandiState



/**
 * Converts a Sofa scene to a verdandi state.
 * @param vector of wrapped mechanical objects in the scene - already describes the ordering in which the objects are saved to verdandi state
 * @param the optimParams object governing the Sofa scene
 * @param verdandiState to save the result to
 */
template <class Type>
void SofaModelWrapperParallel<Type>::StateSofa2VerdandiParallel(const helper::vector<SofaObjectParallel>& mechanicalObjects,
                                                                OptimParams* oparams, state& verdandiState)
{
    // save the objects' positions to the verdandi state
    for (size_t iop = 0; iop < mechanicalObjects.size(); iop++) {
        const SofaObjectParallel& obj = mechanicalObjects[iop];

        if (obj.vecMS != NULL) {            
            typename MechStateVec3d::ReadVecCoord pos = obj.vecMS->readPositions();
            typename MechStateVec3d::ReadVecDeriv vel = obj.vecMS->readVelocities();
            
            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++) {
                    // offset by the reduced state size, as the optimized parameters are listed first in the verdandiState
                    verdandiState(dim_*it->second + d) = pos[it->first][d];
                }


            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    verdandiState(dim_*it->second + d) = vel[it->first][d];
        }

        if (obj.rigidMS != NULL) {
            typename MechStateRigid3d::ReadVecCoord pos = obj.rigidMS->readPositions();
            typename MechStateRigid3d::ReadVecDeriv vel = obj.rigidMS->readVelocities();

            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                defaulttype::Rigid3dTypes::CPos rpos = defaulttype::Rigid3dTypes::getCPos(pos[it->first]);
                for (size_t d = 0; d < dim_; d++)
                    verdandiState(dim_*it->second + d ) = rpos[d];
            }

            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++) {
                defaulttype::Rigid3dTypes::DPos rvel = defaulttype::Rigid3dTypes::getDPos(vel[it->first]);
                for (size_t d = 0; d < dim_; d++)
                    verdandiState(dim_*it->second + d ) = rvel[d];

            }
        }
    }

    // save the param values into the verdandi state    
    oparams->paramsToRawVectorParallel(verdandiState.GetData());
} // StateSofa2VerdandiParallel



/**
 * Uploads the verdandi state into the sofa objects.
 * @param vector of wrapped mechanical objects in the scene - already describes the ordering in which the objects are saved to verdandi state
 * @param the optimParams object governing the Sofa scene
 * @param verdandiState
 */
template <class Type>
void SofaModelWrapperParallel<Type>::StateVerdandi2SofaParallel(helper::vector<SofaObjectParallel>& mechanicalObjects,
                                                                OptimParams* oparams, const state& verdandiState)
{

    for (size_t iop = 0; iop < mechanicalObjects.size(); iop++) {
        SofaObjectParallel& obj = mechanicalObjects[iop];
        //std::cout << "Verdandi => Sofa on ";        

        if (obj.vecMS != NULL) {
            //std::cout<<obj.vecMS->getName()<<"\n";
            typename MechStateVec3d::WriteVecCoord pos = obj.vecMS->writePositions();
            typename MechStateVec3d::WriteVecDeriv vel = obj.vecMS->writeVelocities();

            // map the mechanical-object's verdandi position onto sofa
            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                for (size_t d = 0; d < dim_; d++) {
                    // offset by the reduced state size, as the optimized parameters are listed first in the verdandiState
                    pos[it->first][d] = verdandiState(dim_*it->second + d);
                }
            }

            //map the mechanical object verdandi velocity onto sofa
            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    vel[it->first][d] = verdandiState(dim_*it->second + d);

            /*std::cout << "VERDANDI -> SOFA" << std::endl;
            for (size_t i = 0; i < 5; i++)
                std::cout << pos[100+i] << std::endl;*/
        }

        // analogously
        if (obj.rigidMS != NULL) {
            //std::cout<<obj.rigidMS->getName()<<"\n";
            typename MechStateRigid3d::WriteVecCoord pos = obj.rigidMS->writePositions();
            typename MechStateRigid3d::WriteVecDeriv vel = obj.rigidMS->writeVelocities();


            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                defaulttype::Rigid3dTypes::CPos rpos = defaulttype::Rigid3dTypes::getCPos(pos[it->first]);
                for (size_t d = 0; d < dim_; d++)
                    rpos[d] = verdandiState(dim_*it->second + d );
            }

            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++) {
                defaulttype::Rigid3dTypes::DPos rvel = defaulttype::Rigid3dTypes::getDPos(vel[it->first]);
                for (size_t d = 0; d < dim_; d++)
                    rvel[d] = verdandiState(dim_*it->second + d );

            }
        }

        sofa::core::MechanicalParams mp;
        MechanicalPropagatePositionAndVelocityVisitor(&mp).execute( obj.node ); // propagate the changes to mappings below

        // let the OptimParams to extract the actual values of parameters from the verdandi state
        oparams->rawVectorToParamsParallel(verdandiState.GetData());




    }
} // StateVerdandi2SofaParallel



/**
 * Initializes object wrappers for master and slave nodes. Initializes sigma point structures.
 * Initializes the master verdandi state to match the loaded scene.
 */
template <class Type>
void SofaModelWrapperParallel<Type>::Initialize()
{
    SNCOUTP("== InitializeParallel started")


    /// get fixed nodes
    size_t vsi = 0;    
    
    for (size_t iop = 0; iop < sofaObjectsMaster.size(); iop++) {
        SofaObjectParallel& obj = sofaObjectsMaster[iop];

        /** Determine free node count in of this object. */
        helper::vector<size_t> freeNodes; // only include fixed nodes in freeNodes

        // ineffectively coded loop
        if (obj.vecMS != NULL) {
            for (int msi = 0; msi < obj.vecMS->getSize(); msi++) {
                bool found = false;
                if (obj.vecFC != NULL) {
                    const FixedConstraintVec3d::SetIndexArray& fix = obj.vecFC->f_indices.getValue();
                    for (size_t j = 0; j < fix.size(); j++) {
                        if (int(fix[j]) == msi) {
                            found = true;
                            break;
                        }
                    }
                }
                if (!found)
                    freeNodes.push_back(msi);
            }
        }

        if (obj.rigidMS != NULL) {
            for (int msi = 0; msi < obj.rigidMS->getSize(); msi++) {
                bool found = false;
                if (obj.vecFC != NULL) {
                    const FixedConstraintVec3d::SetIndexArray& fix = obj.rigidFC->f_indices.getValue();
                    for (size_t j = 0; j < fix.size(); j++) {
                        if (int(fix[j]) == msi) {
                            found = true;
                            break;
                        }
                    }
                }
                if (!found)
                    freeNodes.push_back(msi);
            }
        }
        std::cout<<"Free nodes found: "<<freeNodes.size()<<std::endl;
        /** Free node count of this object established. */

        /** Verdandi state only governs freenodes; create pairings in the wrapper to account for this.*
         * note: might very well be saved only once per SofaModelWrapperParallel */

        obj.positionPairs.clear();
        for (int j=0; j<m_slaveCount; j++) t_threadData[j].sofaObjectsSlave[iop].positionPairs.clear();
        obj.velocityPairs.clear();
        for (int j=0; j<m_slaveCount; j++) t_threadData[j].sofaObjectsSlave[iop].velocityPairs.clear();


        for (size_t i = 0; i < freeNodes.size(); i++)
        {
            std::pair<size_t, size_t> pr(freeNodes[i], vsi++);
            if (modelData.positionInState)
            {
                obj.positionPairs.push_back(pr);
                for (int j=0; j<m_slaveCount; j++) t_threadData[j].sofaObjectsSlave[iop].positionPairs.push_back(pr);
            }
            if (modelData.velocityInState)
            {
                obj.velocityPairs.push_back(pr);
                for (int j=0; j<m_slaveCount; j++) t_threadData[j].sofaObjectsSlave[iop].velocityPairs.push_back(pr);
            }
        }
                
    }

    state_size_parallel = dim_* vsi + paramsMaster->size(); // total size of verdandi state
    reduced_state_size_parallel = paramsMaster->size(); // reduced state size
    reduced_state_index_parallel = dim_ * vsi;

    size_t vpi = 0;

    helper::vector<size_t> opv;
    for (size_t i = 0; i < paramsMaster->size(); i++, vpi++) {
        opv.push_back(reduced_state_index_parallel+vpi);
    }
    paramsMaster->setVStateParamIndices(opv);

    for (size_t i = 0; i < m_slaveCount; i++) {
        std::cout << "Setting indices to " << this->t_threadData[i].paramsSlave->getContext()->getName() << "::" << this->t_threadData[i].paramsSlave->getName() << std::endl;
        this->t_threadData[i].paramsSlave->setVStateParamIndices(opv);
    }



    verdandiStateMaster.Nullify();
    verdandiStateMaster.Resize(state_size_parallel);

    /** establish sigma point structures */
    switch (m_sigmaType)
    {
    case SIMPLEX:
        m_sigmaPointCount=reduced_state_size_parallel+1;
        break;
    case CANONICAL:
        m_sigmaPointCount=reduced_state_size_parallel*2;
        break;
    case STAR:
        m_sigmaPointCount=reduced_state_size_parallel*2+1;
        break;
    default:
        SERRP("Unknown sigmaPointType; exiting.")
                exit(1);
    }
    t_sigmaPoints = new state[m_sigmaPointCount];
    for (int i=0;i<m_sigmaPointCount;i++)
    {
        t_sigmaPoints[i].Nullify();
        t_sigmaPoints[i].Resize(state_size_parallel);
    }

    std::cout << "Initializing model (filter type " << modelData.filterTypeP <<")"<<std::endl;
    std::cout << "Size: " << state_size_parallel<< std::endl;
    std::cout << "Reduced state size: " << reduced_state_size_parallel << std::endl;
    std::cout << "Sigma point count: "<<m_sigmaPointCount;

    // initialize the master verdandi state to match the loaded scene
    StateSofa2VerdandiParallel(sofaObjectsMaster, paramsMaster, verdandiStateMaster);

    if (modelData.filterTypeP == ROUKFP) {
        variance_projector_allocated_ = false;
        variance_reduced_allocated_ = false;
    }
    
    time_=double(0.0);
    SNCOUTP("== P: Initialize done")
} // Initialize



/**
 * Prints the actual values of optimized parameters.
 */
template <class Type>
void SofaModelWrapperParallel<Type>::FinalizeStep() {
    std::cout << "Actual parameter values: ";
        for (size_t i = 0; i < reduced_state_size_parallel; i++)
            std::cout << " " << verdandiStateMaster(i);
        std::cout << std::endl;
} // Finalizestep



/**
  * Updates the master verdandi state to match the current state of the master scene.
  * Thereafter returns a REFERENCE to the master verdandi scene, which can be altered outside.
  * @return reference to the master verdandi state
  */
template <class Type>
typename SofaModelWrapperParallel<Type>::state& SofaModelWrapperParallel<Type>::GetState() {
    /// propagate the SOFA state towards verdandi state
    StateSofa2VerdandiParallel(sofaObjectsMaster, paramsMaster, verdandiStateMaster);


    //std::cout << "AKDEBUG " << sofaObjectsMaster.size() << std::endl;
    //std::cout << "AKDEBUG " << sofaObjectsMaster[0].vecMS->getName() << std::endl;

    //std::cout << "AKDEBUG " << verdandiStateMaster(0) << " " << verdandiStateMaster(3234) << std::endl;

    return verdandiStateMaster;
} // GetState



/**
 * Alters the master scene to match the master verdandi state.
 */
template <class Type>
void SofaModelWrapperParallel<Type>::StateUpdated() {
    if (modelData.verbose)
        std::cout << "[" << this->getName() << "]: state updated begin " << std::endl;

    double    dt = rootMaster->getDt();
    core::ExecParams* params = sofa::core::ExecParams::defaultInstance();

    rootMaster->execute< UpdateMappingVisitor >(params);
    {
        UpdateMappingEndEvent ev ( dt );
        PropagateEventVisitor act ( params , &ev );
        rootMaster->execute ( act );
    }

    /*std::cout << "AKDEBUG State updated with: " << std::endl;
    for (size_t i = 0; i < 10; i++)
        std::cout << verdandiStateMaster(100+i) << " ";
    std::cout << " ; ";
    for (size_t i = 0; i < 10; i++)
        std::cout << verdandiStateMaster(3234+i) << " ";
    std::cout << std::endl;*/

    StateVerdandi2SofaParallel(sofaObjectsMaster, paramsMaster, verdandiStateMaster);

    if (modelData.verbose)
        std::cout << "[" << this->getName() << "]: state updated end" << std::endl;
} // StateUpdated()



/**
 * Sets the simulation time of the scene (and propagates it through the scene).
 * note: not thread safe
 * @param new time
 */
template <class Type>
void SofaModelWrapperParallel<Type>::SetTime(double _time) {    
    time_ = _time;
    simulation::Node* gnode = modelData.gnode;
    gnode->setTime ( _time );
    gnode->execute< UpdateSimulationContextVisitor >(execParams);
} // SetTime



/**
 * Static distribution of work among the launched threads.
 * The simulation step takes generally same time no matter the parameter perturbation given.
 */
template <class Type>
void SofaModelWrapperParallel<Type>::distributeWork()
{
    std::cout << "Distribute work of " << m_sigmaPointCount << " sigma points  among " << m_slaveCount << std::endl;
    // set the communication signals
    for (int i=0;i<m_slaveCount;i++)
    {
        t_threadData[i].sigmaPointIndices.clear();
        if (i<m_sigmaPointCount)
        {
            t_threadData[i].threadSignal=WORK_ENQUEUED;
        }
        else
        {
            t_threadData[i].threadSignal=NO_WORK;
        }
    }
    // distribute the sigma points among the threads in a round robin fashion
    for (int i=0;i<m_sigmaPointCount;i++)
    {
        //std::cout<<"\n work "<<i<<"to thread"<<(i%m_slaveCount)<<"\n";

        t_threadData[i%m_slaveCount].sigmaPointIndices.push_back(i);
    }
} // distributeWork

template <class Type>
double SofaModelWrapperParallel<Type>::ApplyOperator(state& /*sigmaPoints*/, bool /*_preserve_state*/, bool /*_update_force*/)  {
    return -2.0f;
}


/**
 * Run simulation steps on all slave scenes in parallel, until all the sigma points are computed. Return the new simulation time.
 * @param array of sigma points
 * @param not used at this point
 * @param not used at this point
 * @return new simulation time
 */
template <class Type>
double SofaModelWrapperParallel<Type>::ApplyOperatorParallel(state* sigmaPoints, bool /*_preserve_state*/, bool /*_update_force*/)  {

    Verb("Apply operator-parallel begin.");

    double    dt = rootMaster->getDt();
    core::ExecParams* params = sofa::core::ExecParams::defaultInstance();

    // make a local copy of the sigma points
    std::cout << "i to: " << m_sigmaPointCount << " j to: " << state_size_parallel << std::endl;
    for (int i=0;i<m_sigmaPointCount;i++)
    {
        for (int j=0;j<state_size_parallel;j++)
        {
            t_sigmaPoints[i](j)=sigmaPoints[i](j);
        }

        /*std::cout << "SPPPPPP: " << i << std::endl;
        for (int j = 0; j < 10; j++)
            std::cout << sigmaPoints[i](3234+j) << " ";
        std::cout << std::endl;*/
    }

    // distribute the work among the threads
    distributeWork();

    {
        AnimateBeginEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        rootMaster->execute ( act );
    }

    pthread_barrier_wait(&t_workAssigned); // let the threads know they have work ready
    Verb("Work Assigned.\n");
    pthread_barrier_wait(&t_workDone); // wait for the threads to let the master know they are done

    // save the results for the Kalman filter
    for (int i=0;i<m_sigmaPointCount;i++)
    {
        for (int j=0;j<state_size_parallel;j++)
        {
            sigmaPoints[i](j)=t_sigmaPoints[i](j);
        }
    }

    {
        //AnimateEndEvent ev ( dt );
        //PropagateEventVisitor act ( params, &ev );
        //rootMaster->execute ( act );
    }

    double newTime=GetTime()+rootMaster->getDt();    
    SetTime(newTime);

#ifndef SOFA_NO_UPDATE_BBOX
    //rootMaster->execute< UpdateBoundingBoxVisitor >(params);
#endif

    Verb("Apply operator-parallel end.");
    return newTime;
} // ApplyOperatorParallel



/**
 * Initiates a simulation step from localRoot, or from the global root if not given.
 * @param not used at this point
 * @param not used at this point
 * @param local root of the subscene where the simulation step is to be performed
 */
template <class Type>
void SofaModelWrapperParallel<Type>::Forward(bool _update_force, bool _update_time, simulation::Node* localRoot)
{
    Verb("forward begin");


    if (constraintSolver)
    {
        Verb("Free motion not implemented yet in parallel.");
    }
    else
    {
        StepDefaultOrig(_update_force, _update_time, localRoot);
    }

    Verb("forward end");
} // Forward

template <class Type>
void SofaModelWrapperParallel<Type>::StepDefaultOrig(bool /*_update_force*/, bool /*_update_time*/, simulation::Node* localRoot) {
    Verb("StepDefault begin.");

    simulation::Node* gnode; // the simulation will start from here
    if (localRoot == NULL)
    {
        Verb("Simulation from global root. Undefined behavior may (will) occur.");
        gnode = modelData.gnode;
    }
    else
    {
        Verb("Simulation from local root.");
        gnode=localRoot;
    }

    //_update_time = false;

    double    dt = gnode->getDt();
    core::ExecParams* params = sofa::core::ExecParams::defaultInstance();


    //sofa::helper::AdvancedTimer::stepBegin("AnimationStep");
    //sofa::helper::AdvancedTimer::begin("Animate");

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    {
        AnimateBeginEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        gnode->execute ( act );
    }

    //double startTime = gnode->getTime();
    BehaviorUpdatePositionVisitor beh(params , dt);
    gnode->execute ( beh );
    AnimateVisitor act(params, dt);
    gnode->execute ( act );

    {
        //std::cout << "[" << this->getName() << "]: animate end" << std::endl;
        AnimateEndEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        gnode->execute ( act );
    }

    //sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
    gnode->execute< UpdateMappingVisitor >(params);
    //sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        UpdateMappingEndEvent ev ( dt );
        PropagateEventVisitor act ( params , &ev );
        gnode->execute ( act );
    }
    //sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");

#ifndef SOFA_NO_UPDATE_BBOX
    //sofa::helper::AdvancedTimer::stepBegin("UpdateBBox");
    gnode->execute< UpdateBoundingBoxVisitor >(params);
    //sofa::helper::AdvancedTimer::stepEnd("UpdateBBox");
#endif
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("Step");
#endif
    //sofa::helper::AdvancedTimer::end("Animate");
    //sofa::helper::AdvancedTimer::stepEnd("AnimationStep");

}

/**
 * Performs a simulation step without collisions on the subtree rooted at localRoot.
 * The simulation is performed by starting a number of visitors at the localRoot.
 * note: Timers are ommited as they are not thread safe. Nevertheless, these are only used to provide information for code optimizations and their use is not mandatory.
 * @param not used at this point
 * @param not used at this point
 * @param local root of the subscene where the simulation step is to be performed
 */
template <class Type>
void SofaModelWrapperParallel<Type>::StepDefault(bool /*_update_force*/, bool _update_time, simulation::Node* localRoot) {

    Verb("StepDefault begin.");

    simulation::Node* gnode; // the simulation will start from here
    if (localRoot == NULL)
    {
        Verb("Simulation from global root. Undefined behavior may (will) occur.");
        gnode = modelData.gnode;
    }
    else
    {
        Verb("Simulation from local root.");
        gnode=localRoot;
    }

    core::ExecParams* params = sofa::core::ExecParams::defaultInstance();

    double    dt = gnode->getDt();

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    {
        AnimateBeginEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        gnode->execute ( act );
    }

    double startTime = gnode->getTime();

    BehaviorUpdatePositionVisitor beh(params , dt);
    gnode->execute ( beh );

    AnimateVisitor act(params, dt);
    gnode->execute ( act );

    if (_update_time) {
        gnode->setTime ( startTime + dt );
        gnode->execute< UpdateSimulationContextVisitor >(params);
    }

    {
        AnimateEndEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        gnode->execute ( act );
    }

    // Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    gnode->execute< UpdateMappingVisitor >(params);

    {
        UpdateMappingEndEvent ev ( dt );
        PropagateEventVisitor act ( params , &ev );
        gnode->execute ( act );
    }

#ifndef SOFA_NO_UPDATE_BBOX
    gnode->execute< UpdateBoundingBoxVisitor >(params);
#endif
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("Step");
#endif
    Verb("StepDefault end.");
} // StepDefault



/**
 * Getter for state error variance.
 * @return attribute state_error_variance
 */
template <class Type>
typename SofaModelWrapperParallel<Type>::state_error_variance& SofaModelWrapperParallel<Type>::GetStateErrorVariance() {
    std::cout << this->getName() << " " << GetTime() << " getStateErrorVariance" << std::endl;
    return state_error_variance_;
} // GetStateErrorVariance



/**
 * Gets the error variance row desired.
 * @param row number
 * @return the variance row in state_error_variance attribute
 */
template <class Type>
typename SofaModelWrapperParallel<Type>::state_error_variance_row& SofaModelWrapperParallel<Type>::GetStateErrorVarianceRow(int row)
{
    if (row == current_row_)
        return state_error_variance_row_;

    GetRow(state_error_variance_, row, state_error_variance_row_);
    current_row_ = row;

    return state_error_variance_row_;
} // GetStateErrorVarianceRow



/**
 * Establishes Error variance projector; a matrix that on right-multiplication extracts the optimized parameters from a verdandi state.
 */
template <class Type>
typename SofaModelWrapperParallel<Type>::state_error_variance& SofaModelWrapperParallel<Type>::GetStateErrorVarianceProjector() {

    //std::cout << "GetSEV_PROJECTOR parallel" << std::endl;
    //std::cout << "START: " << reduced_state_index_parallel << " " << reduced_state_size_parallel << std::endl;
    if (!variance_projector_allocated_)
    {        
        state_error_variance_projector_.Reallocate(state_size_parallel, reduced_state_size_parallel);     
        state_error_variance_projector_.Fill(Type(0.0));        
        for (size_t k = reduced_state_index_parallel, l = 0; k < reduced_state_index_parallel + reduced_state_size_parallel ; k++, l++) {
            //std::cout << "k,l = " << k << " " << l << std::endl;
            state_error_variance_projector_(k, l) = 1;
        }        
        variance_projector_allocated_ = true;
    }
    //std::cout << "DONE" << std::endl;
    return state_error_variance_projector_;
} // GetStateErrorVarianceProjector



template <class Type>
typename SofaModelWrapperParallel<Type>::state_error_variance& SofaModelWrapperParallel<Type>::GetStateErrorVarianceReduced() {

    if (!variance_reduced_allocated_)
    {
        state_error_variance_reduced_.Reallocate(reduced_state_size_parallel,  reduced_state_size_parallel);
        state_error_variance_reduced_.Fill(Type(0.0));

        helper::vector<double> stdev;
        paramsMaster->getStDev(stdev);
        std::cout<<"stdev:\n";
        for (size_t i = 0; i<paramsMaster->size();i++)
        {
            state_error_variance_reduced_(i,i)= Type(Type(1.0) / (stdev[i] * stdev[i]));
            //std::cout<<"stdev["<<i<<"]\t"<<stdev[i]<<std::endl;
        }

        //std::cout << "  Initialize U: " << std::endl;
        //printMatrix(state_error_variance_reduced_, std::cout);

        variance_reduced_allocated_ = true;
        //std::fstream f;
        //f.open("U.mat", std::fstream::out);
        //printMatrixInRow(state_error_variance_reduced_, f);
        //f.close();
    } else {
        //printMatrix(state_error_variance_reduced_, std::cout);
        //std::fstream f;
        //f.open("U.mat", std::fstream::out | std::fstream::app);
        //printMatrixInRow(state_error_variance_reduced_, f);
        //f.close();
    }

    std::cout << "OK" << std::endl;

    return state_error_variance_reduced_;
} // GetStateErrorVarianceReduced



/***********************************************************************************************************
 ***********************************************************************************************************
 *************************************   START OF ROUKF SECTION   ******************************************
 **********************************************************************************************************
 ***********************************************************************************************************/

/**
 * Parameterless constructor of SofaReducedOrderUKFParallel class.
 * note: has a number of Data<..> elements initialized by initData
 */
template <class Model, class ObservationManager>
SofaReducedOrderUKFParallel<Model, ObservationManager>::SofaReducedOrderUKFParallel()    
    : Inherit1()
    , m_outputDirectory( initData(&m_outputDirectory, std::string("output"), "outputDirectory", "working directory of the filter") )
    //, m_configFile( initData(&m_configFile, "configFile", "lua configuration file (temporary)") )
    , m_sigmaPointType( initData(&m_sigmaPointType, std::string("star"), "sigmaPointType", "type of sigma points (canonical|star|simplex)") )
    , m_observationErrorVariance( initData(&m_observationErrorVariance, std::string("matrix_inverse"), "observationErrorVariance", "observationErrorVariance") )
    , m_paramFileName( initData(&m_paramFileName, std::string(""), "paramFileName", "store the parameters at the end of each step to a file") )
    , m_paramVarFileName( initData(&m_paramVarFileName, std::string(""), "paramVarFileName", "store the parameter variance (submatrix) at the end of each step to a file") )
    , m_saveVQ( initData(&m_saveVQ, false, "saveVQ", "m_saveVQ") )
    , m_showIteration( initData(&m_showIteration, false, "showIteration", "showIteration") )
    , m_showTime( initData(&m_showTime, true, "showTime", "showTime") )
    , m_analyzeFirstStep( initData(&m_analyzeFirstStep, false, "analyzeFirstStep", "analyzeFirstStep") )
    , m_withResampling( initData(&m_withResampling, false, "withResampling", "withResampling") )
    , m_positionInState( initData(&m_positionInState, true, "positionInState", "include position in the non-reduced state") )
    , m_velocityInState( initData(&m_velocityInState, false, "velocityInState", "include position in the non-reduced state") )
    //, m_threadCount( initData(&m_threadCount, 1, "threadCount", "degree of shared memory parallelism"))
{
} // SofaReducedOrderUKFParallel



/**
 * Adds the SofaModelWrapperParallel to the scene and performs its basic initialization.
 */
template <class Model, class ObservationManager>
void SofaReducedOrderUKFParallel<Model, ObservationManager>::init() {
    SNCOUTP("== init started")
    SofaModelWrapperParallel<double>::ModelData md;
    md.positionInState = m_positionInState.getValue();
    md.velocityInState = m_velocityInState.getValue();    
    md.filterTypeP = ROUKFP;
    md.gnode = dynamic_cast<simulation::Node*>(this->getContext());
    if (m_sigmaPointType.getValue().compare("simplex") == 0)
    {
        md.sigmaType = SIMPLEX;
    }
    else if (m_sigmaPointType.getValue().compare("canonical") == 0)
    {
        md.sigmaType = CANONICAL;
    }
    else if (m_sigmaPointType.getValue().compare("star") == 0)
    {
        md.sigmaType = STAR;
    }
    else
    {
        SERRP("== ERROR - unknown sigma point type.\n")
                exit(1);
    }

    this->model_.initSimuData(md);
    SNCOUTP("== init done")
} // init



/**
 * Initializes filter parameters and structures.
 */
template <class Model, class ObservationManager>
void SofaReducedOrderUKFParallel<Model, ObservationManager>::InitializeFilter() { //VerdandiROUKFParams* _roukfParams) {
    SNCOUTP("== InitializeFilter started")
    simulation::Node* gnode = dynamic_cast<simulation::Node*>(this->getContext());

    // observation manager provides observations for the filter
    gnode->get(this->observation_manager_, core::objectmodel::BaseContext::SearchDown);
    if (this->observation_manager_) {
        std::cout << this->getName() << " observation manager found: " << this->observation_manager_->getName() << std::endl;
    } else {
        std::cerr << this->getName() << " ERROR: observation manager not found." << std::endl;
        return;
    }

    SNCOUTP("=== InitializeParams...")
    InitializeParams();

    this->model_.Initialize();
    this->observation_manager_->Initialize(this->model_, this->configuration_file_);
    this->observation_manager_->DiscardObservation(false);

    SNCOUTP("=== InitializeStructures...")
    InitializeStructures();

    saveParams=false;
    if (!m_paramFileName.getValue().empty()) {
        std::ofstream paramFile(m_paramFileName.getValue().c_str());
        if (paramFile.is_open()) {
            saveParams = true;
            paramFile.close();
        }
    }

    saveParamVar = false;
    if (!m_paramVarFileName.getValue().empty()) {
        std::ofstream paramFile(m_paramVarFileName.getValue().c_str());
        if (paramFile.is_open()) {
            saveParamVar = true;
            paramFile.close();
        }
    }


    SNCOUTP("== InitializeFilter done")
} // InitializeFilter



/**
 * Simple reassignments of params.
 */
template <class Model, class ObservationManager>
void SofaReducedOrderUKFParallel<Model, ObservationManager>::InitializeParams() {
    this->output_directory_ = m_outputDirectory.getValue();
    this->configuration_file_ = m_configFile.getValue();

    this->saveVQ_ = m_saveVQ.getValue();
    this->analyze_first_step_ = m_analyzeFirstStep.getValue();
    this->with_resampling_ = m_withResampling.getValue();

    this->positionInState = m_positionInState.getValue();
    this->velocityInState = m_velocityInState.getValue();

    this->observation_error_variance_ = m_observationErrorVariance.getValue();
    this->sigma_point_type_ = m_sigmaPointType.getValue();

    this->option_display_["show_iteration"] = m_showIteration.getValue();
    this->option_display_["show_time"] = m_showTime.getValue();


    char comm[100];
    sprintf(comm, "rm %s/roukf*dat", this->output_directory_.c_str());
    std::cout << "Executing: " << comm << std::endl;
    system(comm);
}



/**
 * Saves the step results to outputs, if enabled.
 */
template <class Model, class ObservationManager>
void SofaReducedOrderUKFParallel<Model, ObservationManager>::FinalizeStep() {
    Inherit1::FinalizeStep();

    ///saving the actual structures
    if (saveParams) {
        std::ofstream paramFile(m_paramFileName.getValue().c_str(), std::ios::app);
        if (paramFile.is_open()) {
            typename Inherit1::model_state& st =  this->model_.GetState();
            for (size_t i = this->Nstate_-this->Nreduced_; i < this->Nstate_; i++)
                paramFile << " " << st(i);
            paramFile << '\n';
            paramFile.close();
        }
    }

    ///saving the actual structures
    if (saveParamVar) {
        std::ofstream paramFile(m_paramVarFileName.getValue().c_str(), std::ios::app);
        if (paramFile.is_open()) {
            typename Inherit1::model_state_error_variance& L = this->model_.GetStateErrorVarianceProjector();
            typename Inherit1::model_state_error_variance tmp1(this->Nreduced_, this->Nreduced_), tmp2(this->Nreduced_, this->Nreduced_);

            tmp1.Zero();
            tmp2.Zero();

            int redIx = this->Nstate_ - this->Nreduced_;
            for (int i = 0; i < this->Nreduced_; i++)
                for (int j = 0; j < this->Nreduced_; j++)
                    for (int k = 0; k < this->Nreduced_; k++)
                        tmp1(i,j) += L(redIx+i,k)*this->U_inv_(k,j);


            for (int i = 0; i < this->Nreduced_; i++)
                for (int j = 0; j < this->Nreduced_; j++)
                    for (int k = 0; k < this->Nreduced_; k++)
                        tmp2(i,j) += tmp1(i,k)*L(redIx+j,k);

            for (int i = 0; i < this->Nreduced_; i++)
                paramFile << tmp2(i,i) << " ";

            for (int i = 1; i < this->Nreduced_; i++)
                for (int j = 0; j < i; j++)
                    paramFile << tmp2(i,j) << " ";

            for (int i = 1; i < this->Nreduced_; i++)
                for (int j = 0; j < i; j++)
                    paramFile << tmp2(j,i) << " ";

            paramFile << '\n';
            paramFile.close();
        }
    }
} // InitializeParams



/**
  * Initializes the filter structures by also performing the necessary maths.
  */
 template <class Model, class ObservationManager>
 void SofaReducedOrderUKFParallel<Model, ObservationManager>::InitializeStructures() {
    this->Nstate_ = this->model_.GetNstate(); // state size of verdandi states of the model
    this->Nobservation_ = this->observation_manager_->GetNobservation(); // number of observations

    Copy(this->model_.GetStateErrorVarianceReduced(), this->U_);
    this->U_inv_.Copy(this->U_);

    GetInverse(this->U_inv_);

    this->Nreduced_ = this->U_.GetN();

    std::cout << "NSTATE: " << this->Nstate_ << " Nreduced: " << this->Nreduced_ << std::endl;

    /*** Sigma-points ***/

    typename Inherit1::sigma_point_matrix V_trans;
    if (this->sigma_point_type_ == "canonical")
        Verdandi::ComputeCanonicalSigmaPoint(this->Nreduced_, V_trans, this->D_alpha_,
                                   this->alpha_constant_);
    else if (this->sigma_point_type_ == "star")
        Verdandi::ComputeStarSigmaPoint(this->Nreduced_, V_trans, this->D_alpha_,
                              this->alpha_constant_);
    else if (this->sigma_point_type_ == "simplex")
        Verdandi::ComputeSimplexSigmaPoint(this->Nreduced_, V_trans, this->D_alpha_,
                                 this->alpha_constant_);
    if (this->alpha_constant_)
        this->alpha_ = this->D_alpha_(0);

    this->Nsigma_point_ = V_trans.GetM();

    std::cout << "NSigma: " << this->Nsigma_point_ << std::endl;

    // Initializes transpose of I.
    typename Inherit1::sigma_point_matrix P_alpha_v(this->Nreduced_, this->Nreduced_);
    this->I_trans_.Reallocate(this->Nsigma_point_, this->Nreduced_);

    if (this->alpha_constant_)
    {
        MltAdd(typename Inherit1::Ts(this->alpha_), Seldon::SeldonTrans, V_trans, Seldon::SeldonNoTrans, V_trans,
               typename Inherit1::Ts(0), P_alpha_v);
        GetInverse(P_alpha_v);
        Verdandi::GetCholesky(P_alpha_v);
        MltAdd(typename Inherit1::Ts(1), Seldon::SeldonNoTrans, V_trans, Seldon::SeldonTrans, P_alpha_v,
               typename Inherit1::Ts(0), this->I_trans_);
    }
    else
        throw Verdandi::ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                             "Initialize()", "Calculation not "
                             "implemented for no constant alpha_i.");
    this->I_.Copy(this->I_trans_);
    Transpose(this->I_);


    // Initializes D_v.
    this->D_v_.Reallocate(this->Nsigma_point_, this->Nsigma_point_);
    if (this->alpha_constant_) {
        MltAdd(typename Inherit1::Ts(this->alpha_ * this->alpha_), Seldon::SeldonNoTrans, this->I_trans_, Seldon::SeldonTrans,
               this->I_trans_, typename Inherit1::Ts(0), this->D_v_);
    }
    else {
        throw Verdandi::ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                             "Initialize()", "Calculation not "
                             "implemented for no constant alpha_i.");
    }

    /*** Assimilation ***/

    if (this->analyze_first_step_)
        this->Analyze();

    std::cout << "HERE3" << std::endl;

    //if (initialize_model)
    {
        Verdandi::MessageHandler::Send(*this, "model", "initial condition");
        Verdandi::MessageHandler::Send(*this, "driver", "initial condition");
    }

    Verdandi::MessageHandler::Send(*this, "all", "::Initialize end");

    std::cout << "HERE4" << std::endl;

} // InitializeStructures

 /***********************************************************************************************************
  ***********************************************************************************************************
  *************************      START OF THE OBSERVATION MANAGERS' SECTION      ****************************
  ***********************************************************************************************************
  ***********************************************************************************************************/

 /***********************************************************************************************************
  ******************************      Mapped Points Obervation Manager      *********************************
  ***********************************************************************************************************/

 /**
  * Initialization.
  */
 template <class DataTypes1, class DataTypes2>
 void MappedPointsObservationManagerParallel<DataTypes1, DataTypes2>::init() {
     simulation::Node* gnode = dynamic_cast<simulation::Node*>(this->getContext());

     SNCOUTP("== init started");
     std::cout<<"gnode is "<<gnode->getName()<<std::endl;

     gnode->get(mapping); // look for mapping sibling
     if (mapping) {
         std::cout << "[" << this->getName() << "]: " << "found mapping: " << mapping->getName() << std::endl;
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no mapping found. This component will cause a segfault." << std::endl;

     gnode->get(observationSource); // look for observation source sibling
     if (observationSource) {
         std::cout << "[" << this->getName() << "]: " << "found observation source: " << observationSource->getName() << std::endl;
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no observation source found " << std::endl;

     SofaReducedOrderUKFParallel<SofaModelWrapperParallel<Real1>, SofaLinearObservationManagerParallel<Real1> >* sofaFilter;
     gnode->get(sofaFilter, core::objectmodel::BaseContext::SearchUp);
     if (sofaFilter) {
         sofaModel = sofaFilter->getModel();
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no SOFA model found " << std::endl;

     gnode->get(mappedState); // look for mapped state as a sibling component
     if (mappedState) {
         std::cout << "[" << this->getName() << "]: " << "found mapped mechanical state: " << mappedState->getName() << std::endl;
         typename MappedState::ReadVecCoord mappedPos = mappedState->readPositions();
         mappedStateSize = mappedPos.size();
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no mapped state found " << std::endl;


     m_sofaObjectParallel = NULL;
     masterState = dynamic_cast<MasterState*>(mapping->getFromModel()); // takes master state as the input of the mapping
     if (masterState != NULL) {
         m_sofaObjectParallel = sofaModel->getObject(masterState);
         typename MasterState::ReadVecCoord masterPos = masterState->readPositions();
         masterStateSize = masterPos.size();
         simulation::Node* test1=dynamic_cast<simulation::Node*>(masterState->getContext());
         simulation::Node* test=dynamic_cast<simulation::Node*>((test1->getParents())[0]);
         std::cout << "[" << this->getName() << "]: " << "master state is: "<<test->getName()<<"/"<<test1->getName()<<"/"<<masterState->getName()<<std::endl;
         std::cout << "[" << this->getName() << "]: " << "master state size is "<<masterStateSize<<std::endl;
     }
     else
         std::cerr << "[" << this->getName() << "]: ERROR SOFA MO not found!" << std::endl;

     if (!m_sofaObjectParallel)
         std::cerr << "[" << this->getName() << "]: ERROR SOFA object not found " << std::endl;

     /// initialize noise generator:
     if (m_noiseStdev.getValue() != 0.0) {
         pRandGen = new boost::mt19937;
         pNormDist = new boost::normal_distribution<>(0.0, m_noiseStdev.getValue());
         pVarNorm = new boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >(*pRandGen, *pNormDist);
     }

     actualStep = -1;

     SNCOUTP("== init done")
 } // init

 template <class DataTypes1, class DataTypes2>
 void MappedPointsObservationManagerParallel<DataTypes1, DataTypes2>::bwdInit() {
     SNCOUTP("== bwdInit started")
     typename DataTypes1::VecCoord& inputObservation = *inputObservationData.beginEdit();

     inputObservation = observationSource->getObservation(0.0);

     //std::cout << "Apply mapping on observations" << std::endl;

     sofa::core::MechanicalParams mp;
     mapping->apply(&mp, mappedObservationData, inputObservationData);

     noise.clear();
     noise.resize(3*mappedObservationData.getValue().size());
     SNCOUTP("== bwdInit done")
 } // bwdInit



 template <class DataTypes1, class DataTypes2>
 MappedPointsObservationManagerParallel<DataTypes1,DataTypes2>
 ::Inherit::observation& MappedPointsObservationManagerParallel<DataTypes1, DataTypes2>::GetInnovation(const typename SofaModelWrapperParallel<double>::state& x) {
     //std::cout << "[" << this->getName() << "]: new get innovation " << std::endl;
     //std::cout<<"n observation:" <<GetNobservation()<<"\n";


     //Data<typename DataTypes1::VecCoord> inputObservationData;
     //Data<typename DataTypes2::VecCoord> mappedObservationData;

     typename DataTypes1::VecCoord& inputObservation = *inputObservationData.beginEdit();


     //std::cout<<"AKDEBUG getting obs at time "<<this->time_<<"\n\n";
     inputObservation = observationSource->getObservation(this->time_);
     //std::cout << "AKDEBUG GI " << inputObservation[50] << " " << inputObservation[100] << std::endl;
     

     //std::cout << "Apply mapping on observations" << std::endl;

     sofa::core::MechanicalParams mp;
     mapping->apply(&mp, mappedObservationData, inputObservationData);

     //typename DataTypes2::VecCoord mappedObservation = *mappedObservationData.beginEdit();

     sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObservation = mappedObservationData;
     //std::cout << "AKDEBUG " << mappedObservation << std::endl;

     //std::cout << this->getName() << ": size of mapped observation: " << mappedObservation.size() << std::endl;

     int abberIx = m_abberantIndex.getValue();
     bool doAbber = (abberIx > -1 && abberIx < mappedObservation.size());

     Inherit::observation actualObs(mappedObservation.size()*3);
     for (size_t i = 0; i < mappedObservation.size(); i++) {
         if (doAbber && i == abberIx)
             mappedObservation[i][0] += actualStep*0.001;

         for (size_t d = 0; d < 3; d++) {
             mappedObservation[i][d] += noise[3*i+d];
             actualObs(3*i+d) = mappedObservation[i][d];
         }     
     }

     Data<typename DataTypes1::VecCoord> actualStateData;
     Data<typename DataTypes2::VecCoord> mappedStateData;

     typename DataTypes1::VecCoord& actualState = *actualStateData.beginEdit();
     typename DataTypes2::VecCoord& mappedState = *mappedStateData.beginEdit();

     mappedState.resize(mappedStateSize);
     sofaModel->SetSofaVectorFromVerdandiState(actualState, x, m_sofaObjectParallel);

     //std::cout << "AKDEBUG2 GI " << actualState[50] << " " << actualState[100] << std::endl;

     mapping->apply(&mp, mappedStateData, actualStateData);

     //std::cout << this->getName() << ": size of mapped state: " << mappedState.size() << std::endl;
     this->innovation_.Reallocate(mappedState.size()*3);
     for (size_t i = 0; i < mappedState.size(); i++)
         for (size_t d = 0; d < 3; d++)
             this->innovation_(3*i+d) = mappedState[i][d];

     //std::cout << "AKDEBUG " << this->innovation_ << std::endl;
     //std::cout << "AKDEBUG " << actualObs << std::endl;

     //this->innovation_.Reallocate(this->Nobservation_);
     //this->ApplyOperator(x, this->innovation_);     
     //Inherit::observation predObs  = this->innovation_;
     Mlt(double(-1.0), this->innovation_);     
     //Add(double(1.0), this->GetObservation(), this->innovation_);     
     Add(double(1.0), actualObs, this->innovation_);     
     //std::cout << this->getName() << ": innovation updated" << std::endl;

     //for (size_t i = 0; i < this->innovation_.GetM(); i++)
     //    std::cout << actualObs(i) << " " << predObs(i) << " " << this->innovation_(i) << std::endl;

     //std::cout << "ERROR VARIANCE: " << this->error_variance_ << std::endl;

     inputObservationData.endEdit();
     //mappedObservationData.endEdit();
     actualStateData.endEdit();
     mappedStateData.endEdit();

     //std::cout << "MPP: " << mappedObservationData.getValue() << std::endl;

     return this->innovation_;
 } // GetInnovation



 template <class DataTypes1, class DataTypes2>
 void MappedPointsObservationManagerParallel<DataTypes1, DataTypes2>::Initialize(SofaModelWrapperParallel<double>& /*model*/, std::string /*confFile*/) {
     Verb("initialize mappedPointsObsManager");
     //Inherit1::Initialize(model, confFile);

     this->Delta_t_ = 0.001;
     this->Nskip_= 1;
     this->initial_time_ = 0.0;
     this->final_time_ = 1000.0;


     if (int(masterStateSize) != observationSource->getNParticles()) {
         std::cerr << this->getName() << " ERROR: number of nodes in master state " << masterStateSize << " and observation source " << observationSource->getNParticles() << " differ!" << std::endl;
         return;
     }

     this->error_variance_value_ = m_observationStdev.getValue() * m_observationStdev.getValue();
     this->Nobservation_ = 3*mappedStateSize;
     this->error_variance_.Reallocate(this->Nobservation_, this->Nobservation_);
     this->error_variance_.SetIdentity();
     Mlt(this->error_variance_value_, this->error_variance_);
     this->error_variance_inverse_.Reallocate(this->Nobservation_, this->Nobservation_);
     this->error_variance_inverse_.SetIdentity();
     Mlt(double(double(1.0)/ this->error_variance_value_), this->error_variance_inverse_);

     std::cout << this->getName() << " size of observed state: " << this->Nobservation_ << std::endl;

     return;
 } // Initialize



} // namespace simulation

} // namespace sofa

///temporary hack to compile pthread barriers on apple
#ifdef __APPLE__
int pthread_barrier_init(pthread_barrier_t *barrier, const pthread_barrierattr_t *attr, unsigned int count)
{
    if(count == 0)
    {
        errno = EINVAL;
        return -1;
    }
    if(pthread_mutex_init(&barrier->mutex, 0) < 0)
    {
        return -1;
    }
    if(pthread_cond_init(&barrier->cond, 0) < 0)
    {
        pthread_mutex_destroy(&barrier->mutex);
        return -1;
    }
    barrier->tripCount = count;
    barrier->count = 0;

    return 0;
}

int pthread_barrier_destroy(pthread_barrier_t *barrier)
{
    pthread_cond_destroy(&barrier->cond);
    pthread_mutex_destroy(&barrier->mutex);
    return 0;
}

int pthread_barrier_wait(pthread_barrier_t *barrier)
{
    pthread_mutex_lock(&barrier->mutex);
    ++(barrier->count);
    if(barrier->count >= barrier->tripCount)
    {
        barrier->count = 0;
        pthread_cond_broadcast(&barrier->cond);
        pthread_mutex_unlock(&barrier->mutex);
        return 1;
    }
    else
    {
        pthread_cond_wait(&barrier->cond, &(barrier->mutex));
        pthread_mutex_unlock(&barrier->mutex);
        return 0;
    }
}
#endif
