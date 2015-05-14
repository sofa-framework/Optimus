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

#include <string> /* string */
#include <algorithm>    // std::sort
#include <utility>      // std::pair
#include "boost/bind.hpp"

namespace sofa
{

namespace simulation
{

// dim is 3!
template <class Type>
SofaModelWrapperParallel<Type>::SofaModelWrapperParallel()
    : Inherit()
    , current_row_(-1)
    , dim_(3)
    , state_size_parallel(0)
    , reduced_state_size_parallel(0)
    //, reduced_state_index_parallel(0)
{
    constraintSolver = NULL;
}

template <class Type>
SofaModelWrapperParallel<Type>::~SofaModelWrapperParallel()
{}

template <class Type>
void SofaModelWrapperParallel<Type>::Message(string _message) {
    std::cout << "Message: " << _message << std::endl;
    if (_message.find("initial condition") != string::npos || _message.find("forecast") != string::npos) {
        //Save();
    }
}

template <class Type>
struct threadLoopParam_t
{
    SofaModelWrapperParallel<Type>* model; // model the thread computes on
    int threadID; // the thread's ID in the model

};


template<class Type>
void doWork(SofaModelWrapperParallel<Type>* const& myModel, const int& myTid,
                   typename SofaModelWrapperParallel<Type>::ThreadData_t& myData)
{

    const bool preserve = myModel->t_input_preserveState; // ignored
    const bool update = myModel->t_input_updateForce;

    if (myData.localRoot==NULL)
    {
        std::cout<<"THREAD "<<myTid<<" reports FAILURE -- NULL local root\n";
    }
    for (int i=0; i<myData.sigmaPointIndices.size();i++)
    {
        std::cout<<"BB: Thread "<<myTid<<" has job "<<myData.sigmaPointIndices[i]<<" and total of "<<myData.sigmaPointIndices.size()<<"jobs \n";

        // works correctly
        myModel->StateVerdandi2SofaParallel(myData.sofaObjectsSlave, myData.paramsSlave,
                                            myModel->t_sigmaPoints[myData.sigmaPointIndices[i]]);
        myModel->Forward(update, false, myTid);
        myModel->StateSofa2VerdandiParallel(myData.sofaObjectsSlave, myData.paramsSlave,
                                            myModel->t_sigmaPoints[myData.sigmaPointIndices[i]]);

        std::cout<<"\n";
        std::cout<<"\n";
        std::cout<<"thread "<<myTid<<" managed to get a job done";
    }

}


template<class Type>
void *startThreadLoop (void *in_args)
{
    /** step 1: get args to local copy and sync with others*/
    threadLoopParam_t<Type> args = *((threadLoopParam_t<Type>*) in_args);

    // for code convenience
    SofaModelWrapperParallel<Type>* const myModel = args.model; // model the thread computes on
    const int myTid = args.threadID;
    typename SofaModelWrapperParallel<Type>::ThreadData_t& myData= myModel->t_threadData[myTid];

    std::cout<<"THREAD "<<myTid<<" started properly\n";

    std::cout<<"node"<<myData.localRoot->getName();
    std::cout<<"\nSofaObjectsSlave: ";

    for (int i=0;i<myData.sofaObjectsSlave.size();i++)
    {
        string temp1=myData.sofaObjectsSlave[0].vecMS->getContext()->getName();
        simulation::Node* context = dynamic_cast<simulation::Node*>(myData.sofaObjectsSlave[0].vecMS->getContext());
        string temp2=(context->getParents()[0])->getName();
        std::cout<<temp2<<"/"<<temp1<<"/"<<myData.sofaObjectsSlave[0].vecMS->getName()<<"\n";
    }


    pthread_barrier_wait (&myModel->t_initDone); // ensure all the threads have initiated correctly
    /** step 1 done */

    /** initiate working loop */
    while (true)
    {
        std::cout<<"thread "<<myTid<<" waiting for work\n";
        pthread_barrier_wait(&myModel->t_workAssigned); // wait for master to assign work for everyone

        switch (myModel->t_threadSignals[myTid])
        {
        case WORK_ENQUEUED:
            // work is ready for this thread!
            doWork(myModel, myTid, myData);
            std::cout<<"\nAAAAAAAAAAAAA: Thread ";
            std::cout<<myTid<<" done.\n";
            pthread_barrier_wait(&myModel->t_workDone); // inform the master that the work is done
            break;
        case NO_WORK:
            std::cout<<"\nAAAAAAAAAA: No work for thread "<<myTid<<"\n";
            pthread_barrier_wait(&myModel->t_workDone);
            break;
        case INITIALIZED:
            // same as work done at this point, no break.
            std::cout<<"this should never happen\n";
            return (void*)1;
        case WORK_DONE:
            // wait for work
            std::cout<<"this should never happen\n";
            return (void*)1;

        case CANCEL_THREAD:
            std::cout<<"canceling thread\n";
            return 0;
            break;
        case UNKNOWN:
            std::cout<<"this should never happen\n";
            return (void*)1;
        default:
            std::cout<<"this should never happen\n";
            return (void*) 1;

        }
    }
    return 0; // success
}


template <class Type>
helper::vector<typename SofaModelWrapperParallel<Type>::SofaObjectParallel> SofaModelWrapperParallel<Type>::getSofaObjects(simulation::Node* root)
{
    helper::vector<SofaObjectParallel> objects;

    std::cout << "Searching sofa objects: " << std::endl;
    helper::vector<MechStateVec3d*> mechanicalObjectsV;
    helper::vector<MechStateRigid3d*> mechanicalObjectsR;

    // get all the mechanical objects in the subtree starting with root
    root->get<MechStateVec3d>
            (&mechanicalObjectsV, BaseContext::SearchDown);
    root->get<MechStateRigid3d>
            (&mechanicalObjectsR, BaseContext::SearchDown);

    size_t objVCount=mechanicalObjectsV.size();
    size_t objRCount=mechanicalObjectsR.size();


    BaseMapping* mapping;
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

        if (object->getName().substr(0,4).compare("aux_")==0) continue; // intentionally uncounted object
        context->get(mapping);
        if (mapping!=NULL) continue; // mapped object

        // authentic node
        SofaObjectParallel obj;
        obj.node = context;
        context->get(obj.vecMS);
        context->get(obj.vecFC);
        context->get(obj.rigidMS);
        context->get(obj.rigidFC);
        objects.push_back(obj);

    }
    return objects;
}

// sort the pair only by the string
template <class Type>
bool operator<  (const std::pair<string, typename SofaModelWrapperParallel<Type>::SofaObjectParallel> lhs,
                 const std::pair<string, typename SofaModelWrapperParallel<Type>::SofaObjectParallel> rhs)
{
    return (lhs.first<rhs.first);
}

template<class Type>
vector<typename SofaModelWrapperParallel<Type>::ObjID> SofaModelWrapperParallel<Type>::objIDsFromObjects (vector<SofaObjectParallel> objects)
{
    vector<ObjID> objectIDs;
    ObjID temp;

    // bind node name and object name to the object
    for (int i=0;i<objects.size();i++)
    {
        temp.first=objects[i].node->getName()+objects[i].vecMS->getName();
        temp.second=objects[i];
        objectIDs.push_back(temp);
    }

    return objectIDs;
}

template<class Type>
vector<typename SofaModelWrapperParallel<Type>::ObjID> SofaModelWrapperParallel<Type>::getSortedIDs (helper::vector<SofaObjectParallel>& objects)
{
    vector<ObjID> objectIDs;
    ObjID temp;

    // bind node name and object name to the object
    for (int i=0;i<objects.size();i++)
    {
        temp.first=objects[i].node->getName()+objects[i].vecMS->getName();
        temp.second=objects[i];
        objectIDs.push_back(temp);
    }

    // sort by node name and object name

    std::sort(objectIDs.begin(), objectIDs.end(),
              boost::bind(&ObjID::first, _1) <
              boost::bind(&ObjID::first, _2));

    return objectIDs;
}




template <class Type>
bool SofaModelWrapperParallel<Type>::synchronizeSofaObjects ()
{
    vector<ObjID> masterIDs;
    vector<ObjID> slaveIDs;

    // apply ordering to objects
    masterIDs=getSortedIDs(sofaObjectsMaster);
    for (int i=0;i<masterIDs.size();i++)
    {
        sofaObjectsMaster[i]=masterIDs[i].second;
    }

    // for each slave scene, match its object order to master scene
    for (int i=0; i<m_slaveCount;i++)
    {
        std::cout<<"slave "<<i<<std::endl; // remove later
        slaveIDs=getSortedIDs(t_threadData[i].sofaObjectsSlave);
        if (slaveIDs.size() != masterIDs.size())
        {
            std::cout<<"ERROR: Failed to match Slave to Master; the scenes differ in authentic node count.\n";
            continue;//return false;
        }

        // browse through slaveObjects
        for (int j=0;j<slaveIDs.size();j++)
        {
            std::cout<<"Master: "<<masterIDs[j].first<<"\t\tSlave:"<<slaveIDs[j].first<<std::endl; // remove later
            if (masterIDs[j].first.compare(slaveIDs[j].first))
            {
                // forbids mechanical object in root of subscene!
                std::cout<<"ERROR: Failed to match Slave to Master; Rename the objects or subnodes.\n";
                continue;//return false;
            }
            {
                // mechanical objects matched
                t_threadData[i].sofaObjectsSlave[j]=slaveIDs[j].second;

            }
        }
    }
    return true;
}


template <class Type>
void SofaModelWrapperParallel<Type>::initSimuData(ModelData &_md)
{
    /// set variables given by animation loop
    Verb("initSimuData");

    modelData = _md;
    simulation::Node* gnode = modelData.gnode;

    /// register the object in the scene
    std::cout << "Registering object: " << this->GetName() << std::endl;
    this->setName("SofaModelWrapperParallel");
    gnode->addObject(this);
    gnode->get(constraintSolver, core::objectmodel::BaseContext::SearchDown);
    if (constraintSolver == NULL)
        std::cout << "No ConstraintSolver found, considering the version with no contacts" << std::endl;
    else
        std::cout << "Constraint solver " << constraintSolver->getName() << " found, modeling contacts" << std::endl;


    std::cout << "Searching optim params: " << std::endl;
    helper::vector<OptimParams*> listOP; // mechanical objects
    gnode->get<OptimParams>(&listOP, BaseContext::SearchRoot ); // nall the OptimParams objects from the tree

    // context -> pointer na base mapping ->get SearchLocal

    std::cout<<"params found: "<<listOP.size()<<std::endl;

    //destroy all these somewhen
    m_slaveCount = listOP.size()-1;
    threadLoopParam_t<Type>* args = new threadLoopParam_t<Type>[listOP.size()];
    pthread_barrier_init (&t_barrier, NULL, listOP.size());
    pthread_barrier_init (&t_initDone, NULL, listOP.size());
    pthread_barrier_init (&t_workAssigned, NULL, listOP.size());
    pthread_barrier_init (&t_workDone, NULL, listOP.size());

    t_threadSignals = new ThreadSignal[listOP.size()];
    t_threadData = new ThreadData_t[listOP.size()];
    t_threadHandlers = new pthread_t[listOP.size()];
    //t_execParams = new typename core::ExecParams*[listOP.size()];

    for (size_t iop=0; iop <listOP.size(); iop++)
    {
        t_threadSignals[iop]=INITIALIZED;
    }

    int masterCount = 0; // at this point assumed 1
    int slaveCount = 0; // at this point assumed listOP.size()-1

    vector<ObjID> masterIDs;

    for (size_t iop = 0; iop < listOP.size(); iop++) {

        OptimParams* oparam = listOP[iop];
        simulation::Node* opnode = dynamic_cast<simulation::Node*>(oparam->getContext()); // node in which oparam is rooted

        std::cout<<"param "<<iop<<std::endl;
        // !! we assume we have enough slave nodes and they each have their own optimParams, with optimize toggle off !!

        /** fix, want just one in main scene*/
        /* original*/
        /**
        if (!opnode->getName().compare("MasterScene"))
        {
            opnode = opnode->getChild("Cylinder");
            SofaObject obj;
            obj.node=opnode;
            opnode->get(obj.vecMS);
            opnode->get(obj.rigidMS);

            opnode->get(obj.vecFC);
            opnode->get(obj.rigidFC);

            bool existingObject = false;
            for (size_t i = 0; i < sofaObjects.size(); i++) {
                if (sofaObjects[i].node == obj.node) {
                    sofaObjects[i].oparams.push_back(oparam);
                    existingObject = true;
                    break;
                }
            }

            if (!existingObject) {
                obj.oparams.push_back(oparam);

                if (obj.vecMS == NULL && obj.rigidMS == NULL)
                    std::cerr << "PROBLEM: NO MECHANICAL STATE FOUND! " << std::endl;

                sofaObjects.push_back(obj);
            }

        }
        else
        {
        }
        */
        /* */
    }

    for (size_t iop = 0; iop < listOP.size(); iop++) {

        OptimParams* oparam = listOP[iop];
        simulation::Node* opnode = dynamic_cast<simulation::Node*>(oparam->getContext()); // node in which oparam is rooted

        std::cout<<"param "<<iop<<std::endl;

        // !! we assume we have enough slabe nodes
        if (!opnode->getName().compare("MasterScene"))
        {
            oparam->setOptimize(true); // will probably drop optimize entirely
            std::cout<<"above optimized!\n\n";
            masterCount++;

            sofaObjectsMaster = getSofaObjects(opnode);
            paramsMaster=oparam;

        }
        else
        {
            // sofaObjectsSlave
            t_threadData[slaveCount].localRoot=opnode;
            t_threadData[slaveCount].sofaObjectsSlave = getSofaObjects(opnode);
            t_threadData[slaveCount].paramsSlave=oparam;
            t_threadData[slaveCount].sigmaPointIndices=vector<int>();
            // slave scene - warning, not checking for name
            oparam->setOptimize(false);

            /** step 1: create Thread */
            //std::cout<<"Thread "<<slaveCount<<" started\n";
            args[slaveCount].model=this;
            args[slaveCount].threadID=slaveCount;
            pthread_create(t_threadHandlers+slaveCount, NULL, startThreadLoop<Type>, (void *) (args+slaveCount));
            //std::cout<<"Thread "<<slaveCount<<" done\n";

            slaveCount++;
        }
    }
    synchronizeSofaObjects(); // all the threads get their mechanical states correctly paired
    pthread_barrier_wait (&t_initDone); // ensure all the threads have initiated correctly
    delete[] args; // can now be freed (only after the threads have initiated!)

    std::cout << this->getName() << " NUMBER of SOFA objects: " << sofaObjectsMaster.size() << std::endl;
    numStep = 0;
    //delete[] m_threads;
}



template<class Type>
typename SofaModelWrapperParallel<Type>::SofaObjectParallel* SofaModelWrapperParallel<Type>::getObject(typename SofaModelWrapperParallel<Type>::MechStateVec3d *_state) {
    for (size_t iop = 0; iop < sofaObjectsMaster.size(); iop++) {
        if (sofaObjectsMaster[iop].vecMS == _state) {
            std::cout << this->getName() << " : found SOFA object " << iop << std::endl;
            return &sofaObjectsMaster[iop];
        }
    }
    return(NULL);
}



// works?
template<class Type>
void SofaModelWrapperParallel<Type>::SetSofaVectorFromVerdandiState(defaulttype::Vec3dTypes::VecCoord & vec, const state &_state, SofaObjectParallel* obj) {
    vec.clear();
    typename MechStateVec3d::ReadVecCoord pos = obj->vecMS->readPositions();
    vec.resize(pos.size());
    for (size_t i = 0; i < vec.size(); i++) {
        //std::cout << "[" << i << "]: " << pos[i] << std::endl;
        vec[i] = pos[i];
    }

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj->positionPairs.begin(); it != obj->positionPairs.end(); it++)
        for (size_t d = 0; d < dim_; d++) {
            //std::cout << "x[" << it->first << "]: " << vec[it->first] << std::endl;
            vec[it->first][d] = _state(reduced_state_size_parallel+dim_*it->second + d);
        }

}


/// copy the actual SOFA mechanical object and parameter vector to Verdandi state vector using already created index maps
/**
template <class Type>
void SofaModelWrapperParallel<Type>::StateSofa2Verdandi() {
    std::cout<<"INFO: sofa2Verdandi!\n";
    /// for all sofaObjects (associated with nodes having OptimParams component)
    for (size_t iop = 0; iop < sofaObjects.size(); iop++) {
        SofaObject& obj = sofaObjects[iop];

        if (obj.vecMS != NULL) {
            typename MechStateVec3d::ReadVecCoord pos = obj.vecMS->readPositions();
            typename MechStateVec3d::ReadVecDeriv vel = obj.vecMS->readVelocities();

            // free nodes 0 to N mapped onto
            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    state_(dim_*it->second + d) = pos[it->first][d];

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    state_(dim_*it->second + d) = vel[it->first][d];
        }

        if (obj.rigidMS != NULL) {
            typename MechStateRigid3d::ReadVecCoord pos = obj.rigidMS->readPositions();
            typename MechStateRigid3d::ReadVecDeriv vel = obj.rigidMS->readVelocities();

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                defaulttype::Rigid3dTypes::CPos rpos = defaulttype::Rigid3dTypes::getCPos(pos[it->first]);                
                for (size_t d = 0; d < dim_; d++)
                    state_(dim_*it->second + d ) = rpos[d];
            }

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++) {
                defaulttype::Rigid3dTypes::DPos rvel = Rigid3dTypes::getDPos(vel[it->first]);
                for (size_t d = 0; d < dim_; d++)
                    state_(dim_*it->second + d ) = rvel[d];

            }
        }

        for (size_t opi = 0; opi < obj.oparams.size(); opi++)
        {
            ///
            std::cout<<"INFO: going paramstoraw!\n";
            // takes the vector in params and saves it into the given vector, without checking for its size or alloc.
            std::cout<<"state size:"<<(state_.GetM())<<std::endl;
            double test[10];
            for (int i=0;i<10;i++) test[i]=-2.5;
            obj.oparams[opi]->paramsToRawVector(test, 10); // redundant parameter protected subfunction
            for (int i=0;i<10;i++) std::cout<<i<<": "<<test[i]<<std::endl;//
            obj.oparams[opi]->paramsToRawVector(state_.GetData(), state_.GetM());

        }



    std::cout<<"ERROR ERROR ERROR: should never call sofa2verdandi anymore\n";
    std::cout<<"ERROR ERROR ERROR: should never call sofa2verdandi anymore\n";
    std::cout<<"ERROR ERROR ERROR: should never call sofa2verdandi anymore\n";

}*/

template <class Type>
void SofaModelWrapperParallel<Type>::StateSofa2VerdandiParallel(const helper::vector<SofaObjectParallel>& mechanicalObjects,
                                                                OptimParams* oparams, state& verdandiState)
{
    /// for all given objects
    for (size_t iop = 0; iop < mechanicalObjects.size(); iop++) {
        const SofaObjectParallel& obj = mechanicalObjects[iop];

        if (obj.vecMS != NULL) {
            typename MechStateVec3d::ReadVecCoord pos = obj.vecMS->readPositions();
            typename MechStateVec3d::ReadVecDeriv vel = obj.vecMS->readVelocities();

            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    verdandiState(reduced_state_size_parallel+dim_*it->second + d) = pos[it->first][d];


            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    verdandiState(reduced_state_size_parallel+dim_*it->second + d) = vel[it->first][d];
        }

        if (obj.rigidMS != NULL) {
            typename MechStateRigid3d::ReadVecCoord pos = obj.rigidMS->readPositions();
            typename MechStateRigid3d::ReadVecDeriv vel = obj.rigidMS->readVelocities();

            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                defaulttype::Rigid3dTypes::CPos rpos = defaulttype::Rigid3dTypes::getCPos(pos[it->first]);
                for (size_t d = 0; d < dim_; d++)
                    verdandiState(reduced_state_size_parallel+dim_*it->second + d ) = rpos[d];
            }

            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++) {
                defaulttype::Rigid3dTypes::DPos rvel = Rigid3dTypes::getDPos(vel[it->first]);
                for (size_t d = 0; d < dim_; d++)
                    verdandiState(reduced_state_size_parallel+dim_*it->second + d ) = rvel[d];

            }
        }


        // save the param values into the verdandi state
        oparams->paramsToRawVectorParallel(verdandiState.GetData());

        double* data = verdandiState.GetData();
        std::cout<<"parallel data\n";
        for (int i=0;i<10;i++)
        {
            std::cout<<data[i]<<std::endl;
        }

    }
}

template <class Type>
void SofaModelWrapperParallel<Type>::StateVerdandi2SofaParallel(helper::vector<SofaObjectParallel>& mechanicalObjects,
                                                                OptimParams* oparams, const state& verdandiState)
{

    for (size_t iop = 0; iop < mechanicalObjects.size(); iop++) {
        SofaObjectParallel& obj = mechanicalObjects[iop];
        std::cout << "P: Verdandi => Sofa on " << obj.vecMS->getName() <<  std::endl;
        std::cout<<obj.vecMS->getContext()->getName();
        std::cout<<" : "<<(obj.vecMS)->getName()<<"\n";


        if (obj.vecMS != NULL) {
            std::cout<<"mapping "<<obj.vecMS->getName()<<"\n";
            typename MechStateVec3d::WriteVecCoord pos = obj.vecMS->writePositions();
            typename MechStateVec3d::WriteVecDeriv vel = obj.vecMS->writeVelocities();

            //size_t ii = 0;
            // map the mechanical object verdandi position onto sofa
            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                for (size_t d = 0; d < dim_; d++) {
                    pos[it->first][d] = verdandiState(reduced_state_size_parallel+dim_*it->second + d);
                }
            }
            //std::cout << std::endl;

            //map the mechanical object verdandi velocity onto sofa
            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    vel[it->first][d] = verdandiState(reduced_state_size_parallel+dim_*it->second + d);
        }

        //std::cout<<"yes indeed";

        if (obj.rigidMS != NULL) {
            typename MechStateRigid3d::WriteVecCoord pos = obj.rigidMS->writePositions();
            typename MechStateRigid3d::WriteVecDeriv vel = obj.rigidMS->writeVelocities();


            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                defaulttype::Rigid3dTypes::CPos rpos = defaulttype::Rigid3dTypes::getCPos(pos[it->first]);
                for (size_t d = 0; d < dim_; d++)
                    rpos[d] = verdandiState(reduced_state_size_parallel+dim_*it->second + d );
            }

            for (helper::vector<std::pair<size_t, size_t> >::const_iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++) {
                defaulttype::Rigid3dTypes::DPos rvel = Rigid3dTypes::getDPos(vel[it->first]);
                for (size_t d = 0; d < dim_; d++)
                    rvel[d] = verdandiState(reduced_state_size_parallel+dim_*it->second + d );

            }
        }

        MechanicalParams mp;
        MechanicalPropagatePositionAndVelocityVisitor(&mp).execute( obj.node ); // snad ok

        /// let the OptimParams to extract the actual values of parameters from the verdandi state
        oparams->rawVectorToParamsParallel(verdandiState.GetData());
    }
    std::cout<<"verd2sofa done!\n";

}

/// copy a Verdandi state to SOFA mechanical object and parameter vector into the corresponding OptimParams
/**
template <class Type>
void SofaModelWrapperParallel<Type>::StateVerdandi2Sofa() {
    for (size_t iop = 0; iop < sofaObjects.size(); iop++) {
        SofaObject& obj = sofaObjects[iop];
        std::cout << "Verdandi => Sofa on " << obj.vecMS->getName() <<  std::endl;

        if (obj.vecMS != NULL) {
            typename MechStateVec3d::WriteVecCoord pos = obj.vecMS->writePositions();
            typename MechStateVec3d::WriteVecDeriv vel = obj.vecMS->writeVelocities();

            //size_t ii = 0;
            // map the mechanical object verdandi position onto sofa
            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                for (size_t d = 0; d < dim_; d++) {
                    pos[it->first][d] = state_(dim_*it->second + d);                    
                }
            }
            //std::cout << std::endl;

            //map the mechanical object verdandi velocity onto sofa
            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    vel[it->first][d] = state_(dim_*it->second + d);
        }

        if (obj.rigidMS != NULL) {
            typename MechStateRigid3d::WriteVecCoord pos = obj.rigidMS->writePositions();
            typename MechStateRigid3d::WriteVecDeriv vel = obj.rigidMS->writeVelocities();

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                defaulttype::Rigid3dTypes::CPos rpos(state_(dim_*it->second), state_(dim_*it->second + 1), state_(dim_*it->second + 2));
                defaulttype::Rigid3dTypes::setCPos(pos[it->first], rpos);
                //std::cout << rpos << std::endl;
            }

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++) {
                defaulttype::Rigid3dTypes::DPos rvel(state_(dim_*it->second), state_(dim_*it->second + 1), state_(dim_*it->second + 2));
                defaulttype::Rigid3dTypes::setDPos(vel[it->first], rvel);
            }
        }

        MechanicalParams mp;
        MechanicalPropagatePositionAndVelocityVisitor(&mp).execute( obj.node );

        /// let the OptimParams to extract the actual values of parameters from the verdandi state
        for (size_t opi = 0; opi < obj.oparams.size(); opi++)
            obj.oparams[opi]->rawVectorToParams(state_.GetData(), state_.GetM());


    }
    std::cout<<"ERROR ERROR ERROR: should never call verdandi2sofa anymore!\n";
    std::cout<<"ERROR ERROR ERROR: should never call verdandi2sofa anymore!\n";
    std::cout<<"ERROR ERROR ERROR: should never call verdandi2sofa anymore!\n";

}*/

template <class Type>
void SofaModelWrapperParallel<Type>::Initialize()
{
    SNCOUTP("== InitializeParallel started")
    Verb("initialize SofaModelWrapperParallel");
    /// get fixed nodes


    size_t vsi = 0;
    //size_t vpi = 0;
    //size_t vsi = 0;

    /** own function, for slaves as well*/
    for (size_t iop = 0; iop < sofaObjectsMaster.size(); iop++) {
        SofaObjectParallel& obj = sofaObjectsMaster[iop];

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
        std::cout<<"P: free nodes: "<<freeNodes.size()<<std::endl;

        obj.positionPairs.clear();
        obj.velocityPairs.clear();


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

        //reduced_state_index_parallel = dim_ * vsi; // this many parameters are needed to save the objects so far loaded


        /**
        helper::vector<size_t> opv;
        for (size_t i = 0; i < paramsMaster->size(); i++) {
            opv.push_back(i);
            //opv.push_back(reduced_state_index_+vpi);vpi++;
        }
        paramsMaster->setVStateParamIndices(opv);*/



        //state_size_parallel = dim_* vsi + vpi; // vsi vpi je podezrely u vic objektu
        //reduced_state_size_parallel = vpi;
        //reduced_state_index_parallel = 0;
        state_size_parallel = dim_* vsi + paramsMaster->size();
        reduced_state_size_parallel = paramsMaster->size();
    }
    std::cout << "P: Initializing model (filter type " << modelData.filterTypeP << ") with size " << state_size_parallel<< std::endl;
    std::cout << "P: Reduced state size: " << reduced_state_size_parallel << std::endl;

    verdandiStateMaster.Nullify();
    verdandiStateMaster.Resize(state_size_parallel);
    m_sigmaPointCount=4;
    t_sigmaPoints = new state[m_sigmaPointCount];
    std::cout<<"sigma point count: "<<m_sigmaPointCount<<"\n";
    for (int i=0;i<m_sigmaPointCount;i++)
    {
        t_sigmaPoints[i].Nullify();
        t_sigmaPoints[i].Resize(state_size_parallel);
    }
    std::cout<<"\nstate size parallel:"<<state_size_parallel<<std::endl;

    StateSofa2VerdandiParallel(sofaObjectsMaster, paramsMaster, verdandiStateMaster);

    if (modelData.filterTypeP == ROUKFP) {
        variance_projector_allocated_ = false;
        variance_reduced_allocated_ = false;
    }
    SNCOUTP("== P: Initialize done")
}

template <class Type>
void SofaModelWrapperParallel<Type>::FinalizeStep() {
    std::cout << "Actual parameter values: ";
        for (size_t i = 0; i < reduced_state_size_parallel; i++)
            std::cout << " " << verdandiStateMaster(i);
        std::cout << std::endl;
}



// returns a reference to duplicate state, which is an exact copy of the system state, unique to the model and rewritten each time GetState is called
template <class Type>
typename SofaModelWrapperParallel<Type>::state& SofaModelWrapperParallel<Type>::GetState() {
    /// propagate the SOFA state towards verdandi state
    StateSofa2VerdandiParallel(sofaObjectsMaster, paramsMaster, verdandiStateMaster);

    /// return a reference to duplicate state
    //std::cout<<"sofa: passing a vector "<<verdandiStateMaster.GetDataSize()<<" long\n";
    return verdandiStateMaster;
}


// updates verdandi state to the sofa state of the master node
template <class Type>
void SofaModelWrapperParallel<Type>::StateUpdated() {
    if (modelData.verbose)
        std::cout << "[" << this->getName() << "]: state updated " << std::endl;

    StateVerdandi2SofaParallel(sofaObjectsMaster, paramsMaster, verdandiStateMaster);
}

template <class Type>
void SofaModelWrapperParallel<Type>::SetTime(double _time) {
    time_ = _time;
    simulation::Node* gnode = modelData.gnode;
    gnode->setTime ( _time );
    gnode->execute< UpdateSimulationContextVisitor >(execParams);
}

/**
template <class Type>
void SofaModelWrapperParallel<Type>::GetStateCopy(state& _copy) {
    Verb("get state copy");
    _copy.Reallocate(state_.GetM());
    for (int i = 0; i < state_.GetM(); i++)
        _copy(i) = state_(i);
}*/

template <class Type>
void SofaModelWrapperParallel<Type>::distributeWork()
{

    for (int i=0;i<m_slaveCount;i++)
    {
        t_threadData[i].sigmaPointIndices.clear();
        if (i<m_sigmaPointCount)
        {
            t_threadSignals[i]=WORK_ENQUEUED;
        }
        else
        {
            t_threadSignals[i]=NO_WORK;
        }
    }
    // distribute the sigma points among the threads in a round robin fashion
    for (int i=0;i<m_sigmaPointCount;i++)
    {
        std::cout<<"\nwork "<<i<<"to thread"<<(i%m_slaveCount)<<"\n";

        t_threadData[i%m_slaveCount].sigmaPointIndices.push_back(i);
    }
}

template <class Type>
double SofaModelWrapperParallel<Type>::ApplyOperatorParallel(state* sigmaPoints, bool _preserve_state, bool _update_force)  {

    Verb("apply operator parallel begin\n");
    for (int i=0;i<m_sigmaPointCount;i++) // sigma point count, need to sort out
    {
        //std::cout<<i<<": ";
        for (int j=0;j<state_size_parallel;j++)
        {
            t_sigmaPoints[i](j)=sigmaPoints[i](j);
            //if (j<30) std::cout<<t_sigmaPoints[i](j)<<", ";
        }
        //std::cout<<"\n\n";
    }

    distributeWork();

    std::cout<<("work assigned\n");


    pthread_barrier_wait(&t_workAssigned); // let the threads know they have work
    std::cout<<"past first";
    pthread_barrier_wait(&t_workDone); // wait for the threads to let the master know they are done

    //SetTime(GetTime()+modelData.gnode->getDt());

    for (int i=0;i<m_sigmaPointCount;i++) // sigma point count, need to sort out
    {
        std::cout<<i<<": ";
        for (int j=0;j<state_size_parallel;j++)
        {
            sigmaPoints[i](j)=t_sigmaPoints[i](j);
            if (j<30) std::cout<<t_sigmaPoints[i](j)<<", ";
        }
        std::cout<<"\n\n";
    }
    Verb("apply operator parallel end\n");
    return GetTime()+modelData.gnode->getDt();
}


template <class Type>
void SofaModelWrapperParallel<Type>::Forward(bool _update_force, bool _update_time, int thread)
{
    Verb("forward begin");

    if (constraintSolver)
    {
        Verb("Free motion not implemented yet in parallel.\n");
    }
    else
    {
        StepDefault(_update_force, _update_time, thread);
    }

    Verb("forward end");
}

template <class Type>
void SofaModelWrapperParallel<Type>::StepDefault(bool _update_force, bool _update_time, int thread) {
    if (_update_force) {

    }

    simulation::Node* gnode; // the simulation will start from here
    if (thread == -1)
    {
        std::cout<<"XXX:simulation form GLOBAL root\nWRONG WRONG WRONG \n";
        gnode = modelData.gnode;
    }
    else
    {
        std::cout<<"XXX:simulation form local root\n";
        gnode=t_threadData[thread].localRoot;
    }

    //const core::ExecParams*& params = execParams;
    core::ExecParams* params = sofa::core::ExecParams::defaultInstance();
    //*params = *execParams;

    double    dt = gnode->getDt();

    //std::cout << "[" << this->getName() << "]: step default begin" << std::endl;

    // not thread safe
    //sofa::helper::AdvancedTimer::stepBegin("AnimationStep");
    // not thread safe
    //sofa::helper::AdvancedTimer::begin("Animate");

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    {
        //std::cout << "[" << this->getName() << "]: animate begin" << std::endl;
        AnimateBeginEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        gnode->execute ( act );
    }

    double startTime = gnode->getTime();

    //std::cout << "[" << this->getName() << "]: behaviour update position" << std::endl;
    BehaviorUpdatePositionVisitor beh(params , dt);
    gnode->execute ( beh );

    //std::cout << "[" << this->getName() << "]: animate" << std::endl;
    AnimateVisitor act(params, dt);
    gnode->execute ( act );

    if (_update_time) {
        //std::cout << "[" << this->getName() << "]: update simulation context" << std::endl;
        gnode->setTime ( startTime + dt );
        gnode->execute< UpdateSimulationContextVisitor >(params);
    }

    {
        //std::cout << "[" << this->getName() << "]: animate end" << std::endl;
        AnimateEndEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        gnode->execute ( act );
    }


    // not thread safe
    //sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");

    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    //std::cout << "[" << this->getName() << "]: update mapping" << std::endl;
    gnode->execute< UpdateMappingVisitor >(params);

    // not thread safe
    //sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        //std::cout << "[" << this->getName() << "]: update mapping end" << std::endl;
        UpdateMappingEndEvent ev ( dt );
        PropagateEventVisitor act ( params , &ev );
        gnode->execute ( act );
    }
    // not thread safe
    //sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");

#ifndef SOFA_NO_UPDATE_BBOX
    // not thread safe
    //sofa::helper::AdvancedTimer::stepBegin("UpdateBBox");
    gnode->execute< UpdateBoundingBoxVisitor >(params);
    // not thread safe
    //sofa::helper::AdvancedTimer::stepEnd("UpdateBBox");
#endif
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("Step");
#endif
    // not thread safe
    //sofa::helper::AdvancedTimer::end("Animate");
    //sofa::helper::AdvancedTimer::stepEnd("AnimationStep");
}




template <class Type>
typename SofaModelWrapperParallel<Type>::state_error_variance& SofaModelWrapperParallel<Type>::GetStateErrorVariance() {
    std::cout << this->getName() << " " << GetTime() << " getStateErrorVariance" << std::endl;
    return state_error_variance_;
}


template <class Type>
typename SofaModelWrapperParallel<Type>::state_error_variance_row& SofaModelWrapperParallel<Type>::GetStateErrorVarianceRow(int row)
{
    if (row == current_row_)
        return state_error_variance_row_;

    GetRow(state_error_variance_, row, state_error_variance_row_);
    current_row_ = row;

    return state_error_variance_row_;
}

template <class Type>
typename SofaModelWrapperParallel<Type>::state_error_variance& SofaModelWrapperParallel<Type>::GetStateErrorVarianceProjector() {

    std::cout << "GetSEV_PROJECTOR" << std::endl;
    if (!variance_projector_allocated_)
    {
        state_error_variance_projector_.Reallocate(state_size_parallel, reduced_state_size_parallel);
        state_error_variance_projector_.Fill(Type(0.0));

        for (size_t i = 0, l = 0; i < 1; i++) {   /// only one reduced state - wtf?
            for (size_t k = 0; k < reduced_state_size_parallel; k++) {
                std::cout << "k,l = " << k << " " << l << std::endl;
                state_error_variance_projector_(k, l++) = 1;
            }
        }
        //std::cout << "Initialize L: " << std::endl;
        //printMatrix(state_error_variance_projector_, std::cout);
        variance_projector_allocated_ = true;
    }
    return state_error_variance_projector_;
}

template <class Type>
typename SofaModelWrapperParallel<Type>::state_error_variance& SofaModelWrapperParallel<Type>::GetStateErrorVarianceReduced() {

    if (!variance_reduced_allocated_)
    {
        state_error_variance_reduced_.Reallocate(reduced_state_size_parallel,  reduced_state_size_parallel);
        state_error_variance_reduced_.Fill(Type(0.0));

        helper::vector<double> stdev;
        paramsMaster->getStDev(stdev);
        std::cout<<"stdev:\n";
        for (size_t i; i<paramsMaster->size();i++)
        {
            state_error_variance_reduced_(i,i)= Type(Type(1.0) / (stdev[i] * stdev[i]));
            std::cout<<"stdev["<<i<<"]\t"<<stdev[i]<<std::endl;
        }

        std::cout << "  Initialize U: " << std::endl;
        printMatrix(state_error_variance_reduced_, std::cout);
        std::cout<<"testX";
        variance_reduced_allocated_ = true;
        std::fstream f;
        f.open("U.mat", std::fstream::out);
        printMatrixInRow(state_error_variance_reduced_, f);
        f.close();
    } else {
        printMatrix(state_error_variance_reduced_, std::cout);
        std::fstream f;
        f.open("U.mat", std::fstream::out | std::fstream::app);
        printMatrixInRow(state_error_variance_reduced_, f);
        f.close();
    }
    //std::cout << "U = " << state_error_variance_reduced_ << std::endl;
    std::cout << "OK yes" << std::endl;

return state_error_variance_reduced_;
/**

    if (!variance_reduced_allocated_)
    {
        //int Nreduced = 0;
        //for (unsigned int i = 0; i < reduced_.size(); i++)
        //    Nreduced += x_.GetVector(reduced_[i]).GetSize();
        //std::cout << "  R-Nreduced: " << Nreduced << std::endl;
        // Initializes U.
        //SERR("Reduced size: " << reduced_state_size_)
        state_error_variance_reduced_.Reallocate(reduced_state_size_,  reduced_state_size_);
        state_error_variance_reduced_.Fill(Type(0.0));
        //for (size_t i = 0; i < reduced_state_size_; i++)
        //    state_error_variance_reduced_(i, i) = Type(Type(1.0) / modelData.errorVarianceSofaParams);
        //SERR("SObjs size: " << sofaObjects.size())
        std::cout <<"sobjects: "<<sofaObjects.size()<<std::endl;

        for (size_t soi = 0, vpi = 0; soi < sofaObjects.size(); soi++) {
            SofaObject& obj = sofaObjects[soi];
            //SERR("SObji size: " << obj.oparams.size())
            for (size_t opi = 0; opi < obj.oparams.size(); opi++) {
                OptimParams* oparam = obj.oparams[opi];
                //const helper::vector<Type>& stdev = op->getStdev();
                helper::vector<double> stdev;
                //SERR("getStDev")
                oparam->getStDev(stdev);

                for (size_t pi = 0; pi < oparam->size(); pi++, vpi++)
                    state_error_variance_reduced_(vpi, vpi) = Type(Type(1.0) / (stdev[pi] * stdev[pi]));
            }
        }

        std::cout << "  Initialize U: " << std::endl;
        printMatrix(state_error_variance_reduced_, std::cout);
        variance_reduced_allocated_ = true;
        std::fstream f;
        f.open("U.mat", std::fstream::out);
        printMatrixInRow(state_error_variance_reduced_, f);
        f.close();
    } else {
        printMatrix(state_error_variance_reduced_, std::cout);
        std::fstream f;
        f.open("U.mat", std::fstream::out | std::fstream::app);
        printMatrixInRow(state_error_variance_reduced_, f);
        f.close();
    }
    //std::cout << "U = " << state_error_variance_reduced_ << std::endl;
    std::cout << "OK" << std::endl;

    return state_error_variance_reduced_;
    */
}


/// ROUKF:
template <class Model, class ObservationManager>
SofaReducedOrderUKFParallel<Model, ObservationManager>::SofaReducedOrderUKFParallel()
    //: Inherit1()
    : Inherit2()
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
    , m_threadCount( initData(&m_threadCount, 1, "threadCount", "degree of shared memory parallelism"))

{
}


template <class Model, class ObservationManager>
void SofaReducedOrderUKFParallel<Model, ObservationManager>::init() {
    SNCOUTP("== init started")
    SofaModelWrapperParallel<double>::ModelData md;
    md.positionInState = m_positionInState.getValue();
    md.velocityInState = m_velocityInState.getValue();    
    md.filterTypeP = ROUKFP;
    md.gnode = dynamic_cast<simulation::Node*>(this->getContext());    

    this->model_.initSimuData(md);
    SNCOUTP("== init done")
}


template <class Model, class ObservationManager>
void SofaReducedOrderUKFParallel<Model, ObservationManager>::InitializeFilter() { //VerdandiROUKFParams* _roukfParams) {
    SNCOUTP("== InitializeFilter started")
    simulation::Node* gnode = dynamic_cast<simulation::Node*>(this->getContext());

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

    std::cout<<"alive";
    SNCOUTP("=== InitializeStructures...")
    InitializeStructures();
    std::cout<<"\ndead?\n";

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
}


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


}

/*template <class Model, class ObservationManager>
void SofaReducedOrderUKFParallel<Model, ObservationManager>
::Initialize(std::string configuration_file,
             bool initialize_model, bool initialize_observation_manager)
{
    Verdandi::VerdandiOps configuration(configuration_file);
    Initialize(configuration, initialize_model,
               initialize_observation_manager);
}

template <class Model, class ObservationManager>
void SofaReducedOrderUKFParallel<Model, ObservationManager>::Initialize(Verdandi::VerdandiOps& configuration, bool initialize_model, bool initialize_observation_manager)
{
    Verdandi::MessageHandler::Send(*this, "all", "::Initialize begin");

    this->configuration_file_ = configuration.GetFilePath();

    configuration.Set("output_directory", "", this->configuration_file_, this->output_directory_);

    char comm[100];
    sprintf(comm, "rm %s/roukf*dat", this->output_directory_.c_str());
    std::cout << "Executing: " << comm << std::endl;
    system(comm);

    configuration.SetPrefix("reduced_order_unscented_kalman_filter.");

    configuration.Set("output.saveVQ", this->saveVQ_);

    configuration.Set("model.configuration_file", "", this->configuration_file_,
                      this->model_configuration_file_);

    configuration.Set("observation_manager.configuration_file", "",
                      this->configuration_file_,
                      this->observation_configuration_file_);

    // Should iterations be displayed on screen?
    configuration.Set("display.show_iteration",
                      this->option_display_["show_iteration"]);
    // Should current time be displayed on screen?
    configuration.Set("display.show_time", this->option_display_["show_time"]);

    configuration.Set("data_assimilation.analyze_first_step",
                      this->analyze_first_step_);
    configuration.Set("data_assimilation.with_resampling",
                      this->with_resampling_);
    configuration.Set("data_assimilation.observation_error_variance",
                      "ops_in(v, {'matrix', 'matrix_inverse'})",
                      this->observation_error_variance_);

    configuration.Set("sigma_point.type",
                      "ops_in(v, {'canonical', 'star', 'simplex'})",
                      this->sigma_point_type_);

        configuration.
            SetPrefix("reduced_order_unscented_kalman_filter"
                      ".output_saver.");
        this->output_saver_.Initialize(configuration);
        this->output_saver_.Empty("forecast_time");
        this->output_saver_.Empty("forecast_state");
        this->output_saver_.Empty("analysis_time");
        this->output_saver_.Empty("analysis_state");

        configuration.SetPrefix("reduced_order_unscented_kalman_filter.");

        if (configuration.Exists("output.configuration"))
        {
            std::string output_configuration;
            configuration.Set("output.configuration",
                              output_configuration);
            configuration.WriteLuaDefinition(output_configuration);
        }


    if (configuration.Exists("output.log"))
        Verdandi::Logger::SetFileName(configuration.Get<std::string>("output.log"));

    if (initialize_model)
    {
        this->model_.Initialize(this->model_configuration_file_);
    }
    if (initialize_observation_manager)
    {
        this->observation_manager_.Initialize(this->model_,
                                        this->observation_configuration_file_);
        this->observation_manager_.DiscardObservation(false);
    }
}*/

 template <class Model, class ObservationManager>
 void SofaReducedOrderUKFParallel<Model, ObservationManager>::InitializeStructures() {
    this->Nstate_ = this->model_.GetNstate();
    this->Nobservation_ = this->observation_manager_->GetNobservation();

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
    if (this->alpha_constant_)
        MltAdd(typename Inherit1::Ts(this->alpha_ * this->alpha_), Seldon::SeldonNoTrans, this->I_trans_, Seldon::SeldonTrans,
               this->I_trans_, typename Inherit1::Ts(0), this->D_v_);
    else
        throw Verdandi::ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                             "Initialize()", "Calculation not "
                             "implemented for no constant alpha_i.");
    std::cout<<"tha fuck\n";
    /*** Assimilation ***/

    if (this->analyze_first_step_)
        this->Analyze();
    std::cout<<"wtf\n";
    //if (initialize_model)
    {
        Verdandi::MessageHandler::Send(*this, "model", "initial condition");
        Verdandi::MessageHandler::Send(*this, "driver", "initial condition");
    }
    std::cout<<"WTF\n";
    Verdandi::MessageHandler::Send(*this, "all", "::Initialize end");
}


 template <class DataTypes1, class DataTypes2>
 void MappedPointsObservationManagerParallel<DataTypes1, DataTypes2>::init() {
     simulation::Node* gnode = dynamic_cast<simulation::Node*>(this->getContext());

     SNCOUTP("== init started");
     std::cout<<"gnode is "<<gnode->getName()<<std::endl;

     gnode->get(mapping);
     if (mapping) {
         std::cout << "[" << this->getName() << "]: " << "found mapping: " << mapping->getName() << std::endl;
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no mapping found " << std::endl;

     gnode->get(observationSource);
     if (observationSource) {
         std::cout << "[" << this->getName() << "]: " << "found observation source: " << observationSource->getName() << std::endl;
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no observation source found " << std::endl;

     gnode->get(sofaModel, core::objectmodel::BaseContext::SearchRoot);/** search in root instead*/
     if (sofaModel) {
         std::cout << "[" << this->getName() << "]: " << "found SOFA model: " << sofaModel->getName() << std::endl;
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no SOFA model found " << std::endl;

     gnode->get(mappedState);
     if (mappedState) {
         std::cout << "[" << this->getName() << "]: " << "found mapped mechanical state: " << mappedState->getName() << std::endl;
         typename MappedState::ReadVecCoord mappedPos = mappedState->readPositions();
         mappedStateSize = mappedPos.size();
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no mapped state found " << std::endl;


     m_sofaObjectParallel = NULL;
     masterState = dynamic_cast<MasterState*>(mapping->getFromModel());
     if (masterState != NULL) {
         m_sofaObjectParallel = sofaModel->getObject(masterState);
         typename MasterState::ReadVecCoord masterPos = masterState->readPositions();
         masterStateSize = masterPos.size();
         simulation::Node* test1=dynamic_cast<simulation::Node*>(masterState->getContext());
         simulation::Node* test=dynamic_cast<simulation::Node*>((test1->getParents())[0]);
         std::cout<<"MOPS master state is: "<<test->getName()<<"/"<<test1->getName()<<"/"<<masterState->getName()<<std::endl;
         std::cout<<"its size is "<<masterStateSize<<"\n";
     }
     else
         std::cerr << this->getName() << "ERROR SOFA MO not found!" << std::endl;

     if (!m_sofaObjectParallel)
         std::cerr << this->getName() << "ERROR SOFA object not found " << std::endl;

     /// initialize noise generator:
     if (m_noiseStdev.getValue() != 0.0) {
         pRandGen = new boost::mt19937;
         pNormDist = new boost::normal_distribution<>(0.0, m_noiseStdev.getValue());
         pVarNorm = new boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >(*pRandGen, *pNormDist);
     }

     SNCOUTP("== init done")
 }

 template <class DataTypes1, class DataTypes2>
 void MappedPointsObservationManagerParallel<DataTypes1, DataTypes2>::bwdInit() {
     SNCOUTP("== bwdInit started")
     typename DataTypes1::VecCoord& inputObservation = *inputObservationData.beginEdit();

     inputObservation = observationSource->getObservation(0.0);

     //std::cout << "Apply mapping on observations" << std::endl;

     MechanicalParams mp;
     mapping->apply(&mp, mappedObservationData, inputObservationData);

     noise.clear();
     noise.resize(3*mappedObservationData.getValue().size());
     SNCOUTP("== bwdInit done")
 }



 template <class DataTypes1, class DataTypes2>
 MappedPointsObservationManagerParallel<DataTypes1,DataTypes2>
 ::Inherit::observation& MappedPointsObservationManagerParallel<DataTypes1, DataTypes2>::GetInnovation(const typename SofaModelWrapperParallel<double>::state& x) {
     std::cout << "[" << this->getName() << "]: new get innovation " << std::endl;
     std::cout<<"n observation:" <<GetNobservation()<<"\n";


     //Data<typename DataTypes1::VecCoord> inputObservationData;
     //Data<typename DataTypes2::VecCoord> mappedObservationData;

     typename DataTypes1::VecCoord& inputObservation = *inputObservationData.beginEdit();

     inputObservation = observationSource->getObservation(this->time_);
     std::cout<<"getting obs at time "<<this->time_<<"\n\n";

     //std::cout << "Apply mapping on observations" << std::endl;

     MechanicalParams mp;
     mapping->apply(&mp, mappedObservationData, inputObservationData);

     //typename DataTypes2::VecCoord mappedObservation = *mappedObservationData.beginEdit();

     sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObservation = mappedObservationData;

     //std::cout << this->getName() << ": size of mapped observation: " << mappedObservation.size() << std::endl;
     Inherit::observation actualObs(mappedObservation.size()*3);
     for (size_t i = 0; i < mappedObservation.size(); i++)
         for (size_t d = 0; d < 3; d++) {
             mappedObservation[i][d] += noise[3*i+d];
             actualObs(3*i+d) = mappedObservation[i][d];
         }

     Data<typename DataTypes1::VecCoord> actualStateData;
     Data<typename DataTypes2::VecCoord> mappedStateData;

     typename DataTypes1::VecCoord& actualState = *actualStateData.beginEdit();
     typename DataTypes2::VecCoord& mappedState = *mappedStateData.beginEdit();

     mappedState.resize(mappedStateSize);
     sofaModel->SetSofaVectorFromVerdandiState(actualState, x, m_sofaObjectParallel);

     mapping->apply(&mp, mappedStateData, actualStateData);

     std::cout << this->getName() << ": size of mapped state: " << mappedState.size() << std::endl;
     this->innovation_.Reallocate(mappedState.size()*3);
     for (size_t i = 0; i < mappedState.size(); i++)
         for (size_t d = 0; d < 3; d++)
             this->innovation_(3*i+d) = mappedState[i][d];

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
 }

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
 }

}

}

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
