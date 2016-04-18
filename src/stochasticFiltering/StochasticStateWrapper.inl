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
#ifndef SOFASTATEWRAPPER_INL
#define SOFASTATEWRAPPER_INL

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
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/PipeProcess.h>
#include <sofa/helper/AdvancedTimer.h>



#include "StochasticStateWrapper.h"


namespace sofa
{
namespace component
{
namespace stochastic
{

template <class DataTypes, class FilterType>
StochasticStateWrapper<DataTypes, FilterType>::StochasticStateWrapper()
    :Inherit()
    , velocityInState( initData(&velocityInState, false, "includeVelocity", "include the velocity in the stochastic state") )
{    
}

template <class DataTypes, class FilterType>
StochasticStateWrapper<DataTypes, FilterType>::~StochasticStateWrapper()
{
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::init()
{
    Inherit::init();
    valid = true;
    /// get a mechanical state
    this->gnode->get(mechanicalState,  core::objectmodel::BaseContext::SearchDown);
    if (!mechanicalState) {
        PRNE("No mechanical state found");
        valid=false;
    } else {
        PRNS("Mechanical state found: " << mechanicalState->getName());
    }

    /// get optim params (multiple per node, at least one)
    vecOptimParams.clear();
    this->gnode->template get<OptimParamsBase>(&vecOptimParams, core::objectmodel::BaseContext::SearchDown );
    if (vecOptimParams.empty()) {
        PRNE("No OptimParams found");
        valid=false;
    } else {
        PRNSC("OptimParams found " << vecOptimParams.size() << "x: ");
        if (this->verbose.getValue()) {
            for (size_t i = 0; i < vecOptimParams.size(); i++)
                std::cout << vecOptimParams[i]->getName() << " ";
            std::cout << std::endl;
        }
    }

    /// get fixed constraints (optional)
    this->gnode->get(fixedConstraint,  core::objectmodel::BaseContext::SearchDown);
    if (fixedConstraint) {
        PRNS("Fixed constraint found: " << fixedConstraint->getName());
    }

}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::bwdInit() {
    if (!valid)
        return;

    /// extract free and fixed nodes (fixed nodes cannot be included in the stochastic state)
    fixedNodes.clear();
    freeNodes.clear();
    if (fixedConstraint) {
        const typename FixedConstraint::SetIndexArray& fix = fixedConstraint->f_indices.getValue();
        for (size_t i = 0; i < fix.size(); i++)
            fixedNodes.push_back(size_t(fix[i]));

        for (size_t i = 0; i < mechanicalState->getSize(); i++) {
            helper::vector<size_t>::iterator it = find(fixedNodes.begin(), fixedNodes.end(), i);

            if (it == fixedNodes.end())
                freeNodes.push_back(i);
        }
    } else {
        for (size_t i = 0; i < mechanicalState->getSize(); i++)
            freeNodes.push_back(i);
    }

    /*std::cout << "Fixed nodes: " << std::endl;
    for (size_t i = 0; i < fixedNodes.size(); i++)
        std::cout << fixedNodes[i] << " ";
    std::cout << std::endl;

    std::cout << "Free nodes: " << std::endl;
    for (size_t i = 0; i < freeNodes.size(); i++)
        std::cout << freeNodes[i] << " ";
    std::cout << std::endl;*/

    positionPairs.clear();
    velocityPairs.clear();

    size_t vsi = 0;
    size_t vpi = 0;

    for (size_t i = 0; i < freeNodes.size(); i++) {
        std::pair<size_t, size_t> pr(freeNodes[i], vsi++);
        positionPairs.push_back(pr);
    }

    if (velocityInState.getValue()) {
        for (size_t i = 0; i < freeNodes.size(); i++) {
            std::pair<size_t, size_t> pr(freeNodes[i], vsi++);
            velocityPairs.push_back(pr);
        }
    }

    this->reducedStateIndex = Dim * vsi;
    for (size_t pi = 0; pi < vecOptimParams.size(); pi++) {
        helper::vector<size_t> opv;
        for (size_t i = 0; i < vecOptimParams[pi]->size(); i++, vpi++) {
            opv.push_back(this->reducedStateIndex+vpi);
        }
        vecOptimParams[pi]->setVStateParamIndices(opv);
    }

    this->stateSize = Dim * vsi + vpi;
    this->reducedStateSize = vpi;

    PRNS("Initializing stochastic state with size " << this->stateSize);
    PRNS("Reduced state index: " << this->reducedStateIndex << " size: " << this->reducedStateSize);

    this->state.resize(this->stateSize);
    copyStateSofa2Verdandi();
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::setSofaVectorFromVerdandiVector(EVectorX& _state, typename DataTypes::VecCoord& _vec) {
    if (_vec.size() != mechanicalState->getSize()) {
        PRNE("Input vector not compatible with the actual Sofa state size");
        return;
    }

    typename MechanicalState::ReadVecCoord pos = mechanicalState->readPositions();
    for (size_t fni = 0; fni < fixedNodes.size(); fni++) {
        size_t fn = fixedNodes[fni];
        _vec[fn] = pos[fn];
        //PRNS("Setting fixed[" << fn << "] = " << _vec[fn]);
    }

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
        if (it->first >= _vec.size()) {
            PRNE("Accessing Sofa vector out of bounds: " << it->first <<  " vs. " << _vec.size());
            return;
        }

        if ((Dim*it->second + Dim) >= _state.rows()) {
            PRNE("Accessing DA vector out of bounds: " << Dim*it->second + Dim <<  " vs. " << _state.rows());
            return;
        }

        for (size_t d = 0; d < Dim; d++) {
            _vec[it->first][d] = _state(Dim*it->second + d);
        }
        //PRNS("Setting free[" << it->first << "] = " << _vec[it->first]);
    }
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::copyStateVerdandi2Sofa(const core::MechanicalParams* _mechParams) {
    typename MechanicalState::WriteVecCoord pos = mechanicalState->writePositions();
    typename MechanicalState::WriteVecDeriv vel = mechanicalState->writeVelocities();

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
        for (size_t d = 0; d < Dim; d++) {
            pos[it->first][d] = this->state(Dim*it->second + d);
        }
    }

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = velocityPairs.begin(); it != velocityPairs.end(); it++) {
        for (size_t d = 0; d < Dim; d++) {
            vel[it->first][d] = this->state(Dim*it->second + d);
        }
    }


    sofa::simulation::MechanicalPropagatePositionAndVelocityVisitor(_mechParams).execute( this->gnode );


    /// let the OptimParams to extract the actual values of parameters from the verdandi state
    for (size_t opi = 0; opi < vecOptimParams.size(); opi++)
        vecOptimParams[opi]->vectorToParams(this->state);
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::copyStateSofa2Verdandi() {
    typename MechanicalState::ReadVecCoord pos = mechanicalState->readPositions();
    typename MechanicalState::ReadVecDeriv vel = mechanicalState->readVelocities();

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++)
        for (size_t d = 0; d < Dim; d++) {
            this->state(Dim*it->second + d) = pos[it->first][d];
        }

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = velocityPairs.begin(); it != velocityPairs.end(); it++)
        for (size_t d = 0; d < Dim; d++)
            this->state(Dim*it->second + d) = vel[it->first][d];

    for (size_t opi = 0; opi < vecOptimParams.size(); opi++)
        vecOptimParams[opi]->paramsToVector(this->state);

}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::applyOperator(EVectorX &_vecX, const core::MechanicalParams *_mparams, bool _preserveState, bool _updateForce) {
    EVectorX savedState;

    if (_preserveState)
        savedState = this->state;

    this->state = _vecX;    
    copyStateVerdandi2Sofa(_mparams);
    computeSofaStep(_mparams, false);
    copyStateSofa2Verdandi();
    _vecX = this->state;

    if (_preserveState) {
        this->state = savedState;
        copyStateVerdandi2Sofa(_mparams);
    }
}

/*template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::setSofaTime(const core::ExecParams* _execParams) {
    std::cout << "Setting SOFA time with " << this->actualTime << std::endl;
    this->gnode->setTime (this->actualTime);
    this->gnode->template execute< sofa::simulation::UpdateSimulationContextVisitor >(_execParams);
}*/

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::computeSofaStep(const core::ExecParams* execParams, bool _updateTime) {
    double    dt = this->gnode->getDt();
    //core::ExecParams* execParams = sofa::core::ExecParams::defaultInstance();

    //std::cout << "[" << this->getName() << "]: step default begin at time = " << gnode->getTime() << " update time: " << _update_time << std::endl;

    sofa::helper::AdvancedTimer::stepBegin("AnimationStep");
    //std::cout<<"step "<<step++<<std::endl;
    sofa::helper::AdvancedTimer::begin("Animate");
    //std::cout<<"step "<<step++<<std::endl;

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    {
        //std::cout<<"step "<<step++<<std::endl;
        //std::cout << "[" << this->getName() << "]: animate begin" << std::endl;
        sofa::simulation::AnimateBeginEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act ( execParams, &ev );
        this->gnode->execute ( act );
        //std::cout<<"step "<<step++<<std::endl;
    }

    double startTime = this->gnode->getTime();
    //std::cout<<"step "<<step++<<std::endl;
    //std::cout << "[" << this->getName() << "]: behaviour update position" << std::endl;
    sofa::simulation::BehaviorUpdatePositionVisitor beh(execParams , dt);
    this->gnode->execute ( beh );
    //std::cout<<"step "<<step++<<std::endl;
    //std::cout << "[" << this->getName() << "]: animate" << std::endl;
    sofa::simulation::AnimateVisitor act(execParams, dt);
    this->gnode->execute ( act );

    if (_updateTime) {
        //std::cout << "[" << this->getName() << "]: update simulation context" << std::endl;
        this->gnode->setTime ( startTime + dt );
        this->gnode->template execute<  sofa::simulation::UpdateSimulationContextVisitor >(execParams);
    }
    //std::cout<<"step "<<step++<<std::endl;
    {
        //std::cout << "[" << this->getName() << "]: animate end" << std::endl;
        sofa::simulation::AnimateEndEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act (execParams, &ev );
        this->gnode->execute ( act );
    }

    sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    //std::cout << "[" << this->getName() << "]: update mapping" << std::endl;
    this->gnode->template execute<  sofa::simulation::UpdateMappingVisitor >(execParams);
    sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        //std::cout << "[" << this->getName() << "]: update mapping end" << std::endl;
        sofa::simulation::UpdateMappingEndEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act ( execParams , &ev );
        this->gnode->execute ( act );
    }
    sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");

#ifndef SOFA_NO_UPDATE_BBOX
    sofa::helper::AdvancedTimer::stepBegin("UpdateBBox");
    this->gnode->template execute<  sofa::simulation::UpdateBoundingBoxVisitor >(execParams);
    sofa::helper::AdvancedTimer::stepEnd("UpdateBBox");
#endif
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("Step");
#endif
    sofa::helper::AdvancedTimer::end("Animate");
    sofa::helper::AdvancedTimer::stepEnd("AnimationStep");

}



} // simulation
} // component
} // sofa

#endif // SOFASTATEWRAPPER_INL
