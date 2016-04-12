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

#include <sofa/simulation/common/MechanicalVisitor.h>

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
    , velocityInState( this->initData(&velocityInState, false, "includeVelocity", "include the velocity in the stochastic state") )
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
    this->gnode->get(mechanicalState,  core::objectmodel::BaseContext::SearchRoot);
    if (!mechanicalState) {
        PRNE("No mechanical state found");
        valid=false;
    } else {
        PRNS("Mechanical state found: " << mechanicalState->getName());
    }

    /// get optim params (multiple per node, at least one)
    vecOptimParams.clear();
    this->gnode->template get<OptimParamsBase>(&vecOptimParams, core::objectmodel::BaseContext::SearchRoot );
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
    this->gnode->get(fixedConstraint,  core::objectmodel::BaseContext::SearchRoot);
    if (fixedConstraint) {
        PRNS("Fixed constraint found: " << fixedConstraint->getName());
    }

}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::bwdInit() {
    if (!valid)
        return;

    /// extract free nodes (fixed nodes cannot be included in the stochastic state)
    helper::vector<size_t> freeNodes;
    for (size_t msi = 0; msi < mechanicalState->getSize(); msi++) {
        bool found = false;
        if (fixedConstraint) {
            const typename FixedConstraint::SetIndexArray& fix = fixedConstraint->f_indices.getValue();
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
void StochasticStateWrapper<DataTypes, FilterType>::copyStateVerdandi2Sofa() {
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

    sofa::core::MechanicalParams mp;
    sofa::simulation::MechanicalPropagatePositionAndVelocityVisitor(&mp).execute( this->gnode );

    /// let the OptimParams to extract the actual values of parameters from the verdandi state
    /// actually: rawVector functions in optimParams do nothing, so we omit it here...
    /*for (size_t opi = 0; opi < vecOptimParams.size(); opi++)
        vecOptimParams[opi]->rawVectorToParams(state_.GetData(), state_.GetM());*/

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

}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::applyOperator() {
}

//template <class DataTypes>
//void StochasticStateWrapper<DataTypes>::reinit()
//{
//}

} // simulation
} // component
} // sofa

#endif // SOFASTATEWRAPPER_INL
