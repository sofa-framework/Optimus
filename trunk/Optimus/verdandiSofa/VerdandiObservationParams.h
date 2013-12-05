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
#ifndef SOFA_SIMULATION_VERDANDI_OBSPARAMS_H
#define SOFA_SIMULATION_VERDANDI_OBSPARAMS_H

#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/common/common.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Data.h>
#include <string>

using namespace sofa::core::objectmodel;

namespace sofa
{

namespace simulation
{

/**
  Event fired when needed to stop the animation.
*/
class SOFA_SIMULATION_COMMON_API VerdandiObservationParams : public sofa::core::objectmodel::BaseObject
{
public:
    typedef sofa::core::objectmodel::BaseObject Inherit;

    VerdandiObservationParams();
    ~VerdandiObservationParams();

    /*Data<std::string> m_outputDirectory, m_configFile, m_sigmaPointType, m_observationErrorVariance;
    Data<bool> m_saveVQ, m_showIteration, m_showTime, m_analyzeFirstStep, m_withResampling;
    Data<bool> m_positionInState, m_velocityInState;*/

    //virtual const char* getClassName() const { return "VerdandiROUKFParams"; }

};

} // namespace simulation

} // namespace sofa

#endif
