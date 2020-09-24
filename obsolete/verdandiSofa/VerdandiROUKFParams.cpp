/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "VerdandiROUKFParams.h"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace simulation
{

SOFA_DECL_CLASS(VerdandiROUKFParams)

int VerdandiROUKFParamsClass = core::RegisterObject("Parameters that will be adapted by ROUKF object")
        .add< VerdandiROUKFParams >()
        ;

VerdandiROUKFParams::VerdandiROUKFParams()
    : Inherit()
    , m_outputDirectory( initData(&m_outputDirectory, "outputDirectory", "working directory of the filter") )
    , m_configFile( initData(&m_configFile, "configFile", "lua configuration file (temporary)") )
    , m_sigmaPointType( initData(&m_sigmaPointType, std::string("star"), "sigmaPointType", "type of sigma points (canonical|star|simplex)") )
    , m_observationErrorVariance( initData(&m_observationErrorVariance, std::string("matrix_inverse"), "observationErrorVariance", "observationErrorVariance") )
    , m_saveVQ( initData(&m_saveVQ, true, "saveVQ", "m_saveVQ") )
    , m_showIteration( initData(&m_showIteration, false, "showIteration", "showIteration") )
    , m_showTime( initData(&m_showTime, true, "showTime", "showTime") )
    , m_analyzeFirstStep( initData(&m_analyzeFirstStep, false, "analyzeFirstStep", "analyzeFirstStep") )
    , m_withResampling( initData(&m_withResampling, false, "withResampling", "withResampling") )
    , m_positionInState( initData(&m_positionInState, true, "positionInState", "include position in the non-reduced state") )
    , m_velocityInState( initData(&m_velocityInState, false, "velocityInState", "include position in the non-reduced state") )
{

}

VerdandiROUKFParams::~VerdandiROUKFParams()
{
}


} // namespace simulation

} // namespace sofa
