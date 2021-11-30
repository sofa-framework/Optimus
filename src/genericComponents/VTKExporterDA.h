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
#pragma once


#include "initOptimusPlugin.h"
#include <sofa/core/ObjectFactory.h>
#include <SofaExporter/VTKExporter.h>

//#include <sofa/core/objectmodel/Event.h>
//#include <sofa/helper/AdvancedTimer.h>
//#include <sofa/simulation/PropagateEventVisitor.h>



namespace sofa
{

namespace component
{

namespace misc
{

class SOFA_OPTIMUSPLUGIN_API VTKExporterDA : public VTKExporter
{
public:
    SOFA_CLASS(VTKExporterDA,VTKExporter);

    void handleEvent(sofa::core::objectmodel::Event *) override;
};


} // namespace misc

} // namespace component

} // namespace sofa

