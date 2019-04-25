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

#ifndef OPTIMPARAMS_H_
#define OPTIMPARAMS_H_

//#include <SofaExporter/config.h>

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

}

}

}

#endif

