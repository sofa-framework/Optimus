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

#include "optimusConfig.h"

#include "VTKExporterDA.h"
#include "FilterEvents.h"



namespace sofa
{

namespace component
{

namespace exporter
{



SOFA_DECL_CLASS(VTKExporterDA)

int VTKExporterClassDA = core::RegisterObject("Save geometries in VTK, compatible with Optimus data assimilation")
    .add< VTKExporterDA >();



void VTKExporterDA::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (sofa::component::stochastic::CorrectionEndEvent::checkEventType(event)) {
        unsigned int maxStep = this->exportEveryNbSteps.getValue();
        if (maxStep == 0) return;

        this->stepCounter++;
        if(this->stepCounter >= maxStep)
        {
            this->stepCounter = 0;
            if(fileFormat.getValue())
                writeVTKXML();
            else
                writeVTKSimple();
        }
    }
}



} // namespace misc

} // namespace component

} // namespace sofa

