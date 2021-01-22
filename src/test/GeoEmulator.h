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
/******************************************************************************
  *  detect and handle events from geomagic device
 *****************************************************************************/
#pragma once

/* include files */
#include <SofaDeformable/config.h>
#include <cstdint>
#include "../initOptimusPlugin.h"
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>



namespace sofa {

namespace component {

namespace behavior {


/**
 *        class GeomagicEmulator
 * <p>
 *   description:
 *       class to emulate button
 *       pressing on geomagic device
 */
class GeoEmulator : public core::objectmodel::BaseObject {

public:
    SOFA_CLASS(GeoEmulator, core::objectmodel::BaseObject);

    typedef defaulttype::RigidTypes::Coord Coord;

    // updating and accumulating Von Mises stress
    void handleEvent(sofa::core::objectmodel::Event* event) override;


    // constructor and destructor
    GeoEmulator();
    virtual ~GeoEmulator();

    virtual void reinit() override { }
    virtual void init() override;
    virtual void bwdInit() override;
    void cleanup() override;

protected:
    Data<Coord> d_devicePosition;
    Data<int> d_firstButton;
    Data<int> d_secondButton;

    sofa::core::objectmodel::DataFileName d_positionSourceFilename;
    sofa::core::objectmodel::DataFileName d_buttonSourceFilename;

    Data<helper::vector<double>> timeData;
    helper::vector<int> m_deviceSecondButton;
    helper::vector< helper::vector<Coord> > m_devicePositions;

    size_t m_currentIndex;

    // parce input file
    void parseSourcePositionsFile();
    void parseSourceButtonFile();
};

} // namespace behavior

} // namespace component

} // namespace sofa
