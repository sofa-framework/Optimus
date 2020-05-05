/******************************************************************************
  *  detect and handle events from geomagic device
 *****************************************************************************/
#pragma once

/* include files */
#include <SofaDeformable/config.h>
#include <cstdint>
#include "../initOptimusPlugin.h"
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/Vec3Types.h>
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

