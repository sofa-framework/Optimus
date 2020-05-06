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
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaDeformable/RestShapeSpringsForceField.h>
#include <SofaBaseMechanics/BarycentricMapping.h>



namespace sofa {

namespace component {

namespace behavior {


/**
 *        class GeomagicDeviceListener
 * <p>
 *   description:
 *       class to detect and handle
 *       events from geomagic device
 */
template <class DataTypes>
class GeoListener : public core::objectmodel::BaseObject {

public:
    SOFA_CLASS(SOFA_TEMPLATE(GeoListener, DataTypes), core::objectmodel::BaseObject);

    // updating and accumulating Von Mises stress
    void handleEvent(sofa::core::objectmodel::Event* event) override;


    // constructor and destructor
    GeoListener();
    virtual ~GeoListener();

    //virtual void reinit();
    virtual void init() override;
    void cleanup() override;

    virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const GeoListener<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }


protected:
    Data <std::string> d_geomagicButtonPath;
    Data<bool> d_geomagicButtonPressed;
    Data<bool> d_geomagicSecondButtonPressed;
    Data< defaulttype::RigidTypes::Coord > d_devicePosition;
    Data< defaulttype::RigidTypes::Coord > d_deviceLastPosition;
    Data<bool> d_detachStiffSpring;
    bool m_buttonWasPressed;
    bool m_secondButtonWasPressed;
    Data<bool> d_savePression;
    sofa::core::objectmodel::DataFileName d_pressFilename;


    // geometry data for visualisation
    core::behavior::MechanicalState<DataTypes>* m_mstate;
    forcefield::RestShapeSpringsForceField<DataTypes>* m_spring;
    component::mapping::BarycentricMapping<defaulttype::Vec3dTypes, DataTypes>* m_mapping;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_GEOMAGIC_DEVICE_LISTENER_CPP)
extern template class GeoListener<defaulttype::Vec3Types>;
extern template class GeoListener<defaulttype::Vec6Types>;
extern template class GeoListener<defaulttype::Rigid3Types>;
#endif

} // namespace behavior

} // namespace component

} // namespace sofa

