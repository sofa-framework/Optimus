/******************************************************************************
  *  average von mises stress class registration
 *****************************************************************************/
#define SOFA_COMPONENT_GEO_EMULATOR_CPP

/* include files */
#include "GeoEmulator.inl"
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/TemplatesAliases.h>


namespace sofa {

namespace component {

namespace behavior {

using namespace sofa::defaulttype;


SOFA_DECL_CLASS(GeoEmulator)

int GeoEmulatorClass = core::RegisterObject("Geomagic device manipulating object")
.add<GeoEmulator>();


} // namespace bahavior

} // namespace component

} // namespace sofa
