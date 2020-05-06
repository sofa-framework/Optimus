/******************************************************************************
  *  average von mises stress class registration
 *****************************************************************************/
#define SOFA_COMPONENT_GEO_LISTENER_CPP

/* include files */
#include "GeoListener.inl"
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/TemplatesAliases.h>


namespace sofa {

namespace component {

namespace behavior {

using namespace sofa::defaulttype;


SOFA_DECL_CLASS(GeomagicDeviceListener)

int GeomagicDeviceListenerClass = core::RegisterObject("Geomagic device manipulating object")
        .add< GeoListener<Vec3Types> >(true)
        .add< GeoListener<Vec6Types> >()
        .add< GeoListener<Rigid3Types> >()
        ;

template class GeoListener<Vec3Types>;
template class GeoListener<Vec6Types>;
template class GeoListener<Rigid3Types>;

} // namespace bahavior

} // namespace component

} // namespace sofa

