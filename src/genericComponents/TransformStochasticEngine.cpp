#define SOFA_COMPONENT_ENGINE_TransformStochasticEngine_CPP

#include <genericComponents/TransformStochasticEngine.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace engine
{

SOFA_DECL_CLASS(TransformStochasticEngine)

int TransformStochasticEngineClass = core::RegisterObject("Transform position of 3d points")
        .add< TransformStochasticEngine<defaulttype::Vec3dTypes> >(true) // default template
        ;

template class SOFA_OPTIMUSPLUGIN_API TransformStochasticEngine<defaulttype::Vec3dTypes>;

} // namespace constraint

} // namespace component

} // namespace sofa

