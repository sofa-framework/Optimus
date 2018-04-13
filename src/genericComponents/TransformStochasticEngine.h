#ifndef SOFA_COMPONENT_ENGINE_TransformStochasticEngine_H
#define SOFA_COMPONENT_ENGINE_TransformStochasticEngine_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/topology/BaseMeshTopology.h>

#include <sofa/defaulttype/Quat.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace component
{

namespace engine
{

/**
 * This class transforms the positions of one DataFields into new positions after applying a transformation
This transformation can be either translation, rotation, scale
 */
template <class DataTypes>
class TransformStochasticEngine : public core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(TransformStochasticEngine,DataTypes),core::DataEngine);
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef sofa::core::topology::BaseMeshTopology::SeqTriangles SeqTriangles;
    typedef sofa::core::topology::BaseMeshTopology::SeqQuads SeqQuads;
    typedef typename SeqTriangles::value_type Triangle;
    typedef typename SeqQuads::value_type Quad;

protected:

    TransformStochasticEngine();

    ~TransformStochasticEngine() {}
public:
    void init() override;
    void bwdInit() override;

    void reinit() override;

    void update() override;

    virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const TransformStochasticEngine<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

protected:
    Data<VecCoord> f_inputX; // input position
    Data<VecCoord> f_outputX; // ouput position
    Data<defaulttype::Vector3> translation; // translation
    Data<defaulttype::Vector3> rotation; // rotation
    Data<defaulttype::Quaternion> quaternion; // quaternion rotation
    Data<defaulttype::Vector3> scale; // scale
    Data<bool> inverse;
    Data<helper::vector<double>> d_optimParams;
    Data<SeqTriangles> d_triangles; //< input triangles
    Data<SeqQuads> d_quads; //< input quads
    Data<VecCoord> d_normals; //< ouput normals
    Data<bool> d_stochEstim;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_ENGINE_TransformStochasticEngine_CPP)
extern template class SOFA_GENERAL_ENGINE_API TransformStochasticEngine<defaulttype::Vec3dTypes>;

#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif
